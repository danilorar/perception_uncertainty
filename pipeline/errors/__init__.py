"""Error plugins. Every module in this folder with SETTINGS and a class Error is one plugin.

A configuration lists the plugins to use, in the order they run:

    "errors": [{"type": "size", "scale": 1.1}, {"type": "detection", "miss_rate": 0.1}]

"type" is the module name, "name" is needed only when a type appears twice, the rest are settings.
To write a plugin, copy template.py. A plugin module has:

    SETTINGS = {...}            default of every setting; a configuration may only set these
    RANGES = {...}              optional: (low, high) of number settings
    CHOICES = {...}             optional: allowed values of text settings
    LOG = (...)                 optional: values the plugin writes to target['log'] (perception_applied.tsv)
    def check(settings, run)    optional: further checks; raise ValueError
    class Error:
        def __init__(self, settings, run)           once per run
        def perceive(self, step, t, ego, targets)   before every controller step; change targets in place
        def describe(self)                          optional: saved in summary.json

run has seed, dt_s, steps, duration_s, scenario, ego (name), targets ([{'id', 'name'}]) and label.
A target is a dict:
    read only   id, name, true_range, true (x, y, z, heading, speed, vx, vy, vz, ax, ay, az, length, width, height)
    change      range        centre-to-centre distance; the target moves along the line of sight
                x, y         centre of its box; when changed, they win over range
                heading, speed, vx, vy, vz, ax, ay, az, length, width
                seen         False: the controller does not see the target this step
                log          values for perception_applied.tsv, under the names in LOG
    shared      class_correct (identification sets it, size reads it), predicted (set by detection)
ego has id, name and the same fields as true; it is read only. A negative range hides the target.
The controller sees the result for one step; esmini puts the true state back right after it.
"""
import csv
import importlib
import math
from pathlib import Path
import pkgutil
from types import SimpleNamespace

FOLDER = Path(__file__).resolve().parent
STATE = ('x', 'y', 'z', 'heading', 'speed', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'length', 'width', 'height')
MOTION = ('speed', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')
PREFIX = b'@@PB'
# Error samples run this long past the scenario end, so a run can never run out of them.
MARGIN_S = 2.0
LOG_FIELDS = ['time_s', 'object_name', 'true_range_m', 'raw_range_m', 'valid', 'observed_center_x_m',
              'observed_center_y_m', 'class_correct', 'predicted']
SEEN, POSITION, HEADING, MOVING, SIZE = 1, 2, 4, 8, 16


def all_plugins():
    found = {}
    for info in pkgutil.iter_modules([str(FOLDER)]):
        if not info.name.startswith('_'):
            module = importlib.import_module(f'{__name__}.{info.name}')
            if hasattr(module, 'Error') and hasattr(module, 'SETTINGS'):
                found[info.name] = module
    return dict(sorted(found.items()))


def label(entry):
    return entry.get('name') or entry['type']


def check_setting(where, key, value, default, module):
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise ValueError(f'{where}.{key}: must be true or false')
        return value
    if isinstance(default, (int, float)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{where}.{key}: must be a number')
        low, high = getattr(module, 'RANGES', {}).get(key, (-math.inf, math.inf))
        if not low <= value <= high:
            raise ValueError(f'{where}.{key}: must be between {low} and {high}')
        return float(value) if isinstance(default, float) else value
    choices = getattr(module, 'CHOICES', {}).get(key)
    if choices and value not in choices:
        raise ValueError(f'{where}.{key}: must be one of {", ".join(choices)}')
    return value


def validate(entries, c):
    """Complete and check the "errors" list of configuration c."""
    if not isinstance(entries, list):
        raise ValueError('"errors" must be a list, such as [{"type": "size", "scale": 1.1}].')
    plugins = all_plugins()
    checked, labels = [], set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict) or entry.get('type') not in plugins:
            raise ValueError(f'errors[{index}]: needs a "type" out of {", ".join(plugins)}')
        if 'name' in entry and not (isinstance(entry['name'], str) and entry['name'] and '.' not in entry['name']):
            raise ValueError(f'errors[{index}]: "name" must be a short text without dots')
        where = f'errors.{label(entry)}'
        if label(entry) in labels:
            raise ValueError(f'{where}: used twice; give each a different "name"')
        labels.add(label(entry))
        module = plugins[entry['type']]
        raw = {key: value for key, value in entry.items() if key not in ('type', 'name')}
        extra = set(raw) - set(module.SETTINGS)
        if extra:
            raise ValueError(f'{where}: unknown settings {", ".join(sorted(extra))}; '
                             f'known: {", ".join(module.SETTINGS) or "none"}')
        settings = {key: check_setting(where, key, raw.get(key, default), default, module)
                    for key, default in module.SETTINGS.items()}
        if hasattr(module, 'check'):
            try:
                module.check(settings, SimpleNamespace(seed=c['seed'], dt_s=c['dt_s']))
            except ValueError as error:
                raise ValueError(f'{where}: {error}') from None
        checked.append({'type': entry['type'], **({'name': entry['name']} if 'name' in entry else {}), **settings})
    return checked


class Chain:
    """The plugins of one run. Answers the bridge in esmini (engine/PerceptionBridge.hpp) once per controller step."""

    def __init__(self, c, scenario, log_path):
        plugins = all_plugins()
        self.dt = c['dt_s']
        self.steps = round((c['duration_s'] + MARGIN_S) / c['dt_s']) + 1
        self.ids = {item['name']: item['id'] for item in scenario.objects}
        self.plugins, self.columns = [], []
        for entry in c['errors']:
            module = plugins[entry['type']]
            run = SimpleNamespace(seed=c['seed'], dt_s=c['dt_s'], steps=self.steps, duration_s=c['duration_s'],
                                  scenario=c['scenario'], ego=scenario.ego, targets=scenario.targets, label=label(entry))
            settings = {key: value for key, value in entry.items() if key not in ('type', 'name')}
            self.plugins.append((label(entry), module.Error(settings, run)))
            self.columns += [f'{label(entry)}.{name}' for name in getattr(module, 'LOG', ())]
        self.log = Path(log_path).open('w', encoding='utf-8', newline='')
        self.writer = csv.writer(self.log, delimiter='\t', lineterminator='\n')
        self.writer.writerow(LOG_FIELDS + self.columns)

    def describe(self):
        return [{'plugin': name, **(plugin.describe() if hasattr(plugin, 'describe') else {})} for name, plugin in self.plugins]

    def close(self):
        self.log.close()

    def answer(self, line):
        """One request line from esmini in, one reply line out."""
        cells = line.rstrip(b'\r\n').split(b'\t')
        t = float.fromhex(cells[2].decode('ascii'))
        objects = []
        for index in range(int(cells[3])):
            start = 4 + index * 16
            values = [float.fromhex(cell.decode('ascii')) for cell in cells[start + 1:start + 16]]
            objects.append((cells[start].decode('utf-8'), values))
        if len(cells) != 4 + 16 * len(objects):
            raise RuntimeError(f'Malformed message from esmini at t={t}')
        ego_name, ego_values = objects[0]
        ego = {'id': self.ids.get(ego_name), 'name': ego_name, **dict(zip(STATE, ego_values))}
        targets = []
        for name, values in objects[1:]:
            true = dict(zip(STATE, values))
            targets.append({'id': self.ids.get(name), 'name': name, 'true': true, 'true_range': values[14],
                            'range': values[14], **true, 'seen': True, 'class_correct': True, 'predicted': False,
                            'log': {}})
        step = round(t / self.dt)
        if step >= self.steps:
            raise RuntimeError(f'The run is longer than its error samples (t={t} s).')
        notes = [{} for _ in targets]
        for name, plugin in self.plugins:
            for target in targets:
                target['log'] = {}
            plugin.perceive(step, t, dict(ego), targets)
            for target, note in zip(targets, notes):
                note.update({f'{name}.{key}': value for key, value in target['log'].items()})
        reply = [PREFIX, b'R']
        for target, note in zip(targets, notes):
            reply += self.encode(target)
            self.write_log(t, ego, target, note)
        return b'\t'.join(reply) + b'\n'

    @staticmethod
    def flags(target):
        true = target['true']
        position = target['x'] != true['x'] or target['y'] != true['y']
        seen = bool(target['seen']) and (position or target['range'] >= 0.0)
        return (SEEN * seen | POSITION * position | HEADING * (target['heading'] != true['heading'])
                | MOVING * any(target[key] != true[key] for key in MOTION)
                | SIZE * (target['length'] != true['length'] or target['width'] != true['width']))

    def encode(self, target):
        values = [target['range'], target['x'], target['y'], target['heading'], *(target[key] for key in MOTION),
                  target['length'], target['width']]
        return [str(self.flags(target)).encode('ascii'), *(float(value).hex().encode('ascii') for value in values)]

    def write_log(self, t, ego, target, note):
        flags, true = self.flags(target), target['true']
        if flags & POSITION:
            raw, x, y = math.hypot(target['x'] - ego['x'], target['y'] - ego['y']), target['x'], target['y']
        else:
            # The same arithmetic as the bridge, so the logged centre is where esmini put the target.
            raw = target['range']
            scale = raw / target['true_range'] if target['true_range'] > 1e-12 else 0.0
            x, y = ego['x'] + (true['x'] - ego['x']) * scale, ego['y'] + (true['y'] - ego['y']) * scale
        valid = bool(flags & SEEN)
        self.writer.writerow([repr(t), target['name'], repr(target['true_range']), repr(raw), int(valid),
                              repr(x) if valid else '', repr(y) if valid else '', int(bool(target['class_correct'])),
                              int(bool(target['predicted']))] + [note.get(column, '') for column in self.columns])
