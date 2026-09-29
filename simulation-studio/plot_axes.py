"""Shared, expanding plot scales for comparable runs of the same scenario."""
from contextlib import contextmanager
import csv
import json
import math
import os
from pathlib import Path
import shutil
import time

from matplotlib.ticker import MaxNLocator
from settings import write_json

AXES_VERSION = 1
PLOT_METRICS = {'speed': ('speed', 'acceleration'), 'range': ('range', 'range_error')}


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return default


@contextmanager
def axes_lock(folder):
    # CLI batches and independent workers may finish together. Serialize the
    # registry update AND rendering so an older scale cannot overwrite a new one.
    with (folder / '.plot-axes.lock').open('a+b') as lock:
        if lock.tell() == 0:
            lock.write(b'0')
            lock.flush()
        lock.seek(0)
        if os.name == 'nt':
            import msvcrt
            deadline = time.monotonic() + 180
            while True:
                try:
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                    break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Timed out waiting for shared plot axes')
                    time.sleep(.1)
        else:
            import fcntl
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            lock.seek(0)
            if os.name == 'nt':
                msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def include(bounds, key, value):
    if value in (None, ''):
        return
    number = float(value)
    if math.isfinite(number):
        limits = bounds.setdefault(key, [0., 0.])
        limits[0] = min(limits[0], number)
        limits[1] = max(limits[1], number)


def measure(out, config, bounds, speed_only):
    include(bounds, 'time', config.get('duration_s'))
    with (out / 'ego.csv').open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            include(bounds, 'time', row['time_s'])
            for field in ('speed_kph', 'ideal_ego_speed_kph'):
                include(bounds, 'speed', row.get(field))
            if config.get('controller') in ('original_trajectory', 'aeb_follow'):
                include(bounds, 'acceleration', row.get('trajectory_accel_mps2'))
            if config.get('controller') != 'original_trajectory':
                include(bounds, 'acceleration', row.get('command_accel_mps2'))
    if speed_only:
        return
    with (out / 'observations.csv').open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            include(bounds, 'time', row['time_s'])
            include(bounds, 'range', row['true_range_m'])
            for field in ('observed_range_m', 'estimated_range_m'):
                value = row.get(field)
                include(bounds, 'range', value)
                if value not in (None, ''):
                    include(bounds, 'range_error', float(value) - float(row['true_range_m']))


def scale(bounds, *, symmetric=False, padding=True):
    lo, hi = bounds
    if symmetric:
        radius = max(abs(lo), abs(hi), 1.)
        lo, hi = -radius, radius
    if lo == hi:
        hi = lo + 1.
    margin = (hi - lo) * .04 if padding else 0.
    lo = lo - margin if lo < 0 else lo
    hi = hi + margin if hi > 0 else hi
    ticks = MaxNLocator(nbins=6, steps=[1, 2, 2.5, 5, 10]).tick_values(lo, hi)
    ticks = [float(f'{value:.12g}') for value in ticks]
    return {'limits': [ticks[0], ticks[-1]], 'ticks': ticks}


def sync_plots(out, render, *, speed_only=False):
    out = Path(out).resolve()
    config = read_json(out / 'config.json')
    scenario = config['scenario']
    wanted = ('speed',) if speed_only else ('speed', 'range')
    with axes_lock(out.parent):
        peers = {out: config}
        for directory in sorted(out.parent.iterdir()):
            if directory == out or not directory.is_dir():
                continue
            other = read_json(directory / 'config.json', {})
            summary = read_json(directory / 'summary.json', {})
            ready = (summary.get('status') in ('completed', 'completed_with_warnings')
                     or (summary.get('status') == 'running'
                         and bool(read_json(directory / 'plot_axes.json', {}))))
            if (other.get('scenario') == scenario
                    and ready
                    and (directory / 'ego.csv').is_file()
                    and (speed_only or (directory / 'observations.csv').is_file())):
                peers[directory] = other

        registry_path = out.parent / 'plot-axes.json'
        registry = read_json(registry_path, {'version': AXES_VERSION, 'scenarios': {}})
        bounds = registry['scenarios'].setdefault(scenario, {})
        for directory, other in peers.items():
            measure(directory, other, bounds, speed_only)
        # Retain raw extrema, not rounded limits, to avoid incremental padding
        # growth. Removing a run does not silently shrink future comparisons.
        write_json(registry_path, registry)
        xaxis = scale(bounds.get('time', [0., 1.]), padding=False)
        specs = {
            name: {'version': AXES_VERSION, 'scenario': scenario, 'x': xaxis,
                   'y': [scale(bounds.get(metric, [0., 0.]), symmetric=metric == 'range_error')
                         for metric in PLOT_METRICS[name]]}
            for name in wanted
        }
        for directory in peers:
            manifest_path = directory / 'plot_axes.json'
            manifest = read_json(manifest_path, {})
            changed = [name for name in wanted if directory == out
                       or manifest.get(name) != specs[name]
                       or any(not (directory / f'{name}.{ext}').is_file() for ext in ('png', 'svg'))]
            if not changed:
                continue
            backup = directory / 'postprocessing-backups' / 'before-shared-axes-v1'
            for name in changed:
                for ext in ('png', 'svg'):
                    source = directory / f'{name}.{ext}'
                    if source.is_file() and not (backup / source.name).exists():
                        backup.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(source, backup / source.name)
            render(directory, specs, changed)
            manifest.update({name: specs[name] for name in changed})
            write_json(manifest_path, manifest)


def apply_axes(axes, spec):
    for ax, yaxis in zip(axes, spec['y']):
        ax.set_xticks(spec['x']['ticks'])
        ax.set_yticks(yaxis['ticks'])
        ax.set_xlim(spec['x']['limits'])
        ax.set_ylim(yaxis['limits'])


def save_figure(fig, out, name):
    # Keep identical plotting rectangles even if labels/legends differ by run.
    fig.subplots_adjust(left=.11, right=.98, bottom=.09, top=.93, hspace=.36)
    for ext in ('png', 'svg'):
        path = out / f'{name}.{ext}'
        temporary = out / f'{name}.{ext}.{os.getpid()}.tmp'
        fig.savefig(temporary, format=ext, dpi=160)
        temporary.replace(path)
