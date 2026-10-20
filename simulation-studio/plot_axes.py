"""Comparable plot scales without letting extreme runs flatten their peers."""

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

AXES_VERSION = 4
MAX_SHARED_Y_EXPANSION = 2.0
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
                    time.sleep(0.1)
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
        limits = bounds.setdefault(key, [0.0, 0.0])
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


def scale(bounds, *, padding=True):
    lo, hi = bounds
    if lo == hi:
        hi = lo + 1.0
    margin = (hi - lo) * 0.04 if padding else 0.0
    lo = lo - margin if lo < 0 else lo
    hi = hi + margin if hi > 0 else hi
    ticks = MaxNLocator(nbins=6, steps=[1, 2, 2.5, 5, 10]).tick_values(lo, hi)
    ticks = [float(f'{value:.12g}') for value in ticks]
    return {'limits': [ticks[0], ticks[-1]], 'ticks': ticks}


def metric_scale(bounds, metric):
    limits = bounds.get(metric, [0.0, 0.0])
    if metric == 'range_error' and limits == [0.0, 0.0]:
        # Keep a zero-error line visible when the whole comparison is noiseless.
        return {'limits': [-1.0, 1.0], 'ticks': [-1.0, -0.5, 0.0, 0.5, 1.0]}
    return scale(limits)


def independent_y(out):
    """Explicit per-record plot exceptions; simulation settings stay untouched."""
    policy = read_json(out / 'plot_axes_policy.json', {})
    names = policy.get('independent_y', [])
    if not isinstance(names, list) or any(name not in PLOT_METRICS for name in names):
        raise ValueError(f'Invalid independent_y plot policy: {out}')
    return set(names)


def grouped_y_scales(own_bounds, directories, metric):
    """Share by magnitude, bounding the loss of vertical detail for EVERY run.

    Compare the final rounded axis spans, including padding, against each run's
    standalone axis. Smallest-first grouping prevents a chain of intermediate
    peaks from admitting an arbitrarily large outlier. Always retain all data.
    """
    standalone = {out: metric_scale(own_bounds[out], metric) for out in directories}

    def span(axis):
        return axis['limits'][1] - axis['limits'][0]

    ordered = sorted(
        standalone, key=lambda out: (span(standalone[out]), standalone[out]['limits'], out.name)
    )
    groups = []
    for out in ordered:
        limits = own_bounds[out].get(metric, [0.0, 0.0])
        own_span = span(standalone[out])
        if groups:
            group = groups[-1]
            merged = [min(group['bounds'][0], limits[0]), max(group['bounds'][1], limits[1])]
            axis = metric_scale({metric: merged}, metric)
            min_span = min(group['min_span'], own_span)
            if span(axis) <= MAX_SHARED_Y_EXPANSION * min_span * (1.0 + 1e-12):
                group.update(bounds=merged, axis=axis, min_span=min_span)
                group['members'].append(out)
                continue
        groups.append(
            {
                'bounds': list(limits),
                'axis': standalone[out],
                'min_span': own_span,
                'members': [out],
            }
        )

    assignments, records = {}, []
    for index, group in enumerate(groups, 1):
        axis = {
            **group['axis'],
            'scope': 'group' if len(groups) > 1 else 'scenario',
            'group': index,
            'group_count': len(groups),
            'member_count': len(group['members']),
        }
        for out in group['members']:
            assignments[out] = axis
        records.append(
            {
                'group': index,
                'bounds': group['bounds'],
                'axis': group['axis'],
                'members': sorted(out.name for out in group['members']),
            }
        )
    return assignments, records


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
            ready = summary.get('status') in ('completed', 'completed_with_warnings') or (
                summary.get('status') == 'running'
                and bool(read_json(directory / 'plot_axes.json', {}))
            )
            if (
                other.get('scenario') == scenario
                and ready
                and (directory / 'ego.csv').is_file()
                and (speed_only or (directory / 'observations.csv').is_file())
            ):
                peers[directory] = other

        registry_path = out.parent / 'plot-axes.json'
        registry = read_json(registry_path, {'version': AXES_VERSION, 'scenarios': {}})
        if registry.get('version') != AXES_VERSION:
            registry = {'version': AXES_VERSION, 'scenarios': {}}
        # Recompute from CSV, never from cached extrema: a removed or corrected
        # experiment must not leave every remaining plot stuck at its old peak.
        bounds = {}
        own_bounds = {}
        exceptions = {}
        for directory, other in peers.items():
            own = own_bounds[directory] = {}
            measure(directory, other, own, speed_only)
            exceptions[directory] = independent_y(directory)
            # Time stays common to the scenario, including independent-y runs.
            shared_metrics = {'time'}
            for name in wanted:
                if name not in exceptions[directory]:
                    shared_metrics.update(PLOT_METRICS[name])
            for metric in shared_metrics:
                for value in own.get(metric, []):
                    include(bounds, metric, value)
        assignments, groups = {}, {}
        for name in wanted:
            eligible = [directory for directory in peers if name not in exceptions[directory]]
            for metric in PLOT_METRICS[name]:
                assignments[metric], groups[metric] = grouped_y_scales(own_bounds, eligible, metric)
        previous = registry['scenarios'].get(scenario, {}) if speed_only else {}
        registry['scenarios'][scenario] = {
            **previous,
            **bounds,
            'y_groups': {**previous.get('y_groups', {}), **groups},
            'max_shared_y_expansion': MAX_SHARED_Y_EXPANSION,
        }
        write_json(registry_path, registry)
        xaxis = scale(bounds.get('time', [0.0, 1.0]), padding=False)
        for directory in peers:
            specs = {}
            for name in wanted:
                independent = name in exceptions[directory]
                yaxes = [
                    (
                        {**metric_scale(own_bounds[directory], metric), 'scope': 'run'}
                        if independent
                        else assignments[metric][directory]
                    )
                    for metric in PLOT_METRICS[name]
                ]
                scope = (
                    'run'
                    if independent
                    else ('group' if any(y['scope'] == 'group' for y in yaxes) else 'scenario')
                )
                specs[name] = {
                    'version': AXES_VERSION,
                    'scenario': scenario,
                    'x': xaxis,
                    'y_scope': scope,
                    'max_shared_y_expansion': MAX_SHARED_Y_EXPANSION,
                    'y': yaxes,
                }
            manifest_path = directory / 'plot_axes.json'
            manifest = read_json(manifest_path, {})
            changed = [
                name
                for name in wanted
                if directory == out
                or manifest.get(name) != specs[name]
                or any(not (directory / f'{name}.{ext}').is_file() for ext in ('png', 'svg'))
            ]
            if not changed:
                continue
            backup = directory / 'postprocessing-backups' / f'before-shared-axes-v{AXES_VERSION}'
            if manifest_path.is_file() and not (backup / manifest_path.name).exists():
                backup.mkdir(parents=True, exist_ok=True)
                shutil.copy2(manifest_path, backup / manifest_path.name)
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
    # Reserve the same footer space on every plot, even without a scale note.
    fig.subplots_adjust(left=0.11, right=0.98, bottom=0.12, top=0.93, hspace=0.36)
    for ext in ('png', 'svg'):
        path = out / f'{name}.{ext}'
        temporary = out / f'{name}.{ext}.{os.getpid()}.tmp'
        fig.savefig(temporary, format=ext, dpi=160)
        temporary.replace(path)
