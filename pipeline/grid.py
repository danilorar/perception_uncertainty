"""Many simulations from one JSON file. See README.md, "Run a grid".

{"name": "...", "scenarios": [...], "base": {...},
 "groups": [{"name": "...", "set": {...}, "vary": {"key": [values]}, "seeds": 30 | {"first": 50, "count": 50} | [1, 2]}]}

Every combination of the "vary" values runs on every scenario with every seed. Keys may be dotted;
inside "errors" a part names an entry by its "name" or "type" ("errors.detection.miss_rate").
Runs that already finished with the same configuration are skipped, so a stopped grid continues where it was.
"""
from concurrent.futures import ThreadPoolExecutor
import copy
import csv
import gzip
import itertools
from pathlib import Path
import shutil
import statistics
import subprocess
import sys

from . import scenarios
from .common import NO_WINDOW, OUTPUT, REPO, read_json, write_json
from .config import validate

COMPRESS = {'ego.csv', 'observations.csv', 'metrics.csv', 'perception_applied.tsv'}
KEEP = {'config.json', 'summary.json', 'versions.json', 'error.log'}
DONE = ('completed', 'completed_with_warnings')
RESULT_FIELDS = ['status', 'collision', 'impact_speed_kph', 'relative_impact_speed_kph', 'first_trigger_time_s',
                 'first_brake_time_s', 'true_center_range_at_brake_m', 'minimum_gap_m', 'minimum_ttc_s',
                 'minimum_acceleration_mps2', 'final_speed_kph', 'end_time_s', 'invalid_range_observations',
                 'observed_range_rmse_m']
# Which code made each run: the commit, and how many files differed from it (0 = clean).
CODE_FIELDS = ['commit', 'local_changes']


def entry(entries, name, key):
    """The entry of a list such as "errors" whose name, or type, is `name`."""
    for item in entries:
        if isinstance(item, dict) and (item.get('name') or item.get('type')) == name:
            return item
    raise ValueError(f'{key}: no entry {name!r}; add {{"type": "{name}"}} to the list first, in "base" or "set".')


def assign(config, key, value):
    """Set a possibly dotted key; dictionaries are merged."""
    *parents, last = key.split('.')
    for parent in parents:
        if isinstance(config, list):
            config = entry(config, parent, key)
            continue
        if not isinstance(config.get(parent), (dict, list)):
            config[parent] = {}
        config = config[parent]
    if isinstance(config, list):
        raise ValueError(f'{key}: name a setting inside an entry, such as errors.detection.miss_rate')
    if isinstance(value, dict) and isinstance(config.get(last), dict):
        config[last] |= value
    else:
        config[last] = copy.deepcopy(value)


def seed_list(seeds):
    if seeds is None:
        return [0]
    if isinstance(seeds, int):
        return list(range(seeds))
    if isinstance(seeds, dict):
        return list(range(seeds['first'], seeds['first'] + seeds['count']))
    return list(seeds)


def expand(grid):
    """All runs of a grid: a list of {name, scenario, group, levels, seed, config}."""
    extra = set(grid) - {'name', 'description', 'scenarios', 'base', 'groups', 'set', 'vary', 'seeds'}
    if extra:
        raise ValueError('Unknown grid settings: ' + ', '.join(sorted(extra)))
    groups = grid.get('groups') or [{'name': 'main', **{k: grid[k] for k in ('set', 'vary', 'seeds') if k in grid}}]
    jobs = []
    for scenario, group in itertools.product(grid.get('scenarios') or list(scenarios.all_scenarios()), groups):
        vary = group.get('vary', {})
        for levels in itertools.product(*vary.values()):
            levels = dict(zip(vary, levels))
            for seed in seed_list(group.get('seeds')):
                config = {}
                for key, value in {**grid.get('base', {}), **group.get('set', {})}.items():
                    assign(config, key, value)
                for key, value in levels.items():
                    assign(config, key, value)
                assign(config, 'scenario', scenario)
                assign(config, 'seed', seed)
                short = ''.join(f'__{key.split(".")[-1]}={value}' for key, value in levels.items())
                jobs.append(dict(name=f'{scenario}__{group["name"]}{short}__seed{seed}', scenario=scenario,
                                 group=group['name'], levels=levels, seed=seed, config=validate(config)))
    names = [job['name'] for job in jobs]
    if len(set(names)) != len(names):
        raise ValueError('Two runs of this grid get the same name; give each group its own name.')
    return jobs


def compact(out):
    """A grid keeps only what its analysis needs, compressed."""
    for path in list(out.iterdir()):
        if path.name in COMPRESS:
            with path.open('rb') as source, gzip.open(path.with_name(path.name + '.gz'), 'wb') as target:
                shutil.copyfileobj(source, target)
        if path.name not in KEEP:
            path.unlink() if path.is_file() else shutil.rmtree(path)


def finished(job, out):
    if not (out / 'ego.csv.gz').is_file() or not (out / 'summary.json').is_file():
        return False
    return read_json(out / 'summary.json').get('status') in DONE and read_json(out / 'config.json') == job['config']


def run_job(job, folder):
    out = folder / job['name']
    if finished(job, out):
        return True
    if out.exists():
        shutil.rmtree(out)
    config, log = folder / f'{job["name"]}.json', folder / f'{job["name"]}.log'
    write_json(config, job['config'])
    with log.open('w', encoding='utf-8') as stream:
        subprocess.run([sys.executable, '-X', 'utf8', '-m', 'pipeline', 'run', str(config), '--output', str(out), '--no-plots'],
                       cwd=REPO, stdout=stream, stderr=subprocess.STDOUT, creationflags=NO_WINDOW)
    config.unlink()
    summary = out / 'summary.json'
    if not summary.is_file() or read_json(summary).get('status') not in DONE:
        return False  # keep the log
    compact(out)
    log.unlink()
    return True


def result_row(job, out):
    summary = read_json(out / 'summary.json') if (out / 'summary.json').is_file() else {'status': 'failed'}
    details = summary.get('collision_details') or {}
    row = dict(summary, impact_speed_kph=details.get('ego_speed_kph'), relative_impact_speed_kph=details.get('relative_speed_kph'))
    versions = read_json(out / 'versions.json') if (out / 'versions.json').is_file() else {}
    code = versions.get('code') or {}
    changes = code.get('local_changes')
    return {'run': job['name'], 'scenario': job['scenario'], 'group': job['group'], **job['levels'], 'seed': job['seed'],
            **{key: row.get(key) for key in RESULT_FIELDS},
            'commit': code.get('commit'), 'local_changes': None if changes is None else len(changes)}


def describe_code(rows):
    """One line on which code made the finished runs, and a warning when that is not one clean commit."""
    done = [row for row in rows if row['status'] in DONE]
    commits = sorted({row['commit'] or 'unknown' for row in done})
    if not done:
        return 'Code: no finished runs.'
    if commits == ['unknown']:
        return 'Code: not recorded (made outside a Git checkout or by an older pipeline).'
    if len(commits) > 1:
        return (f'Warning: these runs come from {len(commits)} code versions ({", ".join(c[:10] for c in commits)}). '
                'Run the grid again into a new output folder before comparing them.')
    changed = sum(1 for row in done if row['local_changes'])
    if changed:
        return (f'Warning: {changed} of {len(done)} runs used files that differ from commit {commits[0][:10]}. '
                'Commit the changes and run again, or say so when you share these results.')
    return f'Code: commit {commits[0][:10]}, no local changes.'


def middle(values):
    return statistics.median(values) if values else None


def summary_row(rows, level_keys):
    done = [row for row in rows if row['status'] in DONE]
    impacts = [row['impact_speed_kph'] for row in done if row['collision']]
    gaps = [row['minimum_gap_m'] for row in done if row['minimum_gap_m'] is not None]
    ttcs = [row['minimum_ttc_s'] for row in done if row['minimum_ttc_s'] is not None]
    return {'scenario': rows[0]['scenario'], 'group': rows[0]['group'], **{key: rows[0].get(key) for key in level_keys},
            'runs': len(rows), 'failed': len(rows) - len(done), 'collisions': len(impacts),
            'impact_speed_min_kph': min(impacts, default=None), 'impact_speed_median_kph': middle(impacts),
            'impact_speed_max_kph': max(impacts, default=None), 'minimum_gap_min_m': min(gaps, default=None),
            'minimum_gap_median_m': middle(gaps), 'minimum_ttc_median_s': middle(ttcs),
            'never_triggered': sum(row['first_trigger_time_s'] is None for row in done)}


def write_table(path, rows, fields):
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_results(jobs, folder, output):
    level_keys = list(dict.fromkeys(key for job in jobs for key in job['levels']))
    rows = [result_row(job, folder / job['name']) for job in jobs]
    write_table(output / 'results.csv', rows, ['run', 'scenario', 'group', *level_keys, 'seed', *RESULT_FIELDS, *CODE_FIELDS])
    cells = {}
    for row in rows:
        cells.setdefault((row['scenario'], row['group'], *(row.get(key) for key in level_keys)), []).append(row)
    summaries = [summary_row(cell, level_keys) for cell in cells.values()]
    write_table(output / 'summary.csv', summaries, list(summaries[0]))
    return rows


def run_grid(path, *, workers=8, list_only=False, output=None):
    """Returns the process exit code: 0 if every run finished."""
    grid = read_json(path)
    jobs = expand(grid)
    name = grid.get('name') or Path(path).stem
    counts = {}
    for job in jobs:
        counts[job['group']] = counts.get(job['group'], 0) + 1
    print(f'{name}: {len(jobs)} runs (' + ', '.join(f'{group} {count}' for group, count in counts.items()) + ')', flush=True)
    if list_only:
        return 0
    output = Path(output) if output else OUTPUT / 'grids' / name
    folder = output / 'runs'
    folder.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, output / 'grid.json')
    failed = []
    step = max(1, len(jobs) // 20)
    with ThreadPoolExecutor(workers) as pool:
        for index, (job, ok) in enumerate(zip(jobs, pool.map(lambda job: run_job(job, folder), jobs)), 1):
            if not ok:
                failed.append(job['name'])
            if index % step == 0 or index == len(jobs):
                print(f'{index}/{len(jobs)} failed={len(failed)}', flush=True)
    (output / 'failed.txt').write_text('\n'.join(failed), encoding='utf-8')
    rows = write_results(jobs, folder, output)
    print(f'Results: {output / "results.csv"} and summary.csv', flush=True)
    print(describe_code(rows), flush=True)
    return 1 if failed else 0
