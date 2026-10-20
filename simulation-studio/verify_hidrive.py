"""Re-run the HiDrive-only experiments and check the range-error injection.

For each teacher scene: ideal run, zero-amplitude OU run (must equal ideal),
constant bias z=+1 / z=-1 (target perceived farther / nearer), OU seed 42 twice
(must repeat exactly) and OU seed 7. Writes verification/<time>_hidrive_only/report.json;
pass an existing report directory to resume it.
"""
from datetime import datetime
import csv
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import hidrive_runner
from models import CorrelatedRangeNoise, range_error_parameters
from runner import run
from settings import DEFAULT, ROOT, SCENARIOS, write_json

SCENES = sorted(SCENARIOS)
BIAS_PCT = 20.0


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    with Path(path).open(encoding='utf-8') as f:
        return list(csv.DictReader(f))


def experiment(root, name, config, z=None):
    out = root / name
    if (out / 'summary.json').is_file():
        # Resume an interrupted verification without rerunning finished cases.
        summary = json.loads((out / 'summary.json').read_text(encoding='utf-8'))
        if summary.get('status') == 'completed':
            return out, summary
    original = CorrelatedRangeNoise.sample
    if z is not None:
        # Constant unit state: a pure bias of z·σ(r) at every step.
        hidrive_runner.CorrelatedRangeNoise.sample = lambda self, i, t: (z, None, None)
    cwd = os.getcwd()
    try:
        summary = run(DEFAULT | {'export_video': False} | config, out)
    finally:
        hidrive_runner.CorrelatedRangeNoise.sample = original
        os.chdir(cwd)
    return out, summary


def outcome(summary):
    return {
        'first_trigger_time_s': summary['first_trigger_time_s'],
        'first_brake_time_s': summary['first_brake_time_s'],
        'collision': summary['collision'],
        'collision_speed_kph': (summary['collision_details'] or {}).get('ego_speed_kph'),
        'final_speed_kph': summary['final_speed_kph'],
        'end_time_s': summary['end_time_s'],
        'invalid_range_observations': summary.get('invalid_range_observations'),
        'observed_range_rmse_m': summary['assessment'].get('observed_range_rmse_m'),
    }


def injection_error(out, config):
    """Largest deviation between the applied error and Studio's amplitude law."""
    worst = 0.0
    for row in rows(out / 'observations.csv'):
        if row['observation_valid'] != 'True':
            continue
        true_range = float(row['true_range_m'])
        sigma = range_error_parameters(config, true_range)[1]
        expected = true_range + sigma * float(row['range_error_normalized'])
        worst = max(worst, abs(float(row['observed_range_m']) - expected),
                    abs(float(row['range_error_sigma_m']) - sigma))
    return worst


def target_truth_difference(a, b):
    """Lead-vehicle truth must not depend on the perceived error."""
    def target(path):
        return {(r['time_s'], r['object_name']): (float(r['x_m']), float(r['y_m']))
                for r in rows(path) if r['object_name'] != 'object_1'}
    first, second = target(a / 'truth.csv'), target(b / 'truth.csv')
    common = set(first) & set(second)
    return max((math.dist(first[k], second[k]) for k in common), default=None), len(common)


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else         ROOT / 'verification' / (datetime.now().strftime('%Y%m%dT%H%M%S') + '_hidrive_only')
    root.mkdir(parents=True, exist_ok=True)
    report, checks = {}, {}
    for scene in SCENES:
        base = {'scenario': scene}
        ou = base | {'uncertainty': 'distance_ou_range'}
        ideal_out, ideal = experiment(root, f'{scene}_ideal', base | {'uncertainty': 'recorded_truth'})
        zero_out, zero = experiment(root, f'{scene}_zero', ou | {'noise_reference_mae_pct': 0.0})
        far_out, far = experiment(root, f'{scene}_bias_far', ou | {'noise_reference_mae_pct': BIAS_PCT}, z=1.0)
        near_out, near = experiment(root, f'{scene}_bias_near', ou | {'noise_reference_mae_pct': BIAS_PCT}, z=-1.0)
        a_out, a = experiment(root, f'{scene}_ou42_a', ou)
        b_out, _ = experiment(root, f'{scene}_ou42_b', ou)
        s7_out, s7 = experiment(root, f'{scene}_ou7', ou | {'seed': 7})
        trigger = {k: v['first_trigger_time_s'] for k, v in [('ideal', ideal), ('far', far), ('near', near)]}
        drift, compared = target_truth_difference(ideal_out, a_out)
        report[scene] = {
            'ideal': outcome(ideal), 'zero_amplitude': outcome(zero),
            'bias_far_z+1': outcome(far), 'bias_near_z-1': outcome(near),
            'ou_seed42': outcome(a), 'ou_seed7': outcome(s7),
            'max_injection_error_m': max(injection_error(o, s['config'])
                                         for o, s in [(a_out, a), (far_out, far), (near_out, near), (s7_out, s7)]),
            'max_target_truth_difference_ideal_vs_ou_m': drift, 'target_truth_samples_compared': compared,
        }
        checks[scene] = {
            'zero_amplitude_equals_ideal': all(digest(ideal_out / f) == digest(zero_out / f)
                                               for f in ['ego.csv', 'truth.csv', 'metrics.csv']),
            'ou_seed42_repeats_exactly': all(digest(a_out / f) == digest(b_out / f)
                                             for f in ['ego.csv', 'truth.csv', 'observations.csv']),
            'injection_matches_amplitude_law': report[scene]['max_injection_error_m'] < 1e-6,
            'target_truth_unchanged_by_noise': drift is not None and drift < 1e-6,
            'bias_order_near_le_ideal_le_far': None not in trigger.values()
                and trigger['near'] <= trigger['ideal'] <= trigger['far'],
        }
    passed = all(all(v for v in c.values()) for c in checks.values())
    write_json(root / 'report.json', {'created': datetime.now().isoformat(timespec='seconds'),
                                      'bias_reference_mae_pct': BIAS_PCT, 'passed': passed,
                                      'checks': checks, 'results': report})
    print(json.dumps({'directory': str(root), 'passed': passed, 'checks': checks}, ensure_ascii=False, indent=1))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
