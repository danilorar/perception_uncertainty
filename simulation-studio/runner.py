"""Single-run and batch CLI. Each experiment has its own directory/process."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid
from settings import ROOT, RUNS, validate, write_json


def csv_writer(out, name, fields):
    f = (out / name).open('w', encoding='utf-8', newline='')
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    return f, w


def new_run_id(case):
    return (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        + '_'
        + case
        + '_'
        + uuid.uuid4().hex[:6]
    )


def finish_run(c, out, summary, *, diagnostic_only=False):
    if c['uncertainty'] == 'distance_ou_range':
        summary['range_error_model'] = {
            'model': 'distance_scaled_stationary_ou_v1',
            'distribution': 'At fixed range: N(0, sigma(r)^2), temporally correlated',
            'correlation_time_s': c['noise_correlation_time_s'],
            'transition': 'z_next = exp(-delta_t/tau)*z + sqrt(1-exp(-2*delta_t/tau))*N(0,1)',
            'initialization': 'Stationary N(0,1) independently for each object',
            'calibration': 'Uncalibrated sensitivity model; distance envelope and tau are experiment settings.',
            'sources': ['Turlej 2023, p.6; doi:10.3390/electronics12010071',
                        'Yang et al. 2020, JAIF 15(2), p.93, equations (3), (4), (7)'],
            'distance': 'true bounding-box centre-to-centre range in metres',
            'zero_error_range_m': c['noise_zero_range_m'],
            'reference_range_m': c['noise_reference_range_m'],
            'reference_mean_absolute_error_pct': c['noise_reference_mae_pct'],
            'mean_absolute_error_m': (
                'reference_range * reference_mae_pct / 100 * max(0, r - zero_range) '
                '/ (reference_range - zero_range)'
            ),
            'sigma_m': 'mean_absolute_error_m * sqrt(pi/2)',
            'beyond_reference': 'Continue the same linear MAE slope; no cap.',
            'invalid_measurements': (
                'Negative raw ranges are saved and marked invalid, not clamped or resampled; '
                'for that controller step the target is placed behind the ego, so it is not '
                'perceived ahead.'
            ),
            'sampling': (
                'Independent RNG stream and persistent OU state per object, advanced at every '
                'integration step (the HiDrive controller step); no reset on invalid ranges or '
                'entry into the zero-error zone. No observation filtering.'
            ),
            'injection': (
                'esmini 3.8.1 public hook (hidrive/PerceptionNoise.hpp): targets moved to their '
                'perceived centre along the ego line of sight during the controller step, truth '
                'restored afterwards.'
            ),
        }
    write_json(out / 'summary.json', summary)
    # Plot saved CSV data; the ideal reference process only produces diagnostics.
    if not diagnostic_only:
        write_json(out / 'progress.json', {'stage': 'plots', 'fraction': 0.7})
        from plots import make_plots

        make_plots(out)
    if c['export_video'] and not diagnostic_only:
        write_json(out / 'progress.json', {'stage': 'video', 'fraction': 0.8})
        with (out / 'video.log').open('w', encoding='utf-8') as log:
            try:
                result = subprocess.run(
                    [sys.executable, '-X', 'utf8', str(ROOT / 'video.py'), str(out)],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=240,
                    creationflags=subprocess.CREATE_NO_WINDOW,
                )
                video_failed = result.returncode != 0
            except subprocess.TimeoutExpired:
                log.write('Video export exceeded the 240-second limit.\n')
                video_failed = True
        if video_failed or not (out / 'video.mp4').exists():
            summary['video_status'] = 'failed'
            summary['status'] = 'completed_with_warnings'
            summary['warning'] = '仿真与数据已完成，但视频导出失败；详见 video.log。'
        else:
            summary['video_status'] = 'ready'
            summary['status'] = 'completed'
    else:
        summary['status'] = 'completed'
    log = (out / 'esmini.log').read_text(encoding='utf-8', errors='replace')
    summary['esmini_error_lines'] = [line for line in log.splitlines() if '[error]' in line]
    if summary['esmini_error_lines']:
        summary['status'] = 'completed_with_warnings'
        summary['warning'] = (
            '；'.join(summary.get('source_warnings', []))
            or '仿真完成，esmini 有提示；详见 esmini.log。'
        )
    write_json(out / 'summary.json', summary)
    write_json(out / 'progress.json', {'stage': 'done', 'fraction': 1})
    print(
        json.dumps(
            {
                'directory': str(out),
                'status': summary['status'],
                'collision': summary['collision'],
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return summary


def run(config, out, *, diagnostic_only=False, plot_axes_policy=None):
    """Run the teacher's HiDrive AEB; the only remaining controller."""
    plot_axes_policy = Path(plot_axes_policy).resolve() if plot_axes_policy is not None else None
    c = validate(config)
    from hidrive_runner import run_hidrive
    return run_hidrive(c, out, diagnostic_only=diagnostic_only, plot_axes_policy=plot_axes_policy)


def main():
    parser = argparse.ArgumentParser(description='TME180 Simulation Studio')
    subs = parser.add_subparsers(dest='command', required=True)
    single = subs.add_parser('run')
    single.add_argument('--config', required=True)
    single.add_argument('--output')
    single.add_argument('--diagnostic-only', action='store_true', help=argparse.SUPPRESS)
    single.add_argument('--plot-axes-policy', help=argparse.SUPPRESS)
    batch = subs.add_parser('batch')
    batch.add_argument('--matrix', required=True)
    args = parser.parse_args()
    if args.command == 'run':
        c = validate(json.loads(Path(args.config).read_text(encoding='utf-8-sig')))
        return run(
            c,
            Path(args.output) if args.output else RUNS / new_run_id(c['scenario']),
            diagnostic_only=args.diagnostic_only,
            plot_axes_policy=args.plot_axes_policy,
        )
    matrix = json.loads(Path(args.matrix).read_text(encoding='utf-8-sig'))
    base = validate(matrix.get('base', {}))
    variants = matrix.get('runs', [])
    if not isinstance(variants, list) or not 1 <= len(variants) <= 1000:
        raise ValueError('runs 必须是包含 1–1000 组配置的列表')
    configs = [validate(base | item) for item in variants]
    folder = ROOT / 'batches' / new_run_id('batch')
    folder.mkdir(parents=True)
    shutil.copy2(args.matrix, folder / 'matrix.json')
    results = []
    for i, c in enumerate(configs):
        cp = folder / f'config_{i+1:04}.json'
        write_json(cp, c)
        out = RUNS / new_run_id(c['scenario'])
        with (folder / f'run_{i+1:04}.log').open('w', encoding='utf-8') as log:
            p = subprocess.run(
                [
                    sys.executable,
                    '-X',
                    'utf8',
                    str(ROOT / 'runner.py'),
                    'run',
                    '--config',
                    str(cp),
                    '--output',
                    str(out),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW,
            )
        s = (
            json.loads((out / 'summary.json').read_text(encoding='utf-8'))
            if (out / 'summary.json').exists()
            else {'status': 'failed'}
        )
        results.append(
            {
                'run_id': out.name,
                'scenario': c['scenario'],
                'seed': c['seed'],
                'uncertainty': c['uncertainty'],
                'correlation_time_s': c['noise_correlation_time_s'] if c['uncertainty'] == 'distance_ou_range' else None,
                'zero_error_range_m': (
                    c['noise_zero_range_m']
                    if c['uncertainty'] == 'distance_ou_range'
                    else None
                ),
                'reference_range_m': (
                    c['noise_reference_range_m']
                    if c['uncertainty'] == 'distance_ou_range'
                    else None
                ),
                'reference_mae_pct': (
                    c['noise_reference_mae_pct']
                    if c['uncertainty'] == 'distance_ou_range'
                    else None
                ),
                'first_trigger_time_s': s.get('first_trigger_time_s'),
                'true_center_range_at_brake_m': s.get('assessment', {}).get(
                    'true_center_range_at_brake_m'
                ),
                'invalid_range_observations': s.get('invalid_range_observations'),
                'observed_range_rmse_m': s.get('assessment', {}).get('observed_range_rmse_m'),
                'status': s['status'],
                'exit_code': p.returncode,
                'collision': s.get('collision'),
                'impact_speed_kph': (s.get('collision_details') or {}).get('ego_speed_kph'),
                'brake_time_s': s.get('first_brake_time_s'),
            }
        )
        with (folder / 'summary.csv').open('w', newline='', encoding='utf-8-sig') as f:
            w = csv.DictWriter(f, results[0].keys())
            w.writeheader()
            w.writerows(results)
        print(f'[{i+1}/{len(configs)}] {out.name}: {s["status"]}', flush=True)
    print(str(folder), flush=True)


if __name__ == '__main__':
    main()
