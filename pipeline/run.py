"""One simulation: scene, noise file, esmini, result tables. Works the same for every controller."""
from datetime import datetime, timezone
from pathlib import Path
import platform
import subprocess
import sys
import traceback

from . import controllers, errors, results, scenarios
from .common import ENGINE, ESMINI, NO_WINDOW, NOISE_HEADER, PACKAGE, sha256, write_json
from .engine.build import check_build

# Noise samples run this long past the scenario end, so esmini can never run out of them.
NOISE_MARGIN_S = 2.0
TIMEOUT_S = 300


def write_versions(out, controller, build_info):
    write_json(out / 'versions.json', {
        'python': sys.version, 'platform': platform.platform(),
        'engine': f'esmini {build_info["engine_tag"]} (official source archive)',
        'engine_archive_sha256': build_info['engine_archive_sha256'],
        'executable': str(ESMINI), 'executable_size': ESMINI.stat().st_size,
        'controller': controller.esmini_name, 'controller_sources': controller.source_metadata(),
        'perception_noise_sha256': sha256(NOISE_HEADER),
        'implementation_sha256': {str(p.relative_to(PACKAGE)): sha256(p) for p in sorted(PACKAGE.rglob('*.py'))}})


def run_esmini(out, c, noise_args):
    # Some controllers need a viewer object; --headless with --window gives an off-screen viewer.
    args = [ESMINI, '--osc', out / 'scene.xosc', '--headless', '--window', '0', '0', '320', '180',
            '--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',
            '--csv_logger', out / 'esmini.csv', '--record', out / 'replay.dat',
            '--logfile_path', out / 'esmini.log', '--disable_stdout', '--path', ENGINE / 'resources',
            # Scenarios name '../models/<car>.osgb'; resolve it inside the full model set.
            '--path', ENGINE / 'resources/models', *noise_args]
    with (out / 'esmini_stdout.log').open('w', encoding='utf-8') as log:
        result = subprocess.run([str(a) for a in args], cwd=out, stdout=log, stderr=subprocess.STDOUT,
                                timeout=TIMEOUT_S, creationflags=NO_WINDOW)
    if result.returncode != 0:
        raise RuntimeError(f'esmini stopped with exit code {result.returncode}; see esmini.log.')


def run(c, out, *, plots=True):
    """Run a validated configuration into a new folder; return the summary."""
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / 'config.json', c)
    summary = dict(status='running', run_id=out.name, config=c, collision=None,
                   created_utc=datetime.now(timezone.utc).isoformat())
    write_json(out / 'summary.json', summary)
    scenario = scenarios.get(c['scenario'])
    controller = controllers.get(c['controller'])
    model = errors.MODELS[c['uncertainty']]
    try:
        build_info = check_build(controller)
        properties = scenario.write_scene(out / 'scene.xosc', controller)
        write_versions(out, controller, build_info)
        noise_args = []
        if model.INJECTS:
            steps = round((c['duration_s'] + NOISE_MARGIN_S) / c['dt_s']) + 1
            errors.write_noise_file(out / 'perception_noise.tsv', c, scenario.ego, scenario.targets,
                                    out / 'perception_applied.tsv', steps)
            noise_args = ['--perception_noise', out / 'perception_noise.tsv']
        run_esmini(out, c, noise_args)
        summary.update(results.collect(out, scenario, controller, model.INJECTS))
        summary.update(controller_properties=properties, range_error_model=model.describe(c), status='completed')
        log = (out / 'esmini.log').read_text(encoding='utf-8', errors='replace')
        summary['esmini_error_lines'] = [line for line in log.splitlines() if '[error]' in line]
        if summary['esmini_error_lines']:
            summary['status'] = 'completed_with_warnings'
            summary['warning'] = 'esmini reported errors; see esmini_error_lines and esmini.log.'
        write_json(out / 'summary.json', summary)
    except Exception as error:
        summary.update(status='failed', error=str(error))
        write_json(out / 'summary.json', summary)
        (out / 'error.log').write_text(traceback.format_exc(), encoding='utf-8')
        raise RuntimeError(f'{error} (run folder: {out})') from error
    if plots:
        from .plots import make_plots
        make_plots(out)
    return summary
