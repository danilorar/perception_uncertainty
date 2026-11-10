"""One simulation: scene, esmini, the error plugins on every controller step, result tables.

Works the same for every controller.
"""
from datetime import datetime, timezone
from pathlib import Path
import platform
import subprocess
import sys
import threading
import traceback

from . import controllers, errors, results, scenarios
from .common import BRIDGE_HEADER, ENGINE, ESMINI, NO_WINDOW, PACKAGE, code_version, sha256, write_json
from .engine.build import check_build

TIMEOUT_S = 300


def write_versions(out, controller, build_info):
    write_json(out / 'versions.json', {
        'code': code_version(),
        'python': sys.version, 'platform': platform.platform(),
        'engine': f'esmini {build_info["engine_tag"]} (official source archive)',
        'engine_archive_sha256': build_info['engine_archive_sha256'],
        'executable': str(ESMINI), 'executable_size': ESMINI.stat().st_size,
        'controller': controller.esmini_name, 'controller_sources': controller.source_metadata(),
        'bridge_sha256': sha256(BRIDGE_HEADER),
        'implementation_sha256': {str(p.relative_to(PACKAGE)): sha256(p) for p in sorted(PACKAGE.rglob('*.py'))}})


def talk(args, out, log, chain):
    """Run esmini and answer its bridge on every controller step."""
    expired = []
    with subprocess.Popen(args, cwd=out, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                          creationflags=NO_WINDOW) as process:
        timer = threading.Timer(TIMEOUT_S, lambda: (expired.append(True), process.kill()))
        timer.start()
        try:
            for line in process.stdout:
                if line.startswith(errors.PREFIX + b'\tS\t'):
                    reply = chain.answer(line)
                    try:
                        process.stdin.write(reply)
                        process.stdin.flush()
                    except OSError:
                        break  # esmini stopped; its exit code tells why
                else:
                    log.write(line.decode('utf-8', 'replace'))
            code = process.wait()
        except BaseException:
            process.kill()
            raise
        finally:
            timer.cancel()
    if expired:
        raise RuntimeError(f'esmini did not finish within {TIMEOUT_S} s.')
    return code


def run_esmini(out, c, chain, observer):
    # --headless alone opens no graphics at all. Adding --window crashes where no graphics context exists.
    args = [ESMINI, '--osc', out / 'scene.xosc', '--headless',
            '--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',
            '--csv_logger', out / 'esmini.csv', '--record', out / 'replay.dat',
            '--logfile_path', out / 'esmini.log', '--disable_stdout', '--path', ENGINE / 'resources',
            # Scenarios name '../models/<car>.osgb'; resolve it inside the full model set.
            '--path', ENGINE / 'resources/models']
    if chain:
        args += ['--perception_bridge', observer]
    args = [str(a) for a in args]
    with (out / 'esmini_stdout.log').open('w', encoding='utf-8') as log:
        if chain:
            code = talk(args, out, log, chain)
        else:
            code = subprocess.run(args, cwd=out, stdout=log, stderr=subprocess.STDOUT, timeout=TIMEOUT_S,
                                  creationflags=NO_WINDOW).returncode
    if code != 0:
        raise RuntimeError(f'esmini stopped with exit code {code}; see esmini.log and esmini_stdout.log.')


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
    chain = None
    try:
        build_info = check_build(controller)
        properties = scenario.write_scene(out / 'scene.xosc', controller)
        write_versions(out, controller, build_info)
        if c['errors']:
            chain = errors.Chain(c, scenario, out / 'perception_applied.tsv')
        try:
            run_esmini(out, c, chain, scenario.ego)
        finally:
            if chain:
                chain.close()
        summary.update(results.collect(out, scenario, controller, chain is not None))
        summary.update(controller_properties=properties, error_plugins=chain.describe() if chain else [],
                       status='completed')
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
