"""Command line: python -m pipeline setup | run | grid | list | replay | import-tasc"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import uuid

from .common import ENGINE, OUTPUT, REPLAYER, read_json


def command_setup(args):
    from .engine.build import build
    build(with_controllers=not args.without_controllers, cmake=args.cmake, jobs=args.jobs)


def command_run(args):
    from .config import validate
    from .run import run
    config = validate(read_json(args.config))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    out = Path(args.output) if args.output else OUTPUT / 'runs' / f'{stamp}_{config["scenario"]}_{uuid.uuid4().hex[:6]}'
    summary = run(config, out, plots=not args.no_plots)
    print(f'{summary["status"]}: collision={summary["collision"]}, results in {out}', flush=True)


def command_grid(args):
    from .grid import run_grid
    sys.exit(run_grid(args.grid, workers=args.workers, list_only=args.list, output=args.output))


def command_list(_):
    from . import controllers, errors, scenarios
    print('Controllers:   ', ', '.join(controllers.all_controllers()))
    print('Error plugins: ', ', '.join(errors.all_plugins()))
    print('Scenarios:     ', ', '.join(scenarios.all_scenarios()))
    local = scenarios.local_scenarios()
    if local:
        groups = {}
        for key in local:
            groups[key.rsplit('_', 1)[0]] = groups.get(key.rsplit('_', 1)[0], 0) + 1
        print('Local:         ', ', '.join(f'{group}_* ({count})' for group, count in groups.items()))


def command_import_tasc(args):
    from . import scenarios
    from .scenarios.tasc import import_folder
    rows = import_folder(args.folder, scenarios.LOCAL, bicycle=args.bicycle)
    for row in rows:
        if row['status'] != 'imported':
            print(f'skipped {row["group"]} {row["case"]}: {row["reason"]}')
    done = [row for row in rows if row['status'] == 'imported']
    print(f'{len(done)} of {len(rows)} imported into {scenarios.LOCAL}; report: import_report.csv there.')


def command_replay(args):
    folder = Path(args.run).resolve()
    recording = folder / 'replay.dat'
    if not recording.is_file():
        sys.exit(f'{recording} not found. Grid runs do not keep it; use "python -m pipeline run" for a single run.')
    subprocess.run([str(REPLAYER), '--file', str(recording), '--res_path', str(ENGINE / 'resources'),
                    '--path', str(ENGINE / 'resources/models'), '--window', '60', '60', '1000', '600', *args.extra],
                   cwd=folder)


def main():
    parser = argparse.ArgumentParser(prog='python -m pipeline', description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)

    setup = commands.add_parser('setup', help='download and compile esmini with the controllers')
    setup.add_argument('--cmake', help='path to cmake, if it is not found automatically')
    setup.add_argument('--jobs', type=int, help='parallel compile jobs')
    setup.add_argument('--without-controllers', action='store_true', help=argparse.SUPPRESS)
    setup.set_defaults(function=command_setup)

    run = commands.add_parser('run', help='run one simulation from a configuration file')
    run.add_argument('config')
    run.add_argument('--output', help='results folder (must not exist); default output/runs/...')
    run.add_argument('--no-plots', action='store_true')
    run.set_defaults(function=command_run)

    grid = commands.add_parser('grid', help='run many simulations from a grid file')
    grid.add_argument('grid')
    grid.add_argument('--workers', type=int, default=8)
    grid.add_argument('--list', action='store_true', help='print the number of runs and stop')
    grid.add_argument('--output', help='results folder; default output/grids/<name>')
    grid.set_defaults(function=command_grid)

    commands.add_parser('list', help='show controllers, error plugins and scenarios').set_defaults(function=command_list)

    tasc = commands.add_parser('import-tasc', help='convert TASC case folders into local scenarios (not in Git)')
    tasc.add_argument('folder', help='folder with "Cut-in", "Rear-end car-car" and/or "Rear-end car-cyclist"')
    tasc.add_argument('--bicycle', action='store_true',
                      help='also write each cyclist case as tasc_bicycle_*, with object_2 as a bicycle-sized box')
    tasc.set_defaults(function=command_import_tasc)

    replay = commands.add_parser('replay', help='watch a finished single run in the esmini replayer')
    replay.add_argument('run', help='results folder of the run')
    replay.add_argument('extra', nargs='*', help='extra replayer options, after --')
    replay.set_defaults(function=command_replay)

    args = parser.parse_args()
    try:
        args.function(args)
    except (RuntimeError, ValueError) as error:
        sys.exit(f'Error: {error}')


if __name__ == '__main__':
    main()
