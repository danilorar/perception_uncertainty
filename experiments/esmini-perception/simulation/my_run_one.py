#!/usr/bin/env python3
"""Replay one original scenario using the esmini executable (no Python sensor).

Author: Zhuo Ma
For sensor detections use my_sensor_demo.py. For an externally controlled ego
use my_aeb_demo.py or my_aeb_dropout_demo.py. All source files stay unchanged.
"""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess

from paths import DATA, ESMINI, RESULTS, SCENARIO_IDS, executable_path


def main():
    """Create a unique result folder and run with fixed 10 ms simulation steps."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIO_IDS, default="1549178")
    parser.add_argument("--view", action="store_true", help="Open the viewer (default: headless)")
    parser.add_argument("--results-dir", type=Path, default=RESULTS)
    args = parser.parse_args()
    executable = executable_path()
    scenario = DATA / f"C_original_{args.scenario}.xosc"
    for path in (executable, scenario):
        if not path.is_file():
            raise FileNotFoundError(path)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output_dir = args.results_dir.expanduser().resolve() / f"replay_{args.scenario}_{timestamp}"
    output_dir.mkdir(parents=True)
    command = [
        str(executable), "--osc", str(scenario),
        "--path", str(DATA), str(ESMINI / "resources"),
        "--fixed_timestep", "0.01", "--traj_filter", "0",
        # HiDriveController is not supplied by this project. Replay uses the
        # original timed trajectories without executing that custom controller.
        "--disable_controllers", "--seed", "0", "--collision",
        "--csv_logger", str(output_dir / "states.csv"),
        "--record", str(output_dir / "sim.dat"),
        "--logfile_path", str(output_dir / "run.log"),
    ]
    command += ["--window", "80", "80", "1100", "650"] if args.view else ["--headless"]
    with (output_dir / "console.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, cwd=output_dir, stdout=log,
                                stderr=subprocess.STDOUT, timeout=60)
    if result.returncode:
        raise RuntimeError(f"Simulation exit {result.returncode}; see {output_dir / 'console.log'}")
    print(f"Simulation completed. Results: {output_dir}")


if __name__ == "__main__":
    main()
