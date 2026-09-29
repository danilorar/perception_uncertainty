#!/usr/bin/env python3
"""Replay original trajectories and log the ideal esmini sensor detections.

Author: Zhuo Ma
No dropout or AEB is applied. Source HiDriveController actions are disabled;
both vehicles follow the supplied trajectories. The sensor exposes IDs only;
classification columns are read separately from simulation ground truth.
"""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import time

from esmini_api import (add_sensor, fetch_ids, get_state, initialize, load_library,
                        scenario_arguments, set_overview_camera)
from generate_scenarios import sha256
from paths import DATA, RESULTS, SCENARIO_IDS


def run(args):
    """Run one original scenario; the camera uses its complete original paths."""
    scenario = DATA / f"C_original_{args.scenario}.xosc"
    if not scenario.is_file():
        raise FileNotFoundError(scenario)
    se = load_library()
    dt = 0.01
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output_dir = args.results_dir.expanduser().resolve() / f"sensor_{args.scenario}_{timestamp}"
    output_dir.mkdir(parents=True)
    print(f"Output directory: {output_dir}", flush=True)
    (output_dir / "run_config.json").write_text(json.dumps(dict(
        scenario=args.scenario, mode="ideal_sensor_replay", dt_s=dt,
        source_sha256=sha256(scenario), author="Zhuo Ma",
    ), indent=2) + "\n", encoding="utf-8")
    arguments = scenario_arguments(scenario, output_dir, dt, args.headless, replay=True)

    try:
        initialize(se, arguments)

        # Select the camera explicitly after the viewer has been initialized.
        # Its position and zoom stay fixed while the vehicles move and overlap.
        if not args.headless:
            set_overview_camera(se, scenario)

        host_id = se.SE_GetIdByName(b"object_1")
        target_id = se.SE_GetIdByName(b"object_2")

        if host_id < 0 or target_id < 0:
            raise RuntimeError("Failed to get object IDs.")

        print("object_1 ID:", host_id)
        print("object_2 ID:", target_id)

        # --------------- add sensor to host vehicle ---------------
        sensor_id, detected_ids = add_sensor(se, host_id)
        if not args.headless:
            se.SE_ViewerShowFeature(1, True)

        # --------------- run simulation loop ---------------
        with(output_dir / "sensor_ids.csv").open(
            "w", encoding="utf-8", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "time_s",
                "sensor_id",
                "detected_ids",
                "target_detected",
                "target_object_type", "target_object_category",
                "observed_target_object_type", "observed_target_object_category",
            ])

            step = 0

            while (
                se.SE_GetQuitFlag() == 0 and
                se.SE_GetSimulationTime() < 20.0
            ):
                wall_start = time.perf_counter()
                previous_t = se.SE_GetSimulationTime()

                if se.SE_StepDT(dt) != 0:
                    raise RuntimeError("Simulation step failed.")

                t = se.SE_GetSimulationTime()

                if t<= previous_t:
                    if se.SE_GetQuitFlag():
                        break
                    raise RuntimeError("Simulation time did not advance.")

                # Sensor output is an ID list. Read the selected target's truth
                # separately and mask the classification when it is not detected.
                ids = fetch_ids(se, sensor_id, detected_ids)
                target = get_state(se, target_id)

                writer.writerow([
                    f"{t:.3f}",
                    sensor_id,
                    ";".join(map(str, ids)),
                    int(target_id in ids),
                    target.objectType, target.objectCategory,
                    target.objectType if target_id in ids else "",
                    target.objectCategory if target_id in ids else "",
                ])

                if step % 10 == 0:
                    print(f"Step {step}: time={t:.3f}s, detected_ids={ids}, target_detected={int(target_id in ids)}")

                step += 1

                elapsed = time.perf_counter() - wall_start
                if not args.headless:
                    time.sleep(max(0.0, dt - elapsed))

    finally:
        se.SE_Close()
        se.SE_CloseLogFile()

    print(f"Simulation completed. Results are stored in: {output_dir}")

    return output_dir


def main():
    """Keep simulation side effects out of imports and expose scenario selection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIO_IDS, default="1549178")
    parser.add_argument("--headless", action="store_true", help="Run without the viewer")
    parser.add_argument("--results-dir", type=Path, default=RESULTS)
    return run(parser.parse_args())


if __name__ == "__main__":
    main()
