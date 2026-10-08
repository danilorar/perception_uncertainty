"""Shared ideal/dropout AEB simulation loop for the three original scenarios.

Author: Zhuo Ma
Read this module to follow the data flow:
    ideal IDs -> optional dropout -> masked observation -> AEB -> ego motion.
Both entry scripts use this same loop so an OFF/ON comparison changes only its
selected parameters. Importing a module does not load esmini or start a run.
"""
import argparse
from datetime import datetime
import csv
import json
import math
from pathlib import Path
import time

from aeb_controller import HostReference, aeb_command, integrate, risk_metrics
from esmini_api import (add_sensor, fetch_ids, get_state, initialize, load_library,
                        scenario_arguments, make_visual_scenario, set_3d_camera,)
from generate_scenarios import sha256
from paths import DATA, GENERATED, RESULTS, SCENARIO_IDS
from sensor_model import DetectionDropout


def build_parser(ideal=False):
    """Keep familiar entry-point arguments and share all common experiment options."""
    parser = argparse.ArgumentParser(description=(
        "Ideal sensor + external AEB" if ideal else "Sensor dropout + external AEB"
    ))
    parser.add_argument("--scenario", choices=SCENARIO_IDS, default="1549178")
    parser.add_argument("--aeb", choices=["off", "on"], default="off")
    parser.add_argument("--ttc", type=float, default=2.0, help="Trigger threshold [s]")
    parser.add_argument("--decel", type=float, default=6.0, help="Positive braking magnitude [m/s^2]")
    parser.add_argument("--dt", type=float, default=0.01, help="Simulation timestep [s]")
    parser.add_argument("--headless", action="store_true", help="No viewer or real-time pacing")
    parser.add_argument("--results-dir", type=Path, default=RESULTS)
    parser.add_argument("--perception-seed", type=int, default=42)

    if ideal:
        # The ideal entry cannot accidentally turn on missing detections.
        parser.set_defaults(drop_prob=0.0)
    else:
        parser.add_argument("--drop-prob", type=float, default=0.1,
                            help="Per-object, per-update dropout probability (default: 0.1)")
    return parser


def run(args):
    """Run one case and return its unique output folder. Original data is read-only."""
    source_scenario = DATA / f"C_original_{args.scenario}.xosc"
    scenario = GENERATED / f"C_aeb_{args.scenario}.xosc"

    for path in (source_scenario, scenario):
        if not path.is_file():
            raise FileNotFoundError(f"Missing {path}; run generate_scenarios.py first")

    # Validate inputs/load dependencies before creating an apparently valid run.
    reference = HostReference(source_scenario)
    se = load_library()
    dt = args.dt
    max_objects = 10
    perception = DetectionDropout(args.drop_prob, args.perception_seed, dt)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output_dir = args.results_dir.expanduser().resolve() / (
        f"aeb_{args.aeb}_{args.scenario}_drop{args.drop_prob:g}"
        f"_seed{args.perception_seed}_{timestamp}"
    )
    output_dir.mkdir(parents=True)
    print(f"Output directory: {output_dir}", flush=True)

    config = dict(
        scenario=args.scenario, aeb=args.aeb, drop_probability=args.drop_prob,
        perception_seed=args.perception_seed, dt_s=dt, ttc_threshold_s=args.ttc,
        deceleration_mps2=args.decel, model="independent_per_object_per_tick_dropout",
        world_seed=0, esmini_abi="3.8.1", author="Zhuo Ma",
        source_sha256=sha256(source_scenario), generated_sha256=sha256(scenario),
    )
    (output_dir / "run_config.json").write_text(
        json.dumps(config, indent=2) + "\n", encoding="utf-8"
    )

    display_scenario = (
        make_visual_scenario(scenario, output_dir)
        if not args.headless
        else scenario
    )
    arguments = scenario_arguments(display_scenario, output_dir, dt, args.headless)

    def report_host(pose, speed, pitch=0.0, roll=0.0):
        """Write ego pose and speed for the next step; units are m, rad and m/s."""
        x, y, z, heading = pose
        if se.SE_ReportObjectPos(host_id, x, y, z, heading, pitch, roll) != 0:
            raise RuntimeError("Cannot report host position")
        if se.SE_ReportObjectSpeed(host_id, speed) != 0:
            raise RuntimeError("Cannot report host speed")


    try:
        initialize(se, arguments)

        # Select the camera explicitly after the viewer has been initialized.
        # Its position and zoom stay fixed while the vehicles move and overlap.
        # if not args.headless:
        #     set_overview_camera(se, source_scenario)

        host_id = se.SE_GetIdByName(b"object_1")
        target_id = se.SE_GetIdByName(b"object_2")

        if host_id < 0 or target_id < 0:
            raise RuntimeError("Failed to get object IDs.")

        if not args.headless:
            set_3d_camera(se, host_id)

        print("object_1 ID:", host_id)
        print("object_2 ID:", target_id)

        # --------------- add sensor to host vehicle ---------------
        sensor_id, detected_ids = add_sensor(se, host_id, max_objects)
        # if not args.headless:
        #     se.SE_ViewerShowFeature(1, True)

        def update_sensor_display(frame_time):
            """Set dropout display based on the following frame time."""

            kept_ids, _ = perception.apply(
                [target_id],
                frame_time,
                sensor_id
            )

            visible = target_id in kept_ids

            if not args.headless:
                se.SE_ViewerShowFeature(1, visible)

            return visible

        # --------------- run simulation loop ---------------
        # One nominal step populates the sensor before the first control decision.
        initial_time = se.SE_GetSimulationTime()
        pose, host_distance, speed = reference.at_time(initial_time + dt)
        report_host(pose, speed)

        # Calculate if the cone should be visible
        cone_visible = update_sensor_display(initial_time+dt)

        if se.SE_StepDT(dt) != 0:
            raise RuntimeError("Initial simulation step failed")

        if se.SE_GetSimulationTime() <= initial_time:
            raise RuntimeError("Initial simulation time did not advance")

        if get_state(se, host_id).ctrl_type != 1:
            raise RuntimeError("object_1 must have an active ExternalController")

        latched = False
        first_brake_time = None
        first_collision_time = None
        step = 0

        fields = ["time_s", "target_detected", "ego_x_m", "ego_y_m",
                  "target_x_m", "target_y_m", "target_object_type", "target_object_category",
                  "ego_speed_mps", "target_speed_mps",
                  "gap_m", "closing_speed_mps", "ttc_s",
                  "acceleration_command_mps2", "aeb_active", "collision"]

        fields += [
            "raw_detected_ids", "observed_ids", "dropped_ids",
            "raw_target_detected", "target_dropped", "cone_visible",
            "observed_gap_m", "observed_ttc_s",
            "observed_target_object_type", "observed_target_object_category",
        ]

        with (output_dir / "aeb_telemetry.csv").open(
            "w", encoding="utf-8", newline=""
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()

            while True:
                wall_start = time.perf_counter()
                t = se.SE_GetSimulationTime()
                ego, target = get_state(se, host_id), get_state(se, target_id)
                # The native sensor writes IDs only. State properties above are truth.
                raw_ids = fetch_ids(se, sensor_id, detected_ids)

                # Apply the perception dropout model to the raw sensor detections.
                # ids is the usable perception output; dropped_ids is audit data.
                ids, dropped_ids = perception.apply(raw_ids, t, sensor_id)

                raw_detected = target_id in raw_ids
                detected = target_id in ids
                target_dropped = target_id in dropped_ids

                # Calculate evaluation metrics from truth independently of dropout.
                gap, closing, in_path = risk_metrics(ego, target)
                ttc = (
                    max(gap, 0.0) / closing
                    if in_path and closing > 1e-3
                    else math.inf
                )

                # Mask truth using post-dropout visibility before the controller sees it.
                # Truth remains available separately for collision/KPI evaluation.
                observed_gap = gap if detected else math.inf
                observed_closing = closing if detected else 0.0
                observed_in_path = in_path if detected else False

                acceleration, latched, observed_ttc = aeb_command(
                    detected,
                    observed_in_path,
                    observed_gap,
                    observed_closing,
                    ego.speed,
                    latched,
                    enabled=args.aeb == "on",
                    ttc_threshold=args.ttc,
                    deceleration=args.decel,
                )

                if latched and first_brake_time is None:
                    first_brake_time = t

                collisions = se.SE_GetObjectNumberOfCollisions(host_id)

                if collisions < 0:
                    raise RuntimeError("Cannot read collisions")

                if collisions and first_collision_time is None:
                    first_collision_time = t

                # gap/TTC are ideal ground-truth metrics, also logged when invisible.
                # The command at time t applies to the following time interval.
                writer.writerow(dict(
                    time_s=f"{t:.3f}", target_detected=int(detected),
                    ego_x_m=ego.x, ego_y_m=ego.y,
                    target_x_m=target.x, target_y_m=target.y,
                    target_object_type=target.objectType,
                    target_object_category=target.objectCategory,
                    observed_target_object_type=target.objectType if detected else "",
                    observed_target_object_category=target.objectCategory if detected else "",
                    ego_speed_mps=ego.speed, target_speed_mps=target.speed,
                    gap_m=gap, closing_speed_mps=closing,
                    ttc_s=ttc if math.isfinite(ttc) else "",
                    acceleration_command_mps2=acceleration,
                    aeb_active=int(latched), collision=int(collisions > 0),
                    raw_detected_ids=";".join(map(str, raw_ids)),
                    observed_ids=";".join(map(str, ids)),
                    dropped_ids=";".join(map(str, dropped_ids)),
                    raw_target_detected=int(raw_detected),
                    target_dropped=int(target_dropped),
                    cone_visible=int(cone_visible),
                    observed_gap_m=observed_gap if detected else "",
                    observed_ttc_s=(
                        observed_ttc
                        if detected and math.isfinite(observed_ttc)
                        else ""
                    ),
                ))

                if step % 100 == 0:
                    print(
                        f"t={t:.2f}s, detected={detected}, "
                        f"target_type={target.objectType}, "
                        f"target_category={target.objectCategory}, "
                        f"gap={gap:.2f}m, "
                        f"v={ego.speed:.2f}m/s, "
                        f"TTC={ttc:.2f}s, AEB={latched}"
                    )

                if se.SE_GetQuitFlag():
                    break

                if t >= 20.0:
                    raise RuntimeError("Scenario did not finish within 20 seconds")

                if latched:
                    # Keep the spatial path, but let braking change progress along it.
                    next_speed, distance = integrate(ego.speed, acceleration, dt)
                    next_distance = host_distance + distance
                    next_pose = reference.at_distance(next_distance)
                else:
                    # Before triggering (or with AEB off), preserve the timed replay.
                    next_pose, next_distance, next_speed = reference.at_time(t + dt)

                report_host(next_pose, next_speed, ego.p, ego.r)

                cone_visible = update_sensor_display(t+dt)

                if se.SE_StepDT(dt) != 0:
                    raise RuntimeError("Simulation step failed")

                if se.SE_GetSimulationTime() <= t:
                    if se.SE_GetQuitFlag():
                        # A stop trigger can end the scenario without advancing time.
                        # Restore the current pose so the queued next pose is not used.
                        report_host((ego.x, ego.y, ego.z, ego.h), ego.speed, ego.p, ego.r)
                        break
                    raise RuntimeError("Simulation time did not advance")

                host_distance = next_distance
                step += 1
                elapsed = time.perf_counter() - wall_start
                if not args.headless:
                    time.sleep(max(0.0, dt - elapsed))

        print(f"First AEB trigger: {first_brake_time} s")
        print(f"First collision: {first_collision_time} s")

    finally:
        se.SE_Close()
        se.SE_CloseLogFile()

    print(f"Simulation completed. Results are stored in: {output_dir}")

    return output_dir


def main(ideal=False):
    """Parse and validate command-line input before entering native simulation."""
    parser = build_parser(ideal)
    args = parser.parse_args()
    if not math.isfinite(args.drop_prob) or not 0 <= args.drop_prob <= 1:
        parser.error("drop-prob must be within [0, 1]")
    if not all(math.isfinite(value) and value > 0 for value in (args.ttc, args.decel, args.dt)):
        parser.error("ttc, decel and dt must be finite and positive")
    return run(args)
