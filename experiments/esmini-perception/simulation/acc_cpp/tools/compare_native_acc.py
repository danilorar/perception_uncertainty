#!/usr/bin/env python3
"""Compare persistent ACC target copies with unmodified ACC on live entities.

ReferenceACC is a diagnostic subclass: it calls ControllerACC::Step directly
without substituting entities or changing ego. Each run uses a separate process.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def compare(binary, scenario, directory, dt, time_gap, set_speed):
    for name, reference in (("copies", False), ("native", True)):
        folder = directory / name
        command = [str(binary), "--scenario", str(scenario), "--output-dir", str(folder),
                   "--controller", "acc", "--dropout-p", "0", "--dt", str(dt),
                   "--sensor-period", str(dt), "--time-gap", str(time_gap),
                   "--stop-on-collision", "false", "--detailed-logs", "true",
                   "--native-reference", str(reference).lower()]
        if set_speed is not None:
            command += ["--set-speed", str(set_speed)]
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
    # Full-precision native diagnostics include speed, xyz, selected ID, count and gap.
    paths = [directory / name for name in ("copies", "native")]
    for filename in ("native_acc.csv", "truth.csv", "results.csv", "summary.json"):
        if (paths[0] / filename).read_bytes() != (paths[1] / filename).read_bytes():
            raise RuntimeError(f"Native equivalence failed: {directory.name}/{filename}")
    with (paths[0] / "native_acc.csv").open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows or any(int(row["target_count"]) != 1 for row in rows):
        raise RuntimeError("Expected a nonempty comparison with one target each step")
    return {"scenario": scenario.name, "dt_s": dt, "time_gap_s": time_gap,
            "set_speed_mps": set_speed, "steps": len(rows),
            "max_speed_error_mps": 0, "max_position_error_m": 0,
            "selected_target_mismatches": 0,
            "selected_target_ids": sorted({int(row["lead_id"]) for row in rows})}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/runner/acc_sim")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    reports = []
    for scenario_id in (1549178, 1554254, 1554431):
        scenario = ROOT / f"../../Data/C_original_{scenario_id}.xosc"
        for label, dt, gap, speed in (("default", .01, 1.5, None),
                                      ("fast", .02, 2.0, 20.0),
                                      ("slow", .05, 1.0, 8.0)):
            report = compare(args.binary.resolve(), scenario.resolve(),
                             output / f"{scenario_id}_{label}", dt, gap, speed)
            reports.append(report)
            print(f"PASS {scenario_id} {label}: {report['steps']} steps; exact speed/position/target agreement")
    (output / "comparison.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
