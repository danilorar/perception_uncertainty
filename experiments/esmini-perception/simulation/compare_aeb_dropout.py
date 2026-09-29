#!/usr/bin/env python3
"""Summarize selected AEB run directories without running simulations.

Author: Zhuo Ma
Input folders must contain run_config.json and aeb_telemetry.csv. Output can be
written with --output, avoiding shell-redirection files being mistaken for runs.
"""
import argparse
import csv
import json
import sys
from pathlib import Path


def summarize(folder):
    """Compute descriptive KPIs; one random seed is not a statistical conclusion."""
    folder = Path(folder)
    config = json.loads((folder / "run_config.json").read_text(encoding="utf-8"))
    with (folder / "aeb_telemetry.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"No telemetry rows: {folder}")

    raw = sum(int(r["raw_target_detected"]) for r in rows)
    dropped = sum(int(r["target_dropped"]) for r in rows)
    longest = streak = 0
    for row in rows:
        streak = streak + 1 if int(row["target_dropped"]) else 0
        longest = max(longest, streak)

    brake = next((r for r in rows if int(r["aeb_active"])), None)
    collision_index = next((i for i, r in enumerate(rows) if int(r["collision"])), None)
    collision = rows[collision_index] if collision_index is not None else None
    # Exclude post-impact replay/overlap from minimum-gap evaluation.
    evaluation = rows[:collision_index + 1] if collision is not None else rows
    return dict(
        run=folder.name,
        scenario=config["scenario"],
        aeb=config["aeb"],
        drop_probability=config["drop_probability"],
        perception_seed=config["perception_seed"],
        dt_s=config["dt_s"],
        ttc_threshold_s=config["ttc_threshold_s"],
        deceleration_mps2=config["deceleration_mps2"],
        raw_target_samples=raw,
        dropped_target_samples=dropped,
        observed_drop_rate=dropped/raw if raw else "",
        longest_miss_s=longest*config["dt_s"],
        brake_time_s=brake["time_s"] if brake is not None else "",
        collision=int(collision is not None),
        collision_time_s=collision["time_s"] if collision is not None else "",
        ego_speed_at_collision_mps=collision["ego_speed_mps"] if collision is not None else "",
        closing_speed_at_collision_mps=collision["closing_speed_mps"] if collision is not None else "",
        minimum_gap_until_collision_or_end_m=min(float(r["gap_m"]) for r in evaluation),
        final_ego_speed_mps=rows[-1]["ego_speed_mps"],
    )


def write_comparison(folders, output):
    """Write a comparison table to an already opened text stream."""
    results = [summarize(folder) for folder in folders]
    if not results:
        raise ValueError("No AEB run directories were selected")
    writer = csv.DictWriter(output, fieldnames=list(results[0]))
    writer.writeheader()
    writer.writerows(results)


def main():
    """Validate folder inputs and keep warnings separate from CSV stdout."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", "-o", type=Path, help="Create a new CSV file")
    args = parser.parse_args()
    folders = []
    for folder in args.runs:
        if folder.is_file():
            print(f"Skipping file (expected a run directory): {folder}", file=sys.stderr)
            continue
        if not all((folder / name).is_file() for name in ("run_config.json", "aeb_telemetry.csv")):
            parser.error(f"Not an AEB run directory: {folder}")
        folders.append(folder)
    if not folders:
        parser.error("No valid AEB run directories supplied")
    try:
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", newline="", encoding="utf-8") as output:
                write_comparison(folders, output)
        else:
            write_comparison(folders, sys.stdout)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
