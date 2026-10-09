#!/usr/bin/env python3
"""Run matched identity/dropout cases; summarize truth-based safety outcomes.

Author: Zhuo Ma
Each run is a separate C++ process (esmini has process-global engine state).
Python never computes control or changes ego/target state.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def default_scenario():
    for parent in ROOT.parents:
        candidate = parent / "Data/C_original_1549178.xosc"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("Specify --scenario; cannot locate Data/C_original_1549178.xosc")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve_road(scenario):
    """Match the native adapter: declared LogicFile, then adjacent basename."""
    node = ET.parse(scenario).getroot().find("./RoadNetwork/LogicFile")
    if node is None or not node.get("filepath"):
        raise ValueError("Scenario must declare a road LogicFile")
    declared = Path(node.get("filepath"))
    road = declared if declared.is_absolute() else scenario.parent / declared
    if not road.is_file():
        road = scenario.parent / declared.name
    if not road.is_file():
        raise FileNotFoundError("Cannot resolve scenario OpenDRIVE file")
    return road.resolve()


def execute(binary, scenario, folder, probability, seed, args, logfile=None):
    # Detailed logs are requested explicitly: the target-truth checks here and in
    # run_markov_sensitivity.py, --validate, and replay_perception.py all read them.
    command = [str(binary), "--detailed-logs", "true", "--scenario", str(scenario), "--output-dir", str(folder),
               "--dropout-p", str(probability), "--seed", str(seed), "--dt", str(args.dt),
               "--sensor-period", str(args.sensor_period), "--duration", str(args.duration),
               "--time-gap", str(args.time_gap), "--dropout-model", args.dropout_model,
               "--mean-missing", str(args.mean_missing), "--markov-init", args.markov_init, "--controller", args.controller,
               "--aeb-ttc", str(args.aeb_ttc), "--aeb-deceleration", str(args.aeb_deceleration),
               "--stop-on-collision", "true" if getattr(args,"stop_on_collision",True) else "false"]
    if args.set_speed is not None:
        command += ["--set-speed", str(args.set_speed)]
    if args.esmini_home:
        command += ["--esmini-home", str(args.esmini_home.resolve())]
    if logfile:
        with Path(logfile).open("w", encoding="utf-8") as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    else:
        subprocess.run(command, check=True)
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    if not summary.get("complete"):
        raise RuntimeError("Incomplete run: " + str(folder))
    return summary


def validate_pair(ideal, zero, repeat, dropout, missing):
    # Compare recorded decisions/physics, not path-containing config/log text.
    for name in ("truth.csv", "perceptions.csv", "control.csv", "metrics.csv", "summary.json", "dropout_states.csv",
                 "results.csv"):
        if (ideal / name).read_bytes() != (zero / name).read_bytes():
            raise RuntimeError("Identity equivalence failed: " + name)
        if (dropout / name).read_bytes() != (repeat / name).read_bytes():
            raise RuntimeError("Seed reproducibility failed: " + name)
    with (missing / "perceptions.csv").open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["observed"] != "0" or (row["raw_detected"] == "1" and row["dropped"] != "1"):
                raise RuntimeError("100% dropout exposed a target or failed to delete a detection")
    summary = json.loads((missing / "summary.json").read_text(encoding="utf-8"))
    # Total dropout need not cause a collision in every scene/configuration.
    # The native loop checks truth-box/engine collision agreement on every step.
    # The exogenous target remains the original trajectory in both experiments.
    def targets(folder):
        with (folder / "truth.csv").open(newline="", encoding="utf-8") as file:
            return {row["time_s"]: row for row in csv.DictReader(file) if row["role"] == "target"}
    a = targets(ideal)
    for folder in (zero, dropout, repeat, missing):
        b = targets(folder)
        if not a.keys() & b.keys():
            raise RuntimeError("No shared target timestamps to validate")
        for time in a.keys() & b.keys():
            if a[time] != b[time]:
                raise RuntimeError("Perception error changed target truth at t=" + time)
    return {"identity_equivalence": True, "repeat_seed_equivalence": True,
            "total_dropout_targets_hidden": True,
            "total_dropout_collision_observed": summary["collision"], "target_truth_preserved": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("aeb", "acc"), default="aeb")
    parser.add_argument("--aeb-ttc", type=float, default=1.5)
    parser.add_argument("--aeb-deceleration", type=float, default=.85*9.81)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--dropout-p", type=float, default=.1)
    parser.add_argument("--dropout-model", choices=("markov", "iid"), default="markov")
    parser.add_argument("--mean-missing", type=float, default=.5, help="Markov mean missing duration in seconds")
    parser.add_argument("--markov-init", choices=("stationary", "normal"), default="stationary")
    parser.add_argument("--dt", type=float, default=.01)
    parser.add_argument("--sensor-period", type=float, default=.05)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--time-gap", type=float, default=1.5)
    parser.add_argument("--set-speed", type=float)
    parser.add_argument("--esmini-home", type=Path)
    parser.add_argument("--validate", action="store_true", help="Also run p=0/repeat/p=1 invariant checks")
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(seed < 0 for seed in args.seeds):
        parser.error("seeds must be unique nonnegative integers")
    binary = args.binary.resolve()
    scenario = args.scenario.resolve() if args.scenario else default_scenario().resolve()
    road = resolve_road(scenario)
    before = {"scenario_sha256": sha(scenario), "road_sha256": sha(road), "binary_sha256": sha(binary)}
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ")
    scenario_id = scenario.stem.removeprefix("C_original_")
    batch = args.results_dir.resolve() / ("comparison_" + scenario_id + "_" + timestamp)
    batch.mkdir(parents=True, exist_ok=False)
    rows, validations = [], {}
    for seed in args.seeds:
        ideal, dropout = batch / ("ideal_seed" + str(seed)), batch / ("dropout_seed" + str(seed))
        for name, folder, probability in (("ideal", ideal, 0), ("dropout", dropout, args.dropout_p)):
            summary = execute(binary, scenario, folder, probability, seed, args)
            rows.append(dict(controller=args.controller, model=name, seed=seed, dropout_probability=probability,
                             dropout_model=args.dropout_model, mean_missing_s=args.mean_missing if args.dropout_model=="markov" else None, **summary))
        if args.validate:
            zero, repeat, missing = (batch / (name + "_seed" + str(seed)) for name in ("zero_check", "repeat_check", "missing_check"))
            execute(binary, scenario, zero, 0, seed, args)
            execute(binary, scenario, repeat, args.dropout_p, seed, args)
            execute(binary, scenario, missing, 1, seed, args)
            validations[str(seed)] = validate_pair(ideal, zero, repeat, dropout, missing)
    if sha(scenario) != before["scenario_sha256"] or sha(road) != before["road_sha256"]:
        raise RuntimeError("Original input data changed")
    with (batch / "comparison.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    manifest = dict(author="Zhuo Ma", controller=args.controller, aeb_ttc_s=args.aeb_ttc, aeb_deceleration_mps2=args.aeb_deceleration, scenario=str(scenario), road=str(road), binary=str(binary), created_utc=timestamp,
                    seeds=args.seeds, dropout_probability=args.dropout_p, dropout_model=args.dropout_model,
                    mean_missing_s=args.mean_missing, markov_initialization=args.markov_init, validation=validations, **before)
    (batch / "batch_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (args.results_dir.resolve() / "latest.txt").write_text(str(batch) + "\n", encoding="utf-8")
    print("Comparison:", batch / "comparison.csv")
    if validations:
        print("Validated: identity, seed reproducibility, target truth, p=1 targets hidden; collision checked against engine each step")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, ET.ParseError, OSError, subprocess.CalledProcessError) as error:
        print("Comparison failed:", error, file=sys.stderr)
        sys.exit(1)
