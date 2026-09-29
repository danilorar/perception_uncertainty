#!/usr/bin/env python3
"""Run four matched AEB/dropout conditions and write a comparison CSV.

Author: Zhuo Ma
For each seed: AEB off/on x dropout 0/p. Each run uses a separate process because
esmini owns native global state. All batch simulations run without the viewer.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys

from compare_aeb_dropout import write_comparison
from paths import RESULTS, SCENARIO_IDS, SIMULATION


def main():
    """Run a matched parameter matrix without globbing unrelated old results."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIO_IDS, default="1549178")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--drop-prob", type=float, default=0.1)
    parser.add_argument("--ttc", type=float, default=2.0)
    parser.add_argument("--decel", type=float, default=6.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--results-dir", type=Path, default=RESULTS)
    args = parser.parse_args()
    if not math.isfinite(args.drop_prob) or not 0 < args.drop_prob <= 1:
        parser.error("drop-prob must be within (0, 1]")
    if not all(math.isfinite(v) and v > 0 for v in (args.ttc, args.decel, args.dt)):
        parser.error("ttc, decel and dt must be finite and positive")
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    batch = args.results_dir.expanduser().resolve() / f"comparison_{args.scenario}_{stamp}"
    batch.mkdir(parents=True)
    folders = []
    manifest = []
    for seed in dict.fromkeys(args.seeds):
        for probability in (0.0, args.drop_prob):
            for mode in ("off", "on"):
                command = [
                    sys.executable, str(SIMULATION / "my_aeb_dropout_demo.py"),
                    "--headless", "--scenario", args.scenario, "--aeb", mode,
                    "--drop-prob", str(probability), "--perception-seed", str(seed),
                    "--ttc", str(args.ttc), "--decel", str(args.decel),
                    "--dt", str(args.dt), "--results-dir", str(batch),
                ]
                before = set(batch.iterdir())
                log_path = batch / f"console_{mode}_drop{probability:g}_seed{seed}.log"
                print(f"Running: AEB={mode}, dropout={probability:g}, seed={seed}", flush=True)
                with log_path.open("w", encoding="utf-8") as log:
                    process = subprocess.run(command, cwd=batch, stdout=log,
                                             stderr=subprocess.STDOUT, timeout=120)
                if process.returncode:
                    raise RuntimeError(f"Simulation failed; see {log_path}")
                created = [p for p in set(batch.iterdir()) - before if p.is_dir()]
                if len(created) != 1:
                    raise RuntimeError("Expected one new run directory")
                folders.append(created[0])
                manifest.append(dict(run=created[0].name, aeb=mode,
                                     drop_probability=probability, perception_seed=seed))
    # Summarize exactly this invocation's runs, not every item under RESULTS.
    with (batch / "comparison.csv").open("w", newline="", encoding="utf-8") as output:
        write_comparison(folders, output)
    (batch / "runs.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Comparison: {batch / 'comparison.csv'}")


if __name__ == "__main__":
    main()
