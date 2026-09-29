"""Run the unchanged official scenarios. Python standard library only.

Each invocation creates a fresh directory; old results are never overwritten.
No perception model or ADS controller is added by this runner.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "runtime/esmini-full-v3.8.1"
NCAP = ROOT / "sources/vectorgrp-OSC-NCAP-scenarios-15365d1"
BASE = NCAP / "OpenSCENARIO/NCAP/CA-FC_2026/CCRs.xosc"
VARIATION = BASE.parent / "Variations/SingleExecution/CCRs_50kph.xosc"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(case, timeout):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    out = ROOT / "runs" / (stamp + "_" + case)
    out.mkdir(parents=True, exist_ok=False)
    render = case.endswith("_render")
    is_ncap = case.startswith("ncap")
    scenario = BASE if is_ncap else RUNTIME / "resources/xosc/cut-in.xosc"
    args = [str(RUNTIME / "bin/esmini.exe"), "--osc", str(scenario)]
    if is_ncap:
        args += ["--param_dist", str(VARIATION), "--param_permutation", "0"]
    # The official default config has only window settings. Command line wins.
    args += ["--headless"]
    if render:
        args += ["--window", "0", "0", "960", "540", "--capture_screen"]
    args += ["--fixed_timestep", "0.1" if render else "0.01",
             "--seed", "1", "--collision",
             "--path", str(RUNTIME / "resources"), str(RUNTIME / "resources/models"),
             "--csv_logger", str(out / "telemetry.csv"),
             "--record", str(out / "simulation.dat"),
             "--logfile_path", str(out / "esmini.log"),
             "--log_level", "debug"]
    env = os.environ.copy()
    removed_config = env.pop("ESMINI_CONFIG_FILE", None)
    meta = {"case": case, "started_utc": datetime.now(timezone.utc).isoformat(),
            "cwd": str(out), "argv": args, "timeout_seconds": timeout,
            "python": sys.version, "esmini_sha256": digest(Path(args[0])),
            "scenario": str(scenario), "scenario_sha256": digest(scenario),
            "removed_inherited_ESMINI_CONFIG_FILE": removed_config,
            "perception_model": None, "added_controller": None}
    if is_ncap:
        meta.update(variation=str(VARIATION), variation_sha256=digest(VARIATION),
                    ncap_commit="15365d18bd7d1d6aff46c75938eddaf4325ac8f3")
    quote = lambda s: "'" + str(s).replace("'", "''") + "'"
    command = ("$env:ESMINI_CONFIG_FILE = $null\nSet-Location -LiteralPath "
               + quote(out) + "\n& " + " ".join(map(quote, args))
               + " 1> 'stdout.txt' 2> 'stderr.txt'\n$LASTEXITCODE\n")
    (out / "command.ps1").write_text(command, encoding="utf-8-sig")
    start = time.monotonic()
    try:
        with (out / "stdout.txt").open("wb") as stdout, (out / "stderr.txt").open("wb") as stderr:
            proc = subprocess.Popen(args, cwd=out, env=env, stdout=stdout, stderr=stderr,
                                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            meta["pid"] = proc.pid
            try:
                meta["exit_code"] = proc.wait(timeout=timeout)
                meta["timed_out"] = False
            except subprocess.TimeoutExpired:
                proc.kill()
                meta["exit_code"] = proc.wait()
                meta["timed_out"] = True
    except OSError as exc:
        meta.update(exit_code=None, timed_out=False, launch_error=str(exc))
    meta["elapsed_wall_seconds"] = round(time.monotonic() - start, 3)
    meta["finished_utc"] = datetime.now(timezone.utc).isoformat()
    meta["outputs"] = [{"name": p.name, "bytes": p.stat().st_size, "sha256": digest(p)}
                       for p in sorted(out.iterdir()) if p.is_file()]
    (out / "run.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps({"case": case, "directory": str(out), "exit_code": meta["exit_code"],
                      "timed_out": meta["timed_out"], "elapsed_wall_seconds": meta["elapsed_wall_seconds"]}))
    return meta["exit_code"] == 0 and not meta["timed_out"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=["all", "demo", "ncap", "demo_render", "ncap_render"], default="all")
    parser.add_argument("--timeout", type=int, default=60)
    opts = parser.parse_args()
    cases = ["demo", "ncap"] if opts.case == "all" else [opts.case]
    results = [run(case, opts.timeout) for case in cases]
    sys.exit(0 if all(results) else 1)
