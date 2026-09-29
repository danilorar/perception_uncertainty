"""Validate simulator outputs without changing scenarios or adding models."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def verify(folder):
    run = json.loads((folder / "run.json").read_text(encoding="utf-8"))
    issues = []
    checks = {}
    checks["process_success"] = run["exit_code"] == 0 and not run["timed_out"]
    checks["output_hashes_match"] = all(
        (folder / p["name"]).is_file()
        and hashlib.sha256((folder / p["name"]).read_bytes()).hexdigest() == p["sha256"]
        for p in run["outputs"])
    log = "\n".join(p.read_text(encoding="utf-8", errors="replace") for p in sorted(folder.glob("esmini*.log")))
    issues = [line for line in log.splitlines() if re.search(r"\[(warn|error)\]", line)]
    checks["storyboard_completed"] = "storyBoard runningState -> stopTransition -> completeState" in log
    checks["closed_normally"] = bool(re.search(r"\[info\] Closing", log))
    csvs = sorted(folder.glob("telemetry*.csv"))
    checks["one_csv"] = len(csvs) == 1
    stats = {}
    if len(csvs) == 1:
        text = csvs[0].read_text(encoding="utf-8-sig")
        lines = text.splitlines()
        header = next(i for i, line in enumerate(lines) if line.startswith("Index [-],"))
        reader = csv.DictReader(io.StringIO("\n".join(lines[header:])), skipinitialspace=True)
        rows = list(reader)
        fields = [field.strip() for field in reader.fieldnames if field.strip()]
        # Names and collision IDs are strings; the other fields are numeric.
        numeric = [field for field in fields if "Entity_Name" not in field and "collision_ids" not in field]
        checks["finite_numeric_values"] = bool(rows) and all(
            math.isfinite(float(row[field])) for row in rows for field in numeric)
        times = [float(row["TimeStamp [s]"]) for row in rows]
        dt = float(run["argv"][run["argv"].index("--fixed_timestep") + 1])
        checks["monotonic_fixed_step"] = len(times) > 1 and all(
            abs(b - a - dt) < 2e-6 for a, b in zip(times, times[1:]))
        checks["contiguous_indices"] = [int(r["Index [-]"]) for r in rows] == list(range(len(rows)))
        names = [rows[0][f"#{i} Entity_Name [-]"] for i in (1, 2)]
        collisions = [row for row in rows if row["#1 collision_ids"].strip()]
        stats = {"csv": str(csvs[0].relative_to(ROOT)), "rows": len(rows),
                 "start_time_s": times[0], "last_time_s": times[-1], "timestep_s": dt,
                 "entities": names,
                 "first_collision_time_s": float(collisions[0]["TimeStamp [s]"]) if collisions else None,
                 "entities_statistics": {}}
        for i, name in enumerate(names, 1):
            speeds = [float(row[f"#{i} Current_Speed [m/s]"]) for row in rows]
            positions = [[float(row[f"#{i} World_Position_{axis} [m]"]) for axis in "XYZ"] for row in rows]
            stats["entities_statistics"][name] = {"min_speed_mps": min(speeds), "max_speed_mps": max(speeds),
                                                  "first_xyz_m": positions[0], "last_xyz_m": positions[-1]}
        if run["case"].startswith("ncap"):
            checks["ncap_parameters_applied"] = "Ego_speed_kph: 50" in log and "ImpactLocation: 50" in log
            checks["ncap_ego_50kph"] = all(abs(float(r["#1 Current_Speed [m/s]"]) - 50 / 3.6) < 1e-5 for r in rows)
            checks["ncap_target_stationary"] = all(abs(float(r["#2 Current_Speed [m/s]"])) < 1e-8 for r in rows)
            checks["ncap_collision_recorded"] = bool(collisions) and "Set variable collisionDetected = true" in log
            checks["ncap_collision_stop_trigger"] = bool(re.search(r"\[info\] StopAfterCollision: true\s*$", log, re.M))
            ego, target = rows[0], rows[0]
            # For this straight, zero-heading case, project the original catalog bounding boxes onto x.
            initial_gap = (float(target["#2 World_Position_X [m]"]) + float(target["#2 bb_x [m]"])
                           - float(target["#2 bb_length [m]"]) / 2
                           - float(ego["#1 World_Position_X [m]"]) - float(ego["#1 bb_x [m]"])
                           - float(ego["#1 bb_length [m]"]) / 2)
            expected_contact = initial_gap / (50 / 3.6)
            observed_contact = stats["first_collision_time_s"]
            stats.update(initial_bbox_gap_m=initial_gap, geometric_contact_time_s=expected_contact,
                         first_collision_ego_speed_kph=(float(collisions[0]["#1 Current_Speed [m/s]"]) * 3.6) if collisions else None)
            checks["ncap_collision_matches_geometry"] = observed_contact is not None and abs(observed_contact - expected_contact) <= dt + 1e-5
    dats = sorted(folder.glob("simulation*.dat"))
    checks["nonempty_recording"] = len(dats) == 1 and dats[0].stat().st_size > 1024
    if run["case"].endswith("_render"):
        frames = sorted(folder.glob("screen_shot_*.tga"))
        checks["render_frames_written"] = len(frames) > 1 and all(p.stat().st_size > 1000 for p in frames)
        stats["render_frame_count"] = len(frames)
    result = {"directory": str(folder.relative_to(ROOT)), "case": run["case"],
              "passed": all(checks.values()), "checks": checks, "statistics": stats,
              "log_error_count": sum("[error]" in x for x in issues),
              "log_warning_count": sum("[warn]" in x for x in issues), "log_issues": issues,
              "scope": "Execution and telemetry checks only; not ADS safety validation or NCAP certification."}
    (folder / "validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


if __name__ == "__main__":
    results = [verify(path.parent) for path in sorted((ROOT / "runs").glob("*/run.json"))]
    (ROOT / "evidence/validation-summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    for result in results:
        print(json.dumps({k: v for k, v in result.items() if k != "log_issues"}))
    sys.exit(0 if results and all(r["passed"] for r in results) else 1)
