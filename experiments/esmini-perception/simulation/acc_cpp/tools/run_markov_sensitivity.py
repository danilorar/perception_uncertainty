#!/usr/bin/env python3
"""Compare temporal dropout at a fixed stationary missing probability.

Author: Zhuo Ma
Controls and uncertainty execute only in C++. This script schedules separate
native processes and summarizes truth-based results using the standard library.
Ideal is one deterministic reference per scene; stochastic groups share seeds.
"""
import argparse
import copy
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
from run_comparison import ROOT, execute, resolve_road, sha

RECORDINGS = ("truth.csv", "perceptions.csv", "control.csv", "metrics.csv")


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def equal_recordings(a, b, names=RECORDINGS):
    for name in names:
        if (a / name).read_bytes() != (b / name).read_bytes():
            raise RuntimeError("Recordings differ: " + name + " in " + str(b))


def target_truth(folder):
    with (folder / "truth.csv").open(newline="", encoding="utf-8") as file:
        return {(r["time_s"], r["object_id"]): r for r in csv.DictReader(file) if r["role"] == "target"}


def check_targets(reference, folder):
    current = target_truth(folder)
    if not current or not current.keys() <= reference.keys():
        raise RuntimeError("Target timestamps missing from ideal reference: " + str(folder))
    if any(reference[k] != r for k, r in current.items()):
        raise RuntimeError("Uncertainty changed target truth: " + str(folder))


def wilson(collisions, count):
    z = 1.959963984540054
    p = collisions / count
    denominator = 1 + z * z / count
    center = (p + z * z / (2 * count)) / denominator
    half = z * math.sqrt(p * (1 - p) / count + z * z / (4 * count * count)) / denominator
    return max(0.0, center - half), min(1.0, center + half)


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["scenario_id"], row["model"], row["mean_missing_s"]), []).append(row)
    output = []
    for (sid, model, duration), runs in groups.items():
        collisions = sum(r["collision"] for r in runs)
        n = len(runs)
        low, high = wilson(collisions, n) if model != "ideal" else (None, None)
        aeb_times = [r["first_aeb_s"] for r in runs if r["first_aeb_s"] is not None]
        delays = [r["aeb_trigger_delay_s"] for r in runs if r.get("aeb_trigger_delay_s") is not None]
        impacts = [r["collision_relative_speed_mps"] for r in runs if r["collision_relative_speed_mps"] is not None]
        gaps = [r["minimum_gap_m"] for r in runs if r["minimum_gap_m"] is not None]
        ttcs = [r["minimum_ttc_s"] for r in runs if r["minimum_ttc_s"] is not None]
        raw = sum(r["raw_detections"] for r in runs)
        dropped = sum(r["dropped_detections"] for r in runs)
        output.append(dict(controller=runs[0]["controller"], scenario_id=sid, model=model, mean_missing_s=duration, runs=n,
                           collisions=collisions, collision_rate=collisions/n,
                           aeb_triggered_runs=len(aeb_times),
                           mean_first_aeb_s=statistics.mean(aeb_times) if aeb_times else None,
                           mean_trigger_delay_s=statistics.mean(delays) if delays else None,
                           maximum_trigger_delay_s=max(delays) if delays else None,
                           mean_collision_relative_speed_mps=statistics.mean(impacts) if impacts else None,
                           collision_ci95_low=low, collision_ci95_high=high,
                           pooled_realized_dropout_rate=dropped/raw if raw else None,
                           mean_realized_dropout_rate=statistics.mean(r["realized_dropout_rate"] for r in runs),
                           mean_minimum_gap_m=statistics.mean(gaps) if gaps else None,
                           worst_minimum_gap_m=min(gaps) if gaps else None,
                           median_minimum_gap_m=statistics.median(gaps) if gaps else None,
                           mean_minimum_ttc_s=statistics.mean(ttcs) if ttcs else None,
                           worst_minimum_ttc_s=min(ttcs) if ttcs else None,
                           mean_longest_missing_s=statistics.mean(r["longest_missing_s"] for r in runs),
                           maximum_longest_missing_s=max(r["longest_missing_s"] for r in runs)))
    return output


def report_markdown(path, aggregated, args):
    lines = ["# " + args.controller.upper() + " Markov 漏检敏感性 / Markov dropout sensitivity", "", "作者 / Author: Zhuo Ma", "",
             f"控制器 / Controller: {args.controller}.", ""]
    if args.controller == "aeb":
        lines += [f"AEB TTC < {args.aeb_ttc} s，最大减速度 {args.aeb_deceleration} m/s²。"
                  "调用上游 R157 ReferenceDriver 的 AEB 组件，不执行驾驶员反应/巡航。"
                  "原生制动建立时间 0.6 s，漏检后保持已触发制动直到停车。", "",
                  f"AEB TTC < {args.aeb_ttc} s; maximum braking: {args.aeb_deceleration} m/s². "
                  "The upstream R157 ReferenceDriver AEB component excludes human reaction/cruise. "
                  "The native 0.6 s ramp and latch preserve braking through misses until stop.", ""]
    else:
        lines += [f"原生 ControllerACC::Step()，time gap={args.time_gap} s，速度请求由共同车辆模型执行。", "",
                  f"Native ControllerACC::Step(), time gap={args.time_gap} s; speed requests use the common vehicle model.", ""]
    lines += [f"稳态缺失概率 {args.dropout_p:.1%}，感知周期 {args.sensor_period} s，"
              f"seed {args.seed_start}..{args.seed_start + args.seed_count - 1}，初始化 {args.markov_init}，"
              f"最长 {args.duration} s 或首次接触。", "",
             f"Stationary miss probability: {args.dropout_p:.1%}. Sensor period: {args.sensor_period} s. "
             f"Seeds: {args.seed_start}..{args.seed_start + args.seed_count - 1}. "
             f"Initialization: {args.markov_init}. Duration: {args.duration} s or first contact.", "",
             "| 场景 / Scene | 模型 / Model | 平均缺失 / Mean loss s | 碰撞/运行 / Collisions/runs | 实际缺失 / Actual fraction | 平均最小gap / Mean min m | 最差gap / Worst m | 平均最小TTC / Mean min s | 平均最长缺失 / Mean longest s |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    def number(x):
        return "—" if x is None else f"{x:.3f}"
    for r in aggregated:
        lines.append(f"| {r['scenario_id']} | {r['model']} | {number(r['mean_missing_s'])} | "
                     f"{r['collisions']}/{r['runs']} | {number(r['pooled_realized_dropout_rate'])} | "
                     f"{number(r['mean_minimum_gap_m'])} | {number(r['worst_minimum_gap_m'])} | "
                     f"{number(r['mean_minimum_ttc_s'])} | {number(r['mean_longest_missing_s'])} |")
    lines += ["", "理想感知是确定性参考，不是 Monte Carlo 样本。保留理想感知碰撞；辅助完整时域目标真值验证可穿过接触点而不建模碰撞响应，排除在安全统计外。", "",
              "The ideal reference is deterministic; its run count is not a Monte Carlo sample.",
              "An ideal-perception collision is retained. If needed, a separate full-horizon target-truth validation run continues through contact without crash response; this auxiliary run is excluded from safety statistics.",
              "", "p 是稳态规律，不强制每个短场景的实际缺失比例。首次接触停止会截断检测机会和缺失段。TTC 使用纵向投影近似，CSV 的碰撞区间为 seed 间 Wilson 95% 区间。这是设定误差的敏感性结果，未校准真实传感器。", "",
              "The configured probability is a stationary law, not an enforced fraction in every short run.",
              "Colliding runs stop at first contact, so recorded exposure and missing bursts are censored.",
              "Reported TTC is the existing longitudinal projection approximation.",
              "Collision intervals in sensitivity_summary.csv use the Wilson 95% interval over seeds.",
              "These are assumed-error sensitivity results, not a calibrated sensor model.", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("aeb", "acc"), default="aeb")
    parser.add_argument("--aeb-ttc", type=float, default=1.5)
    parser.add_argument("--aeb-deceleration", type=float, default=.85*9.81)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--scenarios", type=Path, nargs="+")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument("--seed-count", type=int, default=100)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--dropout-p", type=float, default=.1)
    parser.add_argument("--mean-missing", type=float, nargs="+", default=[.2, .5, 1.0])
    parser.add_argument("--markov-init", choices=("stationary", "normal"), default="stationary")
    parser.add_argument("--dt", type=float, default=.01)
    parser.add_argument("--sensor-period", type=float, default=.05)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--time-gap", type=float, default=1.5)
    parser.add_argument("--set-speed", type=float)
    parser.add_argument("--esmini-home", type=Path)
    parser.add_argument("--validate", action="store_true", help="Check identity/repeat/p=1 for each scene and model at the first seed")
    args = parser.parse_args()
    if args.seed_count < 1 or args.seed_start < 0 or args.seed_start + args.seed_count > 2**64:
        parser.error("seed range must be nonempty and fit uint64")
    if not 0 < args.dropout_p < 1:
        parser.error("sensitivity requires 0 < dropout-p < 1")
    if len(set(args.mean_missing)) != len(args.mean_missing) or any(
            not math.isfinite(t) or t < args.sensor_period or
            args.dropout_p*args.sensor_period/(t*(1-args.dropout_p)) > 1 for t in args.mean_missing):
        parser.error("mean-missing values must be unique and give valid Markov transitions")
    scenarios = [p.resolve() for p in args.scenarios] if args.scenarios else [
        ROOT.parent.parent / "Data" / f"C_original_{sid}.xosc" for sid in ("1549178", "1554254", "1554431")]
    ids = [p.stem.removeprefix("C_original_") for p in scenarios]
    if len(set(ids)) != len(ids):
        parser.error("scenario filenames must have unique IDs")
    binary = args.binary.resolve()
    input_hashes = {str(p): sha(p) for scenario in scenarios for p in (scenario, resolve_road(scenario))}
    source_files = [p for base in (ROOT / "src", ROOT / "include") for p in base.rglob("*") if p.is_file()]
    source_files += [Path(__file__), ROOT / "tools/run_comparison.py"]
    source_hashes = {str(p.relative_to(ROOT)): sha(p) for p in source_files}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ")
    batch = args.results_dir.resolve() / (args.controller + "_markov_sensitivity_" + stamp)
    batch.mkdir(parents=True)
    (batch / "logs").mkdir()
    rows, validation, case_count = [], {}, 0
    truth_reference_runs=0
    seeds = range(args.seed_start, args.seed_start + args.seed_count)
    families = [("iid", None)] + [("markov", t) for t in args.mean_missing]
    total = len(scenarios) * (1 + args.seed_count * len(families))
    for scenario, sid in zip(scenarios, ids):
        scene_dir = batch / sid
        reference_dir = scene_dir / "ideal"
        settings = copy.copy(args)
        settings.dropout_model = "iid"
        settings.mean_missing = .5
        ideal = execute(binary, scenario, reference_dir, 0, args.seed_start, settings,
                        batch / "logs" / f"{sid}_ideal.log")
        if ideal["end_reason"] not in ("duration", "collision"):
            raise RuntimeError("Unexpected ideal scenario termination")
        truth_reference_dir=reference_dir
        if ideal["end_reason"]=="collision":
            # A collision in the ideal baseline is a result, never a reason to
            # tune the controller or discard the scene. A separate validation
            # run continues the identical model to collect exogenous target truth
            # for the full horizon; it is excluded from all safety statistics.
            truth_reference_dir=scene_dir/"checks"/"full_horizon_target_truth"
            truth_settings=copy.copy(settings)
            truth_settings.stop_on_collision=False
            truth_summary=execute(binary,scenario,truth_reference_dir,0,args.seed_start,truth_settings,
                                  batch/"logs"/f"{sid}_full_horizon_target_truth.log")
            if truth_summary["end_reason"]!="duration":
                raise RuntimeError("Incomplete target-truth validation horizon")
            truth_reference_runs+=1
        reference = target_truth(truth_reference_dir)
        check_targets(reference,reference_dir)
        rows.append(dict(controller=args.controller, scenario_id=sid, model="ideal", mean_missing_s=None, seed=None,
                         dropout_probability=0, run_directory=str(reference_dir), aeb_trigger_delay_s=0 if ideal["first_aeb_s"] is not None else None, **ideal))
        case_count += 1
        for model, duration in families:
            settings.dropout_model = model
            settings.mean_missing = duration if duration is not None else .5
            label = model if duration is None else "markov_" + format(duration, ".9g") + "s"
            for seed in seeds:
                folder = scene_dir / label / ("seed" + str(seed))
                summary = execute(binary, scenario, folder, args.dropout_p, seed, settings,
                                  batch / "logs" / f"{sid}_{label}_seed{seed}.log")
                if summary["end_reason"] not in ("duration", "collision"):
                    raise RuntimeError("Unexpected early scenario stop: " + str(folder))
                check_targets(reference, folder)
                rows.append(dict(controller=args.controller, scenario_id=sid, model=model, mean_missing_s=duration, seed=seed,
                                 dropout_probability=args.dropout_p, run_directory=str(folder),
                                 aeb_trigger_delay_s=summary["first_aeb_s"]-ideal["first_aeb_s"]
                                 if summary["first_aeb_s"] is not None and ideal["first_aeb_s"] is not None else None, **summary))
                case_count += 1
                if args.validate and seed == args.seed_start:
                    check_dir = scene_dir / "checks" / label
                    zero, repeat, missing = [check_dir / name for name in ("zero", "repeat", "missing")]
                    for name, path, p in (("zero", zero, 0), ("repeat", repeat, args.dropout_p), ("missing", missing, 1)):
                        execute(binary, scenario, path, p, seed, settings,
                                batch / "logs" / f"{sid}_{label}_{name}.log")
                        check_targets(reference, path)
                    equal_recordings(reference_dir, zero)
                    equal_recordings(folder, repeat, RECORDINGS + ("summary.json", "dropout_states.csv"))
                    with (missing / "perceptions.csv").open(newline="") as file:
                        if any(r["observed"] != "0" or (r["raw_detected"] == "1" and r["dropped"] != "1") for r in csv.DictReader(file)):
                            raise RuntimeError("Total dropout leaked a target")
                    validation[sid + ":" + label] = dict(zero_identity=True, repeat_seed=True,
                                                         total_dropout_hidden=True, target_truth_preserved=True)
                if (seed - args.seed_start + 1) % max(1, args.seed_count // 5) == 0:
                    print(f"Progress {case_count}/{total}: {sid} {label} seed {seed}", flush=True)
    if any(sha(Path(p)) != expected for p, expected in input_hashes.items()):
        raise RuntimeError("Original scenario/road data changed")
    if any(sha(ROOT / p) != expected for p, expected in source_hashes.items()):
        raise RuntimeError("Source changed during experiment")
    aggregated = summarize(rows)
    write_csv(batch / "sensitivity_runs.csv", rows)
    write_csv(batch / "sensitivity_summary.csv", aggregated)
    report_markdown(batch / "REPORT.md", aggregated, args)
    manifest = dict(author="Zhuo Ma", controller=args.controller, aeb_ttc_s=args.aeb_ttc, aeb_deceleration_mps2=args.aeb_deceleration, complete=True, created_utc=stamp, binary=str(binary),
                    binary_sha256=sha(binary), input_sha256=input_hashes, source_sha256=source_hashes,
                    stationary_missing_probability=args.dropout_p, mean_missing_s=args.mean_missing,
                    seeds=list(seeds), initialization=args.markov_init, dt_s=args.dt,
                    sensor_period_s=args.sensor_period, duration_s=args.duration, time_gap_s=args.time_gap,
                    main_runs=total, additional_validation_runs=3*len(validation),
                    additional_target_truth_reference_runs=truth_reference_runs,
                    target_truth_preserved_all_runs=True, validation=validation,
                    censoring="stop at first truth contact; realized ratios/bursts use recorded pre-contact exposure",
                    model_scope="per-target independent Markov chains; exact ego; ideal spatial-path driver; no tracker")
    (batch / "experiment_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.results_dir.resolve() / ("latest_"+args.controller+"_markov_sensitivity.txt")).write_text(str(batch) + "\n")
    print("Sensitivity:", batch / "REPORT.md", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, subprocess.CalledProcessError) as error:
        print("Sensitivity failed:", error, file=sys.stderr)
        sys.exit(1)
