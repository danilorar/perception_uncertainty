# 原生 AEB + 两状态 Markov / Native AEB with two-state Markov loss

作者 / Author: Zhuo Ma · 完成 / Completed: 2026-10-06, macOS

固定 esmini v3.8.1 / `19d26b68f7f473de4e55046a794b5fd2f91c58c8`。原生 AEB 位于 ALKS_R157SM ReferenceDriver，调用 `Process` / `UpdateAEB` / `ReactCritical`，排除完整驾驶员反应和巡航；上游源码未改动。桥接提供隔离的处理后观测及持久制动状态。

esmini v3.8.1 is pinned to `19d26b68f7f473de4e55046a794b5fd2f91c58c8`. Native AEB is in ALKS_R157SM ReferenceDriver: `Process` / `UpdateAEB` / `ReactCritical`, excluding full driver reaction and cruise. Upstream source is unchanged; the bridge supplies isolated processed observations and persistent braking state.

参数：TTC < 1.5 s，0.85g = 8.3385 m/s²，0.6 s 制动建立及原生前方重叠条件，车辆 ±10 m/s² 限幅。准确自车反馈，触发前保持初始速度、沿原空间路径行驶；目标执行原时间轨迹。感知 dt=0.05 s，物理 dt=0.01 s，p=0.1，stationary 初始化，最长 10 s 或首次真值接触。

Settings: TTC < 1.5 s, 0.85g = 8.3385 m/s², a 0.6 s brake ramp and native front-overlap checks, with ±10 m/s² vehicle limits. Ego feedback is exact; before triggering it holds initial speed along the original spatial path. The target follows its original timed trajectory. Sensor dt=0.05 s, physics dt=0.01 s, p=0.1, stationary initialization, and a 10 s horizon or first truth contact.

Markov 平均缺失 T=0.2/0.5/1.0 s，beta=0.05/T，alpha=p*beta/(1-p)。保留 IID 基准，各随机组 seed=0..99。共 1203 次主实验、36 次边界/复现验证、1 次完整时域目标真值参考，共 1240 次；三个 ideal 单次是确定性参考。

Markov mean loss T=0.2/0.5/1.0 s, beta=0.05/T, alpha=p*beta/(1-p). IID is retained, with seeds 0..99 in each random group. There are 1203 main runs, 36 boundary/reproduction checks and one full-horizon target-truth reference, totaling 1240. The three single ideal runs are deterministic references.

## 碰撞和实际缺失 / Collision and realized loss

| 场景 / Scene | 模型 / Model | T / s | 碰撞/运行 / Collisions/runs | 实际缺失 / Realized loss | 平均最小 gap / Mean min m | Wilson 95% |
|---|---|---:|---:|---:|---:|---|
| 1549178 | ideal | — | 0/1 | 0.00% | 1.553 | — |
| 1549178 | iid | — | 0/100 | 9.99% | 1.517 | 0.00%–3.70% |
| 1549178 | markov | 0.200 | 4/100 | 10.64% | 1.464 | 1.57%–9.84% |
| 1549178 | markov | 0.500 | 8/100 | 11.42% | 1.415 | 4.11%–15.00% |
| 1549178 | markov | 1.000 | 9/100 | 12.42% | 1.389 | 4.81%–16.23% |
| 1554254 | ideal | — | 0/1 | 0.00% | 0.880 | — |
| 1554254 | iid | — | 1/100 | 9.98% | 0.856 | 0.18%–5.45% |
| 1554254 | markov | 0.200 | 7/100 | 10.60% | 0.804 | 3.43%–13.75% |
| 1554254 | markov | 0.500 | 9/100 | 11.51% | 0.793 | 4.81%–16.23% |
| 1554254 | markov | 1.000 | 9/100 | 12.30% | 0.796 | 4.81%–16.23% |
| 1554431 | ideal | — | 1/1 | 0.00% | -0.028 | — |
| 1554431 | iid | — | 100/100 | 9.89% | -0.029 | 96.30%–100.00% |
| 1554431 | markov | 0.200 | 100/100 | 11.80% | -0.031 | 96.30%–100.00% |
| 1554431 | markov | 0.500 | 100/100 | 11.24% | -0.031 | 96.30%–100.00% |
| 1554431 | markov | 1.000 | 100/100 | 12.05% | -0.031 | 96.30%–100.00% |

10% 是稳态参数，不强制有限运行实际比例。首次碰撞截止和 AEB 停车导致离开 FOV 会改变记录的检测机会/缺失段。表中是设定误差敏感性，不是实测传感器风险。

10% is stationary probability rather than a forced finite-run ratio. First-contact termination and leaving FOV after AEB stopping alter recorded detection exposure and bursts. These are assumed-error sensitivity results rather than measured sensor risk.

## 触发和接触速度 / Trigger and contact speed

| 场景 / Scene | 模型 / Model | T / s | 已触发/运行 / Triggered/runs | 首次触发均值 / Mean first s | 延迟均值 / Mean delay s | 接触相对速度均值 / Mean relative m/s |
|---|---|---:|---:|---:|---:|---:|
| 1549178 | ideal | — | 1/1 | 3.280 | 0.000 | — |
| 1549178 | iid | — | 100/100 | 3.283 | 0.003 | — |
| 1549178 | markov | 0.200 | 100/100 | 3.295 | 0.015 | 4.232 |
| 1549178 | markov | 0.500 | 99/100 | 3.305 | 0.025 | 5.443 |
| 1549178 | markov | 1.000 | 97/100 | 3.313 | 0.033 | 6.868 |
| 1554254 | ideal | — | 1/1 | 3.530 | 0.000 | — |
| 1554254 | iid | — | 100/100 | 3.533 | 0.003 | 5.179 |
| 1554254 | markov | 0.200 | 100/100 | 3.550 | 0.020 | 6.211 |
| 1554254 | markov | 0.500 | 98/100 | 3.548 | 0.018 | 7.522 |
| 1554254 | markov | 1.000 | 96/100 | 3.550 | 0.020 | 10.590 |
| 1554431 | ideal | — | 1/1 | 3.420 | 0.000 | 8.270 |
| 1554431 | iid | — | 100/100 | 3.423 | 0.003 | 8.252 |
| 1554431 | markov | 0.200 | 100/100 | 3.446 | 0.026 | 8.599 |
| 1554431 | markov | 0.500 | 99/100 | 3.444 | 0.024 | 8.679 |
| 1554431 | markov | 1.000 | 97/100 | 3.445 | 0.025 | 8.859 |

触发延迟相对同场景 ideal，只对已触发运行求均值；接触速度只对碰撞运行求均值，无碰撞动力学。触发计数包含未触发情况。1549178 的 Markov 碰撞为 4/8/9 次，1554254 为 7/9/9 次；100 seed 不支持将所有差异都解释为严格单调或统计显著。

Delay is relative to the same scenario's ideal trigger, averaged over triggered runs only. Contact speed averages colliding runs only, without crash dynamics. Trigger counts expose non-triggering runs. Markov collisions are 4/8/9 for 1549178 and 7/9/9 for 1554254. One hundred seeds do not establish strict monotonicity or statistical significance of every difference.

1554431 理想在 3.42 s 触发、5.14 s 接触，相对速度约 8.270 m/s。当前控制器/车辆/驾驶模型组合不能避免该理想碰撞，所有随机组碰撞率饱和；Markov 接触速度 8.599/8.679/8.859 m/s 仍反映误差影响。不能把该场景全部碰撞归因于漏检；未为消除碰撞调高阈值或制动能力。

For 1554431, ideal triggers at 3.42 s and contacts at 5.14 s with relative speed about 8.270 m/s. The current controller/vehicle/driver combination cannot avoid that ideal collision; random-group collision rates saturate. Markov contact speeds 8.599/8.679/8.859 m/s still show error effects. Not all collisions are attributable to misses, and thresholds/brake capability were not raised to remove the result.

## 验证 / Validation

7 项 CTest 通过：原 ACC、原生 AEB 触发/方向/重叠、准确速度向量、漏检后锁存、制动坡度/上限、停车释放、run 状态隔离和 Markov 等。三场景 × 四误差族验证 p=0 等价、同 seed 复现、p=1 无目标观测。逐步核对真值几何与引擎碰撞，核对全部目标真值。

Seven CTests passed, covering original ACC, native AEB trigger/direction/overlap, exact velocity vectors, latch through misses, ramp/limits, stop release, run isolation and Markov behavior. Three scenarios × four error families checked p=0 equivalence, reproduction and p=1 observation removal. Truth geometry matched engine collision step by step, with all target truth checked.

1554431 额外参考运行穿过接触点继续到 10 s，不建模碰撞响应、不计安全统计。六个原始 XOSC/XODR 和上游 ACC/ALKS 源码指纹不变。旧 ACC seed42 IID 的既有字段与历史记录一致。Windows/Linux 未执行。

The extra 1554431 reference continues through contact to 10 s without crash response and is excluded from safety statistics. Six original XOSC/XODR and upstream ACC/ALKS hashes remained unchanged. Existing ACC seed42 IID fields matched historical records. Windows/Linux were not executed.

## 精选数据和复现 / Selected data and reproduction

[汇总 CSV / Summary CSV](aeb_markov_20261006/sensitivity_summary.csv)、[逐次指标 / Per-run metrics](aeb_markov_20261006/sensitivity_runs.csv)、[参数与指纹 / Settings and hashes](aeb_markov_20261006/experiment_manifest.json)、[导出来源 / Export provenance](aeb_markov_20261006/export_manifest.json)。原批次名为 `aeb_markov_sensitivity_20261006T071308-294610Z`，本机原始结果保留但不进入 Git。

The linked CSVs/manifests retain summaries, per-run metrics, settings and provenance. Original batch: `aeb_markov_sensitivity_20261006T071308-294610Z`. Local raw results are retained and ignored by Git.

从 acc_cpp/ 运行 / Run from acc_cpp/:

```bash
python3 tools/build_native.py
python3 tools/run_markov_sensitivity.py --binary build/runner/acc_sim \
  --controller aeb --dropout-p 0.1 --mean-missing 0.2 0.5 1.0 \
  --seed-count 100 --seed-start 0 --dt 0.01 --sensor-period 0.05 \
  --duration 10 --markov-init stationary --validate
```

切 ACC 使用 `--controller acc --time-gap 1.5`；它不等于 AEB TTC，AEB 不使用 `--set-speed`。Windows 使用 `python`、`build/runner/Release/acc_sim.exe` 和单行命令。

Switch to ACC with `--controller acc --time-gap 1.5`; this is distinct from AEB TTC, and AEB does not use `--set-speed`. On Windows use `python`, `build/runner/Release/acc_sim.exe` and a single-line command.
