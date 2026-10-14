# ACC architecture

Author: Zhuo Ma

The current runner supports ACC only. See the [file map and notebook workflow](../simulation/acc_cpp/README.md). AEB/custom movement and their tools have been retired; historical reports remain unchanged.

`main.cpp` prepares the scenario using `scenario.cpp`, initializes `esmini_adapter.cpp`, then calls `advance_native(dt)` → `SE_StepDT(dt)` → `PersistentAcc::Step(dt)` in `src/controllers/acc.cpp` → upstream `ControllerACC::Step(dt)`. Logging stays in `main.cpp`; `metrics.cpp` evaluates world truth independently of controller observations.

The integration connects observations to native ACC; it does not implement the algorithm. One controller stays attached to the real ego. It optionally updates one upstream `ObjectSensor` immediately before ACC, then copies detected targets (or all non-ego targets in baseline mode) in scenario order, preserving native road/lane positions and bounding boxes. Copies with live relative-position links or unsupported relative position types are rejected. The reference mode uses the same lifecycle with live native targets. Native ACC/esmini own ego movement; no external state reporting or custom vehicle integration remains.

The scenario preparer retains original target trajectories and initial conditions, removes ego's timed trajectory and assigns `PerceptionACC` (or `ReferenceACC`). Recorded inputs contain two explicit vehicles and flat, near-parallel forward WorldPosition trajectories. Action-based base XOSC inputs can instead use catalog vehicles and native Init speed/position actions. Ego is selected by name; native ACC captures initialized speed when cruise speed is omitted. Target actions and native stop conditions remain. Later ego maneuver groups are rejected rather than silently competing with ACC. Parameter-distribution variation files are not yet supported. Optional NCAP speed/braking overrides pass through the adapter’s native parameter-declaration callback; esmini re-evaluates dependent expressions before Init and the callback is cleared before returning from initialization. There is no general controller plugin framework.

`perception.cpp` provides the existing Markov implementation, compiled into esminiLib alongside the ACC integration. `controllers/acc.cpp` updates native ObjectSensor, filters its detected IDs through one persistent per-run Markov instance, then supplies native-position copies of retained targets. Each initial non-ego ID has an independent chain, advancing at dt even outside visibility; new IDs are rejected. No observation states are reconstructed from world coordinates. Reference and all-target modes remain available; Markov requires ideal sensing and target-copy mode. Gaussian remains paused.

Initialization at t=0 samples stationary states without a transition. Repeated zero-time calls reuse the cached frame; later physical steps advance once per dt. Logging consumes captured IDs without advancing random state. Optional live rendering changes wall-clock pacing only. `detections.csv` logs ideal-detected, dropped and supplied IDs separately.

Collision uses truth oriented 2D boxes with height overlap, checked against engine flags each step. Distance is box-surface distance; gap is projected along ego heading; TTC is gap/closing speed for an approaching in-path target. These metrics do not feed ACC.

The following describes the shared Markov component now used by persistent ACC. IID remains separately tested and is not exposed by this runner.

## 两状态 Markov 原理 / Two-state Markov model

每个目标独立维护正常 G / 缺失 M。G 保留理想观测，M 删除整条观测。模型每个感知周期推进一次，视场外也推进但不生成观测。重复序号不推进，跳帧/乱序报错。当前没有目标跟踪器。

Each target independently maintains normal G / missing M. G retains ideal observations; M removes them. State advances once per sensor period, including outside the FOV, without inventing observations. Repeated sequences do not advance state; skipped/out-of-order frames are rejected. There is no tracker.

```text
                alpha                 beta
          G --------------> M ----------------> G
          |                 |
          +-- 1-alpha --> G  +-- 1-beta --> M

P = [[1-alpha, alpha], [beta, 1-beta]]    (row/column order: G, M)
beta = sensor_period / mean_missing_duration
alpha = p * beta / (1-p)
stationary P(M) = alpha / (alpha + beta) = p
```

缺失段服从几何分布，平均时长 T，每段不固定。`stationary` 以 p 概率从 M 开始；`normal` 从 G 开始，短场景前段不再稳态。p=0/1 恒正常/恒缺失，有限平均时长不适用；一般参数要求 alpha、beta 在 [0,1] 内。

Missing burst length is geometrically distributed with mean T, rather than fixed length. `stationary` starts in M with probability p; `normal` starts in G and is not initially stationary. p=0/1 force always-normal/always-missing states, where finite mean duration does not apply. General parameters require alpha and beta in [0,1].

| p=0.1、周期 0.05 s / p=0.1, period 0.05 s | alpha G→M | beta M→G |
|---|---:|---:|
| T=0.2 s | 0.027778 | 0.25 |
| T=0.5 s | 0.011111 | 0.10 |
| T=1.0 s | 0.005556 | 0.05 |

IID 每次理想检测独立以 p 概率删除；Markov 相同 p 带来连续缺失。整数随机映射由 seed、测量序号、目标 ID 和域决定。有限场景实际漏检率不保证等于 p，首次碰撞结束也截断检测机会和缺失段。

IID independently removes each detection with probability p; Markov introduces bursts at the same stationary p. Integer random mapping uses seed, sequence, target ID and domain. Finite-run miss rates need not equal p; first-contact termination also censors exposure and burst duration.

Referenced action-based maneuver catalogs are checked for variable-setting observer actions only. Movement, entity/custom actions, nested references and missing/ambiguous entries are rejected. Catalogs are read for validation, not expanded or executed by the preparer; esmini retains those responsibilities.
