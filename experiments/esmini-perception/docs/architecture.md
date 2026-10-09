# 架构与接口 / Architecture and interfaces

作者 / Author: Zhuo Ma

本页描述 `simulation/acc_cpp/` 原生闭环；构建与运行命令见 [原生运行指南](../simulation/acc_cpp/README.md)。

This page describes the native loop in `simulation/acc_cpp/`; see the [native guide](../simulation/acc_cpp/README.md) for build and run commands.

## 文件与依赖 / Files and dependencies

| 文件（相对 acc_cpp） / File (relative to acc_cpp) | 输入 / Input | 输出及责任 / Output and responsibility |
|---|---|---|
| `tools/build_native.py`, `tools/add_bridge.cmake` | 固定 esmini commit、桥接源码 / Pinned commit, bridge source | 带 ABI 2 桥接的库、runner、测试 / Library with ABI 2 bridges, runner, tests |
| `src/scenario.cpp` | 原始 XOSC/XODR / Original XOSC/XODR | 生成副本、初始速度、限制、自车空间路径 / Working copy, speeds, limits, ego path |
| `src/esmini_adapter.cpp` | 生成场景、自车状态 / Working scenario, ego state | 同步世界真值、理想感知快照 / Synchronized truth, ideal snapshot |
| `src/perception.cpp` | 理想 `PerceptionFrame`、seed、p、T / Ideal frame, seed, p, T | 删除漏检目标后的新帧及审计 / Processed frame and loss audit |
| `src/esmini_aeb.cpp`, `src/native_aeb_bridge.cpp` | 准确自车、缓存观测、持久 context / Exact ego, cached observations, persistent context | 上游 AEB 组件产生请求 / Upstream AEB component produces request |
| `src/esmini_acc.cpp`, `src/native_acc_bridge.cpp` | 同一输入边界 / Same input boundary | 上游 ACC 速度请求转加速度 / Upstream ACC speed request converted to acceleration |
| `src/vehicle.cpp` | 请求、限制、dt、路径 / Request, limits, dt, path | 限幅、积分、下一自车状态 / Limits, integration, next ego state |
| `src/metrics.cpp` | `WorldTruthFrame` | 真值间隙、TTC、碰撞 / Truth gap, TTC, collision |
| `src/main.cpp` | 配置及上述模块 / Configuration and modules above | 调度、缓存、状态提交、记录 / Scheduling, caching, reporting, logs |
| `tools/run_comparison.py` | 程序、场景、seed / Binary, scenario, seeds | 配对比较及不变量检查 / Paired comparison and invariant checks |
| `tools/run_markov_sensitivity.py` | 多场景、共同 seed、T / Scenarios, shared seeds, T | 逐次/聚合统计、报告、指纹 / Per-run/aggregate statistics, report, hashes |
| `tools/replay_perception.py` | 已存真值、控制及感知 / Saved truth, control, perception | 确定性回放、cone 显隐 / Deterministic replay, cone visibility |
| `tools/export_report.py` | 已完成批次 / Completed batch | 小型可提交报告 / Compact reports for Git |

`include/accsim/types.hpp` 定义值类型；`include/accsim/control.hpp` 定义控制器接口；`include/native_acc_bridge.h` 定义 ACC/AEB 共用的跨库 C ABI。依赖为 esmini/fmt 与已附带的 pugixml 1.14；pugixml 使用独立命名空间，避免与 esmini 内部版本冲突。

`include/accsim/types.hpp` defines value types, `include/accsim/control.hpp` defines controller interfaces, and `include/native_acc_bridge.h` defines the shared ACC/AEB C ABI. Dependencies are esmini/fmt and the included pugixml 1.14. A separate pugixml namespace avoids conflicts with esmini's internal version.

## 场景如何进入闭环 / How scenarios enter the loop

输入 ID 为 `1549178`、`1554254`、`1554431`，`object_1` 是自车，`object_2` 是目标。`Data/` 只读。每次运行在自己的 `generated/` 内创建场景及字节一致的道路快照，修复道路引用和初始化 action 结构，将自车控制接口改成 `ExternalController`，移除自车原时间轨迹控制，保留目标原时间轨迹。

Input IDs are `1549178`, `1554254` and `1554431`; `object_1` is ego and `object_2` is the target. `Data/` is read only. Each run creates a scenario and byte-identical road snapshot in its own `generated/`, fixes road references and initialization actions, assigns `ExternalController` to ego, removes ego's timed trajectory control, and retains the target's timed trajectory.

初始速度由原轨迹第一段推导：原 XOSC 初始 SpeedAction 为零，但紧接着轨迹开始运动。自车沿原空间路径行驶，进度取决于实际积分距离。两组共用初始条件、配置和模型；闭环反应不同，后续自车位置允许不同。

Initial speeds come from the first trajectory segment: the original SpeedAction starts at zero although the trajectory immediately moves. Ego follows the original spatial path with progress determined by integrated distance. Cases share initial conditions, configurations and models; their later ego positions may differ because of closed-loop responses.

`ExternalController` 是提交状态的接口，真正的决策来自桥接调用的原生 AEB/ACC。原 XOSC 中的 `HiDriveController` 声明不代表该控制器正在运行。

`ExternalController` is the state-reporting interface; decisions come from native AEB/ACC through the bridge. The original XOSC declaration of `HiDriveController` does not mean that controller is running.

## 一帧的时序 / Frame timing

1. 读取 `t_k` 世界真值，自车作为准确反馈。 / Read world truth at `t_k`; use exact ego feedback.
2. 每 0.05 s 用 `SE_AddObjectSensor` / `SE_FetchSensorObjectList` 获取理想 ID，再在同一帧读取完整目标属性。 / Every 0.05 s, obtain ideal sensor IDs and read complete target properties in the same engine frame.
3. IID/Markov 处理一次，缓存 `PerceptionFrame`。 / Apply IID/Markov once and cache the processed frame.
4. 每 0.01 s 控制器接收当前准确自车和最新缓存观测。 / Every 0.01 s, the controller receives exact ego and cached observations.
5. 限幅并积分，提交自车状态，推进目标场景。 / Limit and integrate, report ego state, and advance the target scenario.
6. 下一帧真值用于评价及记录，默认首次接触结束。 / Evaluate and log next-frame truth; stop at first contact by default.

初始化 `SE_StepDT(0)` 填充传感器而不推进时间。感知周期必须是物理步长的整数倍。非采样步复用旧帧；measurement time、sequence、age 和 fresh 记录其新旧状态。

Initialization uses `SE_StepDT(0)` without advancing time. The sensor period must be an integer multiple of the physics step. Non-sampling steps reuse the frame; measurement time, sequence, age and fresh record its freshness.

## 真值和观测隔离 / Truth and observation isolation

`WorldTruthFrame` 含准确自车和真实目标，用于理想采样及评价。`ControllerInput` 只含自车、dt/time 和独立 `PerceptionFrame`，不含真实目标容器。跨库传递 POD 值副本，不传活动实体指针或 STL 容器。

`WorldTruthFrame` contains exact ego and real targets for ideal sampling and evaluation. `ControllerInput` contains ego, dt/time and an independent `PerceptionFrame`, without target-truth containers. The C ABI passes POD value copies rather than live pointers or STL containers.

桥接只从处理后目标建立隔离实体视图。漏检目标完全不存在；速度/加速度向量、类型、尺寸、位置一并经过误差模块，不在控制步补读真值。显示与日志也复用同一帧。

The bridge constructs an isolated entity view solely from processed targets. Missing targets are absent entirely. Velocity/acceleration vectors, type, size and pose all pass through the error module, without control-time truth lookup. Display and logs reuse the same frame.

ACC 原生 `Step()` 遍历 `entities_`；其桥接每步从准确自车和固定巡航目标重建实例。AEB 用每次运行持久 context 调用 `Process()`、`UpdateAEB()`、`ReactCritical()`，保存锁存和制动坡度；漏检不释放已触发制动，停车时释放。

Native ACC `Step()` iterates `entities_`; its bridge rebuilds the instance each step from exact ego and the fixed cruise target. AEB uses a persistent per-run context calling `Process()`, `UpdateAEB()` and `ReactCritical()`, preserving latch and ramp. Misses do not release active braking; stopping releases it.

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

## 模型范围 / Model scope

默认前向 sensor 位置 `(2.5,0,0.5)` m，范围 0.1–60 m，FOV π/3 rad（60°）。这是对象几何可见性，不是雷达/相机物理模型。v3.8.1 实现使用 radians。

The default forward sensor is at `(2.5,0,0.5)` m, with range 0.1–60 m and FOV π/3 rad (60°). It models geometric object visibility rather than radar/camera physics. The v3.8.1 implementation uses radians.

车辆采用限幅恒加速度积分及理想空间路径跟随，没有轮胎、执行器延迟、融合或碰撞响应。支持两个显式车辆、t=0 开始的绝对 WorldPosition 轨迹、平坦近似同向向前运动。没有累计横向偏移上限，但检查高度变化 ≤0.05 m、相对初始航向变化 ≤0.05 rad、pitch/roll ≤0.02 rad。

The vehicle uses bounded constant-acceleration integration and ideal path following, without tires, actuator delay, fusion or crash response. Inputs contain two explicit vehicles and absolute WorldPosition trajectories from t=0 on flat, near-parallel forward motion. There is no cumulative lateral-offset cap, but height variation ≤0.05 m, heading variation ≤0.05 rad relative to initial heading, and pitch/roll ≤0.02 rad are checked.

碰撞用真值有向二维包围盒与高度重叠，逐步核对引擎标志。距离是包围盒表面最短距离；gap 沿自车方向投影；TTC 是同路径接近目标的 gap/closing speed。接触速度是运动学量，不是碰撞动力学。

Collision uses truth oriented 2D boxes with height overlap and is checked against engine flags each step. Distance is the minimum box-surface distance; gap is projected along ego heading; TTC is gap/closing speed for an approaching in-path target. Contact speed is kinematic, not crash dynamics.
