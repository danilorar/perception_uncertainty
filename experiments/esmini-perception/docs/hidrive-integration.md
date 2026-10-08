# HiDrive 接入计划 / HiDrive integration plan

作者 / Author: Zhuo Ma

状态：尚未收到老师 `.cpp/.hpp`，没有 HiDrive 实现、注册或运行验证。当前 `acc_sim` 只支持 `aeb|acc`。本页是收到源码后的步骤，不能作为 HiDrive 已完成的证据。

Status: the teacher's `.cpp/.hpp` has not been received. There is no HiDrive implementation, registration or runtime validation. `acc_sim` currently supports only `aeb|acc`. These are steps for future integration, not evidence of a completed controller.

## 1. 确认原接口 / Establish the original interface

请老师一起提供所需 esmini commit/分支、构建依赖、参数与最小运行示例。确认它是 esmini `Controller` 子类，还是独立 C++ 控制算法；确认输入来源、输出是速度/加速度还是油门/制动/转向，以及内部状态生命周期。

Obtain the required esmini commit/branch, dependencies, parameters and minimal example. Determine whether it subclasses esmini `Controller` or is a standalone C++ algorithm, how it obtains input, whether output is speed/acceleration or throttle/brake/steering, and how it retains state.

老师原 XOSC 声明 `HiDriveController`，参数包含 `timeGap=1.0`、`mode=additive`、`ADF_enabled=true`、`Speed_limit=100`。这些不是已验证接口；尤其不能仅凭名称推断单位或 additive 对轨迹的作用。

Supplied XOSCs declare `HiDriveController` with `timeGap=1.0`, `mode=additive`, `ADF_enabled=true` and `Speed_limit=100`. These are not a verified interface; names alone cannot establish units or additive trajectory semantics.

## 2. 先运行老师的 ideal 示例 / Run the teacher's ideal example first

保留 `Data/` 和现有 AEB/ACC 结果，另建工作副本/分支。将老师源码放入版本管理目录，例如 `src/controllers/hidrive/`，不要放在 `build/_deps/`。如需 esmini 内部类，使用固定源码构建并完成源码加入、工厂注册、实例化和实际 update 调用。

Preserve `Data/` and existing AEB/ACC results; create working copies/a branch. Store teacher source in a versioned directory such as `src/controllers/hidrive/`, outside `build/_deps/`. If it depends on esmini internals, build source and complete source inclusion, factory registration, instantiation and actual update execution.

先确认原示例 ideal 能运行，再确定 HiDrive 对原自车时间轨迹的依赖。现有 `scenario.cpp` 移除自车时间轨迹、采用外部状态 override，不应未经检查直接用于老师的 additive 控制器。

Verify the original ideal example first, then determine its dependence on ego's timed trajectory. Existing `scenario.cpp` removes ego's timed trajectory and uses external state override; do not reuse that behavior blindly for the teacher's additive controller.

## 3. 接入处理后观测 / Connect processed observations

保持目标输入边界为 `ControllerInput{exact ego, processed PerceptionFrame}`。新增 `EsminiHiDrive::update()` 适配器和对应桥接，扩展 C ABI、构建清单、CLI 控制器选择与源码指纹。一个有状态实例贯穿一次 run，结束销毁，下一 run 重新创建。

Keep target input within `ControllerInput{exact ego, processed PerceptionFrame}`. Add an `EsminiHiDrive::update()` adapter and bridge, extending the C ABI, build list, CLI selection and source fingerprints. Retain one stateful instance throughout a run and recreate it for the next run.

若它遍历 `entities_`，只提供准确自车及处理后目标副本。若它通过 `GetSensorHits()` 或类似 hits 接口读取目标，须检查老师实现，建立来自同一处理后缓存的 hits，数量、ID、位置、尺寸和向量一起过滤；p=1 应无目标 hits。不能继续使用指向真实目标的原 hits、活动 `player_`、备用 sensor 或补读 `SE_GetObjectState` 绕过漏检。

If it iterates `entities_`, provide only exact ego and processed target copies. If it reads targets through `GetSensorHits()` or a similar hits interface, inspect the implementation and construct hits from the same processed cache, filtering count, IDs, pose, box and vectors together. At p=1 there must be no target hits. Original truth-backed hits, live `player_`, alternate sensors or fallback `SE_GetObjectState` must not bypass loss.

传感器测量每次只抽样一次；控制、显示、logger 都使用该帧。目标被漏检时允许老师自己的跟踪/记忆继续工作，但新的真值观测不能偷偷补入；记录记忆策略，区别于当前无跟踪器模型。

Sample errors once per sensor update; control, display and logger use that frame. Teacher tracking/memory may continue during misses, but new truth observations must not be injected. Document memory behavior and distinguish it from the current tracker-free model.

## 4. 执行输出 / Execute outputs

明确速度请求如何转加速度，油门/制动如何作用于车辆模型、单位与限幅。若有转向/横向输出，扩展 `ControlRequest`、C ABI 和车辆模型，并停用冲突的固定路径跟随；仅保存转向数值不会改变实际轨迹。

Define speed-to-acceleration conversion, throttle/brake execution, units and limits. If steering/lateral outputs exist, extend `ControlRequest`, the C ABI and vehicle model, disabling conflicting fixed-path following. Logging a steering value alone does not change the trajectory.

## 5. 验证再比较 / Validate before comparison

依次验证三场景 ideal、p=0 等价、同 seed 复现、p=1 无新目标真值泄漏、目标轨迹不变、控制状态不跨 run 污染、输出限幅及真值碰撞一致。之后保留 IID，扩展 Markov 时长和多 seed，对比同一 HiDrive 配置。

Check three-scenario ideal behavior, p=0 equivalence, same-seed reproduction, no fresh target-truth leak at p=1, unchanged target trajectories, per-run state isolation, limits and truth/engine collision agreement. Then retain IID and add Markov durations and multiple seeds with the same HiDrive configuration.

记录老师源码版本/许可、输入指纹、控制和车辆参数、感知周期及跟踪假设。原生 AEB 和 ACC 历史结果保留各自标签，不改名成 HiDrive 结果。

Record teacher-source version/license, input hashes, controller/vehicle settings, sensor period and tracking assumptions. Historical native AEB/ACC results retain their original labels instead of being relabeled as HiDrive.
