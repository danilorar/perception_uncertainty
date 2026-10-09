# 输出字段 / Output schema

作者 / Author: Zhuo Ma

本页说明 `simulation/acc_cpp` 原生结果。

This page describes native `simulation/acc_cpp` outputs.

## 原生单次运行 / Native single run

| 文件 / File | 内容 / Content |
|---|---|
| `results.csv` | 主结果：每个仿真步一行 / Main results: one row per simulation step |
| `run_config.json` | 参数、原输入/源码指纹、esmini commit、模型约定 / Settings, input/source hashes, commit, assumptions |
| `generated/C_acc_<id>.xosc` | 外部自车状态接口和原目标轨迹；历史文件名 / External ego interface and original target trajectory; historical filename |
| `generated/*.xodr` | 原道路字节一致快照 / Byte-identical road snapshot |
| `truth.csv` | 逐时刻逐对象真值 / Per-time, per-object truth |
| `perceptions.csv` | 逐感知帧原始/处理后目标审计 / Per-sensor-frame raw/processed target audit |
| `dropout_states.csv` | Markov 状态和持续帧数；IID 仅表头 / Markov state and duration; header only for IID |
| `control.csv` | 控制使用的缓存序号、请求与执行 / Consumed sequence, request and applied command |
| `metrics.csv` | 同步真值风险与双碰撞检查 / Synchronized truth risk and dual collision checks |
| `summary.json` | 安全和感知汇总，`complete=true` 正常完成 / Safety/perception summary; `complete=true` means successful completion |
| `sim.dat`, `run.log` | 原生记录与引擎日志 / Native recording and engine log |
| `sensor_replay/` | 可选 cone 回放输出 / Optional cone replay outputs |

### 主结果 / Main results (`results.csv`)

每个循环步一行，时间 `t_k = k·dt`：该时刻真值、当时可用的缓存观测，以及在 `t_k` 决定并作用于 `[t_k, t_k+dt)` 的指令。空单元表示该量不可用或非有限值，不表示零。

One row per loop step at `t_k = k·dt`: truth at that time, the cached observation available then, and the command decided at `t_k` that acts over `[t_k, t_k+dt)`. An empty cell means the value is unavailable or non-finite, never zero.

| 字段 / Fields | 含义 / Meaning |
|---|---|
| `step,time_s` | 步序号与真值时间 / Step index and truth time |
| `ego_x_m,ego_y_m,ego_heading_rad,ego_speed_mps` | 准确自车真值 / Exact ego truth |
| `ego_progress_m` | 沿参考路径的积分距离 / Integrated distance along the reference path |
| `sensor_refreshed` | 本步是否新采样感知帧，0/1 / Whether a new sensor frame was taken at this step |
| `perception_sequence,perception_age_s` | 所用缓存帧序号及其年龄；首帧前为空 / Cached frame in use and its age; empty before the first frame |
| `ideal_detections,observed_detections,dropped_detections` | 该缓存帧的理想/保留/删除目标数；帧间重复，求和时只取 `sensor_refreshed=1` 行 / Counts for that cached frame; repeated between refreshes, so sum only rows with `sensor_refreshed=1` |
| `selected_target_id,selected_target_gap_m,selected_target_ttc_s` | 控制器根据观测选出的目标及感知间隙/TTC（TTC 仅 AEB）；无选中目标时为空。漏检某目标时控制器仍可能选中另一可见目标 / Target the controller selected from observations, with perceived gap/TTC (TTC for AEB only); empty when none is selected. A missed object does not necessarily empty these: another visible object may be selected |
| `controller_updated` | 本步是否计算新指令，0/1 / Whether a new command was computed at this step |
| `desired_speed_mps,requested_acceleration_mps2,applied_acceleration_mps2,acceleration_limited` | 控制请求、限幅后执行值与是否限幅 / Request, bounded applied value, and whether it was limited |
| `aeb_active` / `close_gap_stop` | AEB 激活（仅 AEB）/ ACC <1 m 停车请求（仅 ACC）；另一控制器为空 / AEB active (AEB only) / ACC <1 m stop request (ACC only); empty for the other controller |
| `truth_min_gap_m,truth_min_gap_target_id` | 所有路径内目标中的最小真值包围盒间隙及其目标；无路径内目标时为空 / Smallest true box gap over in-path targets and that target; empty without an in-path target |
| `truth_min_ttc_s` | 路径内接近目标的最小真值 TTC；空表示没有接近中的目标，不是 TTC=0。可能与最小间隙属于不同目标 / Smallest true TTC over closing in-path targets; empty means no closing target, not TTC=0. May belong to a different target than the minimum gap |
| `truth_min_distance_m,collision` | 任意目标的最小包围盒距离与真值碰撞，0/1 / Smallest box distance to any target and truth collision |

`selected_target_*` 是控制器基于漏检后观测的判断；`truth_min_*` 是对所有目标的真值评价，两者可能指向不同对象。

`selected_target_*` reflects the controller's view from post-dropout observations; `truth_min_*` is truth over all targets. They may refer to different objects.

运行结束时最后一行只记录真值：碰撞停止（仅 `stop_on_collision=true`）、到达时长或场景结束时，不再采样感知、不计算新指令，`sensor_refreshed=0`、`controller_updated=0`，指令和选中目标列为空。上一条指令在前一行，已作用于最后一个步长。

The final row records truth only. When the run stops at first collision (only with `stop_on_collision=true`), at the duration, or at scenario end, no new sensor frame or command is computed: `sensor_refreshed=0`, `controller_updated=0`, and the command and selected-target columns are empty. The previous command is in the preceding row and already acted over the last step.

### 真值 / Truth

| 字段 / Fields | 单位和含义 / Units and meaning |
|---|---|
| `time_s`, `object_id`, `role` | s、引擎 ID、`ego`/`target` / Seconds, engine ID, role |
| `x_m,y_m,z_m,h_rad,speed_mps` | 世界参考点位置、航向、速度 / World reference-point position, heading, speed |
| `length_m,width_m,height_m` | 包围盒尺寸，m / Box dimensions in m |
| `center_x_m,center_y_m,center_z_m` | 包围盒中心相对对象参考点的局部偏移，m / Local box-center offsets from object reference point, m |
| `object_type,object_category` | esmini 枚举；类别须结合 type / esmini enums; interpret category with type |
| `vx_mps,vy_mps,vz_mps` | 世界速度向量，m/s / World velocity vector |
| `ax_mps2,ay_mps2,az_mps2` | 世界加速度向量，m/s² / World acceleration vector |

### 感知和状态 / Perception and state

| 字段 / Fields | 含义 / Meaning |
|---|---|
| `sequence`, `measurement_time_s`, `delivery_time_s`, `track_id` | 测量序号、采样/交付时间、目标 ID / Sequence, sampling/delivery time, target ID |
| `raw_detected`, `observed`, `dropped` | 理想可见、实际输出、随机删除，0/1 / Ideal visibility, delivered observation, deletion, 0/1 |
| `raw_*` | 误差前属性，仅用于审计 / Pre-error properties for audit only |
| `observed_*` | 处理后属性；无输出时为空 / Processed properties; empty if absent |
| `missing`, `changed`, `run_length_frames` | Markov 是否 M、是否转移、当前状态连续帧数 / Missing state, transition, current state duration |

`raw_*` 不属于控制器输入。当前漏检删除完整 `Observation`，包括向量和 bounding box；CSV 空值不代表目标位置或距离为零。不在 FOV 内的目标也推进 Markov，但 `raw_detected=0` 不算随机删除一次理想检测。

`raw_*` is not controller input. Loss removes the complete `Observation`, including vectors and box; empty CSV cells do not mean zero position or distance. Markov advances outside FOV, but `raw_detected=0` is not a random deletion of an ideal detection.

### 控制和评价 / Control and evaluation

| 字段 / Fields | 含义 / Meaning |
|---|---|
| `perception_sequence`, `measurement_time_s`, `age_s`, `fresh` | 实际使用的帧、采样时间、年龄、是否新帧 / Consumed frame, sample time, age, freshness |
| `lead_id`, `observed_gap_m`, `observed_ttc_s` | 控制器选择目标及观测风险 / Selected target and observed risk |
| `requested_acceleration_mps2`, `desired_speed_mps` | 控制请求 / Controller request |
| `applied_acceleration_mps2`, `limited` | 车辆实际执行和限幅标志 / Applied acceleration and limit flag |
| `aeb_active`, `close_gap_stop` | AEB 原生锁存、ACC 近距离停车请求 / Native AEB latch, ACC close-gap stop request |
| `collision`, `engine_collision` | 独立真值几何和引擎碰撞 / Independent truth geometry and engine collision |
| `minimum_distance_m`, `minimum_gap_m`, `ttc_s` | 真值表面距离、纵向 gap、近似 TTC / Truth surface distance, longitudinal gap, approximate TTC |

控制日志在做出请求的 `t_k` 记录；真值/metrics 也记录初始化及推进后的状态，因此不要假定所有 CSV 的行数和最后时间相同，按时间与感知序号关联。安全指标由真值计算，不使用观测间隙。

Control is logged at request time `t_k`. Truth/metrics include initialization and advanced states, so do not assume equal row counts or final times across files; join by time and perception sequence. Safety metrics use truth, not observed gaps.

### 汇总 / Summary

`first_aeb_s` 是原生 AEB 首次 active；`first_deceleration_s` 是首次实际负加速度，两者应分开。`collision_relative_speed_mps` 是首次接触相对速度；`minimum_*` 只统计至结束。无有限数值在 JSON 中为 null、CSV 中为空。

`first_aeb_s` is the first native active AEB state; `first_deceleration_s` is the first applied negative acceleration. Keep them distinct. `collision_relative_speed_mps` is relative speed at first contact; `minimum_*` covers the recorded horizon. Undefined finite quantities appear as JSON null or empty CSV cells.

`realized_dropout_rate = dropped_detections / raw_detections`。`longest_missing_s = longest_missing_frames × sensor_period`，只统计理想检测机会中的连续随机丢失；`model_missing_fraction` / `longest_model_missing_frames` 则反映完整 Markov 状态，包括 FOV 外。碰撞提前结束会截断统计。

`realized_dropout_rate = dropped_detections / raw_detections`. `longest_missing_s = longest_missing_frames × sensor_period` measures consecutive random loss among ideal detection opportunities. `model_missing_fraction` / `longest_model_missing_frames` describe the complete Markov state, including outside FOV. Early collision truncates these statistics.

## 批次与可提交汇总 / Batches and shareable summaries

| 文件 / File | 内容 / Content |
|---|---|
| `comparison.csv`, `batch_manifest.json` | 配对组指标、参数、哈希与检查 / Paired outcomes, settings, hashes, checks |
| 导出 / Export `run_configs.json` | 配对组完整单次参数及输入指纹 / Full per-run paired settings and input hashes |
| `sensitivity_runs.csv` | 每个场景/模型/seed 的汇总及运行引用 / Summary and run reference per scenario/model/seed |
| `sensitivity_summary.csv` | 碰撞计数、Wilson 95% 区间、触发延迟和速度等 / Collision counts, Wilson 95% intervals, trigger delay, speeds |
| `REPORT.md`, `experiment_manifest.json` | 自动双语汇总、完整参数和验证指纹 / Automatic bilingual report, full settings and validation hashes |
| `reports/<name>/export_manifest.json` | 原批次名称、原文件/导出 SHA256、路径转换和行数 / Original batch name, source/export hashes, path normalization, row counts |

ideal 是确定性参考，不是随机样本。触发延迟均值只含已触发运行，接触速度均值只含碰撞运行；计数用于解释选择范围。精选导出不含逐帧日志，重新回放需保留本机原运行或重跑。

Ideal is deterministic, not a random sample. Mean trigger delay includes triggered runs only; mean contact speed includes colliding runs only. Counts explain these subsets. Compact exports omit frame logs; replay needs local raw runs or a rerun.
