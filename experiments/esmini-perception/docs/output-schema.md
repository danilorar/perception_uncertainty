# 输出字段 / Output schema

作者 / Author: Zhuo Ma

本页先说明 `simulation/acc_cpp` 原生结果，再说明 Python 教学结果。两者参数、控制器和 CSV 格式不同，分析时不要混用。

Native `simulation/acc_cpp` outputs are described first, followed by Python teaching outputs. Parameters, controllers and CSV formats differ; do not mix them in analysis.

## 原生单次运行 / Native single run

| 文件 / File | 内容 / Content |
|---|---|
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

## Python 教学结果 / Python teaching outputs

教学入口输出 `aeb_telemetry.csv`、`run_config.json`、`sim.dat`、`run.log`。纯 ideal sensor 输出 `sensor_ids.csv`，原始可执行回放输出 `states.csv`。

Teaching entries produce `aeb_telemetry.csv`, `run_config.json`, `sim.dat` and `run.log`. Ideal sensor replay writes `sensor_ids.csv`; executable trajectory replay writes `states.csv`.

| 字段 / Fields | 含义 / Meaning |
|---|---|
| `raw_detected_ids,observed_ids,dropped_ids` | 分号分隔的理想/保留/删除 ID / Semicolon-separated ideal/retained/deleted IDs |
| `raw_target_detected,target_detected,target_dropped` | 单目标检测审计，0/1 / Single-target detection audit |
| `ego_*`, `target_*`, `gap_m`, `closing_speed_mps`, `ttc_s` | 真值状态及纵向风险 / Truth state and longitudinal risk |
| `observed_target_*`, `observed_gap_m`, `observed_ttc_s` | 目标检测存在时的观测，否则空 / Observed target properties/risk, otherwise empty |
| `acceleration_command_mps2,aeb_active,collision` | 简化 AEB 请求、锁存、引擎碰撞 / Simplified AEB command, latch, engine collision |

教学控制器在目标缺失时将风险输入屏蔽为 gap=∞、closing=0、in_path=false；已触发制动仍保持。其 `comparison.csv` 漏检率为 dropped_target_samples/raw_target_samples，最长缺失不含原本不在 sensor 内的时段。

When a target is missing, teaching controller risk input is gated to gap=∞, closing=0 and in_path=false; active braking stays latched. Its comparison miss rate is dropped_target_samples/raw_target_samples, and longest loss excludes periods outside the ideal sensor.
