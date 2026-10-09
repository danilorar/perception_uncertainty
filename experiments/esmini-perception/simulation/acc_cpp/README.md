# 原生 AEB/ACC 运行指南 / Native AEB/ACC run guide

作者 / Author: Zhuo Ma

主入口为 `tools/build_native.py` → `build/runner/acc_sim` → `tools/run_comparison.py`。默认控制器 AEB，默认误差 Markov。完整原理见 [架构](../../docs/architecture.md)，研究结果见 [报告](reports/README.md)。以下命令均从本目录执行。

The main path is `tools/build_native.py` → `build/runner/acc_sim` → `tools/run_comparison.py`. Defaults are AEB and Markov loss. See [architecture](../../docs/architecture.md) for principles and [reports](reports/README.md) for results. Run all commands below from this directory.

## 1. 构建 / Build

需要 Git、C++17 编译器、Python ≥3.9、CMake ≥3.21。首次下载需要网络。构建脚本固定 esmini v3.8.1 commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8`，获取 fmt 子模块，检查 ACC/ALKS_R157SM 四个源码文件未改动，再加入项目桥接。OSG/viewer、OSI、SUMO、PROJ、上游 GTest 等可选功能关闭。

Requirements: Git, a C++17 compiler, Python ≥3.9 and CMake ≥3.21. The initial fetch needs network access. The script pins esmini v3.8.1 commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8`, fetches fmt, verifies the four ACC/ALKS_R157SM source files are unchanged, and adds the project bridges. Optional OSG/viewer, OSI, SUMO, PROJ and upstream GTest features are disabled.

```bash
python3 tools/build_native.py
# CMake 未安装时先执行 / If CMake is missing, first run:
# python3 -m pip install cmake==3.31.6
```

| 本机产物 / Local product | 内容 / Content |
|---|---|
| `build/_deps/esmini-src/` | 独立稀疏源码 checkout / Independent sparse source checkout |
| `build/engine/` | 引擎编译产物 / Engine build products |
| `build/engine-home/` | 带 ABI 2 桥接的无窗口库、头文件与指纹 / Headless library with ABI 2, header and hashes |
| `build/runner/acc_sim` | macOS/Linux 程序 / macOS/Linux executable |
| `build/runner/Release/acc_sim.exe` | 默认 Windows 程序 / Default Windows executable |
| `build/*-build.log` | 编译日志 / Build logs |

仓库根目录 `esmini/` 与 `build/_deps/esmini-src/` 独立。不要把老师源码长期放在可再生成的 `_deps` 中。官方 demo 库没有本项目桥接 ABI，不能直接替换实验库。

Repository-root `esmini/` and `build/_deps/esmini-src/` are independent. Do not store teacher source permanently in regenerable `_deps`. An official demo library lacks the project bridge ABI and cannot directly replace the experiment library.

## 2. 参数与单次运行 / Parameters and a single run

| 参数 / Option | 默认值 / Default | 含义 / Meaning |
|---|---|---|
| `--controller` | `aeb` | `aeb` 或 `acc` / `aeb` or `acc` |
| `--dt`, `--sensor-period` | 0.01, 0.05 s | 物理/控制步与感知周期 / Physics/control step and sensor period |
| `--duration` | 10 s | 最长时域，默认首次接触结束 / Horizon, normally stop at first contact |
| `--dropout-model`, `--dropout-p` | `markov`, 单次 0 / 批比较 0.1 / single 0, batch 0.1 | 模型与稳态缺失率 / Model and stationary miss probability |
| `--mean-missing` | 0.5 s | Markov 平均缺失时长 / Markov mean missing duration |
| `--markov-init` | `stationary` | 稳态或 `normal` / Stationary or `normal` |
| `--seed`（单次） / (single run) | 42 | 随机种子 / Random seed |
| `--aeb-ttc` | 1.5 s | 严格 TTC < 阈值，另需前方重叠 / Strict TTC < threshold, plus front overlap |
| `--aeb-deceleration` | 8.3385 m/s² | 0.85g，原生建立时间 0.6 s / 0.85g, native 0.6 s ramp |
| `--time-gap` | 1.5 s | 仅 ACC 时距 / ACC time gap only |
| `--set-speed` | 场景初始自车速度 / Initial ego speed | ACC 巡航目标 m/s；AEB 拒绝该选项 / ACC cruise speed m/s; rejected by AEB |
| `--detailed-logs` | `false` | 另写 truth/perceptions/control/metrics/dropout_states CSV；回放和验证需要 / Also write the detailed CSVs; replay and validation need them |

```bash
build/runner/acc_sim --config configs/ideal.ini --output-dir results/my_ideal_run
build/runner/acc_sim --config configs/dropout.ini --output-dir results/my_dropout_run

# 显式配置 / Explicit configuration
build/runner/acc_sim --controller aeb --scenario ../../Data/C_original_1549178.xosc \
  --output-dir results/custom_run --dt 0.01 --sensor-period 0.05 --duration 10 \
  --seed 10 --dropout-model markov --dropout-p 0.1 --mean-missing 0.5 \
  --markov-init stationary --aeb-ttc 1.5 --aeb-deceleration 8.3385
```

每次输出目录必须新建且为空。INI 中场景路径相对 INI 所在目录，CLI 场景路径相对当前目录。两个 INI 仅漏检概率不同，为 0 / 0.1。

Output directories must be new and empty. INI scenario paths are relative to the INI directory; CLI scenario paths are relative to the current directory. The two INIs differ only in loss probability, 0 / 0.1.

ACC 调用原 `ControllerACC::Step()`，把速度请求换成加速度，再由共同车辆模型执行 ±10 m/s² 限幅和积分。小于 1 m 间隙的零速度请求也经过该模型，不瞬间停车。AEB 只调用 ReferenceDriver 的 AEB 组件，保留跨步状态，排除驾驶员反应和巡航。

ACC calls original `ControllerACC::Step()`, converts its speed request to acceleration, then applies the common vehicle model with ±10 m/s² limits and integration. Even a zero-speed request for gaps below 1 m passes through this model instead of stopping instantly. AEB calls only the ReferenceDriver AEB component, retains state, and excludes driver reaction and cruise.

## 3. 配对对比 / Paired comparison

```bash
python3 tools/run_comparison.py --binary build/runner/acc_sim \
  --scenario ../../Data/C_original_1549178.xosc --controller aeb \
  --dropout-model markov --dropout-p 0.1 --mean-missing 0.5 --seeds 10 --validate

# IID 基准 / IID baseline
python3 tools/run_comparison.py --binary build/runner/acc_sim --dropout-model iid --seeds 42 --validate

# 切换原生 ACC / Switch to native ACC
python3 tools/run_comparison.py --binary build/runner/acc_sim --controller acc --time-gap 1.5 --seeds 42 --validate
```

每个 seed 跑 ideal/dropout 两组；`--validate` 另跑 p=0、相同 seed 重复、p=1，检查等价、复现、目标真值不变、完全删除观测。每步核对碰撞几何与引擎标志。普通批运行通过检查后可省略验证选项。

Each seed runs ideal/dropout cases. `--validate` adds p=0, same-seed repeat and p=1 checks for identity, reproducibility, unchanged target truth and complete observation removal. Collision geometry is checked against engine flags each step. Routine batches can omit validation after it passes.

通过 `--seeds 10 42 43` 增加种子，`--scenario` 切换场景，`--results-dir results/my_batch_group` 分组保存。每批创建 UTC 目录，其 results-dir 下 `latest.txt` 指向最近比较。`--time-gap` 不等于 AEB TTC 阈值。

Use `--seeds 10 42 43` for more seeds, `--scenario` for another case, and `--results-dir results/my_batch_group` to group batches. Each batch gets a UTC directory; its results directory's `latest.txt` points to the latest comparison. `--time-gap` is not the AEB TTC threshold.

Windows 使用 `python` 和 `build/runner/Release/acc_sim.exe`。PowerShell 将上述多行命令写成一行，或用反引号续行。

On Windows, use `python` and `build/runner/Release/acc_sim.exe`. Enter multiline commands on one line in PowerShell, or use backticks for continuation.

## 4. 三场景敏感性 / Three-scenario sensitivity

```bash
python3 tools/run_markov_sensitivity.py --binary build/runner/acc_sim \
  --controller aeb --dropout-p 0.1 --mean-missing 0.2 0.5 1.0 \
  --seed-count 100 --seed-start 0 --dt 0.01 --sensor-period 0.05 \
  --duration 10 --markov-init stationary --validate
```

默认各场景一个确定性 ideal、100 次 IID、三组各 100 次 Markov，共 1203 次主运行、36 次检查。若 ideal 提前碰撞，另跑完整时域目标真值参考，排除在安全统计外。`--scenarios` 可选一个或多个 XOSC。改为 `--controller acc` 可跑 ACC。

Each scenario has one deterministic ideal, 100 IID and three groups of 100 Markov runs: 1203 main runs and 36 checks. If ideal collides early, a full-horizon target-truth reference is excluded from safety statistics. `--scenarios` accepts one or more XOSCs. Use `--controller acc` for ACC.

`results/latest_aeb_markov_sensitivity.txt` 或 `latest_acc_markov_sensitivity.txt` 指向新批次。旧 `latest_markov_sensitivity.txt` 是历史 ACC 指针。输出见 [字段](../../docs/output-schema.md)，统计解释见 [报告](reports/README.md)。

`results/latest_aeb_markov_sensitivity.txt` or `latest_acc_markov_sensitivity.txt` points to the new batch. The old `latest_markov_sensitivity.txt` is a historical ACC pointer. See [fields](../../docs/output-schema.md) and [reports](reports/README.md).

## 5. 感知 cone 回放 / Replay with the perception cone

普通 `replayer --file sim.dat` 不恢复漏检。此工具用图形版 esminiLib 重放 `truth.csv`，按 `control.csv` 实际使用的序号查找 `perceptions.csv`。需要详细日志：批处理工具总是写出；单次运行需加 `--detailed-logs true`。处理后目标列表非空显示 cone，单目标漏检隐藏 cone；目标真实运动始终可见。

Ordinary `replayer --file sim.dat` does not restore loss. This tool replays `truth.csv` with graphical esminiLib and looks up perception by the sequence consumed in `control.csv`. It needs detailed logs: batch tools always write them; add `--detailed-logs true` to a single run. A nonempty processed list shows the cone; single-target loss hides it. True target motion stays visible.

```bash
task_batch="$(cat results/latest.txt)"
python3 tools/replay_perception.py --run-dir "$task_batch/ideal_seed10"
python3 tools/replay_perception.py --run-dir "$task_batch/dropout_seed10"

# 自选图形版安装 / Select a graphical installation
python3 tools/replay_perception.py --run-dir "$task_batch/dropout_seed10" \
  --esmini-home /path/to/graphical/esmini --camera rear --speed 1

# 无窗口核对与截图 / Headless check and snapshots
python3 tools/replay_perception.py --run-dir "$task_batch/dropout_seed10" --check-only
python3 tools/replay_perception.py --run-dir "$task_batch/dropout_seed10" \
  --no-realtime --snapshots 3.25 3.55
```

默认用仓库根目录 `esmini/` 的平台库、俯视相机、半速、末帧保留 3 s。图形库须 v3.8.1，含 `bin/` 和 `resources/models/`，与 Python 架构匹配。不要指向无 viewer 的 `build/engine-home`。仅依赖标准库，不需 Pillow。

Defaults use the platform library in repository-root `esmini/`, a top camera, half speed and a 3 s final hold. Graphical v3.8.1 needs `bin/` and `resources/models/` and must match Python's architecture. Do not use viewer-disabled `build/engine-home`. Only the standard library is needed, without Pillow.

回放不执行控制器或重抽误差；每帧核对姿态、速度、时间和输入 SHA256。输出 `sensor_replay/` 包含显隐时间线、核对结果、可视场景及截图。cone 是观测可用性提示，不表示硬件停机。当前支持本实验两车平坦道路。

Replay does not run controllers or resample errors. It checks pose, speed, time and input SHA256. `sensor_replay/` contains visibility timelines, check results, the visual scenario and snapshots. Cone visibility indicates observation availability, not hardware failure. Support covers this experiment's two-car flat-road runs.

## 6. 导出与检查 / Export and checks

```bash
task_sensitivity="$(cat results/latest_aeb_markov_sensitivity.txt)"
python3 tools/export_report.py --batch-dir "$task_sensitivity" --name my_aeb_sensitivity

# 构建脚本已执行 CTest；修改 C++ 后先重新构建
# The build script runs CTest; rebuild first after changing C++
ctest --test-dir build/runner -C Release --output-on-failure
```

导出含白名单内汇总/逐 seed CSV、manifest、双语 README。绝对本机路径改成相对引用，数值不变。新目录不覆盖旧报告，原 results 文件不修改。Git 操作见 [GitHub 指南](../../docs/github-workflow.md)。

Exports contain whitelisted summary/per-seed CSVs, manifests and a bilingual README. Local absolute paths become relative references; numbers stay unchanged. New exports do not overwrite old reports or modify original results. See the [GitHub guide](../../docs/github-workflow.md).

## 上游来源 / Upstream sources

- esmini v3.8.1：[ACC 源码 / ACC source](https://github.com/esmini/esmini/blob/v3.8.1/EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp)、[ALKS AEB 源码 / ALKS AEB source](https://github.com/esmini/esmini/blob/v3.8.1/EnvironmentSimulator/Modules/Controllers/ControllerALKS_R157SM.cpp)，MPL 2.0。
- [控制器文档 / Controller docs](https://esmini.github.io/controllers.html)、[构建文档 / Build guide](https://esmini.github.io/build-guide.html)。
- pugixml 1.14：源码及 MIT 许可 / Source and MIT license in `third_party/pugixml/`。

原始数据和第三方署名/许可不改写；项目新增代码标注 Zhuo Ma。HiDrive 接入见 [独立指南](../../docs/hidrive-integration.md)。

Original data and third-party attribution/license text are preserved. Added project code credits Zhuo Ma. See the [separate guide](../../docs/hidrive-integration.md) for HiDrive integration.
