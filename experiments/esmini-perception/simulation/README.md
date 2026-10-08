# Python 教学与原始回放 / Python examples and original replay

作者 / Author: Zhuo Ma

当前研究主流程在 [acc_cpp](acc_cpp/README.md)。本目录 Python 示例保留简化 TTC AEB 与 IID 漏检，帮助理解接口，并保留历史对照。它们不调用原生 AEB、没有 Markov 模型，也没有接入 HiDrive。

The current research workflow is in [acc_cpp](acc_cpp/README.md). Python examples here retain a simplified TTC AEB and IID loss for learning and historical comparisons. They do not call native AEB, implement Markov loss, or integrate HiDrive.

## 入口与依赖 / Entries and dependencies

| 文件 / File | 作用 / Purpose |
|---|---|
| `generate_scenarios.py` | 原始输入 → Python 外部自车场景和来源 manifest / Originals → Python external-ego scenarios and provenance |
| `my_sensor_demo.py` | 原时间轨迹上的理想对象 sensor / Ideal object sensor on original timed trajectories |
| `my_aeb_demo.py`, `my_aeb_dropout_demo.py` | 理想或 IID 漏检的 AEB off/on 入口 / Ideal or IID loss, AEB off/on entries |
| `aeb_runner.py` | 上述 AEB 共用闭环和记录 / Shared teaching AEB loop and logging |
| `aeb_controller.py` | 简化 TTC、制动锁存、路径与积分 / Simplified TTC, brake latch, path and integration |
| `sensor_model.py` | 可复现逐对象/逐步 IID 删除 / Reproducible per-object/per-step IID deletion |
| `esmini_api.py` | ctypes State、SE API、sensor、相机；原生 cone 回放也复用 State / State, SE API, sensor, camera; native cone replay reuses State |
| `paths.py` | 路径和环境覆盖 / Paths and environment overrides |
| `run_comparison.py`, `compare_aeb_dropout.py` | 教学四组批跑与已有结果汇总 / Four teaching conditions and existing-run summaries |
| `run_scenarios.py`, `my_run_one.py` | 原始时间轨迹核对及回放 / Original timed-trajectory checks and replay |
| `render_replays.py` | 可选原始轨迹图像/GIF，需要 Pillow / Optional original-trajectory images/GIF, requires Pillow |

不要把此处 `run_comparison.py` 与 `acc_cpp/tools/run_comparison.py` 混用；前者 Python 简化 AEB 四组，后者 C++ 原生控制器两组。

Do not confuse this directory's `run_comparison.py` with `acc_cpp/tools/run_comparison.py`: the first compares four simplified Python AEB conditions; the second compares two native C++ controller conditions.

## 安装与路径 / Setup and paths

Python ≥3.9，标准库。另准备完整图形版 esmini v3.8.1，含 `bin/` 与 `resources/`，库架构须与 Python 匹配。主原生构建的无窗口库可支持无窗口 SE API，但不能提供教学 GUI。

Use Python ≥3.9 and the standard library. Prepare a complete graphical esmini v3.8.1 installation with `bin/` and `resources/`, matching Python's CPU architecture. The native build's headless library supports headless SE APIs but cannot provide the teaching GUI.

以下从 Git 仓库根目录开始，进入 `esmini-perception`。默认安装位置 `simulation/esmini-demo/`；本机仓库根目录的 `esmini/` 有完整库与资源时，可用以下覆盖。

Start at the Git repository root and enter `esmini-perception`. The default installation path is `simulation/esmini-demo/`; use the override below when repository-root `esmini/` has the full library and resources.

```bash
cd experiments/esmini-perception
export ESMINI_HOME="$(cd ../../esmini && pwd)"
# 或 / Or: export ESMINI_HOME="/path/to/graphical/esmini"
python3 simulation/generate_scenarios.py --scenario all
```

生成副本不进入 Git；重建已有副本加 `--overwrite`，只覆盖 `simulation/generated/`。原始 `Data/` 不改写，自车时间轨迹移除但空间路径仍从原输入读取。教学生成副本复用原道路相对路径，不能脱离 `Data/` 单独使用。

Generated copies are ignored by Git. Add `--overwrite` to regenerate existing copies; only `simulation/generated/` is overwritten. `Data/` stays unchanged; ego's timed trajectory is removed but its spatial path is read from the original. Teaching copies reference the original road and require `Data/`.

| 环境变量 / Variable | 默认 / Default |
|---|---|
| `ESMINI_HOME` | `simulation/esmini-demo/` |
| `TME180_DATA` | `Data/` |
| `TME180_GENERATED` | `simulation/generated/` |
| `TME180_RESULTS` | `~/TME180-local/results/data-scenarios/` |

## 运行 / Run

```bash
python3 simulation/my_sensor_demo.py --scenario 1549178
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb on --headless
python3 simulation/my_aeb_dropout_demo.py --scenario 1549178 --aeb on \
  --drop-prob 0.1 --perception-seed 42 --headless
python3 simulation/run_comparison.py --scenario 1549178 --seeds 42

# 原始时间轨迹，无 HiDrive 控制 / Original timed trajectories, without HiDrive control
python3 simulation/run_scenarios.py
python3 simulation/run_scenarios.py --view all
python3 simulation/my_run_one.py --scenario 1549178 --view

# 无原生引擎依赖的教学模型检查 / Model tests without the native engine
python3 -m unittest discover -s tests -v
```

教学单次 AEB 默认 off，需明确 `--aeb on`；ideal 固定漏检 p=0，dropout 默认 p=0.1。默认 dt=0.01 s、TTC=2 s、减速度=6 m/s²，和原生 AEB 的 1.5 s / 8.3385 m/s² 不同。普通单次默认窗口，加 `--headless` 禁用；批比较默认无窗口。

Single teaching AEB runs default to off; explicitly use `--aeb on`. Ideal fixes p=0; dropout defaults to p=0.1. Defaults are dt=0.01 s, TTC=2 s and braking=6 m/s², unlike native AEB's 1.5 s / 8.3385 m/s². Single entries normally open a viewer; `--headless` disables it. Batch comparison is headless by default.

批比较为 AEB off/on × ideal/dropout，每 seed 四次；相同 tick/目标的随机映射保持配对，但 ego 闭环改变后可见样本数可能不同。只汇总已有结果时给运行目录，不给 CSV 路径。

Batches compare AEB off/on × ideal/dropout, four runs per seed. Random mapping stays paired for the same tick/object, but changed ego motion may alter visible sample counts. To summarize existing runs, pass run directories rather than CSV files.

```bash
python3 simulation/compare_aeb_dropout.py /path/to/run_off /path/to/run_on \
  --output /path/to/comparison.csv
```

输出和历史验证见 [字段](../docs/output-schema.md)、[验证](../docs/validation.md)。原始回放禁用未配置控制器；教学 AEB 使用 `ExternalController`，不会运行 HiDrive。旧 `.command` 快捷方式已移除，统一使用上述跨平台 Python 入口；Windows 使用 `python`，多行命令写成一行。

See [fields](../docs/output-schema.md) and [validation](../docs/validation.md). Original replay disables unconfigured controllers; teaching AEB uses `ExternalController`, without HiDrive. Old `.command` shortcuts were removed in favor of these Python entries. On Windows, use `python` and enter multiline commands on one line.
