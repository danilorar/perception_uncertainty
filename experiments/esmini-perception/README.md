# esmini 感知误差与原生 AEB/ACC / Perception errors with native AEB/ACC

作者 / Author: **Zhuo Ma**

使用老师提供的三个追尾场景，对比理想感知、IID 随机漏检及两状态 Markov 连续漏检对闭环控制的影响。当前主流程是 C++ 原生 AEB；ACC 可切换，Python 负责构建、批运行和回放。`HiDriveController` 尚未收到、尚未实现。

This experiment uses three supplied rear-end scenarios to compare ideal perception, IID detection loss and two-state Markov bursts in a closed loop. The main workflow uses native C++ AEB, with an ACC option. Python handles builds, batches and replay. The teacher's `HiDriveController` source has not been received or implemented.

固定 esmini **v3.8.1**，commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8`。AEB 调用 `ALKS_R157SM::ReferenceDriver` 的 AEB 组件；不执行完整驾驶员模型。目录名 `acc_cpp` 和可执行程序名 `acc_sim` 保留原命名。

esmini is pinned to **v3.8.1**, commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8`. AEB calls the AEB component of `ALKS_R157SM::ReferenceDriver`, excluding the full driver model. The historical names `acc_cpp` and `acc_sim` are retained.

## 目录 / Layout

```text
esmini-perception/
  Data/                       # 老师的原始 XOSC/XODR，只读 / Supplied originals, read only
  docs/                       # 架构、字段、验证与 Git 指南 / Architecture, schema, checks, Git
  simulation/
    acc_cpp/                  # 当前原生 AEB/ACC / Current native AEB/ACC
      src/ include/           # C++ 闭环及桥接 / C++ loop and bridges
      configs/ tests/ tools/  # 配置、测试、Python 工具 / Configs, tests, Python tools
      reports/                # 可提交的精选结果 / Selected results for Git
      third_party/            # 所需 XML 源码与上游许可 / Required XML code and licenses
      build/ results/         # 本机生成，不提交 / Local generated files, ignored
```

## 从零运行 / Start from a clean checkout

需要 Git、Python ≥3.9、CMake ≥3.21 和 C++17 编译器。macOS 使用 Xcode Command Line Tools，Linux 使用 GCC/Clang，Windows 使用 Visual Studio C++ Build Tools。核心 Python 工具仅依赖标准库。当前实际验证平台为 macOS；Windows/Linux 提供构建适配和 CI 模板，尚未在本机验证。

Install Git, Python ≥3.9, CMake ≥3.21 and a C++17 compiler. Use Xcode Command Line Tools on macOS, GCC/Clang on Linux, or Visual Studio C++ Build Tools on Windows. Core Python tools use only the standard library. macOS has been tested locally; Windows/Linux have build support and a CI template, without local validation.

以下命令从 Git 仓库根目录开始；进入 `acc_cpp` 后执行其余命令。

Start at the Git repository root, then run the remaining commands inside `acc_cpp`.

```bash
cd experiments/esmini-perception/simulation/acc_cpp
# 若已安装 CMake，可跳过 / Skip if CMake is installed
python3 -m pip install cmake==3.31.6
# 下载固定源码、构建带桥接的无窗口库与 runner、运行 CTest
# Fetch pinned source, build the headless library/runner, and run CTest
python3 tools/build_native.py

# 理想感知与连续漏检比较，加边界/复现检查
# Ideal versus burst loss, with boundary/reproducibility checks
python3 tools/run_comparison.py --binary build/runner/acc_sim \
  --scenario ../../Data/C_original_1549178.xosc --controller aeb \
  --dropout-model markov --dropout-p 0.1 --mean-missing 0.5 \
  --seeds 10 --validate
```

Windows 将 `python3` 改为 `python`，程序路径改为 `build/runner/Release/acc_sim.exe`。Bash 的续行符 `\` 在 PowerShell 中需改为反引号，或将命令写为一行。

On Windows, use `python` and `build/runner/Release/acc_sim.exe`. Replace Bash's `\` continuation with PowerShell backticks, or enter the command on one line.

输出为 `results/comparison_<scenario>_<UTC>/`，`results/latest.txt` 指向该批次。场景 ID 可替换为 `1554254` 或 `1554431`。单次原生运行必须使用新的空输出目录。

Output goes to `results/comparison_<scenario>_<UTC>/`; `results/latest.txt` points to the batch. Replace the scenario ID with `1554254` or `1554431`. Single native runs require a new, empty output directory.

## 仿真流程 / Simulation flow

```text
Data XOSC/XODR -> generated working copy -> esmini world truth
                                             |
                         ideal object sensor, every 0.05 s
                                             |
                          IID / Markov loss -> cached observations
                                             |
                    exact ego + observations -> native AEB / ACC
                                             |
                acceleration limit + integration, every 0.01 s
                                             |
                          report next ego state to esmini

world truth -> collision/gap/TTC metrics -> logs and summary CSV
saved truth + saved observations -> replay with cone visibility
```

误差只作用于外部目标观测。自车反馈准确，目标继续原时间轨迹；控制器无法读取活动场景中的目标真值。漏检删除整条观测，不填零、不补读真值。物理步之间共用一次感知处理的缓存，安全评价独立使用真值。

Errors affect only external-object observations. Ego feedback is exact and the target follows its original timed trajectory. Controllers cannot read live target truth. A miss removes the entire observation, without zeros or truth fallback. Physics steps reuse one processed sensor frame; safety metrics independently use truth.

## 完整实验和回放 / Full experiment and replay

```bash
# 三场景：ideal + IID + Markov 0.2/0.5/1.0 s，每随机组 100 个 seed
# Three scenarios: ideal + IID + Markov 0.2/0.5/1.0 s, 100 seeds per random group
python3 tools/run_markov_sensitivity.py --binary build/runner/acc_sim \
  --controller aeb --dropout-p 0.1 --mean-missing 0.2 0.5 1.0 \
  --seed-count 100 --seed-start 0 --dt 0.01 --sensor-period 0.05 \
  --duration 10 --markov-init stationary --validate

# 本机图形版 esmini，含 bin/ 和 resources/ / Local graphical esmini installation
export ESMINI_VIEWER_HOME="/path/to/graphical/esmini"
task_batch="$(cat results/latest.txt)"
python3 tools/replay_perception.py --run-dir "$task_batch/dropout_seed10" \
  --esmini-home "$ESMINI_VIEWER_HOME"

# 导出精简结果，原始数据继续留在 results/ / Export compact results; retain raw runs locally
python3 tools/export_report.py --batch-dir "$task_batch" --name my_aeb_comparison
```

回放需要带 viewer 的 v3.8.1 库和模型资源。默认寻找仓库根目录 `esmini/`；原生实验构建的 `build/engine-home` 没有图形功能。回放只读取已存日志：观测存在时显示 cone，单目标漏检时隐藏 cone，不重新运行控制器或抽样。

Replay requires a viewer-enabled v3.8.1 library and model resources. It looks in repository-root `esmini/` by default; the experiment's `build/engine-home` has no graphics. Replay reads saved logs: the cone appears when observations exist and disappears during single-target loss, without rerunning the controller or sampling errors.

## 文档和结果 / Documentation and results

| 入口 / Entry | 内容 / Content |
|---|---|
| [原生运行指南 / Native guide](simulation/acc_cpp/README.md) | 构建、参数、批运行、回放 / Build, parameters, batches, replay |
| [架构与接口 / Architecture](docs/architecture.md) | 文件依赖、时序、Markov 原理 / Dependencies, timing, Markov model |
| [输出字段 / Output schema](docs/output-schema.md) | CSV/JSON 字段及单位 / CSV/JSON fields and units |
| [HiDrive 接入 / HiDrive integration](docs/hidrive-integration.md) | 收到老师源码后的步骤 / Steps after receiving teacher source |
| [验证记录 / Validation](docs/validation.md) | 当前检查与历史实验的区别 / Current checks and historical evidence |
| [GitHub 指南 / GitHub guide](docs/github-workflow.md) | 提交边界、结果导出、Git 命令 / Tracking policy, exports, Git commands |
| [精选报告 / Selected reports](simulation/acc_cpp/reports/README.md) | AEB 敏感性、ACC 历史结果、seed10 演示 / AEB sensitivity, historical ACC, seed10 demo |

早期 Python 简化 AEB 示例已移除，原生流程为唯一实现；如需查看，见 Git 提交 `3792c21` 及更早版本的 `simulation/*.py`。

The earlier simplified Python AEB examples were removed; the native workflow is the only implementation. They remain in Git history as `simulation/*.py` at commit `3792c21` and earlier.

`1554431` 在当前参数下理想感知也碰撞，不能把所有碰撞归因于漏检。Markov 参数是设定误差的敏感性研究，尚未用真实传感器数据校准。

With current parameters, `1554431` collides even under ideal perception; its collisions cannot all be attributed to misses. Markov parameters represent assumed-error sensitivity and have not been calibrated against real sensor data.

项目自写文档采用中英文；第三方源码和许可原文保持完整。原始场景的 CAP unit / Vehicle Safety Division / Chalmers 署名保留，数据提供方的分享范围仍适用于原始及派生场景。

Project-authored documentation is bilingual. Third-party source and license text remain intact. Original CAP unit / Vehicle Safety Division / Chalmers attribution is preserved; the data provider's sharing terms apply to both original and derived scenarios.
