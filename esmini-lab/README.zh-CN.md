# esmini 完整环境与实际运行记录

## 2026-09-29：当前使用完整安装

当前运行目录是 `runtime/esmini-full-v3.8.1`。按照[官方 Get complete esmini](https://esmini.github.io/getting-started.html#_get_complete_esmini)，使用同一 [v3.8.1 release](https://github.com/esmini/esmini/releases/tag/v3.8.1) 的 `esmini-fullsrc.zip`、`esmini-bin_Windows.zip`，再补齐官方完整模型包。共包含 13 个二进制/库文件、79 个场景目录文件、20 个道路目录文件、93 个模型目录文件及完整源码、测试和示例。

新增 `esminiRMLib.dll`、`osireceiver.exe`、`esmini-dyn.exe` 等工具；Demo 与完整包共同包含的 7 个二进制/库文件 SHA-256 完全一致。完整安装扩展工具和资源；当前项目的仿真算法保持原定义。

安装入口为 `scripts/setup_full.py`，使用 Python 标准库和 Windows 自带 tar，后台下载及解压。`evidence/full-install/installation.json` 保存来源、哈希和文件清单。源码与 Windows 二进制包匹配 GitHub 发布校验值，模型包记录本地哈希。`scripts/run_cases.py`、Simulation Studio 和数据兼容性探针已指向完整安装目录。

原 `runtime/esmini-demo`、下载包与历史运行记录保留。下文为 2026-09-13 初始 Demo 安装的历史记录；`scripts/setup.ps1` 和 `scripts/verify_sources.py` 继续用于复核该记录。

## 2026-09-13：初始安装记录

验证日期：2026-09-13，Europe/Berlin。工作目录：`E:\Chalmers course\P5\TME180 project\esmini-lab`。

已完成 Windows x64 预编译 Demo 的下载、校验、解压及两种官方场景的执行。自带 cut-in 与 Vector CCRs 50 km/h 均正常到达场景终止条件，生成日志、原始 CSV 和 DAT。另完成两个场景的离屏渲染并人工检查截图。新增系统依赖为 **0**。本次结果只证明运行环境、所选场景及数据记录链路可用。

**任务书与范围。** 已提取并逐页查看上级目录中两页的 `H - Perception Models For Autonomous Driving In Virtual Simulations.pdf`，提取文本保存在 [project-brief.txt](evidence/project-brief.txt)。任务书要求把已开发的感知不确定性模型接入虚拟仿真，比较理想感知和感知不确定性条件下的 ADS 安全表现，并研究距离、速度、视距、开角等属性的影响。初始目录只有任务书，没有正式模型、模型参数或 ADS 控制器。因此，本次只落实环境和官方场景验证，没有实现、接入或编造任何感知模型、噪声模型、融合模型或 AEB/ADS 控制器。

**官方依据与选择。** [esmini Getting started](https://esmini.github.io/getting-started.html) 建议首次使用匹配平台的 Demo，解压即可使用；本次遵循这一方式，没有源码编译。所用 [v3.8.1 release](https://github.com/esmini/esmini/releases/tag/v3.8.1) 发布于 2026-09-08，为查询当时的 latest。命令参数参考 [Command reference](https://esmini.github.io/command-reference.html) 并以本机二进制 `--help` 输出核对。两份网页快照已保存至 `evidence/official-*.html`。

[Vector 官方仓库](https://github.com/vectorgrp/OSC-NCAP-scenarios) 提供基础场景、单次参数组合和参数变化文件，当前文件使用 OpenSCENARIO XML 1.3。使用其原始 2026 CCRs 场景及 50 km/h 单次参数文件，避免自行设计测试用例。仅核实了这一个 NCAP 场景；不能据此推断所有 1.3 功能、转弯场景或全套 NCAP 文件均兼容。

**本机环境与固定版本。** 完整机器可读记录见 [environment.json](evidence/environment.json)。

| 项目 | 实测值 |
|---|---|
| 操作系统 | Windows 11 家庭中文版，10.0.26200，64 位 |
| CPU / 内存 | Intel Core i7-14650HX，16,890,322,944 bytes，约 15.73 GiB |
| 显卡 | Intel UHD Graphics，驱动 32.0.101.7076；NVIDIA RTX 5060 Laptop GPU，驱动 32.0.15.8157 |
| VC++ x64 运行库 | 已安装 v14.50.35719.00，未重新安装 |
| Python / PowerShell / Git | 已有 Python 3.11.9 / PowerShell 7.6.5 / Git 2.52.0.windows.1 |
| CMake | PATH 中未发现；使用预编译包，无需安装 |
| esmini | v3.8.1，git revision `v3.8.1-0-19d26b68`，build 6383 |
| 二进制架构 | PE Machine `0x8664`，AMD64/x64，与本机匹配 |
| NCAP 源版本 | commit `15365d18bd7d1d6aff46c75938eddaf4325ac8f3`，2026-08-17 |

渲染确已产生可读图像；没有测定实际由哪块 GPU 执行，不能将显卡清单当作 GPU 使用证明。PDF 提取、渲染和预览格式转换复用了 Codex 已有 Python/Poppler/Pillow；模拟器及运行、验证脚本不依赖这些文档工具。Python 脚本只使用标准库。未安装 CUDA、PyTorch、OpenCV、PCL、ROS、Docker、编译工具链或 Python 第三方包，也未更改全局 PATH。

**下载与原始文件校验。** [setup-receipt.json](evidence/setup-receipt.json)、[esmini-release.json](evidence/esmini-release.json)、[ncap-commit.json](evidence/ncap-commit.json) 保存了固定下载来源与版本元数据。原始 ZIP 保留在 `downloads/`，官方许可文件随包保留。

| 下载文件 | 字节数 | SHA-256 |
|---|---:|---|
| esmini-demo_Windows-v3.8.1.zip | 130793013 | `2983be14adba66e5cc22f52714c82e1c06c766facf6864b59c911a1860b9a319` |
| OSC-NCAP-scenarios-15365d18.zip | 213541 | `11134e3450b8351a740b8ace16ac635ea99404e3c1756ed72e6e4b0a93295c99` |

Demo 的 SHA-256 与 GitHub release asset API 公布值一致；NCAP ZIP 的 SHA-256 是本次下载后记录的本地校验值，不是发布方签名。NCAP 从官方 API 的固定 commit ZIP 下载，解压时保留 OpenSCENARIO / OpenDRIVE / Catalogs 的相对路径。[source-integrity.json](evidence/source-integrity.json) 确认 Demo 的 248 个文件和 NCAP 的 147 个文件均与各自压缩包逐文件一致，未修改原始场景、目录、道路或软件配置。

**实际运行结果。** 正式数据记录使用固定步长 0.01 s、随机种子 1、`--headless --collision --csv_logger --record`。每次运行以独立目录作为工作目录，绝对路径通过参数数组传递，支持项目路径中的空格。场景运行时没有继承 `ESMINI_CONFIG_FILE`；原包 `config.yml` 只有窗口设置，命令行 `--headless` 覆盖它。

| 场景与用途 | 退出码 | CSV 数据行数 | 仿真结束时间 | 首次几何碰撞 |
|---|---:|---:|---:|---:|
| 官方 cut-in，0.01 s | 0 | 2175 | 21.74 s | 12.44 s，Ego / OverTaker |
| 官方 CCRs 50 km/h，0.01 s | 0 | 572 | 5.71 s | 4.70 s，Ego / Target |
| cut-in 离屏图形检查，0.1 s | 0 | 224 | 22.30 s | 13.00 s |
| CCRs 离屏图形检查，0.1 s | 0 | 59 | 5.80 s | 4.70 s |

图形检查为减少帧文件量采用 0.1 s 步长，保留了 cut-in 225 帧、NCAP 60 帧原始 TGA；其结果不得与 0.01 s 主运行混合作为同一时间序列。cut-in 的结束时间及碰撞时间随步长出现变化，本次没有完成步长收敛研究。

NCAP 实际输入是 `OpenSCENARIO/NCAP/CA-FC_2026/CCRs.xosc` 加 `Variations/SingleExecution/CCRs_50kph.xosc`，`--param_permutation 0` 选择唯一组合。日志显示 `Ego_speed_kph=50`、`ImpactLocation=50`、目标速度为零、`isTargetbraking=false`。`ImpactLocation=50` 是官方字段值，公式给出横向偏移零；这里不将其改称 50% 车辆重叠率。

CSV 证实 Ego 全程 13.888889 m/s，Target 全程 0 m/s。根据原始目录包围盒，初始纵向净间距为 65.232945 m，匀速几何接触时刻约为 4.696772 s，与 0.01 s 采样下的 4.70 s 一致。日志在 4.70 s 设置 `collisionDetected=true`，之后经原始 `StopAfterCollision` 条件及采样延迟在 5.71 s 结束。未接入制动控制，因此碰撞后的对象继续按场景运动；这里没有真实碰撞动力学或碰撞伤害评估。

[validation-summary.json](evidence/validation-summary.json) 保存了全部检查结果：退出码、无超时、正常关闭、Storyboard 完成、CSV 索引连续、时间严格递增且步长正确、数值有限、DAT 非空、输出哈希一致；NCAP 另外核验参数、双方速度、碰撞变量、终止触发和几何接触时间。`passed` 只代表这些明列的检查通过，日志中的 error/warn 另行计数，不表示日志无错误。

**已知异常与未成功的诊断步骤。** 所有四次场景执行均正常结束，且 stderr 为空，但存在以下需保留的事实。

- cut-in 每次含 4 条 `[error] Unsupported object type`：`rail-pole` 和 `guide-post` 各两条，模拟器按 `NONE` 解释。此次未修改原始道路去消除报错；该道路对象分类未被完整识别。
- cut-in 无图形运行另有 2 条碰撞开始/解除 warn；图形运行共 45 条 warn，包括道路设施缺少尺寸时采用模型包围盒、部分对象 s 坐标超出道路长度，以及碰撞开始/解除。这些问题没有阻止本次运行和截图，但本次不证明道路设施语义正确。
- NCAP 两次运行均无 `[error]`，各有 2 条碰撞开始/解除 warn。
- `esmini.exe --version` 和 `--help` 均输出了正常版本/帮助文本，但实际退出码为 **-1**；原值保存在 [introspection-exits.json](evidence/introspection-exits.json)，未将其改写为 0。版本同时由原包 `version.txt` 及实际场景日志交叉核对，环境可用性由场景执行验证。
- 第一次 PDF 提取已写出 UTF-8 文本，但向控制台打印项目符号时出现 GBK `UnicodeEncodeError`；随后成功读取已保存的 UTF-8 文件并查看全部两页渲染。一次检索误用了 `esmini_0_of_1.log`，实际文件名为 `esmini_1_of_1.log`，已更正。详见 [diagnostic-attempts.json](evidence/diagnostic-attempts.json)。

NCAP 的官方车辆目录不提供专属 3D 模型。日志显示 esmini 自动使用 Demo 自带 `car_white.osgb` / `car_red.osgb` 作显示，路面由 OpenDRIVE 生成；这些只是现成的可视化素材，车辆尺寸和运动来自原始场景/目录，没有把显示素材用作正式感知模型。

**文件入口。** 每个运行目录包含 `command.ps1`、`run.json`、`validation.json`、stdout/stderr、CSV、日志及 DAT；NCAP 的单次参数运行由 esmini 自动添加 `_1_of_1` 后缀。

| 用途 | CSV | 场景日志 |
|---|---|---|
| cut-in 主结果 | [telemetry.csv](runs/20260913T153713_529656Z_demo/telemetry.csv) | [esmini.log](runs/20260913T153713_529656Z_demo/esmini.log) |
| NCAP 主结果 | [telemetry_1_of_1.csv](runs/20260913T153722_252575Z_ncap/telemetry_1_of_1.csv) | [esmini_1_of_1.log](runs/20260913T153722_252575Z_ncap/esmini_1_of_1.log) |
| cut-in 渲染 | [telemetry.csv](runs/20260913T153733_645901Z_demo_render/telemetry.csv) | [esmini.log](runs/20260913T153733_645901Z_demo_render/esmini.log) |
| NCAP 渲染 | [telemetry_1_of_1.csv](runs/20260913T153753_370425Z_ncap_render/telemetry_1_of_1.csv) | [esmini_1_of_1.log](runs/20260913T153753_370425Z_ncap_render/esmini_1_of_1.log) |

CSV 前 6 行为 esmini 版本/场景元信息，第 7 行才是列名；单位随列名提供，数据为仿真对象真值，不是传感器观测。CSV 行末可能有空列，`verify_runs.py` 已按实际格式处理。

**复现命令。** 在 PowerShell 中执行；已配置本机可直接从第二步开始。`setup.ps1` 复用匹配哈希的本地 ZIP，仅在缺包时从固定官方 URL 下载，不覆盖已有解压文件；`verify_sources.py` 用于识别任何原始文件变化。若 GitHub 后续改变同一 commit ZIP 的封装字节导致哈希不符，脚本会停止，应使用已保留 ZIP 或人工核对来源后更新记录，不应跳过校验。

```powershell
Set-Location -LiteralPath 'E:\Chalmers course\P5\TME180 project\esmini-lab'
& '.\scripts\setup.ps1'
python -X utf8 '.\scripts\verify_sources.py'
python -X utf8 '.\scripts\run_cases.py' --case all
python -X utf8 '.\scripts\verify_runs.py'
```

单独重跑：

```powershell
python -X utf8 '.\scripts\run_cases.py' --case demo
python -X utf8 '.\scripts\run_cases.py' --case ncap
```

复现实际检查过的离屏图形路径：

```powershell
python -X utf8 '.\scripts\run_cases.py' --case demo_render
python -X utf8 '.\scripts\run_cases.py' --case ncap_render
python -X utf8 '.\scripts\verify_runs.py'
```

每次生成新的 UTC 时间戳目录，默认超时为 60 秒。建议使用上面的 runner 复现以保留旧证据；各运行内 `command.ps1` 是本次实际 esmini 参数的逐项记录，直接执行会覆盖该目录中的同名日志。运行脚本不依赖当前工作目录；整个 `esmini-lab` 迁移后仍可通过脚本相对位置定位软件和场景，但旧的 `command.ps1` 记录保留原机绝对路径。

**双击观看 DAT 回放。** DAT 保存仿真记录，不是 MP4 视频。已补充 [replay-ncap.cmd](replay-ncap.cmd) 和 [replay-demo.cmd](replay-demo.cmd)，在资源管理器双击即可由自带 replayer 打开对应记录并循环播放。空格暂停/继续，左右方向键逐帧，Ctrl+左方向键回到开头，Esc 退出；可在三维窗口改变视角。两个 DAT 均经 replayer 离屏窗口回放检查，退出码为 0，命令与日志见 `evidence/replayer-check.json` 和 `evidence/replayer-check-*.log`。cut-in 回放仍保留原始道路对象兼容性报错。双击脚本会更新 `evidence/replay-ncap.log` 或 `evidence/replay-demo.log`，不会改写原始模拟结果。

**原始渲染预览。** 从实测 TGA 无损转换为 PNG，未修饰内容；转换对应关系见 `evidence/render-previews.json`。本次没有测试人工点击和键盘交互，已验证的是模拟器创建离屏窗口后的图形输出。

![官方 cut-in，8.0 s](evidence/demo-render.png)

![官方 CCRs，4.0 s](evidence/ncap-render.png)

**后续待办，尚未执行。**

- 获取项目组正式感知不确定性模型：模型文件、合法使用条件、训练/标定版本、输入输出、坐标系、单位、时间戳、更新频率、参数及适用范围。
- 获取正式 ADS/AEB 控制器及车辆接口：观测输入、制动/转向输出、时延、控制周期、车辆动力学与参数。当前恒速动作是官方场景本身的定义，不能作为理想感知 ADS 基线。
- 正式组件到位后再确定 esmini API/OSI 接口、理想感知对照和不确定性感知对照、参数矩阵、随机重复、步长收敛及安全指标；尚未执行闭环控制、感知不确定性对比或安全收益评估。
- 未执行全部 NCAP 场景、协议评分或认证；转弯控制点等特定兼容性需届时逐项核对，不从此次 CCRs 结果外推。
