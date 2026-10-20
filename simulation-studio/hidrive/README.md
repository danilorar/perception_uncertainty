# HiDrive AEB 控制器接入（esmini 3.8.1）

老师提供的 `ControllerHiDrive.cpp/.hpp` 原件（esmini 2.40 接口）在项目上级目录 `AEB Controler/`。
2026-10-20 起 Studio 使用由用户手工移植到 esmini 3.8.1 接口的版本，放在
`AEB Controler (esmini 3.8.1)/`（改法见 `MIGRATE-3.8.1.zh.md`）。任何 AI agent 都不得打开、
搜索、解析、哈希或打印这些文件，也不得阅读可能引用其源码的编译日志。脚本只复制文件、
比较文件大小和修改时间。

## 运行

HiDrive AEB 是 Studio 唯一的控制器。`hidrive_runner.py` 用隔离编译的 esmini 3.8.1
（`esmini-lab/runtime/esmini-hidrive-v3.8.1/bin/esmini.exe`）运行老师原场景。场景中 object_1
已声明 `HiDriveController`，其属性（timeGap、mode 等）原样保留，只把道路和目录路径改为绝对路径。

- 运行器使用 `--headless --window ...`：离屏 pbuffer 画面，桌面上不出现窗口。
- OU 测距误差通过公开引擎钩子注入（`PerceptionNoise.hpp`，编译时复制为
  `ScenarioEngine/SourceFiles/StudioPerceptionNoise.hpp`，并在 `ScenarioEngine.cpp`
  的控制器 `Step` 前后各加一行调用；`--perception_noise` 选项登记在 `EnumConfig.hpp` 与
  `playerbase.cpp`）。Studio 按种子生成每个目标每步的单位 OU 样本（每个目标一条独立随机流，
  种子为 `种子:range-ou:目标ID`），写入 `perception_noise.tsv`，通过 `--perception_noise` 传给 esmini。
  esmini 在 HiDrive 控制器步进前，用实时真值中心距离计算误差幅度，把目标沿视线移到感知中心；
  步进后立即恢复真值，所以碰撞检测、CSV 记录和回放仍是真值。实际注入的误差写入
  `perception_applied.tsv`，Studio 据此生成 `observations.csv`。
- 误差每个积分步更新；Studio 的传感器量程/视场与观测周期不用于 HiDrive。测得距离为负视为
  无效，该步把目标放到自车后方 1000 m。未选测距误差时不传该参数，行为与原版完全相同。
- 车辆状态来自 esmini CSV 记录；`AEB_active` 与 `Acceleration` 取自日志中
  `[时间] [info] LOG_VALUE:名称:值` 行。首次触发为 `AEB_active=1`，首次制动为该加速度小于 -0.01 m/s²。
- 首次接触即截断输出；视频由 Studio 的完整 esmini 3.8.1 回放。

## 控制器更新后重新编译

改 `AEB Controler (esmini 3.8.1)/` 里的两个文件，然后运行：

```
.venv\Scripts\python.exe hidrive\build.py
```

脚本首次运行时从已校验的官方源码包 `esmini-lab/downloads/esmini-fullsrc-v3.8.1.zip` 解出隔离源码树，
依赖包（osg/osi/sumo/implot/proj）从 `esmini-full-v3.8.1/externals` 复制；然后复制两份控制器文件，
加入注册代码与误差钩子，后台编译并安装到 `bin/`。编译输出只写入
`esmini-lab/evidence/hidrive-build.log`。不要直接改隔离源码树里的控制器副本，下次编译会被覆盖。
控制器文件与已编译副本的大小或时间不一致时，Studio 会拒绝运行并提示重新编译。

`esmini-lab/runtime/esmini-hidrive-v2.40.0/` 是停用的旧接入方式，仅作对照保留。
