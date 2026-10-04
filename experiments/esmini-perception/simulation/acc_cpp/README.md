# esmini 原生 C++ ACC 感知不确定性试验

Author: Zhuo Ma

这套流程直接调用 esmini **v3.8.1 的原始 `ControllerACC::Step()`**。控制器原 `.cpp/.hpp` 文件不修改、不重新实现控制律。新代码只为控制器准备独立的对象快照，并将输出目标速度交给共同的车辆模型执行。

固定版本：`19d26b68f7f473de4e55046a794b5fd2f91c58c8`。

## 已定义的研究边界

- 只对外部目标感知施加误差；ego 状态始终为当前仿真真值。
- 两组共用相同初始条件、ACC 参数、车辆模型、驾驶模型和传感器配置，ego 后续轨迹允许不同。
- 外部目标继续执行原场景的时间轨迹。
- 碰撞、间隙、距离、TTC 全部由真实状态评价。
- 输入仍限定为两辆显式定义的车辆、ego 名称 `object_1`、从 t=0 开始的绝对时间 WorldPosition 轨迹。保留平坦、近似同向、向前运动等检查；一般路口和换道决策未纳入模型。
- 不再限制轨迹相对初始方向的累计横向偏移，也不限制场景文件名。新增覆盖 1549178、1554254、1554431 的场景准备检查。
- 横向驾驶仍采用原始空间路径的理想跟随。感知误差改变速度与路径进度，因此同一时刻的 x/y 可以不同；尚未实现由感知驱动的转向或换道决策。

## 为什么 XOSC 里仍然是 ExternalController

XOSC 的 `ExternalController` 是 ego 状态提交接口。真正的纵向决策发生在编译进 esminiLib 的 `ControllerACC::Step()` 中。适配层给它提供自己的 `Entities` 列表：准确 ego 副本 + uncertainty 输出的目标副本。它不会获得正在运行的场景实体列表。

控制器设为 additive 以便仅修改副本速度，不推进实体位置。每次计算从准确 ego 速度初始化，再设置固定巡航目标。这使 ACC 的当前速度与实际车辆状态一致，避免未执行的旧速度请求改变巡航目标。

原 ACC 中小于 1 m 的车间隙会直接请求速度为零。本流程保留这一请求，转换为加速度请求；共同车辆模型执行 ±10 m/s² 限幅，再连续积分。因此本流程使用原 ACC 决策和筛选逻辑，并使用明确、独立的车辆执行模型。

## 从源码构建

需要 Git、CMake >=3.21、Python >=3.9，以及本平台的 C++ 编译器：macOS 的 Xcode Command Line Tools、Windows 的 Visual Studio C++ Build Tools、Linux 的 GCC/Clang 和 Make。

进入目录：

```bash
cd "$HOME/TME180-project/experiments/esmini-perception/simulation/acc_cpp"
```

如果 CMake 尚未安装，可以在自己的 Python 环境安装：

```bash
python3 -m pip install cmake==3.31.6
```

构建和运行单元检查：

```bash
python3 tools/build_native.py
```

Windows 对应使用 `python`。编译脚本会下载固定版本 esmini 和它的固定 fmt 子模块，禁用图形/OSI/SUMO 等可选功能，并将适配层加入 esminiLib。它仅调整上游 CMake 的测试资源下载条件，避免拉取本试验不需要的 ALKS/NCAP 资源。构建前验证原 ACC 源文件与固定 commit 逐字节相同。

编译输出保存在 `build/`，不进入 Git。源码下载需要网络；重复构建使用缓存。完整日志在 `build/engine-build.log` 和 `build/runner-build.log`。

本机配置不要提交到 Git。`CMakeUserPresets.json` 已忽略；`CMakePresets.json` 提供可选 Ninja 配置，日常可直接用以上 Python 构建入口。

## 运行理想感知与 10% dropout 对比

macOS/Linux：

```bash
python3 tools/run_comparison.py --binary build/runner/acc_sim --seeds 42 --validate
```

Windows：

```powershell
python tools/run_comparison.py --binary build/runner/Release/acc_sim.exe --seeds 42 --validate
```

`--validate` 另外检查零概率等价、相同种子复现、目标真值保持，以及 100% dropout 下所有原始检测都被删除。每次运行逐步核对独立真值几何和 esmini 的碰撞标志。100% dropout 是否发生碰撞作为结果记录，不假定每个场景和配置都必然碰撞。测试通过后，普通批运行可以省略此选项。

通过 `--scenario` 更换原始场景。批目录和生成文件名随输入场景变化。macOS/Linux 依次验证三个场景：

```bash
for id in 1549178 1554254 1554431; do
  python3 tools/run_comparison.py --binary build/runner/acc_sim \
    --scenario "../../Data/C_original_${id}.xosc" --seeds 42 --validate || break
done
```

Windows PowerShell：

```powershell
foreach ($id in 1549178, 1554254, 1554431) {
  python tools/run_comparison.py --binary build/runner/Release/acc_sim.exe --scenario "../../Data/C_original_$id.xosc" --seeds 42 --validate
  if ($LASTEXITCODE -ne 0) { break }
}
```

多种子试验：

```bash
python3 tools/run_comparison.py --binary build/runner/acc_sim --seeds 42 43 44 45 46
```

单独运行一个配置：

```bash
build/runner/acc_sim --config configs/ideal.ini --output-dir results/my_ideal_run
build/runner/acc_sim --config configs/dropout.ini --output-dir results/my_dropout_run
```

结果目录必须是新的空目录。每次批运行自动生成独立时间目录，`results/latest.txt` 指向最新一次比较。

不使用 INI 时，命令行可显式设置：

```bash
build/runner/acc_sim --scenario ../../Data/C_original_1549178.xosc --output-dir results/custom_run --dropout-p 0.1 --seed 42 --dt 0.01 --sensor-period 0.05 --duration 10 --time-gap 1.5
```

`--set-speed` 单位 m/s。省略时使用所选场景 ego 第一段轨迹的平均速度（1549178 为 13.1529464 m/s）。初始 ego/target 速度也分别由第一段轨迹推导，因为原 XOSC 的初始 SpeedAction 为 0，之后却立即执行运动轨迹；该调整只发生在生成副本中。

## 模块与接口

| 模块 | 文件 | 输入 | 输出 |
|---|---|---|---|
| 场景准备、参考路径 | `src/scenario.cpp` | 原 `.xosc/.xodr` | 生成场景、原 ego 空间参考、初始速度、性能限制 |
| esmini 适配器 | `src/esmini_adapter.cpp` | 生成场景、ego 下一步状态 | 同步世界真值、理想目标快照 |
| 感知误差 | `src/perception.cpp` | `PerceptionFrame`、概率、seed | 新观测列表及单独的丢失审计列表 |
| ACC 输入/输出封装 | `src/esmini_acc.cpp` | 当前 ego 真值、缓存观测 | 目标速度、加速度请求、目标 ID |
| 原生 ACC 桥接 | `src/native_acc_bridge.cpp` | C ABI 的 ego/目标值副本 | 调用原 `ControllerACC::Step()` 的结果 |
| 仲裁及车辆模型 | `src/vehicle.cpp` | 控制请求、车辆限制、dt、参考路径 | 执行命令与 ego 下一步状态 |
| 安全评价 | `src/metrics.cpp` | `WorldTruthFrame` | 真实距离、车间隙、TTC、碰撞 |
| 调度和记录 | `src/main.cpp` | 配置与各模块 | 固定时步闭环、CSV、JSON、DAT |
| 批运行 | `tools/run_comparison.py` | 原生程序、配置、种子列表 | 比较 CSV、输入 SHA256、验证结果 |

数据结构见 `include/accsim/types.hpp`。使用 m、s、m/s、m/s²、rad 和世界坐标。保留 bounding box 中心相对参考点的偏移，避免把参考点距离误当成车间隙。

`WorldTruthFrame` 包含真实 ego 和真实目标，仅供场景适配、理想快照构造和评价使用。控制入口 `ControllerInput` 接收准确 ego 与独立的 `PerceptionFrame`，没有目标真值容器。跨动态库接口 `include/native_acc_bridge.h` 只传 POD/C ABI 数据，不传 STL 容器或 C++ 实体指针。

## 时步顺序

1. 读取时间 `t_k` 的同步世界真值，准确 ego 状态作为反馈。
2. 每五个物理步采样一次 ideal sensor，形成完整目标快照。
3. uncertainty 对这个快照计算一次；之后控制与日志读取缓存。
4. 原生 ACC 根据准确 ego 和最新可用观测计算速度请求。
5. 转换为加速度请求，执行限幅，车辆模型积分至 `t_(k+1)`。
6. ego 沿原空间路径行驶，进度由实际行驶距离决定，不跳回原时间轨迹。
7. 将 ego 状态提交给 esmini，推进目标场景；检查 ego 未被原场景覆盖。
8. 下一帧使用同步真值评价、记录；接触发生时结束。

初始化使用 `SE_StepDT(0)` 填充传感器结果，不推进物理时间。感知与物理周期必须是整数倍关系。新空感知帧表示本次没有目标输出；非采样步继续读取上一个完整快照。日志记录 measurement time、sequence、age 和 fresh。

## Ideal sensor 和 dropout 的含义

本实现调用 esmini `SE_AddObjectSensor` / `SE_FetchSensorObjectList`，先获取 ID，再在同一引擎帧查询这些对象的属性。v3.8.1 实现的 FOV 参数为 radians，尽管其头文件注释写 degrees；默认使用 π/3，即 60°。默认安装位置为 ego `(2.5,0,0.5)` m，范围 0.1–60 m。

这是理想对象视场筛选，不代表完整雷达、相机或多传感器融合。当前只有一个理想前向 sensor。

dropout 对每个原始检测目标、每个感知帧独立以 10% 概率删除整条观测。删除不会修改场景实体，也不会把位置设为零。整数混合算法由 seed、测量序号和稳定对象 ID 决定，不受日志调用次数影响。

本实现没有目标跟踪器。新帧丢失目标后，ACC 采用自身的无前车巡航逻辑；后续帧重新检测到时再恢复跟车。参数属于设定误差的敏感性试验，尚未用实测 sensor 数据校准。

## 输出文件

| 文件 | 内容 |
|---|---|
| `run_config.json` | 参数、源数据指纹、模型约定、esmini commit |
| `generated/C_acc_<场景ID>.xosc` | ego 外部状态桥接、原 target 时间轨迹 |
| `generated/<原道路文件名>.xodr` | 原路网字节一致的快照，生成输入可跨盘符使用 |
| `truth.csv` | 每个时刻的真实位置、速度、尺寸、类型和类别 |
| `perceptions.csv` | 原始检测、dropout、实际观测、原始和观测类型/类别 |
| `control.csv` | 缓存帧年龄、ACC 目标、请求加速度、执行加速度、限幅 |
| `metrics.csv` | 真实间隙、距离、TTC；独立几何与 esmini 碰撞标志 |
| `summary.json` | 安全和感知汇总；`complete=true` 表示正常完成 |
| `sim.dat`、`run.log` | esmini 回放记录及引擎日志 |
| 批目录 `comparison.csv` | 所有种子的 ideal/dropout 比较 |
| 批目录 `batch_manifest.json` | 原始输入/程序 SHA256 与验证结果 |

CSV 中 `raw_*` 是审计用理想结果，`observed_*` 才是控制器可使用的目标信息。被丢弃时观测字段为空。所有安全指标仍来自 `truth.csv`。

## 评价和模型限制

碰撞由真实的有向二维 bounding box 加高度重叠判定，并逐步核对 esmini 自带碰撞标志。最小距离是两包围盒表面的几何距离；车间隙沿 ego 方向投影；TTC 是同路径前车且正在接近时的间隙/接近速度，其他情况无定义。

当前纵向车辆模型采用恒加速度积分，停止发生在步内时不会倒退；横向使用理想空间路径跟随。没有轮胎、执行器延迟或碰撞响应。首次接触时结束，因此碰撞速度是接触时的运动学速度，不是碰撞动力学结果。

累计横向偏移不再作为场景拒绝条件，也不裁剪运行结果。当前仍检查高度变化 <=0.05 m、相对初始 ego 航向变化 <=0.05 rad、俯仰/侧倾 <=0.02 rad 和沿初始方向向前运动。该范围支持所提供的三个近似同向场景，不表示一般弯道或交叉口已验证。真实有向包围盒可处理平面朝向变化；纵向 TTC 仍是当前方向上的间隙/接近速度近似。

`truth.csv` 保留 ego 的真实 x/y/h。观察横向变化时应使用这些真值，并明确参考方向，例如相对初始 ego 航向的横向投影；不要对它施加累计偏移上限。

## 更换其他 esmini 控制器

当前没有 `--controller` 选项，不能仅改 XOSC 控制器名称。生成场景中的 `ExternalController` 是状态提交接口；真正调用的控制器由 `src/native_acc_bridge.cpp` 中的 C++ 类型决定。

更换纵向控制器时，先检查其 `Step()` 所需依赖、状态和输入。适配层必须继续只提供准确 ego 与 uncertainty 后目标；不能让新控制器通过 `player_`、其他传感器或场景实体重新读取目标真值。修改原生桥接的控制器类型、参数、初始化和输出提取，并同步 `src/esmini_acc.cpp`、`include/native_acc_bridge.h`、`include/accsim/control.hpp` 和控制器测试，再重新构建 esminiLib 与 runner。可以保留共用的感知、纵向车辆模型和真值评价。

当前跨库对象包主要包含 pose、标量速度、类别和 bounding box。有的控制器还使用横向速度、加速度或额外车辆参数；这些输入必须显式补齐。外部目标的新字段也要经过 uncertainty 接口，不能为满足控制器输入而绕过感知误差模型。

当前 ACC 桥接每步重新构造控制器并从准确 ego 初始化。其他控制器如果含积分器、目标跟踪、状态机或历史缓存，必须改为每次仿真持久创建、逐步更新、结束销毁，不能照搬每步重建的生命周期。

若新控制器还输出转向或横向控制，需扩展 `ControlRequest` 和跨库输出，并让车辆模型执行转向；同时停用冲突的固定路径横向驾驶。否则新控制器的横向输出不会影响实际 ego 运动。`tools/build_native.py` 的上游源码校验也应增加新控制器，记录版本、输入输出约定，并重新验证 ideal/dropout 的隔离与复现。

## 去除累计横向偏移限制后的本机验证

macOS 上五项 CTest 全部通过。三个场景分别执行 ideal、10% dropout、零概率重复、同种子重复及 100% dropout，共 15 次原生仿真。以下六组主比较均运行到 10 s，使用 dt=0.01 s、sensor period=0.05 s、timeGap=1.5 s、seed=42，各场景巡航目标默认采用自身初始 ego 速度。

| 场景 | 感知 | 碰撞 | 最小真实间隙 / m | 最小纵向 TTC / s |
|---|---|---|---|---|
| 1549178 | ideal | 否 | 5.721 | 2.611 |
| 1549178 | 10% dropout | 否 | 5.206 | 2.124 |
| 1554254 | ideal | 否 | 8.317 | 2.372 |
| 1554254 | 10% dropout | 否 | 7.595 | 2.240 |
| 1554431 | ideal | 否 | 13.557 | 1.972 |
| 1554431 | 10% dropout | 否 | 12.685 | 1.867 |

三场景均通过零概率等价、同种子逐字节复现、目标真值保持和 100% dropout 观测删除检查；100% dropout 实测分别在 4.34、4.75、4.70 s 首次碰撞。1549178 的 ideal/dropout 记录与修改前验证记录逐字节一致。六个原始 .xosc/.xodr 文件和上游 ACC 源码 SHA256 均保持不变。

每组 dropout 为 26/200 次原始检测消失，有限样本实际比例为 13%，不要求恰好 10%。同一时刻比较两组 ego 的真实横向投影，最大差异分别约为 0.00858、0.01171、0.01787 m；这来自速度变化导致的空间路径进度差异。

本次汇总路径保存在 `results/latest_revalidation.txt`；对应目录包含 `comparison_all_scenarios.csv`、`validation_report.json` 和各场景的完整运行记录。构建及运行结果仍由 .gitignore 排除，不自动提交到 Git。上述结果是本次种子与模型参数的验证记录，不代表所有随机种子和控制器。

10% 是每次随机试验的概率，不要求有限样本恰好消失 10%。比较应保留实际漏检比例，并扩展多个种子。不要仅凭一个 seed 推断一般安全结论。

## 三平台验证

统一源码和构建入口支持 macOS、Windows、Linux；每个平台单独编译。macOS 已在当前开发环境验证。`ci/native-acc.yml` 是三平台 CI 模板，若需要启用，复制到仓库根目录 `.github/workflows/`，并调整路径触发条件；本地没有执行 Windows/Linux job。

运行最小检查可使用 `python3 tools/build_native.py`；运行闭环不变量检查使用 `run_comparison.py --validate`。跨平台比较采用数值容差，而不要求浮点 CSV 逐字节一致；随机漏检的整数决策应一致。

图形功能在默认精简库中关闭。`sim.dat` 可使用同版本完整 esmini 的 replayer 回放，借助其 `resources` 模型目录。原生程序自身不依赖 Python 感知/AEB 脚本。

## 上游来源与许可

- [esmini v3.8.1 ControllerACC.cpp](https://github.com/esmini/esmini/blob/v3.8.1/EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp)：MPL 2.0，源码由构建脚本按 commit 获取，ACC 文件保持原样。
- [esmini 控制器文档](https://esmini.github.io/controllers.html)。
- [esmini 构建文档](https://esmini.github.io/build-guide.html)。
- pugixml 1.14 用于本程序场景准备，源码及 MIT 许可放在 `third_party/pugixml/`。为避免与 esmini 内部 pugixml 1.9 的符号冲突，编译时使用独立命名空间。

本程序的新增文件标注 Author: Zhuo Ma。原始数据的 FileHeader author 保持原数据作者。
