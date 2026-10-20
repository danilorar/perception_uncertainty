# HiDrive 控制器移植到 esmini 3.8.1：改法对照表

本表只依据 esmini **公开源码** v2.40.0 与 v3.8.1 的差异整理，AI 没有打开过
`ControllerHiDrive.cpp/.hpp`。你在 VS 里改控制器，碰到表里没有的报错再把报错文字发给我。

## 0. 准备（一次）

1. `AEB Controler\` 里的原件是 2.40 版，**不要动**，当备份。
2. 用 Visual Studio 打开
   `esmini-lab\runtime\esmini-full-v3.8.1\build-hidrive\esmini.slnx`，
   配置选 **Release / x64**，右键 `esmini` 项目 →「生成」。
3. 要改的是这个工程里的副本（与原件完全相同）：
   `esmini-full-v3.8.1\EnvironmentSimulator\Modules\Controllers\ControllerHiDrive.cpp / .hpp`。
   注册代码（`Controller.hpp` 的 `CONTROLLER_TYPE_HIDRIVE`、`ScenarioReader.cpp` 的
   `RegisterController("HiDriveController", ...)`）已经加好，不用改。
4. **第一步先做**：在 `.hpp` 里，凡是重写基类的虚函数（`Init`、`InitPostPlayer`、`Step`、
   `Activate`、`GetType`、`GetTypeName`、`ReportKeyEvent`、`LinkObject` 等）末尾都加上
   `override`。这样签名和 3.8.1 对不上的地方会直接报错，而不是编译通过却永远不被调用。

## 1. 会报编译错误的改动（按报错逐条改）

| 2.40 写法 | 3.8.1 改成 |
|---|---|
| `#include "ScenarioGateway.hpp"` | 删除。需要引擎时 `#include "ScenarioEngine.hpp"`；用日志加 `#include "logger.hpp"` |
| `gateway_->updateObjectSpeed(id, 0.0, v)` | `object_->SetSpeed(v);` |
| `gateway_->updateObjectPos(id, 0.0, &object_->pos_)` | **直接删掉**（`object_->MoveAlongS(...)` 或改 `pos_` 已经生效） |
| `gateway_->updateObjectAcc(id, 0.0, ax, ay, az)` | `object_->SetAcc(ax, ay, az);` |
| `gateway_->updateObjectVel(id, 0.0, vx, vy, vz)` | `object_->SetVel(vx, vy, vz);` |
| `gateway_->updateObjectAngularVel(id, 0.0, h, p, r)` | `object_->SetAngularVel(h, p, r);` |
| `gateway_->updateObjectWorldPosXYH(id, 0.0, x, y, h)` | `object_->pos_.SetInertiaPos(x, y, h);`<br>`object_->dirty_.SetBits(Object::DirtyBit::LONGITUDINAL \| Object::DirtyBit::LATERAL);` |
| `gateway_->updateObjectWheelAngle(id, 0.0, a)` | `object_->wheel_angle_ = a;`<br>`object_->dirty_.SetBits(Object::DirtyBit::WHEEL_ANGLE);` |
| `gateway_->getObjectStatePtrById(id)->state_.pos` | `entities_->GetObjectById(id)->pos_` |
| 其它任何 `gateway_->...` | 把那一行用到的函数名发给我 |
| `args->entities`、`args->gateway`、`gateway_ = ...` | 删除。基类构造函数已从 `args->scenario_engine` 设好 `entities_` 和 `scenario_engine_` |
| 重写了 `SetScenarioEngine(...)` | 删除（3.8.1 没有这个函数，`scenario_engine_` 构造时就有） |
| `static const char* GetTypeNameStatic()` + `virtual const char* GetTypeName()` | 合成一个：`const char* GetTypeName() const override { return "HiDriveController"; }`（返回原来那个名字常量即可） |
| `static int GetTypeStatic()` + `virtual int GetType()` | `Type GetType() const override { return CONTROLLER_TYPE_HIDRIVE; }`；别处调用 `GetTypeStatic()` 的改成 `CONTROLLER_TYPE_HIDRIVE` |
| `int Activate(ControlActivationMode lat, ControlActivationMode lon, ControlActivationMode light, ControlActivationMode anim)` | `int Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]) override`<br>函数体里：`lat` → `mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)]`，`lon` → `...DOMAIN_LONG)]`，`light`/`anim` 同理；<br>`Controller::Activate(lat, lon, light, anim)` → `Controller::Activate(mode)` |
| `ControlDomains::DOMAIN_NONE / DOMAIN_ALL / DOMAIN_LAT_AND_LONG` | `ControlDomainMasks::DOMAIN_MASK_NONE / DOMAIN_MASK_ALL / DOMAIN_MASK_LAT_AND_LONG` |
| `object_->SetSensorPosition(x, y, z)`、`sensor_pos_` | `object_->SetLookaheadSensorPosition(x, y, z)`、`lookahead_sensor_pos_` |
| `player_->SteeringSensorSetVisible(id, b)` | `player_->LookaheadSensorSetVisible(id, b)` |
| `object_->SetDirtyBits(b)` / `ClearDirtyBits(b)` / `CheckDirtyBits(b)` / `GetDirtyBitMask()` | `object_->dirty_.SetBits(b)` / `dirty_.ClearBits(b)` / `dirty_.Check(b)` / `dirty_.Get()` |
| `ControlDomain2Str(mask)` | `ControlDomainMask2Str(mask)` |
| `LOG(...)` / `LOG_TRACE(...)` | `LOG_INFO(...)`（调试信息用 `LOG_DEBUG`，默认不输出）——**格式串要按第 2 节改** |
| `LOG_ONCE(...)` / `LOG_AND_QUIT(...)` | `LOG_WARN_ONCE(...)` / `LOG_ERROR_AND_QUIT(...)` |
| `SE_getSystemTime()` / `SE_sleep(ms)` | `SE_getSystemTimeMilliseconds()` / `SE_sleepMilliseconds(ms)` |
| `FileExists(path)` | `std::filesystem::exists(path)`（加 `#include <filesystem>`） |
| `...GetOptions().GetOptionArg("x")` | `...GetOptions().GetOptionValue("x")` |

`GetSpeed()`、`GetId()`、`pos_.GetX()`、`FreeSpaceDistance(...)`、`Distance(...)`、`MoveAlongS(...)`、
`GetMaxDeceleration()`、`boundingbox_`、`entities_->object_[i]`、`InstantiateControllerHiDrive(void* args)`
这些在 3.8.1 里还在，不用改。

## 2. 编译不报错、但结果会错的地方（必须自己搜一遍）

1. **`ControlDomains::DOMAIN_LONG` / `DOMAIN_LAT` / `DOMAIN_LIGHT` / `DOMAIN_ANIM`**
   名字没变，**数值变了**：2.40 是位掩码（LONG=1，LAT=2），3.8.1 变成序号（LONG=0，LAT=1）。
   所以 `IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LONG))` 能编译，
   但会变成“永远为真”，`DOMAIN_LAT` 会变成“纵向”。
   → 搜 `ControlDomains::`：凡是当掩码用的（`IsActiveOnDomains`、`operating_domains_ =`、`& / |` 运算）
   全部改成 `ControlDomainMasks::DOMAIN_MASK_LONG` 等；只有当数组下标（如 `mode[...]`）时才保留 `ControlDomains::`。
2. **日志格式串**：3.8.1 用 fmt，不认 `%d %f %s`。
   `LOG("v=%.2f id=%d name=%s", v, id, name.c_str())` →
   `LOG_INFO("v={:.2f} id={} name={}", v, id, name)`。
   只改宏名不改格式串也能编译，但输出会是字面的 `%.2f`。消息里如果有字面 `{` `}` 要写成 `{{` `}}`。
   **格式符和变量类型不匹配也能编译，但运行到那一行会抛异常 `invalid format specifier`，esmini 直接退出。**
   对照：`%f`/`%lf` → `{:f}`（只能给 double/float）；`%.2f` → `{:.2f}`；`%d`/`%i`/`%u`/`%ld`/`%zu` → `{}`；
   `%s` → `{}`；**bool 想输出 0/1 要用 `{:d}`**（`{}` 会输出 true/false）。
   `{:lf}`、`{:i}`、`{:u}`、`{:ld}`，以及把 `{:f}` 用在 int/bool 上，都是非法格式。
3. **Studio 依赖的 `LOG_VALUE` 行**：保持 `LOG_VALUE:名字:值` 这段文字不变（如
   `LOG_INFO("LOG_VALUE:AEB_active:{}", active)`），且必须用 `LOG_INFO`，不能用 `LOG_DEBUG`。
   3.8.1 的行首格式变成 `[12.340] [info] ...`，Studio 那边的解析由我来改。
4. **没加 `override` 的旧签名虚函数**：如 4 参数的 `Activate`、非 const 的 `GetType()`，
   在 3.8.1 里能编译，但引擎永远不调用它们。第 0 节第 4 步加了 `override` 就能全部暴露。

## 3. 需要你发给我的信息

- **A. 编译错误清单**：VS「错误列表」里每条的 *错误代码 + 说明 + 行号*（不用贴代码行）。
  表里能对上的你直接改；对不上的发我，我告诉你那一行怎么改。
- **B. 搜索结果**（在两个文件里 Ctrl+F，告诉我每个词出现几次即可）：
  `gateway_`、`ControlDomains::`、`LOG`、`Activate(`、`GetType`、`player_`、`viewer`、`osi`
- **C.（可选）** 控制器有没有在 `Init` / `InitPostPlayer` / `Step` 里用 `player_` 或 viewer。
  2.40 下纯 `--headless` 会崩溃，很可能跟这个有关；3.8.1 下要一起确认。

## 4. 编译通过后由我来做（2026-10-20 已完成）

移植版已放到 `AEB Controler (esmini 3.8.1)/`，以后改控制器请改那里的文件，再运行 `hidrive/build.py`
（见 `README.md`）。`esmini-full-v3.8.1/build-hidrive` 里的 VS 工程只是移植时用的草稿工作区。

- `hidrive/build.py` 改成基于 3.8.1 编译（仍然只复制控制器文件、不读取）。
- 把测距误差钩子 `PerceptionNoise.hpp` 移植到 3.8.1 的 `ScenarioEngine.cpp`（控制器 `Step` 那一行在 3.8.1 里位置相同）。
- `hidrive_runner.py`：日志正则改为 3.8.1 格式，核对 CSV 列和命令行参数，去掉 2.40 专用的处理。
- 回归：同一批老师场景分别用 2.40 和 3.8.1 跑，对比 AEB 触发时间、首次制动、最小距离、是否碰撞；
  结果见 `README.zh-CN.md` 0.12.0。Studio 已切到 3.8.1；2.40 隔离版本暂留作对照。
