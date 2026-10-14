# 验证记录 / Validation record

作者 / Author: Zhuo Ma

不同日期与控制器的证据分开记录。历史记录表示当时完成的检查，不代表本次重新执行 1200 次实验或验证所有平台。

Evidence is separated by date and controller. Historical records describe checks at that time, not a new 1200-run campaign or validation of every platform.

## 2026-10-08：整理检查 / Organization checks

本次检查通过：16 份项目双语文档及本地链接、Git 排除规则、3 组精选导出的数值/哈希和拒绝覆盖行为、六个原始输入指纹、7 项原生 CTest、10 项 Python 测试、13 个 CLI 入口、Python 语法，以及在临时目录从零生成三个教学场景。

Checks passed for 16 project bilingual documents/local links, Git ignores, three faithful exports and overwrite refusal, six input hashes, seven native CTests, ten Python tests, thirteen CLI entries, Python syntax, and fresh generation of three teaching scenarios in a temporary directory.

重新运行 1549178/seed10 的 ideal、Markov dropout 及三项配对检查（5 次），结果与既有演示一致；小型敏感性批次完成 5 个主运行、6 个检查，确认新自动报告有中英文。另完成一次 INI ideal 无窗口运行。验证产物留在被忽略的 `results/cleanup_validation/`。

Re-running 1549178/seed10 ideal, Markov loss and three paired checks (five runs) reproduced the existing demo. A small sensitivity batch completed five main runs and six checks, confirming bilingual automatic reports. One INI ideal headless run also passed. Check products remain in ignored `results/cleanup_validation/`.

整理范围不改变 C++ 控制逻辑、模型参数或老师输入。当前新增导出工具只读取汇总，自动报告只修改文字；编译缓存和原始实验目录保留。

Organization does not change C++ control logic, model settings or supplied input. The added exporter reads summaries only; automatic report changes affect text only. Build caches and original experiment directories are retained.

## 2026-10-08：cone 回放 / Cone replay

场景 1549178、AEB、Markov p=0.1、T=0.5 s、seed10。已完成实际 GUI 和截图检查：3.25 s 漏检 cone 隐藏，3.55 s 重新检测 cone 恢复。先前日志回放核对 ideal 1001 帧、dropout 458 帧、p=1 435 帧，位置/航向/速度/时间最大误差为零，输入日志指纹不变。

Scenario 1549178, AEB, Markov p=0.1, T=0.5 s, seed10. Actual GUI/snapshot checks showed the cone hidden during loss at 3.25 s and restored on detection at 3.55 s. Earlier replay checks covered 1001 ideal, 458 dropout and 435 p=1 frames, with zero maximum pose/heading/speed/time error and unchanged input hashes.

这是本机已有证据；本次文档整理不重新执行 GUI。演示解释和精选指标见 [seed10 报告](../simulation/acc_cpp/reports/aeb_demo_20261008.md)。

This is existing local evidence; documentation organization does not rerun the GUI. See the [seed10 report](../simulation/acc_cpp/reports/aeb_demo_20261008.md) for interpretation and selected metrics.

## 2026-10-06：原生 AEB Markov / Native AEB Markov

固定 esmini v3.8.1，7 项 CTest，1203 次主实验、36 次不变量检查、1 次额外完整时域目标真值参考。检查 p=0 等价、同 seed 复现、p=1 完全删除、目标真值不变、逐步碰撞一致，以及源输入和上游控制器不变。

Pinned esmini v3.8.1; seven CTests, 1203 main experiments, 36 invariant checks and one extra full-horizon target-truth reference. Checks cover p=0 equivalence, seed reproduction, p=1 removal, unchanged target truth, stepwise collision agreement, and unchanged inputs/upstream controllers.

1549178 的 IID/Markov T=0.2/0.5/1.0 s 碰撞为 0/4/8/9 次（每随机组 100 次）；1554254 为 1/7/9/9 次。1554431 理想已碰撞，其所有随机组均 100/100；完整时域参考不计入安全统计。

For 1549178, IID/Markov T=0.2/0.5/1.0 s collisions were 0/4/8/9 per 100 random runs; for 1554254 they were 1/7/9/9. Scenario 1554431 already collided under ideal perception, and every random group had 100/100 collisions. The full-horizon reference is excluded from safety statistics.

完整参数、统计范围及精选 CSV 见 [AEB 报告](../simulation/acc_cpp/reports/aeb_markov_20261006.md)。

See the [AEB report](../simulation/acc_cpp/reports/aeb_markov_20261006.md) for settings, statistical scope and selected CSVs.

## 历史原生 ACC / Historical native ACC

旧 IID seed42 三场景和旧输出回归验证保留原字段；Markov 引入前后旧 IID truth/perception/control/metrics 逐字节一致。ACC 的 1203 次主实验与 36 次检查另有标记，结果与 AEB 不混用，见 [ACC 报告](../simulation/acc_cpp/reports/acc_markov_historical.md)。

Old IID seed42 three-scenario regressions retain their original fields; truth/perception/control/metrics were byte-identical before and after adding Markov. ACC's 1203 main runs and 36 checks are separately labeled and not mixed with AEB; see the [ACC report](../simulation/acc_cpp/reports/acc_markov_historical.md).

## 2026-09-29：Python 教学整理 / Python teaching organization

历史记录环境为 macOS、Python 3.12.14，另用系统 Python 3.9.6 检查。10 项模型测试通过；三场景 ideal sensor、12 次 AEB/感知组合、原始轨迹八项核对和六个输入 SHA256 检查通过。1549178 整理前后四组既有字段逐行一致；独立复制后的模型测试与 ideal AEB 无窗口运行也通过。此次记录不含 GUI/GIF 视觉验证。

The historical environment was macOS with Python 3.12.14, additionally checked using system Python 3.9.6. Ten model tests, three ideal-sensor runs, twelve AEB/perception conditions, eight original-trajectory checks and six input SHA256 checks passed. Existing 1549178 fields matched row by row across organization; model tests and a headless ideal AEB run also passed in an independent copy. That record did not include GUI/GIF inspection.

教学参数 dt=0.01 s、TTC=2 s、减速度=6 m/s²、seed42。它是简化 Python AEB，和当前原生参数不同。

Teaching settings: dt=0.01 s, TTC=2 s, braking=6 m/s², seed42. This is simplified Python AEB with different settings from native AEB.

| 场景 / Scene | AEB | p | 首次制动 / Brake s | 碰撞 / Collision | 接触 / Contact s |
|---|---|---:|---:|---|---:|
| 1549178 | off | 0 | — | 是 / yes | 4.910 |
| 1549178 | on | 0 | 3.310 | 否 / no | — |
| 1549178 | off | 0.1 | — | 是 / yes | 4.910 |
| 1549178 | on | 0.1 | 3.310 | 否 / no | — |
| 1554254 | off | 0 | — | 是 / yes | 4.920 |
| 1554254 | on | 0 | 3.200 | 否 / no | — |
| 1554254 | off | 0.1 | — | 是 / yes | 4.920 |
| 1554254 | on | 0.1 | 3.200 | 否 / no | — |
| 1554431 | off | 0 | — | 是 / yes | 5.080 |
| 1554431 | on | 0 | 3.140 | 是 / yes | 5.800 |
| 1554431 | off | 0.1 | — | 是 / yes | 5.080 |
| 1554431 | on | 0.1 | 3.180 | 是 / yes | 5.670 |

以上是模型与软件验证，不是工业 AEB、安全认证或真实传感器验证。只在 macOS 实际运行；Windows/Linux CI 模板未执行。原路网末端警告保留，原轨迹核对显示所用轨迹位于末端之前。

These are model/software checks rather than industrial AEB certification or real-sensor validation. Actual runs are on macOS; Windows/Linux CI templates were not executed. Original road-end warnings remain; trajectory checks showed used paths end before the road endpoint.
