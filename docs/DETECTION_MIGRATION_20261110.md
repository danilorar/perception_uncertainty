# Detection pipeline 迁移与基准验证：2026-11-10

五项操作已经完成：保留旧基准、合入 main、建立 pipeline 开发入口、接回 cone、验证原 Markov/IID 与 HiDrive 基准。

## 开发位置与 Git 状态

| 用途 | 工作目录 | 分支 / 提交 |
| --- | --- | --- |
| 后续开发 | `/Users/zhuoma/TME180-detection-v2` | `feature/esmini-perception` |
| 旧 Studio、原始数据和图形运行环境 | `/Users/zhuoma/TME180-project` | `archive/studio-local-20261110`，`833d370` |
| 原生 C++ Markov/AEB 基准 | 共享 Git 仓库中的 tag | `baseline/native-markov-20261006`，`484675b` |
| 旧 Aron 代码 | 共享 Git 仓库中的 tag | `baseline/aron-before-pipeline-20261110`，`6f9ae90` |
| 本地 HiDrive、敏感性报告与 cone 快照 | 共享 Git 仓库中的 tag | `baseline/hidrive-studio-local-20261110`，`833d370` |

合并提交 `e47289d` 同时保留原 feature 历史与 `origin/main@247f6cc`；合并无冲突。
实现提交为 `75bfeabc07edd1d43b10fa163556b6fe76b2dc82`，下述 1203 次实验全部在该提交、无本地修改时运行。
最后重新查询远程，main 仍为 `247f6cc`。所有变更均在本地，尚未 push。

旧目录中的原始实验、构建环境和原视频保留。旧目录原有的 `esmini` 子模块状态与未跟踪输出保留；
该目录用于旧环境复现，后续请在新目录开发。

## 验证结果

| 验证层次 | 实测结果 |
| --- | --- |
| 原 C++ 生成器 → pipeline Python 生成器 | 编译 `484675b` 的三个公开 perception 文件；800 个流、160,000 个标志逐帧完全一致 |
| 原 HiDrive 敏感性报告 → pipeline | 1203/1203 运行完成，0 失败；碰撞结果全部一致，触发时间、制动时间、最小间距和相对碰撞速度最大数值差均为 0 |
| 项目测试 | `.venv/bin/python -m unittest discover tests -q`：51 项通过 |
| 旧 Studio 环境独立重跑 seed18 | 与 pipeline 的 seed18 指标完全一致 |
| cone 视频 | 完整解码通过，所有 261 个仿真帧的显隐与保存的检测标志一致；源数据哈希保持不变 |

生成器比较包括目标 ID 1/7、seed 0–99、IID 及 Markov 平均漏检持续时间 0.2/0.5/1.0 s，
每个流 200 个传感器帧。HiDrive 闭环设计为三种场景、3 次理想参考与 1200 次随机实验，
漏检率参数 0.1、检测周期 0.05 s、仿真步长 0.01 s，其余误差关闭。
场景文件中的 `gidas_*` 命名沿用仓库，数据来源为 TASC。

新旧 HiDrive 实验的碰撞次数相同：

| 场景 | 理想参考 | IID | Markov 0.2 s | Markov 0.5 s | Markov 1.0 s |
| --- | --- | --- | --- | --- | --- |
| 1549178 | 0/1 | 0/100 | 6/100 | 6/100 | 9/100 |
| 1554254 | 0/1 | 0/100 | 13/100 | 12/100 | 12/100 |
| 1554431 | 1/1 | 100/100 | 100/100 | 100/100 | 100/100 |

以上闭环结果使用 HiDrive；原生 AEB 与 HiDrive 的闭环结果应各自比较。
这些参数是敏感性分析假设，结果不代表实测传感器风险。

## cone 与展示实验

`configs/detection_markov_seed18.json` 对应场景 1549178、Markov 0.5 s、seed18：

- 首次 AEB 触发：3.70 s；首次制动：3.90 s。
- 碰撞：5.19 s；自车速度 16.06329 km/h，相对碰撞速度 10.043606510354143 km/h。
- 例如 3.20–3.68 s 检测漏失，cone 消失；3.70 s 检测恢复，cone 出现。

视频为 `output/videos/detection_seed18_cone.mp4`，1280×720、50 fps、6.22 s，末尾定格 1 s。
保持之前的斜向俯视跟随角度、白色自车和红色目标车。
cone 从保存的 `detection.missed` 读取状态，车辆运动来自 `truth.csv`，回放过程不重新抽样误差或运行 HiDrive。
cone 的 24 m / 40° 仅用于展示范围，不作为控制器传感器模型的标定值。

## 继续研究

在 IDE 打开 `/Users/zhuoma/TME180-detection-v2`。现成 `.venv` 与 esmini 3.8.1 已构建，可直接运行：

```bash
cd /Users/zhuoma/TME180-detection-v2
.venv/bin/python -m pipeline run configs/detection_markov_seed18.json --output output/markov_seed18_research
.venv/bin/python -m pipeline cone output/markov_seed18_research \
  --output output/videos/markov_seed18_research.mp4 \
  --visual-runtime /Users/zhuoma/TME180-project/esmini-lab/runtime/esmini-full-v3.8.1
```

保留固定参数 Markov 作为对照，复制配置后新增模块关系变体。
`errors` 的执行顺序本身不会建立条件概率依赖；新模型需要明确检测状态如何供下游模块复用，
漏检时下游如何处理，以及随机流如何分配。单帧历史指上一帧检测结果。
本次已建立开发和验证入口，模型关系的具体形式仍应作为下一阶段研究单独定义。

完整命令见 [开发说明](DETECTION_DEVELOPMENT.md)，紧凑机器验证记录见
[detection_migration_20261110.json](detection_migration_20261110.json)。
本地完整输出保存在 `output/grids/detection_baseline_100/`、`output/demo_seed18/` 与 `output/validation/`；
这些大文件按仓库规则忽略，基准 CSV、检测序列哈希与本报告已纳入版本控制。
