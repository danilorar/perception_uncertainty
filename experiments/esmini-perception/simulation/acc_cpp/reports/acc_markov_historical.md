# 历史原生 ACC Markov / Historical native ACC Markov

作者 / Author: Zhuo Ma

本页保留 AEB 切换前的 ACC 敏感性结果。原批次 `markov_sensitivity_20261004T232223-126392Z`，不是 HiDrive 或当前 AEB 结果。三个场景均 ideal 不碰撞。

This retains ACC sensitivity results from before the AEB switch. Original batch: `markov_sensitivity_20261004T232223-126392Z`. These are neither HiDrive nor current AEB results. All three ideal references avoid collision.

参数：esmini v3.8.1、原 `ControllerACC::Step()`、timeGap=1.5 s，各场景巡航速度为初始自车速度；dt=0.01 s、感知周期 0.05 s、p=0.1、stationary、seed0..99、最长 10 s/首次接触。1203 主运行、36 次边界/复现验证。原记录完成六项 CTest。

Settings: esmini v3.8.1, original `ControllerACC::Step()`, timeGap=1.5 s, cruise speed from each scene's initial ego speed; dt=0.01 s, sensor period=0.05 s, p=0.1, stationary, seeds0..99, and 10 s/first contact. There are 1203 main runs and 36 boundary/reproduction checks. The historical record completed six CTests.

| 模型 / Model | 1549178 碰撞 / Collisions | 1554254 碰撞 / Collisions | 1554431 碰撞 / Collisions |
|---|---:|---:|---:|
| IID | 0/100 | 0/100 | 0/100 |
| Markov T=0.2 s | 3/100 | 1/100 | 1/100 |
| Markov T=0.5 s | 22/100 | 17/100 | 13/100 |
| Markov T=1.0 s | 31/100 | 28/100 | 26/100 |

Markov 实际聚合缺失约 10.7%–11.7%，IID 约 10%；短场景抽样及首次接触截止影响比例。0/100 不表示风险为零，区间和 gap/TTC 分布见 CSV。这是设定误差敏感性，未校准实测传感器。

Pooled realized Markov loss is about 10.7%–11.7%, versus about 10% IID. Short-run sampling and first-contact censoring affect ratios. 0/100 does not establish zero risk; CSVs contain intervals and gap/TTC statistics. This is assumed-error sensitivity without real-sensor calibration.

历史 Markov 500000 帧单元检查得缺失比例约 0.10067、完整缺失段均值约 0.50333 s（p=0.1，T=0.5 s）；其他检查覆盖边界、顺序、复现和 FOV 外状态推进。旧 IID seed42 的 truth/perception/control/metrics 保持一致，原输入及上游 ACC 未改动。

The historical 500000-frame Markov check gave loss fraction about 0.10067 and mean complete burst about 0.50333 s (p=0.1, T=0.5 s). Other checks covered boundaries, order, reproduction and state progression outside FOV. Old seed42 IID truth/perception/control/metrics stayed identical; inputs and upstream ACC were unchanged.

[汇总 / Summary](acc_markov_20261004/sensitivity_summary.csv)、[逐 seed 指标 / Per-seed metrics](acc_markov_20261004/sensitivity_runs.csv)、[来源 / Provenance](acc_markov_20261004/README.md)。当前重跑使用 [原生指南](../README.md) 的敏感性命令，设 `--controller acc --time-gap 1.5`；程序已增加 AEB 字段，历史 manifest 保留当时指纹。

The links retain historical summaries, per-seed metrics and provenance. To rerun with current code, use the [native guide](../README.md) sensitivity command with `--controller acc --time-gap 1.5`. The program now includes AEB fields; the historical manifest retains its original hashes.
