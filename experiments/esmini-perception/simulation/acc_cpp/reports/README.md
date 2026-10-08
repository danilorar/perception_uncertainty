# 精选实验结果 / Selected experiment results

作者 / Author: Zhuo Ma

这里提交报告、聚合 CSV、每 seed 一行指标及来源指纹；逐帧 CSV、DAT、日志、GUI 图像和视频仍在本机 `results/`。每个导出目录的双语 README 说明路径转换，export manifest 保留原文件/导出文件 SHA256。历史控制器结果不混用。

This directory tracks reports, aggregate CSVs, one metrics row per seed and provenance hashes. Frame CSVs, DAT, logs, GUI images and videos stay in local `results/`. Each export's bilingual README explains path normalization; the export manifest records source/export SHA256. Different controllers' historical results remain separate.

| 报告 / Report | 控制器和规模 / Controller and scope | 精选数据 / Selected data |
|---|---|---|
| [AEB Markov 2026-10-06](aeb_markov_20261006.md) | 原生 AEB，1203 主运行 / Native AEB, 1203 main runs | [CSV/manifest](aeb_markov_20261006/README.md) |
| [历史 ACC / Historical ACC](acc_markov_historical.md) | 原生 ACC，1203 主运行 / Native ACC, 1203 main runs | [CSV/manifest](acc_markov_20261004/README.md) |
| [seed10 演示 / seed10 demo](aeb_demo_20261008.md) | 1549178，ideal/Markov 两组 / Two ideal/Markov cases | [CSV/manifest](aeb_demo_1549178_seed10_20261008/README.md) |

复现入口见 [原生指南](../README.md)。导出新批次使用 `tools/export_report.py --batch-dir <batch> --name <new_name>`，再将新报告加入此表。不要手工编辑 CSV 数值来匹配报告叙述。

See the [native guide](../README.md) for reproduction. Export a new batch using `tools/export_report.py --batch-dir <batch> --name <new_name>` and add its report to this table. Do not manually edit CSV values to fit narrative conclusions.
