# esmini 仿真入口

Author: Zhuo Ma

当前三场景工作流请从项目根目录的 [README](../README.md) 开始。
完整说明见 [运行与 GitHub 指南](../docs/运行与GitHub指南.md)。

- `my_sensor_demo.py`：原始轨迹上的 ideal sensor。
- `my_aeb_demo.py`：ideal sensor 与可选 AEB。
- `my_aeb_dropout_demo.py`：dropout sensor 与可选 AEB。
- `run_comparison.py`：AEB off/on × ideal/dropout 批量比较。
- `generate_scenarios.py`：从只读原始数据生成外部控制场景。

旧的 Vector/NCAP 示例仍保留在 `aeb/`，本流程不再依赖它。
整理前说明和脚本位于 `archive/pre_cleanup_20260929/`，不提交 GitHub。
