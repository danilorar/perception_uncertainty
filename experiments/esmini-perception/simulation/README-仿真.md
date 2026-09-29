# 三个原始场景的仿真与感知实验

Author: Zhuo Ma

场景编号：1549178、1554254、1554431。

在项目根目录运行：

```bash
python3 simulation/generate_scenarios.py --scenario all --overwrite
python3 simulation/my_sensor_demo.py --scenario 1549178
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb on
python3 simulation/my_aeb_dropout_demo.py --scenario 1549178 --aeb on --drop-prob 0.1
python3 simulation/run_comparison.py --scenario 1549178 --seeds 42
```

单次 sensor/AEB 运行可加 `--headless`。批比较默认无窗口。
原始数据放 `Data/`，生成场景放 `simulation/generated/`，结果默认放
`~/TME180-local/results/data-scenarios/`。

纯原始回放及完整轨迹检查：`python3 simulation/run_scenarios.py`。
原始回放禁用自定义控制器；AEB 场景使用 ExternalController，不运行 HiDriveController。

见 [代码与 GitHub 指南](../docs/运行与GitHub指南.md)、
[输出字段](../docs/输出字段.md)、[验证记录](../docs/验证记录.md)。
