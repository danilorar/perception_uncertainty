# TME180 — ideal perception, detection dropout and AEB

Author: **Zhuo Ma**

This project replays three supplied rear-end scenarios in esmini and studies how
missing detections affect a simple external AEB controller. The original dataset
credits CAP unit, Vehicle Safety Division, Chalmers University of Technology;
its original file headers and contents are preserved.

中文逐步说明：[运行、代码结构与 GitHub 上传](docs/运行与GitHub指南.md)。

## Files

```text
Data/                              Original .xosc/.xodr pairs; read only
simulation/
  my_sensor_demo.py                Ideal sensor on original trajectory replay
  my_aeb_demo.py                   Ideal perception + AEB off/on
  my_aeb_dropout_demo.py           Detection dropout + AEB off/on
  esmini_api.py                    Native API, State, object sensor and camera
  sensor_model.py                  Reproducible per-object/per-tick dropout
  aeb_controller.py                TTC logic and ego path integration
  aeb_runner.py                    Shared ideal/dropout experiment loop
  generate_scenarios.py            Original data -> external-AEB scenario copies
  generated/                      Generated XOSC files and provenance manifest
  run_comparison.py               Four matched conditions, one or more seeds
  compare_aeb_dropout.py           Summarize existing run directories
  paths.py                        Portable paths and environment overrides
  my_run_one.py / run_scenarios.py Original replay and data validation
  render_replays.py                Optional native replay rendering (Pillow)
tests/                            Tests without native simulator dependencies
docs/                             Usage, output fields and validation record
```

## Setup

- Python **3.9 or later**; the core workflow uses the Python standard library.
- esmini **3.8.1**, using its double-precision C ABI. Install the full official
  [demo package](https://github.com/esmini/esmini/releases/tag/v3.8.1), including
  `bin/` and `resources/`, under `simulation/esmini-demo/`.
- Alternatively, set `ESMINI_HOME` to your esmini installation directory.
- The validated platform is macOS. Other platforms require matching esmini and
  Python architectures; this project does not claim they have been tested.
- Only `render_replays.py` needs the optional Pillow package.

Run commands from the repository root:

```bash
# Rebuild the three generated scenario copies; original data is never rewritten.
python3 simulation/generate_scenarios.py --scenario all --overwrite

# View the ideal sensor while replaying the original scenario.
python3 simulation/my_sensor_demo.py --scenario 1549178

# Ideal perception, AEB enabled. Add --headless to skip the viewer.
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb on

# 10% dropout, AEB enabled, reproducible seed.
python3 simulation/my_aeb_dropout_demo.py --scenario 1549178 --aeb on --drop-prob 0.1 --perception-seed 42

# AEB off/on x dropout 0/0.1; writes one comparison.csv.
python3 simulation/run_comparison.py --scenario 1549178 --seeds 42

# Test the models and scenario transformation without starting esmini.
python3 -m unittest discover -s tests -v
```

Replace `1549178` with `1554254` or `1554431` to select another supplied case.
Run any entry with `--help` for its parameters. Single-run AEB entries default to
**AEB off**. The ideal entry fixes dropout at zero; the dropout entry defaults to
**0.1**, which can be explicitly changed with `--drop-prob`.

## Inputs, observations and limitations

`SE_FetchSensorObjectList` returns a count and object IDs. Position, size, speed,
type and category are read separately through `SE_GetObjectState` as ground
truth. The controller receives target properties only while that target remains
detected after dropout. Truth is retained separately for evaluation.

The ideal sensor is an object-level geometric visibility model. This experiment
does not implement camera/radar/lidar physics, sensor fusion, classification
uncertainty, false positives or tracking. The dropout probability is per object
per simulation update, not a guaranteed fraction of samples in a finite run.

The AEB controller uses an approximate longitudinal TTC for near-parallel
rear-end motion. It follows the ego's original spatial path under braking; the
target continues its prescribed trajectory. Braking remains latched after the
first trigger, including during later dropouts. It is a simulation teaching
model, not a production AEB or a real-road safety validation.

## Results and sharing

Results default to `~/TME180-local/results/data-scenarios/`, outside OneDrive.
Override with `--results-dir` or `TME180_RESULTS`. Each run creates a new folder;
it does not overwrite previous experiments. See [output definitions](docs/输出字段.md).

Track the original data, generated XOSC/manifest, scripts, tests and documentation.
The `.gitignore` excludes local simulator downloads, environments, results and
old experiment backups. `project-files.txt` explicitly lists the files to copy
when importing this workflow into an existing team repository.

Dataset availability is distinct from script authorship: the supplied data has
no redistribution license in this checkout. Use the data provider's agreed
sharing scope for both original and derived scenarios; no license is inferred here.
