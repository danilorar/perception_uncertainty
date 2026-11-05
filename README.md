# Perception errors on esmini

This pipeline runs a rear-end scenario in [esmini](https://github.com/esmini/esmini) with an AEB controller,
adds a range error to what the controller sees, and saves what the car did. The controller reacts to the error, so
the loop is closed.

Error values are assumed, so results show sensitivity, not measured sensor risk.

Works on Windows and macOS. The default controller is HiDrive.

## Setup

You need Python 3.11 or newer, about 3 GB of disk, and a C++ compiler:

- **Windows:** Visual Studio with "Desktop development with C++" (it includes CMake).
- **macOS:** run `xcode-select --install` and `brew install cmake`.

```bash
git clone -b pipeline https://github.com/danilorar/perception_uncertainty.git
cd perception_uncertainty
```

Create and activate a Python environment.

Windows:

```bat
py -m venv .venv
.venv\Scripts\activate
```

macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Then, on both:

```bash
pip install -r pipeline/requirements.txt
python -m pipeline setup
python -m unittest discover tests
```

`setup` downloads esmini 3.8.1 and compiles it with the controllers. The first time takes 10-30 minutes.
Run it again whenever a controller or `pipeline/engine/PerceptionNoise.hpp` changes; a run refuses to start on a stale
build.

## Run one simulation

```bash
python -m pipeline run configs/lost_target.json --output output/my_run
python -m pipeline replay output/my_run
python -m pipeline list
```

Without `--output`, results go to a new folder in `output/runs/`. `replay` opens the run in the esmini viewer. `list` shows
the available controllers, error models and scenarios.

A configuration lists only what differs from the defaults in `pipeline/config.py`:

```json
{"scenario": "gidas_1549178", "uncertainty": "distance_ou_range", "noise_reference_mae_pct": 5.0, "seed": 0}
```

| Result file | Content |
|---|---|
| `summary.json` | Collision, impact speed, trigger and brake times, smallest gap and TTC, status |
| `ego.csv` | Ego speed, position and controller acceleration per step |
| `observations.csv` | True and observed range per step and target |
| `truth.csv` | True state of every vehicle per step |
| `metrics.csv` | Distance, gap and TTC per step. These use the truth; the controller never sees them |
| `speed.png`, `range.png` | Plots of this run |
| `config.json`, `versions.json` | Full settings and versions; `config.json` can be run again |
| `perception_noise.tsv`, `perception_applied.tsv` | Error samples given to esmini, and errors it applied |
| `esmini.log`, `esmini.csv`, `replay.dat`, `scene.xosc` | Raw esmini input and output |

## Run a grid

```bash
python -m pipeline grid experiments/smoke.json
python -m pipeline grid experiments/range_factors.json --list
python -m pipeline grid experiments/range_factors.json --workers 16
```

Results go to `output/grids/<name>/`: `results.csv` has one row per run, `summary.csv` one row per scenario and level,
`failed.txt` lists failed runs. A stopped grid continues where it stopped. A grid keeps compressed tables only; it does not
keep plots or replay files.

```json
{
  "name": "my_grid",
  "scenarios": ["gidas_1549178"],
  "base": {"uncertainty": "range_factors", "range_factors.miss_rate": 0.1},
  "groups": [
    {"name": "markov", "vary": {"range_factors.miss_mean_s": [0.2, 0.5, 1.0]}, "seeds": 50},
    {"name": "iid", "set": {"range_factors.miss_model": "iid"}, "seeds": {"first": 50, "count": 50}}
  ]
}
```

Every combination of the `vary` values runs on every scenario with every seed. Without `scenarios`, all scenarios run.
`seeds` is a count (from 0), a range, or a list like `[1, 2]`; without it, seed 0 runs.

| Grid in `experiments/` | Question | Runs |
|---|---|---|
| `smoke` | Does the pipeline work? | 6 |
| `ou_tau` | OU error: size against correlation time | 1803 |
| `range_factors` | Size, Identification, Detection and OU, each alone | 1899 |
| `detection_seeds_50_99` | More Detection seeds for `range_factors` | 600 |
| `lost_target` | What the car receives while the target is lost | 3000 |
| `ou_size` | How large must the OU error be before safety changes? | 282 |

Tables from `range_factors`, `lost_target` and `ou_size` are in `results/`.

## Error models

Set with `"uncertainty"`:

| Value | Meaning | Settings |
|---|---|---|
| `recorded_truth` | No error | |
| `distance_ou_range` | Correlated error that grows with distance | `noise_reference_mae_pct` (mean error at `noise_reference_range_m`), `noise_correlation_time_s`, `noise_zero_range_m` |
| `range_factors` | Size, Identification, Detection, lost target, optional OU | `range_factors`: see the comments in `pipeline/errors/factors.py` |

How it works: before each controller step, esmini moves the targets to where the controller should perceive them, and
puts them back right after. Collisions, logs and replays therefore stay true, and any controller that reads target
positions from esmini gets the error with no extra code.

## Add a controller

1. Copy `pipeline/controllers/example_aeb` to `pipeline/controllers/my_aeb`, then rename the files and the class, and
   give it its own number in `GetType`.
2. Edit `controller.json`:

   | Field | Meaning |
   |---|---|
   | `esmini_name` | Name registered in esmini and written into the scene |
   | `sources`, `header`, `factory` | Files to compile, the header, and the function that creates the controller |
   | `properties` | Controller properties in the scene. `null` keeps those in the scenario file; an object replaces them |
   | `signals` | Names of the two `LOG_VALUE` signals below |

3. Run `python -m pipeline setup`, then use `{"controller": "my_aeb"}` in a configuration.

The controller must read targets from `entities_->object_` inside `Step`, and log these two values each step, or trigger
and brake times stay empty:

```cpp
LOG_INFO("LOG_VALUE:AEB_active:{}", braking ? 1 : 0);
LOG_INFO("LOG_VALUE:Acceleration:{}", acceleration);
```

In the scenarios the ego starts with speed 0 and follows a recorded trajectory. With `"mode": "override"` that trajectory is
ignored and the controller moves the car itself, as `example_aeb` does.

## Add an error model

1. Copy `pipeline/errors/ou.py` to `pipeline/errors/my_model.py`. `pipeline/errors/__init__.py` lists the functions it needs.
2. Add it to `MODELS` in that file, and add any new settings to `DEFAULT` in `pipeline/config.py`.
3. Use `{"uncertainty": "my_model"}` in a configuration.

No compile is needed if the model only produces the existing inputs: a unit error `z`, `missed` and `wrong` flags per step,
and fixed scale and bias. Anything else also needs a change in `pipeline/engine/PerceptionNoise.hpp` and another `setup`.

## Add a scenario

Put the `.xosc` and its `.xodr` (same name) in `pipeline/scenarios/data/` and add a line to
`pipeline/scenarios/scenarios.json`. The `.xosc` needs:

- an ego (default `object_1`) with an `ObjectController` and an `ActivateControllerAction` in `Init`;
- a storyboard `StopTrigger` with a `SimulationTimeCondition`.

## Files

```
pipeline/
  __main__.py       command line
  config.py         settings and checks
  run.py            one simulation
  grid.py           many simulations
  results.py        result tables and metrics
  plots.py          plots of one run
  controllers/      one folder per controller
  errors/           error models
  scenarios/        scenario files
  engine/           esmini build and error hook
configs/            example configurations
experiments/        grids
results/            tables from earlier grids
tests/              python -m unittest discover tests
```

Not used by the pipeline, and left as they are on `main`: `OSC-NCAP-scenarios/`, `PerceptionModels.pdf`, `esmini`,
`requirements.txt`. Created locally and not in Git: `runtime/` (compiled esmini), `output/` (results).

## Good to know

- `gidas_*` are the three scenario IDs. The data are TASC data, even where a file says GIDAS.
- The course does not allow a Kalman filter, so controllers get the observations directly.
- The same seed gives the same result on the same machine. Windows and macOS may differ slightly.
- If a Windows build fails on long paths, set `TME180_RUNTIME` to a short folder, such as `C:\t`, and run `setup` again.
- AI agents must not read the HiDrive sources: see `pipeline/controllers/hidrive/AGENTS.md`.
