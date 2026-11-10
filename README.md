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
git clone https://github.com/danilorar/perr-esmini.git
cd perr-esmini
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

`setup` downloads esmini 3.8.1 and about 1 GB of dependencies, then compiles it with the controllers. The first time takes a
few minutes. Run it again whenever a controller or `pipeline/engine/PerceptionBridge.hpp` changes; a run refuses to start on a
stale build. Error plugins are Python and need no compile. If a download fails, `setup` says which one; run it again
later and it downloads only what is missing.

## Run one simulation

```bash
python -m pipeline run configs/lost_target.json --output output/my_run
python -m pipeline replay output/my_run
python -m pipeline list
```

Without `--output`, results go to a new folder in `output/runs/`. `replay` opens the run in the esmini viewer. `list` shows
the available controllers, error plugins and scenarios.

A configuration lists only what differs from the defaults in `pipeline/config.py`:

```json
{"scenario": "gidas_1549178", "errors": [{"type": "ou", "mae_pct": 5.0}], "seed": 0}
```

| Result file | Content |
|---|---|
| `summary.json` | Collision, impact speed, trigger and brake times, smallest gap and TTC, status |
| `ego.csv` | Ego speed, position and controller acceleration per step |
| `observations.csv` | True and observed range per step and target |
| `truth.csv` | True state of every vehicle per step |
| `metrics.csv` | Distance, gap and TTC per step. These use the truth; the controller never sees them |
| `speed.png`, `range.png` | Plots of this run |
| `config.json`, `versions.json` | Full settings and versions; `config.json` can be run again. `versions.json` also has the Git commit and any files that differ from it |
| `perception_applied.tsv` | Per step: true range, range given to the controller, and the values each error plugin logs |
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

To compare results with someone else, run the same commit with no local changes. The last columns of `results.csv`
give each run's commit and how many files differed from it, and the grid warns at the end when that is not one clean
commit.

```json
{
  "name": "my_grid",
  "scenarios": ["gidas_1549178"],
  "base": {"errors": [{"type": "detection", "miss_rate": 0.1}]},
  "groups": [
    {"name": "markov", "vary": {"errors.detection.miss_mean_s": [0.2, 0.5, 1.0]}, "seeds": 50},
    {"name": "iid", "set": {"errors.detection.miss_model": "iid"}, "seeds": {"first": 50, "count": 50}}
  ]
}
```

Every combination of the `vary` values runs on every scenario with every seed. Without `scenarios`, all scenarios run.
A key like `errors.detection.miss_rate` changes one setting of the entry with that `name` or `type`; the entry must
already be in the list, from `base` or `set`. A `set` of `"errors"` replaces the whole list.
`seeds` is a count (from 0), a range, or a list like `[1, 2]`; without it, seed 0 runs.

Ready-made grids are in `experiments/`. Start with `smoke`, which checks that the pipeline works.

## Error plugins

Errors are set with `"errors"`, a list of plugins that run in that order on every controller step:

```json
"errors": [{"type": "size", "scale": 1.1}, {"type": "ou", "mae_pct": 1.4}, {"type": "detection", "miss_rate": 0.1}]
```

`type` is the file name in `pipeline/errors/`. The other keys are settings; a setting left out keeps its default. Each
plugin gets the result of the one before it, so the order matters. To use one type twice, give each entry a `name`.

| Type | What the controller gets | Settings (default) |
|---|---|---|
| `size` | range + (scale - 1) * true range + bias | `scale` (1.0), `bias_m` (0.0) |
| `identification` | in wrong-class frames: range + (wrong_scale - 1) * true range + wrong_bias; `size` skips those frames | `wrong_rate` (0), `wrong_mean_s` (0.5), `wrong_scale` (1.0), `wrong_bias_m` (0), `sensor_period_s` (0.05) |
| `ou` | range + a correlated error that grows with distance | `mae_pct` (5.0, mean error at `reference_range_m`), `correlation_time_s` (0.5), `zero_range_m` (3), `reference_range_m` (100) |
| `detection` | in lost frames: no target, or a predicted range | `miss_rate` (0), `miss_model` (`markov` or `iid`), `miss_mean_s` (0.5), `sensor_period_s` (0.05), `lost_target` (`vanish` or `predict`), `lost_target_motion` (`live` or `frozen`), `lost_target_distance` (`frozen_difference` or `frozen_object`) |
| `template` | range + white noise; a starting point for new plugins | `sigma_m` (0.0) |

Put `identification` before `size`, and `detection` last, so its prediction starts from the range with all other
errors. The docstring at the top of each file gives the details.

How it works: before each controller step, esmini sends the true state of every object to Python. The plugins change
what the controller should see, and esmini moves the targets there for that one step and puts them back right after.
Collisions, logs and replays therefore stay true, and any controller that reads targets from esmini gets the error with
no extra code. `perception_applied.tsv` records, per step, the true and given range and every value the plugins log.

## Extend the pipeline

To add an error plugin, a controller or a scenario, see [docs/EXTENDING.md](docs/EXTENDING.md).

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
  errors/           error plugins
  scenarios/        scenario files
  engine/           esmini build and the perception bridge
configs/            example configurations
experiments/        grids
tests/              python -m unittest discover tests
docs/               EXTENDING.md (add plugins, controllers, scenarios), EXPERIMENTS.md (grid log)
AGENTS.md           rules for AI agents (CLAUDE.md loads it for Claude Code)
```

Not used by the pipeline, and left as they are on `main`: `OSC-NCAP-scenarios/`, `PerceptionModels.pdf`, `esmini`,
`requirements.txt`. Created locally and not in Git: `runtime/` (compiled esmini), `output/` (results).

## Good to know

- `gidas_*` are the three scenario IDs. The data are TASC data, even where a file says GIDAS.
- The course does not allow a Kalman filter, so controllers get the observations directly.
- The same seed gives the same result on the same machine. Windows and macOS may differ slightly.
- macOS was tested on a cloud Mac with no display, so `replay` has not been tried there yet.
- If a Windows build fails on long paths, set `TME180_RUNTIME` to a short folder, such as `C:\t`, and run `setup` again.
- AI agents must not read the HiDrive sources: see `pipeline/controllers/hidrive/AGENTS.md`.
