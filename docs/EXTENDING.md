# Extending the pipeline

How to add an error plugin, a controller or a scenario. For setup and running, see the
[README](../README.md).

## Add an error plugin

1. Copy `pipeline/errors/template.py` to `pipeline/errors/my_error.py`.
2. Change `SETTINGS` and `perceive`.
3. Use `"errors": [{"type": "my_error", ...}]`. No compile is needed, and `python -m pipeline list` shows the new type.

```python
import random

SETTINGS = {'sigma_m': 0.0}        # every setting with its default
RANGES = {'sigma_m': (0, 50)}      # optional: allowed range of number settings
LOG = ('error_m',)                 # optional: columns in perception_applied.tsv

class Error:
    def __init__(self, settings, run):           # once per run
        self.sigma = settings['sigma_m']
        self.random = {t['name']: random.Random(f'{run.seed}:{run.label}:{t["id"]}') for t in run.targets}

    def perceive(self, step, t, ego, targets):   # before every controller step
        for target in targets:
            error = self.random[target['name']].gauss(0.0, self.sigma)
            target['range'] += error
            target['log']['error_m'] = error
```

`run` has `seed`, `dt_s`, `steps`, `duration_s`, `scenario`, `ego`, `targets` and `label`. `ego` holds the true state of the
ego. Each target is a dictionary:

| Key | Meaning |
|---|---|
| `true`, `true_range` | True state and true centre-to-centre range. Read only |
| `range` | Range the controller gets. The target moves along the line of sight from the ego |
| `x`, `y` | Centre of the target box. If a plugin changes them, they win over `range` |
| `heading`, `speed`, `vx`, `vy`, `vz`, `ax`, `ay`, `az`, `length`, `width` | Other values the controller gets |
| `seen` | `False` hides the target for this step |
| `class_correct`, `predicted` | Flags shared between plugins: `identification` sets the first, `detection` the second |
| `log` | Values for `perception_applied.tsv`, under the names in `LOG` |

Optional: `CHOICES` for text settings, `check(settings, run)` to reject bad combinations with `ValueError`, and
`describe()` to save a description in `summary.json`. Use a random stream per plugin and target, seeded as above: the
same seed then gives the same errors, and adding another plugin or target does not change them. A plugin cannot add
objects that are not in the scenario.

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

## Add a scenario

Put the `.xosc` and its `.xodr` (same name) in `pipeline/scenarios/data/` and add a line to
`pipeline/scenarios/scenarios.json`. The `.xosc` needs:

- an ego (default `object_1`) with an `ObjectController` and an `ActivateControllerAction` in `Init`;
- a storyboard `StopTrigger` with a `SimulationTimeCondition`.

The teacher's TASC case files lack both. `python -m pipeline import-tasc <folder>` adds them (HiDrive properties as
in the shared files, a 10 s stop), makes the striking vehicle `object_1`, skips cases whose boxes overlap from the
start, and writes the result to `pipeline/scenarios/local/`, which is not in Git. See `pipeline/scenarios/tasc.py`.
