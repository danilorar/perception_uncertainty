# Native ACC simulation
The everyday entry point is [run_single_acc.ipynb](run_single_acc.ipynb). Python chooses settings and launches `acc_sim`; simulation and controller execution stay in C++. AEB and the custom movement path have been retired. No sensor filtering or Markov/Gaussian integration is connected to ACC yet. Live animation is optional.

## Build once, then use the notebook

From this directory:

```bash
python3 tools/build_native.py
```

Requires Git, a C++17 compiler, Python ≥3.9 and CMake ≥3.21. The first build needs network access. The build downloads esmini v3.8.1 commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8` and fmt, verifies upstream ACC source is unchanged, and compiles the integration into esminiLib (headless by default). No root esmini submodule is required. Rebuild after C++ changes. The project bridge ABI is now **4**; rebuild older installations.

Open the notebook, edit the first cell and run the cells in order:

1. Set `ACC_CPP_DIR` to this directory if the kernel starts elsewhere.
2. Choose scenario, timestep, duration, time gap, optional cruise speed and stopping/logging settings. Leave `NATIVE_REFERENCE=False` for target copies.
3. Run the launch cell. It creates a unique directory under `results/` and calls `acc_sim` using `subprocess`.
4. Run the final cell to load `results.csv` and `summary.json`.

Linux/macOS binary: `build/runner/acc_sim`. On Windows, set the notebook's `BINARY_FILE` to `build/runner/Release/acc_sim.exe`.

## Optional live animation

Build graphics support once:

```bash
python3 tools/build_native.py --viewer
```

On Ubuntu, graphics builds also require the development packages `libgl-dev`, `libx11-dev`, `libxrandr-dev`, `libxinerama-dev`, `libfontconfig1-dev` and `zlib1g-dev` (including their dependencies). OSG and the vehicle-model package are downloaded using the pinned upstream dependency definitions. Vehicle assets and `model_ids.txt` are kept locally in `build/engine-home/resources/`, which esmini finds beside its library. A working graphical desktop/OpenGL display is required.

In the notebook set `VIEWER = True`, then rerun the settings and launch cells. The simulation opens a live esmini window and runs at approximately real-time speed. Closing the window ends the run. Set `VIEWER = False` for fast headless runs with the same graphics-enabled binary. From the CLI, use `--viewer true` or `--viewer false` (default false).

The viewer changes rendering and wall-clock pacing only; ACC and the fixed simulation timestep stay the same. The supplied ego and target use the downloaded white and red car models. Missing models fall back to box geometry. `--esmini-home` can supply a graphics installation's resource directory. Running `build_native.py` without `--viewer` builds a headless library again.

## Files and dependencies

| File | Responsibility | Input → output |
|---|---|---|
| `run_single_acc.ipynb` | Settings, launch and result loading | Settings → command, loaded CSV/JSON |
| `src/main.cpp` | Argument handling, run loop and all CSV/JSON logging | Arguments and truth → run outputs |
| `src/scenario.cpp` | Prepare XOSC and road snapshot; retain target trajectory | Original scenario and ACC settings → generated inputs |
| `src/esmini_adapter.cpp` | Register ACC, initialize esmini, step and read truth | Generated inputs, `dt` → native steps and world states |
| `src/controllers/acc.cpp` | Connect independent observations to persistent native ACC | Native target states → copies, native ACC call, diagnostics |
| Upstream `ControllerACC.cpp` | Native ACC algorithm and override ego movement | Real ego, target view and settings → speed and road position |
| `src/metrics.cpp` | Truth-based safety measurements | World states → gap, TTC, distance and collision |
| `src/perception.cpp` | Shared IID/Markov processing, retained for later connection | Observation frames → filtered frames; currently tested separately |
| `include/accsim/` | Shared types and scenario/engine/perception/metrics declarations | C++ interfaces |
| `include/native_acc_bridge.h` | ACC registration and diagnostic C boundary | Runner ↔ esminiLib |
| `tools/build_native.py`, `tools/add_bridge.cmake` | Build pinned engine, integration, runner and tests | Source → local binaries and provenance |
| `tools/compare_native_acc.py` | Validate copied targets against native live targets | Runner → exact comparison report |

```text
Notebook → main.cpp → scenario.cpp → generated XOSC
                 → esmini_adapter.cpp → SE_StepDT(dt)
                   → controllers/acc.cpp → upstream ControllerACC::Step(dt)
                 ← world truth + ACC diagnostics → metrics + saved results
```

There is one ACC integration source and no custom ego integration. `PerceptionACC` and `ReferenceACC` use the same persistent class and upstream implementation: the former substitutes independent target copies, the latter retains live native entities for validation. `PerceptionACC` is a future-facing name; currently it receives every non-ego target in original scenario order. Copies preserve native road/lane coordinates. Relative-position links/types are rejected rather than retained in snapshots.

The scenario preparer supports the supplied two-vehicle, flat, near-parallel rear-end cases. This is not yet a general scenario/controller framework.

## Settings and outputs

The runner is ACC-only. `--controller acc` is accepted for explicit commands; AEB and its settings are rejected. Supported settings are `--scenario`, `--output-dir`, `--config`, `--dt`, `--duration`, `--time-gap`, `--set-speed`, `--esmini-home`, `--native-reference`, `--stop-on-collision` `--detailed-logs` and `--viewer`. `--dropout-p` accepts only zero. Unused sensor/Markov settings have been removed from the CLI. `--esmini-home` supplies optional resources, not a replacement engine library.

Defaults: `dt=0.01 s`, duration `10 s`, time gap `1.5 s`, cruise speed from the scenario's initial ego speed, stop on collision enabled, detailed logs disabled. Duration must be an integer multiple of `dt`. Config paths are relative to the config file; CLI paths are relative to the current directory. Output directories must be new or empty. `configs/ideal.ini` remains a valid optional ACC configuration.

Each run writes:

- `results.csv`: boundary ego truth, reference-path progress and truth safety metrics.
- `native_acc.csv`: controller-time input speed, post-step ego speed/position, selected lead, supplied target count and diagnostic gap.
- `summary.json`: collision, safety, deceleration and final-speed outcomes.
- `run_config.json`: resolved settings, source fingerprints and pinned engine identity (schema 4).
- `generated/`: prepared XOSC and road snapshot; `run.log` and `sim.dat`: esmini logs/recording.
- With detailed logs: `truth.csv` and `metrics.csv`.

The old empty perception/control/dropout files and unused results/summary fields are no longer emitted. See [output schema](../../docs/output-schema.md). Existing results and reports are historical and remain untouched.

## Validation and retained tools

```bash
ctest --test-dir build/runner --output-on-failure
python3 tools/compare_native_acc.py --output-dir results/native_acc_identity
```

Choose a new comparison directory each time. On Windows also pass `--binary build/runner/Release/acc_sim.exe` and use `ctest -C Release`.

Six CTests cover persistent ACC, shared perception, Markov, metrics and preparation of three supplied scenarios. ACC fixtures check cruise across steps, following calculations, nearest-target selection, rear/lateral/lookahead exclusion, bounding-box offsets/headings, speed/deceleration limits, close-gap stopping and native movement. The comparison runs three scenarios with three setting combinations and requires byte-identical native diagnostics, truth, results and summaries between copy and reference modes.

The obsolete ACC packet wrapper, AEB bridges/test and custom movement code were removed. The dropout comparison, Markov sensitivity and perception replay tools were removed because their workflows require the retired path. The historical batch exporter was also removed because no supported ACC workflow uses it. [Reports](reports/README.md) may contain historical commands that no longer exist.

Upstream [ACC source](https://github.com/esmini/esmini/blob/v3.8.1/EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp) is MPL 2.0. Included pugixml 1.14 retains its MIT license. Original data and third-party attribution are preserved.
