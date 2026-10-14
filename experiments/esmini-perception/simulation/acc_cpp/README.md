# Native ACC simulation
The everyday entry point is [run_single_acc.ipynb](run_single_acc.ipynb). Python chooses settings and launches `acc_sim`; simulation and controller execution stay in C++. AEB and the custom movement path have been retired. Optional ideal detection uses esmini’s existing `ObjectSensor`. Optional persistent Markov dropout filters ideal detections. Gaussian is not connected. Live animation is optional.

## Build once, then use the notebook

From this directory:

```bash
python3 tools/build_native.py
```

Requires Git, a C++17 compiler, Python ≥3.9 and CMake ≥3.21. The first build needs network access. The build downloads esmini v3.8.1 commit `19d26b68f7f473de4e55046a794b5fd2f91c58c8` and fmt, verifies upstream ACC source is unchanged, and compiles the integration into esminiLib (headless by default). No root esmini submodule is required. Rebuild after C++ changes. The project bridge ABI is now **7**; rebuild older installations.

Open `run_single_acc.ipynb`, edit the settings cells and run the cells in order:

Choose `SCENARIO_MODE="recorded"` for trajectory-based `C_original` files, or `"ncap_2023"` for the supported NCAP CCR base file. Recorded mode uses trajectory-defined motion and ignores the NCAP speed/braking settings; NCAP mode applies those overrides and the ego-speed sweep. Switching modes requires no commented command arguments.

1. Set `ACC_CPP_DIR` to this directory if the kernel starts elsewhere.
2. Choose scenario, timestep, time gap, optional cruise speed and stopping/logging settings. Leave `NATIVE_REFERENCE=False` for target copies. Choose `IDEAL_SENSOR_ENABLED=True` for sensor filtering and set the sensor mount, range and horizontal FOV in the settings cells.
3. Choose `EGO_SPEEDS_KPH` (for the NCAP ego-speed parameter) and `MARKOV_SEEDS`. The launch cell runs one ideal baseline and every Markov seed at each ego speed using `subprocess`. Recorded `C_original` scenarios do not support the NCAP speed override. Each gets a subfolder inside a unique `results/acc_batch_*` directory. Use `MARKOV_SEEDS=[]` for only the ideal baseline.
4. Load the saved results and summaries, then compare ego speed, actual gap and missed detections. Plots require `DETAILED_LOGS=True`; the target-speed line assumes target ID 1. Runs may end at different times.

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
| `src/controllers/acc.cpp` | Connect independent observations to persistent native ACC | Native ObjectSensor hits (or all targets) → copies, native ACC call, diagnostics |
| Upstream `ControllerACC.cpp` | Native ACC algorithm and override ego movement | Real ego, target view and settings → speed and road position |
| `src/metrics.cpp` | Truth-based safety measurements | World states → gap, TTC, distance and collision |
| `src/perception.cpp` | Shared dropout mechanisms; Markov connected to ACC | Ideal detection IDs → retained IDs and dropped IDs |
| `include/accsim/` | Shared types and scenario/engine/perception/metrics declarations | C++ interfaces |
| `include/native_acc_bridge.h` | ACC registration, sensor configuration and diagnostic C boundary | Runner ↔ esminiLib |
| `tools/build_native.py`, `tools/add_bridge.cmake` | Build pinned engine, integration, runner and tests | Source → local binaries and provenance |
| `tools/compare_native_acc.py` | Validate copied targets against native live targets | Runner → exact comparison report |

```text
Notebook → main.cpp → scenario.cpp → generated XOSC
                 → esmini_adapter.cpp → SE_StepDT(dt)
                   → controllers/acc.cpp → upstream ControllerACC::Step(dt)
                 ← world truth + ACC diagnostics → metrics + saved results
```

There is one ACC integration source and no custom ego integration. `PerceptionACC` and `ReferenceACC` use the same persistent class and upstream implementation: the former substitutes independent target copies, the latter retains live native entities for validation. `PerceptionACC` receives either all non-ego targets or native `ObjectSensor` detections, in original scenario order. Copies preserve native road/lane coordinates. Relative-position links/types are rejected rather than retained in snapshots.

The recorded-trajectory preparer supports the supplied flat, near-parallel two-vehicle rear-end cases. Action-based scenarios can use inline vehicles or catalog references, with ego selected by `--ego-name`. Their Init actions, target actions and native stop conditions are preserved. Catalogs/parameters are resolved by esmini, not a custom evaluator. Later ego maneuver groups, dynamic actor selection and existing controller actions are currently rejected for explicit review. Referenced maneuver catalogs are validated before use: only `GlobalAction/VariableAction/SetAction` observer actions are supported. Other actions, nested references, missing/duplicate entries, parameterized names and ambiguous catalog files are rejected. Catalogs must resolve from declared directories; unchecked fallback resources are not used. esmini still evaluates parameters and executes the catalog. This is not unrestricted OpenSCENARIO support.

The NCAP base file can now run through the same ACC pipeline:

```bash
build/runner/acc_sim --scenario ../../Data/OSC-NCAP-scenarios/OpenSCENARIO/NCAP/AEB_C2C_2023/NCAP_AEB_C2C_CCR_2023.xosc --ego-name Ego --output-dir results/ncap_base --duration 5 --ideal-sensor true --detailed-logs true
```

Use a new output directory each time. This base file's current defaults initialize both cars at 20 km/h and enable delayed target braking; it is not the CCRm variation. Parameter-distribution files in `Variations/` are not supported by the preparer yet.

Optional arguments `--ego-speed-kph`, `--target-speed-kph` and `--target-braking true|false` override NCAP `Ego_speed_kph`, `GVT_init_speed_kph` and `isCCRbraking`. Speeds must be nonnegative. Omitted values retain XOSC defaults; unsupported parameter names/types fail explicitly rather than being ignored. The adapter applies them with esmini's parameter-declaration callback, lets esmini re-evaluate dependent expressions before Init, then clears the callback. `run_config.json` records requested overrides under `scenario_parameter_overrides` (`null` means omitted).

For a moving target setup, add `--ego-speed-kph 50 --target-speed-kph 20 --target-braking false`. For a stationary target, use `--target-speed-kph 0 --target-braking false`. For braking, enable `--target-braking true`; the scenario still defines final target speed, deceleration and delay. These are parameterized runs of the base XOSC, not execution of the distribution files. In the notebook, append `"--ego-name", "Ego"` to the command when choosing this base file. The notebook launch can also pass the optional speed/braking arguments below.

For action-based inputs, omit `--set-speed` to let native ACC capture the resolved initial speed on activation. Explicit cruise speed still works. Resolved speed and vehicle limits are recorded after initialization at simulation time zero. Original Init and target relative placement are left to esmini; unresolved relative target positions are still rejected when snapshots are created.

## Ideal sensor timing and limits

The notebook enables ideal sensing; the runner default remains all-target copies so `compare_native_acc.py` retains its original validation. `--native-reference true` uses live targets and cannot be combined with `--ideal-sensor true`.

One owned native `ObjectSensor` updates immediately before each ACC step. Its hit pointers are used only to choose targets to snapshot; ACC receives independent copies, never the live hit pointers. No player-managed sensor or extra simulation step is added. Capacity covers every current entity and grows if needed because upstream does not enforce the hit buffer limit. Native zero-time controller initialization can also update the sensor; output rows describe executed physical timesteps only.

Defaults: mount `(0, 0, 0.5)` metres relative to ego, heading `0` radians, near/far range `0/100` metres, horizontal FOV `pi/3` radians (60 degrees). Settings and the update period (`dt`) are saved in `run_config.json`. The pinned implementation uses horizontal object-reference-point range/FOV, skips ghosts and sensor-invisible objects, and has no occlusion model. Its heading/FOV behaviour is reused unchanged; initial tests cover a forward-facing sensor. This is ideal detection, not a realistic camera/radar model.

## Markov dropout

The notebook runs Markov for each entry in `MARKOV_SEEDS`; keep `IDEAL_SENSOR_ENABLED=True` and `NATIVE_REFERENCE=False`. Choose `MARKOV_MISS_PROBABILITY` and `MARKOV_MEAN_MISSING_SECONDS`. The ideal baseline always sends zero dropout. Example CLI addition: `--perception markov --ideal-sensor true --dropout-p .1 --mean-missing-s .5 --seed 10`.

Each persistent ACC owns one existing `MarkovDropout` instance, with an independent chain for every initial non-ego ID. All chains advance even outside sensor visibility; only ideal detections can be supplied or dropped. IDs and order are filtered before copying native target positions. New target IDs during a run are rejected rather than silently restarting chains. Real targets are never altered.

The sensor/Markov period is the simulation timestep. Initialization at time zero samples the stationary missing state without a transition; repeated zero-time samples are cached. Every subsequent dt advances once. Probability must be in [0,1], mean missing duration at least dt, and transition probabilities valid. Missing periods are geometric, not fixed-length. p=0 is identity; p=1 is total dropout. The same seed reproduces results for identical run settings. Reference/all-target validation modes remain available; Markov requires sensor-filtered target-copy mode.

`detections.csv` separates ideal `detected_target_ids`, Markov `dropped_target_ids` and `supplied_target_ids`. The supplied list equals the detected list minus dropped IDs, in original order. `native_acc.csv` target_count counts supplied targets. Settings and stationary initialization policy are recorded in `run_config.json`. Historical output files retain their older schemas.

## Settings and outputs

The runner is ACC-only. `--controller acc` is accepted for explicit commands; AEB and its settings are rejected. Supported settings are `--ego-name` (default `object_1`), `--scenario`, `--output-dir`, `--config`, `--dt`, `--duration`, `--time-gap`, `--set-speed`, `--esmini-home`, `--native-reference`, `--stop-on-collision` `--detailed-logs` and `--viewer`. `--perception ideal|markov` selects the observation model (default ideal). Nonzero `--dropout-p` requires Markov. Ideal sensing uses `--ideal-sensor true`, `--sensor-x-m`, `--sensor-y-m`, `--sensor-z-m`, `--sensor-heading-rad`, `--sensor-near-m`, `--sensor-far-m` and `--sensor-horizontal-fov-rad`. Markov accepts `--mean-missing-s` (default 0.5 s) and `--seed` (default 0). `--esmini-home` supplies optional resources, not a replacement engine library.

Defaults: `dt=0.01 s`, end at the scenario’s own stopping condition, time gap `1.5 s`, cruise speed from the scenario's initial ego speed, stop on collision enabled, detailed logs disabled. `--duration` is an optional upper limit and must be a positive integer multiple of `dt`. Without it, esmini decides when the scenario ends; the viewer closes when the run finishes. A limit longer than the scenario is allowed. `run_config.json` records `duration_s: null` when no override is set. Config paths are relative to the config file; CLI paths are relative to the current directory. Output directories must be new or empty. `configs/ideal.ini` remains a valid optional ACC configuration.

Each run writes:

- `results.csv`: boundary ego truth, progress and truth safety metrics. Recorded inputs use reference-path projection; action-based inputs use accumulated planar ego travel. `progress_source` in run_config identifies the definition.
- `native_acc.csv`: controller-time input speed, post-step ego speed/position, selected lead, supplied target count and diagnostic gap.
- `detections.csv`: one row per ACC step with step, controller time, observation mode and JSON lists of detected, dropped and supplied target IDs (including empty lists).
- `summary.json`: collision, safety, deceleration and final-speed outcomes.
- `run_config.json`: resolved settings, source fingerprints and pinned engine identity (schema 7).
- `generated/`: prepared XOSC and road snapshot; `run.log` and `sim.dat`: esmini logs/recording.
- With detailed logs: `truth.csv` and `metrics.csv`.

The old empty control/dropout files and unused results/summary fields are no longer emitted. See [output schema](../../docs/output-schema.md). Existing results and reports are historical and remain untouched.

## Validation and retained tools

```bash
ctest --test-dir build/runner --output-on-failure
python3 tools/compare_native_acc.py --output-dir results/native_acc_identity
```

Choose a new comparison directory each time. On Windows also pass `--binary build/runner/Release/acc_sim.exe` and use `ctest -C Release`.

Sensor fixtures additionally check near/far range, rear/FOV exclusion, detection order, current-step timing and all-visible speed/position/selection identity. Seven CTests cover persistent ACC, shared perception, Markov, metrics and preparation of three supplied scenarios. ACC fixtures check cruise across steps, following calculations, nearest-target selection, rear/lateral/lookahead exclusion, bounding-box offsets/headings, speed/deceleration limits, close-gap stopping and native movement. The NCAP test additionally verifies catalog initialization, native cruise capture at time zero, parameter re-evaluation, moving/stationary/braking target setups, exact copy/ideal/reference identity, and callback cleanup after successful and failed runs. Persistent Markov integration tests additionally verify p=0 identity, p=1 exclusion, same-seed reproduction, recovery, zero-time caching, per-timestep updates and unchanged target truth. The comparison runs three scenarios with three setting combinations and requires byte-identical native diagnostics, truth, results and summaries between copy and reference modes.

The obsolete ACC packet wrapper, AEB bridges/test and custom movement code were removed. The dropout comparison, Markov sensitivity and perception replay tools were removed because their workflows require the retired path. The historical batch exporter was also removed because no supported ACC workflow uses it. [Reports](reports/README.md) may contain historical commands that no longer exist.

Upstream [ACC source](https://github.com/esmini/esmini/blob/v3.8.1/EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp) is MPL 2.0. Included pugixml 1.14 retains its MIT license. Original data and third-party attribution are preserved.

## Replay a batch together

Viewer builds (`python3 tools/build_native.py --viewer`) also build esmini’s `replayer`. After running a notebook batch, use its optional replay cell. `tools/replay_batch.py` copies each saved `sim.dat` into a uniquely named replay folder inside the batch, adds generated-road and model search paths, and opens one overlay window. It prints the recording-to-run mapping. Tab switches vehicles; Space pauses; Esc closes. The replay closes at the end. This plays saved movements without executing controllers again; overlapping vehicles are expected.
