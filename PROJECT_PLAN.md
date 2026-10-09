# Project plan: esmini perception pipeline

Shared working file for Danilo, ChatGPT (architect and reviewer) and Claude Code (code editor).
Danilo approves the direction. Claude reads this file before working and updates it after each task.

- **Branch:** `danilo-esmini-perception` (on GitHub: `danilorar/perception_uncertainty`)
- **Code:** `experiments/esmini-perception/simulation/acc_cpp/`
- **Last updated:** 2026-10-09

## Goal

Simplify the existing pipeline while preserving its behavior.

**Agreed flow (one native C++ implementation):**
ideal sensor → Markov perception → native controller bridge (ACC or AEB) → vehicle update → esmini

| Stage | File (in `acc_cpp/src/`) |
|---|---|
| Ideal sensor, truth, report ego to esmini | `esmini_adapter.cpp` |
| Markov / i.i.d. dropout | `perception.cpp` |
| Controller wrappers (our types → C structs) | `esmini_acc.cpp`, `esmini_aeb.cpp` |
| Bridges compiled into esminiLib (call upstream ACC/AEB) | `native_acc_bridge.cpp`, `native_aeb_bridge.cpp` |
| Actuator limits and kinematics | `vehicle.cpp` |
| Truth metrics | `metrics.cpp` |
| Run loop and output files | `main.cpp` |

## Working rules

- One small change at a time. Every task ends with validation and a short report for review.
- Behavior-preserving changes are proven with **byte-identical outputs before vs. after** (see Validation).
- Commit each completed change separately; push to the branch.
- Keep ACC and AEB working. Keep `OSC-NCAP-scenarios/`, `Data/` and `reports/`.

## Done

| Commit | Change |
|---|---|
| `3792c21` | `MarkovDropout::apply()` readability (validation / state update / filtering sections) |
| `e7c3162` | Removed the obsolete Python AEB pipeline; `replay_perception.py` owns its esmini `State` struct |
| `f19bff9` | `main.cpp` `run()` readable: labeled stages, extracted CSV writers |
| `f89a64f` | New per-step master output `results.csv` (schema in `docs/output-schema.md`) |
| `e4234f9` | `--detailed-logs true|false` (default false); batch tools request detailed logs explicitly |

## Current state

- A plain `acc_sim` run writes `results.csv`, `run_config.json`, `summary.json`, `sim.dat`, `run.log`.
- `truth.csv`, `perceptions.csv`, `control.csv`, `metrics.csv` and `dropout_states.csv` are written only with `--detailed-logs true`.
- `run_comparison.py` and `run_markov_sensitivity.py` always pass `--detailed-logs true`: target-truth checks, `--validate` and replay need those files. Don't weaken the sensitivity tool's target-truth checks to save files.
- `results.csv` rules: empty cell = unavailable or non-finite (empty TTC means no closing target, not zero). `selected_target_*` is what the controller believes; `truth_min_*` is ground truth over all targets and may be a different object. The last row records truth only, with no new command.

## Next tasks (in order)

1. **Plotting script** (approved for after the cleanup). First figures per run: ego speed, gap (perceived selected-target gap vs. truth min gap), requested/applied acceleration, and detection state (`observed_detections` / `dropped_detections`, using only rows with `sensor_refreshed=1` for totals). Read `results.csv` only. matplotlib/pandas are already in the root `requirements.txt`.
2. **Converter extraction** (paused). `esmini_acc.cpp` (two `packet` overloads) and `esmini_aeb.cpp` (`values<T>`) duplicate the same 21-field conversion to `ACCSIM_Object`. Proposal: one shared `to_bridge_object(x, id)` template. Library boundary and behavior unchanged.
3. **Bridge-side duplication** (later). `copy_vehicle` / `fill` and `precise()` are duplicated in the two bridge files inside esminiLib.

## Open questions

- Should `control.csv` and `metrics.csv` eventually be removed? `results.csv` covers them, but replay still reads `control.csv`.
- The README says only macOS was tested locally. Linux (Ubuntu 24.04, g++ 13.3, CMake 3.28, Python 3.12) now has a passing build, tests, comparison, sensitivity and replay check. Update the README line?
- `experiments/esmini-perception/.gitignore` still lists old Python demo folders (`/simulation/generated/`, `/simulation/results/`). They're harmless; remove them?
- CI (`acc_cpp/ci/native-acc.yml`) is a template and inactive until copied to `.github/workflows/`.

## Setting up on a new machine

```bash
git clone https://github.com/danilorar/perception_uncertainty.git
cd perception_uncertainty
git checkout danilo-esmini-perception
cd experiments/esmini-perception/simulation/acc_cpp
python3 tools/build_native.py          # fetches pinned esmini v3.8.1, builds, runs 7 CTests
```

Needs Git, CMake ≥ 3.21, a C++17 compiler and Python ≥ 3.9. The first build needs network access. `build/` and `results/` are not in git, so expect to rebuild and rerun.

**Quick check that everything works:**

```bash
python3 tools/run_comparison.py --binary build/runner/acc_sim \
  --scenario ../../Data/C_original_1549178.xosc --controller aeb \
  --dropout-model markov --dropout-p 0.1 --mean-missing 0.5 --seeds 10 --validate
```

Expected for seed 10: ideal has no collision; Markov collides at about 4.57 s with 33/92 detections dropped. These numbers match `reports/aeb_demo_1549178_seed10_20261008/comparison.csv` up to the last 1–2 digits.

**Replay with the viewer** needs a graphics-enabled esmini v3.8.1 with `bin/libesminiLib.*` and `resources/`. The repository-root `esmini/` submodule is source only and not built. Options:

- Build it with the viewer (`USE_OSG=ON`).
- Or download the v3.8.1 release package and pass `--esmini-home /path/to/it`.

`build/engine-home` has no viewer and can't be used for replay. Single `acc_sim` runs need `--detailed-logs true` to be replayable.

## Validation recipe (for behavior-preserving changes)

1. Before editing, save a copy of the current `build/runner/acc_sim`.
2. Run the same cases with the old and the new binary, scenario `C_original_1549178`, seed 10, dt 0.01, sensor period 0.05, duration 10:
   - AEB + Markov p=0.1, mean missing 0.5
   - ACC + Markov p=0.1, mean missing 0.5, time gap 1.5
   - AEB + i.i.d. p=0.1
   - AEB ideal (p=0)
   - AEB + Markov p=0.1, `--stop-on-collision false`
   - ACC + Markov p=0.3, mean missing 1.0, `--stop-on-collision false`

   Add `--detailed-logs true` when the detailed files must be compared.
3. Compare every output file. After replacing the output-folder path:
   - CSVs, `summary.json` and `sim.dat` must be byte-identical;
   - `run.log` differs only in its timestamp line.
4. Run CTest, `run_comparison --validate` for AEB and ACC, and, if tools changed, a reduced `run_markov_sensitivity --validate --seed-count 2`.

**Checks on `results.csv`** (they were done with a temporary script; recreate it if needed):

- row count equals the `metrics.csv` row count, and `control.csv` has one row fewer;
- truth columns match `metrics.csv`, with `inf` written as empty;
- ego columns match the `truth.csv` ego rows;
- command columns match `control.csv`;
- frame counts match `perceptions.csv` for each sequence;
- counts repeat between refreshes, and each refresh advances the sequence by 1 with age 0;
- the last row has truth but no command;
- `ego_progress_m` steps match constant-acceleration kinematics.
