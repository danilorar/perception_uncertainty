# Project plan: esmini perception pipeline


- **Branch:** `danilo-esmini-perception` (on GitHub: `danilorar/perception_uncertainty`)
- **Code:** `experiments/esmini-perception/simulation/acc_cpp/`
- **Last updated:** 2026-10-12

## Goal

Simplify the existing pipeline while preserving its behavior.

**Current ACC flow:** `run_single_acc.ipynb` → `acc_sim` → scenario preparation → esmini stepping → `src/controllers/acc.cpp` → native ACC and movement → saved results. See the [current file map](experiments/esmini-perception/simulation/acc_cpp/README.md).

AEB/custom movement and their interfaces/tests are removed. Shared perception/Markov and metrics remain. Sensor/uncertainty integration is paused. Optional native live animation is available through the notebook viewer setting. Build tools and native comparison remain; dropout batches, perception replay and their historical report exporter are retired.

## Working rules

- One small change at a time. Every task ends with validation and a short report for review.
- Behavior-preserving changes are proven with **byte-identical outputs before vs. after** (see Validation).
- Commit each completed change separately; push to the branch.
- Keep the active ACC pipeline working. Keep `Data/` (including `Data/OSC-NCAP-scenarios/`) and `reports/`.

## Done

| Commit | Change |
|---|---|
| `3792c21` | `MarkovDropout::apply()` readability (validation / state update / filtering sections) |
| `e7c3162` | Removed the obsolete Python AEB pipeline; `replay_perception.py` owns its esmini `State` struct |
| `f19bff9` | `main.cpp` `run()` readable: labeled stages, extracted CSV writers |
| `f89a64f` | New per-step master output `results.csv` (schema in `docs/output-schema.md`) |
| `e4234f9` | `--detailed-logs true|false` (default false); batch tools request detailed logs explicitly |

## Current state

- ACC-only runner, bridge ABI 5 and output schema 5; rebuild older libraries.
- Logging remains in `main.cpp`; `results.csv` is boundary truth, `native_acc.csv` is controller diagnostics, and detailed mode adds truth/metrics CSVs.
- Six CTests cover persistent ACC, shared perception/Markov/metrics and three scenario fixtures. Native comparison checks three scenarios × three settings.
- Notebook is the everyday entry point. Historical reports and local results remain intact.
- Optional native ObjectSensor detection is connected before each ACC step; notebook settings and detected-ID logs are available. All-target/reference baselines remain. Markov/Gaussian are not connected. Optional native live animation is supported. No commit requested.

## Validation

From `experiments/esmini-perception/simulation/acc_cpp/`:

```bash
python3 tools/build_native.py
python3 tools/compare_native_acc.py --output-dir results/native_acc_identity
```
