# Detection development after merging main

Continue on `feature/esmini-perception`, using `pipeline/` for new experiments.
The original `experiments/esmini-perception/` remains a historical implementation.
The scenario IDs retain `gidas_`, but these are TASC data.

## Preserved baselines

- `baseline/native-markov-20261006`: original public C++ generator and native AEB/ACC source, `484675b`.
- `baseline/aron-before-pipeline-20261110`: old Studio source, `6f9ae90`.
- `archive/studio-local-20261110` and `baseline/hidrive-studio-local-20261110`: saved local Mac integration,
  HiDrive detection reports and cone exporter, `833d370`. Raw results and graphics runtimes stay in the legacy worktree.
- Merge `e47289d` includes `origin/main@247f6cc` and the original feature history.

The existing legacy worktree is `/Users/zhuoma/TME180-project`.
The new development worktree is `/Users/zhuoma/TME180-detection-v2`.
Both share one Git repository. No remote branch is changed by these local operations.

## Setup and one run

```bash
cd /Users/zhuoma/TME180-detection-v2
python3 -m venv .venv
.venv/bin/python -m pip install -r pipeline/requirements.txt
.venv/bin/python -m pipeline setup
.venv/bin/python -m unittest discover tests
.venv/bin/python -m pipeline run configs/detection_markov_seed18.json --output output/demo_seed18
```

The default controller is the supplied HiDrive controller on esmini 3.8.1. Private source files are opaque compiler inputs.
For a configuration with several plugins, their order is the order in `errors`. That alone does not define a conditional
probability model or implement a detection gate for the other modules.

Keep fixed-parameter IID/Markov as the reference while developing dependencies. Add a separately named variant and
explicitly define the shared detection state, what downstream modules do on a missed frame, and the random streams.
Use one sampled detection state per target and sensor frame; plotting and control must reuse it. A future one-frame
history input means the preceding detection result, not accumulated track age or missing duration.

## Original sensitivity design on the new pipeline

```bash
.venv/bin/python -m pipeline grid experiments/detection_baseline_100.json --list
.venv/bin/python -m pipeline grid experiments/detection_baseline_100.json --workers 4
.venv/bin/python tools/compare_detection_baseline.py --grid output/grids/detection_baseline_100
```

This is 1203 main runs: 3 ideal references plus IID and Markov T=0.2/0.5/1.0 s,
100 seeds (0–99) per stochastic group and three scenarios. All other errors stay off.
The physics period is 0.01 s and detection period 0.05 s. Results stop at first contact or the scenario stop condition.
The comparison fixture contains the previous **HiDrive** study, not the earlier native-AEB study.
It compares collision, trigger/brake time, minimum gap and relative impact speed; the allowed absolute difference is 1e-9.
Grid results remain in ignored `output/`; no plots or replays are retained for a grid.

## Cone replay

The cone reads `detection.missed` from the saved `perception_applied.tsv`. A `valid=0` caused by a negative OU
range is not a detection dropout. With a named detection plugin, its `<name>.missed` column is used.
Prediction can supply an observation while a target is missed; the cone still follows the raw detection flag.
No error samples are generated during replay.

Video dependencies and a graphics-enabled public esmini 3.8.1 library are separate from closed-loop setup:

```bash
.venv/bin/python -m pip install -r pipeline/visualization/requirements.txt
.venv/bin/python -m pipeline cone output/demo_seed18 \
  --output output/videos/detection_seed18.mp4 \
  --visual-runtime /Users/zhuoma/TME180-project/esmini-lab/runtime/esmini-full-v3.8.1
```

`--visual-runtime` points to the public graphics installation, not a controller source folder.
It may also be set through `TME180_VISUAL_RUNTIME`.
On Windows the installation needs `bin/esminiLib.dll`; on macOS, `bin/libesminiLib.dylib`.
Vehicles follow recorded `truth.csv` through ExternalController solely for rendering; HiDrive is not run again.
The camera retains the prior 14-degree road angle and following top view. The cone geometry is illustrative,
not a fitted sensor range/FOV. The output JSON records every frame's source time, cone state and source-data hashes.

## Validate the actual original generator

```bash
.venv/bin/python tools/validate_markov_baseline.py
```

This compiles only the three original public perception files at the baseline tag.
It compares 800 streams (IDs 1 and 7, 100 seeds, IID plus three Markov durations), or 160,000 flags,
with the pipeline generator. Frozen hashes from that C++ executable are covered by `tests/test_markov_legacy.py`.

Keep spatially conditioned or cross-module variants distinct from this fixed-parameter baseline. Treat all parameter
values as sensitivity assumptions, and evaluate safety using restored world truth.
