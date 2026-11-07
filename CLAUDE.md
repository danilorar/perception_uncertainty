# aep — perception errors on esmini

Hard rules for every AI agent (HiDrive files, no Kalman filter, wording of results):
@AGENTS.md

## Project
Closed-loop sensitivity study: Python plugins add assumed range errors to what an AEB controller sees
in esmini rear-end scenarios; we measure how collisions, impact speed and TTC change.
Working branch: `pipeline`. Teammates use Windows and macOS; this machine is Linux.

## Where to read
- `README.md`: setup, running one simulation or a grid, error plugin settings, file map.
- `docs/EXTENDING.md`: adding an error plugin, a controller or a scenario.
- `docs/EXPERIMENTS.md`: protocol and log of every grid. Add a row for every grid you run.

## Commands
- `source .venv/bin/activate`
- `python -m unittest discover tests`   (must pass before any commit)
- `python -m pipeline setup`            (after controller or PerceptionBridge.hpp changes)
- `python -m pipeline run configs/<cfg>.json --output output/<name>`
- `python -m pipeline grid experiments/<grid>.json --workers N`   (start with smoke.json)

## Structure rules — keep the repo easy to follow
- Docs: only `README.md`, `CLAUDE.md` and `AGENTS.md` at the root; every other .md goes in `docs/`.
  Exception: `pipeline/controllers/hidrive/AGENTS.md` stays next to the files it protects.
- Prefer extending an existing file over adding a new script. A new error type is one file in
  `pipeline/errors/`; a new single run is one JSON in `configs/`; a new grid is one JSON in `experiments/`.
- No new top-level folders without asking. Results stay in `output/` (not in Git).
- Never search `runtime/`; for esmini source, read the zip in `runtime/downloads/`.

## Known pitfalls
- A run refuses to start on a stale build: run `setup` again.
- Windows long-path build failure: set `TME180_RUNTIME` to a short folder (e.g. `C:\t`).
