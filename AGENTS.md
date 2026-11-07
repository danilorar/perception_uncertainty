# Rules for AI agents

- Never open, read, search, parse, hash or print `ControllerHiDrive.cpp` or `ControllerHiDrive.hpp`, in any copy, or their
  object files, or `runtime/build.log`. Copy them, compare size and time, compile them. Details:
  `pipeline/controllers/hidrive/AGENTS.md`. Do not search `runtime/esmini-3.8.1/`, which holds copies; read public esmini
  code from the source archive in `runtime/downloads/`.
- The course does not allow a Kalman filter. Controllers get the observations directly.
- Error values are assumed. Describe results as sensitivity results, not as measured sensor risk.
- The scenarios are TASC data, even where a file or an ID says GIDAS.
- Check changes with `python -m unittest discover tests`. A change to `pipeline/run.py`, `results.py` or `errors/` must keep
  results identical unless the change is meant to alter them.
