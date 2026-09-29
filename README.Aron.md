# Aron's Simulation Studio

This branch contains the TME180 perception uncertainty and AEB experiment project
(Simulation Studio 0.6.1). It starts from the group's `main` branch and adds the
application under `simulation-studio/`, the supplied scenario pairs under `DATA/`,
and simulator setup/validation scripts under `esmini-lab/`.

The interface supports Chinese and English. The current scenario menu uses GIDAS
1549178, 1554254, and 1554431. Experiments combine esmini's native ideal object
sensor, configurable distance uncertainty, raw or Kalman observations, and
trajectory replay or AEB takeover. Results include CSV data, plots, and optional
MP4 video. Implementation details and version history are in
[the project documentation](simulation-studio/README.zh-CN.md).

## Get this branch

```powershell
git clone --branch Aron https://github.com/danilorar/perception_uncertainty.git
cd perception_uncertainty
```

The commands below run from the repository root. The application currently
targets Windows x64 and uses Python 3.11. It uses the complete esmini 3.8.1
installation at `esmini-lab/runtime/esmini-full-v3.8.1`.

## Set up a fresh checkout

1. Create the application environment and install its pinned dependencies:

   ```powershell
   py -3.11 -m venv simulation-studio/.venv
   & './simulation-studio/.venv/Scripts/python.exe' -m pip install -r './simulation-studio/requirements-lock.txt'
   ```

2. Download the pinned simulator packages and NCAP sources, then install the
   complete simulator. Keep this order: the full installer compares its binaries
   with the demo installation created by the first script. Experiments use the
   complete installation afterwards.

   ```powershell
   & './esmini-lab/scripts/setup.ps1'
   & './simulation-studio/.venv/Scripts/python.exe' -X utf8 './esmini-lab/scripts/setup_full.py'
   ```

   The scripts verify the configured download hashes and store installation
   receipts locally. Downloads include the full model package and require an
   internet connection. Windows `tar` must support the `.7z` model archive.

3. Generate local reference data for the three supplied scenarios:

   ```powershell
   & './simulation-studio/.venv/Scripts/python.exe' -X utf8 './simulation-studio/teacher_data.py'
   ```

4. Start the interface with the windowless Python launcher:

   ```powershell
   Start-Process -FilePath './simulation-studio/.venv/Scripts/pythonw.exe' -ArgumentList '"./simulation-studio/launcher.pyw"' -WindowStyle Hidden
   ```

   The launcher opens the local interface in a browser. An optional desktop
   shortcut can be installed with `./simulation-studio/install-shortcut.ps1`.

## Code map

| Path | Purpose |
| --- | --- |
| `simulation-studio/engine.py` | esmini DLL interface and native object sensor |
| `simulation-studio/models.py` | Distance uncertainty and AEB logic |
| `simulation-studio/tracking.py` | Kalman distance estimation |
| `simulation-studio/runner.py` | Individual and batch experiments |
| `simulation-studio/teacher_data.py` | Import the supplied timed trajectories |
| `simulation-studio/app.py`, `web/` | Local service and browser interface |
| `simulation-studio/configs/` | Reusable experiment configurations |
| `DATA/` | Original OpenSCENARIO/OpenDRIVE pairs used by this application |
| `simulation-studio/assets/ncap/` | Retained NCAP resources with upstream license |
| `esmini-lab/scripts/` | Pinned installation and validation tools |

The inherited root-level `OSC-NCAP-scenarios/` and `esmini` Git entry belong to
the original group baseline. Simulation Studio uses the paths documented above;
its setup does not require initializing that inherited Git entry.

## Results and local files

Generated references, runs, videos, verification outputs, logs, Python virtual
environments, downloaded simulator packages, and development backups are ignored
by Git. They remain on Aron's computer and can be regenerated locally. The
historical documentation contains links to those local evidence files; the
branch does not include that experiment archive. Local course paperwork and team
agreement files are also excluded from this code upload.

Run an example after completing setup:

```powershell
& './simulation-studio/.venv/Scripts/python.exe' -X utf8 './simulation-studio/runner.py' run --config './simulation-studio/configs/aeb_kalman.json'
```

The AEB and uncertainty models are research prototypes. Model assumptions and
historical behavior changes are documented in the project README; results are
not an NCAP certification or a validated production vehicle safety assessment.

## Working with the group

Continue committing and pushing personal work on `Aron`:

```powershell
git status
git add <files-to-share>
git commit -m "Describe the change"
git push origin Aron
```

Pushing this branch does not merge it into `main` or change another member's
branch. Teammates can browse the branch on GitHub or fetch it for comparison.
Integrate changes into the group baseline through a separately reviewed pull
request when the group is ready.
