"""Overlay saved batch recordings using esmini's replayer; no simulation logic."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def replay_batch(runs, acc_cpp_dir):
    """Replay the notebook's completed runs together in one window."""
    root = Path(acc_cpp_dir).resolve()
    home = root / "build/engine-home"
    binary = home / "bin" / ("replayer.exe" if os.name == "nt" else "replayer")
    if not binary.is_file():
        raise FileNotFoundError("Build replayer first: python3 tools/build_native.py --viewer")
    if not runs:
        raise ValueError("Run or load a batch before replaying it.")


    # Validate every recording before preparing the overlay folder.
    recordings = [Path(run["output_dir"]).resolve() / "sim.dat" for run in runs]
    for recording in recordings:
        if not recording.is_file():
            raise FileNotFoundError(recording)

    replay_dir = Path(tempfile.mkdtemp(prefix="replay_", dir=recordings[0].parent.parent))
    command = [
        str(binary),
        "--res_path", str(home / "resources"),
        "--window", "60", "60", "1000", "700", "--quit_at_end",
        "--logfile_path", str(replay_dir / "replayer.log"),
    ]
    for index, (run, recording) in enumerate(zip(runs, recordings)):
        # Numeric names keep ordering clear and avoid using labels as paths.
        name = f"sim_{index:02d}.dat"
        shutil.copy2(recording, replay_dir / name)
        command += ["--path", str(recording.parent / "generated")]
        print(f"{name}: {run['label']}")

    if len(recordings) == 1:
        command += ["--file", str(replay_dir / "sim_00.dat")]
    else:
        command += ["--dir", str(replay_dir), "--file", "sim_"]

    print("Replay folder:", replay_dir)
    subprocess.run(command, cwd=replay_dir, check=True)
    return replay_dir
