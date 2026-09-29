"""Central paths for the original-data simulation workflow.

Author: Zhuo Ma
Paths are anchored to this file, so scripts work from any current directory.
Environment variables let each collaborator keep their own esmini/results paths.
Importing this module never creates directories or starts a simulation.
"""
import os
from pathlib import Path
import sys

SIMULATION = Path(__file__).resolve().parent
ROOT = SIMULATION.parent


def configured_path(variable, default):
    """Resolve an optional environment override, expanding a user's home (~)."""
    return Path(os.environ.get(variable, str(default))).expanduser().resolve()


# Preserve the spelling Data: case matters when the project moves to Linux.
DATA = configured_path("TME180_DATA", ROOT / "Data")
GENERATED = configured_path("TME180_GENERATED", SIMULATION / "generated")
ESMINI = configured_path("ESMINI_HOME", SIMULATION / "esmini-demo")

# Write frequent telemetry updates outside OneDrive to avoid sync/write timeouts.
# Set TME180_RESULTS, or pass --results-dir to a runner, to choose another location.
RESULTS = configured_path(
    "TME180_RESULTS", Path.home() / "TME180-local/results/data-scenarios"
)
LATEST = RESULTS / "latest.txt"
SCENARIO_IDS = ("1549178", "1554254", "1554431")


def library_path():
    """Choose the library filename; the installed binary must match Python's CPU."""
    name = {"darwin": "libesminiLib.dylib", "win32": "esminiLib.dll"}.get(
        sys.platform, "libesminiLib.so"
    )
    return ESMINI / "bin" / name


def executable_path(name="esmini"):
    """Locate an esmini command-line application in the selected installation."""
    return ESMINI / "bin" / (name + (".exe" if sys.platform == "win32" else ""))
