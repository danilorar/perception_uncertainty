"""Paths and small helpers shared by the whole pipeline."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

PACKAGE = Path(__file__).resolve().parent
REPO = PACKAGE.parent
# Set TME180_RUNTIME to keep the downloaded and compiled esmini somewhere else.
RUNTIME = Path(os.environ.get('TME180_RUNTIME') or REPO / 'runtime').resolve()
OUTPUT = REPO / 'output'
ENGINE = RUNTIME / 'esmini-3.8.1'
WINDOWS = os.name == 'nt'
EXE_SUFFIX = '.exe' if WINDOWS else ''
ESMINI = ENGINE / 'bin' / f'esmini{EXE_SUFFIX}'
REPLAYER = ENGINE / 'bin' / f'replayer{EXE_SUFFIX}'
BUILD_INFO = ENGINE / 'build.json'
BRIDGE_HEADER = PACKAGE / 'engine' / 'PerceptionBridge.hpp'
NO_WINDOW = subprocess.CREATE_NO_WINDOW if WINDOWS else 0


def code_version(folder=REPO):
    """The Git commit of this checkout and the files that differ from it, so a result shows which code made it.

    Ignored files (output/, runtime/, .venv) do not count. Outside a Git checkout every value is None.
    """
    try:
        status = subprocess.run(['git', '--no-optional-locks', 'status', '--porcelain=v2', '--branch',
                                 '--untracked-files=all'], cwd=folder, capture_output=True, encoding='utf-8',
                                errors='replace', timeout=30, creationflags=NO_WINDOW)
    except (OSError, subprocess.SubprocessError):
        status = None
    if status is None or status.returncode != 0:
        return {'commit': None, 'branch': None, 'local_changes': None}
    commit = branch = None
    changes = []
    for line in status.stdout.splitlines():
        if line.startswith('# branch.oid '):
            commit = line.split(' ', 2)[2]
        elif line.startswith('# branch.head '):
            branch = line.split(' ', 2)[2]
        elif line[:2] in ('1 ', '2 ', 'u ', '? '):
            fields = {'1': 9, '2': 10, 'u': 11, '?': 2}[line[0]]  # the path is the last field
            changes.append(line.split(' ', fields - 1)[-1].split('\t')[0])
    return {'commit': commit, 'branch': branch, 'local_changes': sorted(changes)}


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    for attempt in range(5):
        try:
            temporary.replace(path)
            return
        except PermissionError:  # Windows: a virus scanner or indexer can hold the file for a moment
            if attempt == 4:
                raise
            time.sleep(0.2 * (attempt + 1))


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()
