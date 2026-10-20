"""Open allowlisted flow excerpts in the local VS Code workspace."""

import ast
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
from types import SimpleNamespace
import shutil
import subprocess

from settings import ROOT


@dataclass(frozen=True)
class SourceTarget:
    filename: str
    symbol: str
    anchor: str = ''
    label: str = ''


# IDs are the only launch input accepted from the browser. Paths and positions
# always come from this registry and the current source files on disk.
TARGETS = {
    'overview': SourceTarget('hidrive_runner.py', 'run_hidrive', "noisy = c['uncertainty'] == 'distance_ou_range'"),
    'config': SourceTarget('runner.py', 'run', 'c = validate(config)'),
    'reference': SourceTarget('hidrive_runner.py', 'build_hidrive_scene'),
    'clock': SourceTarget('hidrive_runner.py', 'run_hidrive', "duration = max(c['duration_s']"),
    'world': SourceTarget('hidrive_runner.py', 'run_hidrive', 'args = [EXECUTABLE'),
    'noise.process': SourceTarget('models.py', 'CorrelatedRangeNoise.sample'),
    'noise.file': SourceTarget('hidrive_runner.py', 'write_noise_file'),
    'inject.begin': SourceTarget('hidrive/PerceptionNoise.hpp', '@cpp', 'void Begin(',
                                 'hidrive/PerceptionNoise.hpp · Begin'),
    'inject.apply': SourceTarget('hidrive/PerceptionNoise.hpp', '@cpp', 'double z = sequence',
                                 'hidrive/PerceptionNoise.hpp · Begin (apply)'),
    'controller': SourceTarget('hidrive/build.py', '@cpp', 'studio::PerceptionNoise::Inst().Begin',
                               'hidrive/build.py · ScenarioEngine hook'),
    'restore': SourceTarget('hidrive/PerceptionNoise.hpp', '@cpp', 'void End()',
                            'hidrive/PerceptionNoise.hpp · End'),
    'audit': SourceTarget('hidrive_runner.py', 'run_hidrive', 'hits = [int(v) for v in rows[ego_name]'),
    'output.runner': SourceTarget('runner.py', 'finish_run', 'if not diagnostic_only:'),
    'output.video': SourceTarget('video.py', '_export_language', 'for s in frames[t]:'),
    'overlay.snapshot': SourceTarget('video_overlay.py', 'ObservationOverlay.boxes_at'),
    'overlay.corners': SourceTarget('video_overlay.py', 'PerceivedBox.corners'),
    'overlay.draw': SourceTarget('video_overlay.py', 'ObservationOverlay.draw', 'boxes = self.boxes_at(time_s)'),
}

FLOW_TARGETS = {
    'overview': ('overview',),
    'config': ('config',),
    'reference': ('reference',),
    'clock': ('clock',),
    'world': ('world',),
    'noise': ('noise.process', 'noise.file'),
    'inject': ('inject.begin', 'inject.apply'),
    'controller': ('controller',),
    'restore': ('restore',),
    'audit': ('audit',),
    'output': ('output.runner', 'output.video'),
    'overlay': ('overlay.snapshot', 'overlay.corners', 'overlay.draw'),
}


def flow_source_links():
    links = {}
    for flow_id, target_ids in FLOW_TARGETS.items():
        links[flow_id] = []
        for target_id in target_ids:
            try:
                links[flow_id].append(_resolve_source(target_id))
            except (RuntimeError, OSError):
                # Editing or moving one source file must not break the rest
                # of the application bootstrap or other navigation links.
                continue
    return links


def _resolve_source(target_id):
    if not isinstance(target_id, str) or target_id not in TARGETS:
        raise ValueError('未知源码链接；请刷新页面后重试。')
    target = TARGETS[target_id]
    root = ROOT.resolve()
    path = (root / target.filename).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise RuntimeError('源码文件不存在或不在项目目录内。')
    try:
        source = path.read_text(encoding='utf-8-sig')
        if target.symbol == '@cpp':
            hits = [i for i, text in enumerate(source.splitlines(), 1) if target.anchor in text]
            if len(hits) != 1:
                raise StopIteration
            node = SimpleNamespace(lineno=hits[0], end_lineno=min(len(source.splitlines()), hits[0] + 24))
        else:
            node = ast.parse(source, filename=str(path))
            for name in target.symbol.split('.'):
                node = next(
                    child
                    for child in node.body
                    if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                    and child.name == name
                )
    except (OSError, UnicodeError, SyntaxError, StopIteration) as exc:
        raise RuntimeError(f'无法定位源码 {target.filename} · {target.symbol}。') from exc
    line = node.lineno
    if target.anchor:
        matches = [
            index
            for index, text in enumerate(source.splitlines(), 1)
            if node.lineno <= index <= node.end_lineno and target.anchor in text
        ]
        # A changed excerpt still opens the surrounding function. Never guess
        # another occurrence elsewhere in the file or preserve a stale line.
        if len(matches) == 1:
            line = matches[0]
    return {
        'id': target_id,
        'label': target.label or (
            f'{Path(target.filename).name} · C++' if target.symbol == '@cpp'
            else f'{target.filename} · {target.symbol}'
        ),
        'path': str(path),
        'line': line,
        # The teaching excerpts omit code and add translated comments. Give
        # the browser current source lines so displayed numbers stay accurate.
        'lines': source.splitlines()[line - 1:node.end_lineno],
    }


def _find_vscode():
    candidates = []
    for name in ('code', 'Code.exe'):
        command = shutil.which(name)
        if not command:
            continue
        path = Path(command)
        if path.suffix.lower() in ('.cmd', '.bat'):
            candidates.append(path.parent.parent / 'Code.exe')
        else:
            candidates.append(path)
    candidates.append(Path('F:/CodeSoftware/Microsoft VS Code/Code.exe'))
    for variable, relative in (
        ('LOCALAPPDATA', 'Programs/Microsoft VS Code/Code.exe'),
        ('ProgramFiles', 'Microsoft VS Code/Code.exe'),
        ('ProgramFiles(x86)', 'Microsoft VS Code/Code.exe'),
    ):
        if os.environ.get(variable):
            candidates.append(Path(os.environ[variable]) / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise RuntimeError('未找到 Visual Studio Code，请安装 VS Code 后重试。')


def open_source(target_id):
    """Open one current excerpt with its project folder in a new editor window."""
    entry = _resolve_source(target_id)
    editor = _find_vscode()
    environment = os.environ.copy()
    environment.pop('ELECTRON_RUN_AS_NODE', None)
    command = [
        str(editor),
        '--new-window',
        str(ROOT.resolve()),
        '--goto',
        f"{entry['path']}:{entry['line']}:1",
    ]
    log_path = ROOT / 'work/source-links/vscode-launch.log'
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open('ab') as log:
            log.write(f'\n{datetime.now(timezone.utc).isoformat()} {target_id}\n'.encode('utf-8'))
            log.flush()
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0,
            )
            try:
                result = process.wait(timeout=0.75)
            except subprocess.TimeoutExpired:
                result = None
            if result not in (None, 0):
                raise RuntimeError(f'VS Code 启动失败（退出码 {result}）；日志：{log_path}')
    except OSError as exc:
        raise RuntimeError(f'无法启动 VS Code：{exc}；日志：{log_path}') from exc
    return entry
