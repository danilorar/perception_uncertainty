"""Local-only desktop launcher and HTTP UI. No account or cloud service."""

import argparse
import ctypes
import getpass
import json
import mimetypes
import os
from pathlib import Path
import re
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, unquote
from urllib.request import Request, urlopen
import webbrowser
from settings import (
    ROOT,
    RUNS,
    DEFAULT,
    SCENARIOS,
    VERSION,
    validate,
    write_json,
    REFERENCES,
    original_duration,
)
from runner import new_run_id
from scenario import object_label
from source_links import flow_source_links, open_source
from literature import paper_catalog, paper_file

TOKEN = secrets.token_urlsafe(24)
LOCK = threading.Lock()
ACTIVE = {}
RUN_JOBS = {}


class RunConflict(ValueError):
    pass


def process_user():
    # Environment variables can belong to the desktop user even when a helper
    # actually runs under a restricted account. Ask Windows for the token user.
    if os.name == 'nt':
        name = ctypes.create_unicode_buffer(257)
        size = ctypes.c_ulong(len(name))
        if not ctypes.windll.advapi32.GetUserNameW(name, ctypes.byref(size)):
            raise ctypes.WinError()
        return name.value
    return getpass.getuser()


def reuse_service(url, health):
    """Reuse only this workspace's service running as the launching user.

    A desktop launch must not inherit an agent/sandbox-owned HTTP service:
    that service can write results but cannot open the user's Explorer.
    Retire an idle incompatible service through its normal authenticated API.
    The server itself rejects shutdown while experiments are active.
    """
    if health.get('app') != 'tme180-simulation-studio' or health.get('root') != str(ROOT):
        return False
    if health.get('process_user') == process_user():
        return True
    try:
        with urlopen(url + '/api/bootstrap', timeout=2) as response:
            token = json.load(response)['token']
        request = Request(
            url + '/api/shutdown',
            data=b'{}',
            method='POST',
            headers={'Content-Type': 'application/json', 'X-Studio-Token': token},
        )
        with urlopen(request, timeout=2) as response:
            if not json.load(response).get('ok'):
                raise RuntimeError('后台服务未确认退出')
    except Exception as exc:
        raise RuntimeError('旧后台服务无法退出；若实验正在运行，请等其完成后重新启动。') from exc
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            with urlopen(url + '/api/health', timeout=0.3):
                pass
        except OSError:
            return False
        time.sleep(0.1)
    raise RuntimeError('旧后台服务尚未退出，请稍后重新启动。')


def open_results_folder(ident):
    folder = run_dir(ident)
    try:
        os.startfile(str(folder), 'explore')
    except PermissionError as exc:
        raise OSError(
            '后台服务无法访问桌面，未能打开文件夹。请通过桌面快捷方式重新启动应用。'
            f'结果仍保存在：{folder}'
        ) from exc


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return default


def run_dir(ident):
    if not isinstance(ident, str) or not re.fullmatch(r'[A-Za-z0-9_\-]+', ident):
        raise ValueError('无效运行编号')
    root = RUNS.resolve()
    p = root / ident
    if p.resolve() != p:
        raise ValueError('无效运行目录')
    if not p.is_dir():
        raise ValueError('运行记录不存在')
    return p


def delete_runs(idents):
    """Remove a validated batch from history, retaining recoverable raw results."""
    if (
        not isinstance(idents, list)
        or not 1 <= len(idents) <= 100
        or any(not isinstance(ident, str) for ident in idents)
    ):
        raise ValueError('请选择 1 至 100 条有效记录。')
    idents = list(dict.fromkeys(idents))
    from plot_axes import axes_lock
    from plots import make_plots

    with LOCK:
        with axes_lock(RUNS):
            # Validate the entire batch before moving anything. Resolving both
            # ends prevents symlinks/junctions from escaping the workspace.
            folders = [run_dir(ident) for ident in idents]
            summaries = [read_json(folder / 'summary.json', {}) for folder in folders]
            if any(
                ident in ACTIVE or summary.get('status') in ('running', 'starting')
                for ident, summary in zip(idents, summaries)
            ):
                raise ValueError('所选记录中有正在运行的实验，请等待完成或停止后再删除。')
            scenarios = {
                read_json(folder / 'config.json', summary.get('config', {})).get('scenario')
                for folder, summary in zip(folders, summaries)
            } - {None}
            work = ROOT.resolve() / 'work'
            archive = work / 'deleted-runs' / f'{time.time_ns()}_{secrets.token_hex(4)}'
            if archive.resolve() != archive:
                raise ValueError('无效备份目录')
            archive.mkdir(parents=True)
            moved = []
            try:
                for folder in folders:
                    destination = archive / folder.name
                    folder.rename(destination)
                    moved.append((folder, destination))
            except OSError:
                for folder, destination in reversed(moved):
                    destination.rename(folder)
                raise

        warnings = []
        # make_plots acquires the same cross-process axes lock as run workers.
        # Keep LOCK held so another API deletion/start cannot race this refresh.
        for scenario in scenarios:
            try:
                peers = []
                for folder in sorted(RUNS.iterdir()):
                    if not folder.is_dir() or folder.resolve() != folder:
                        continue
                    summary = read_json(folder / 'summary.json', {})
                    ready = summary.get('status') in ('completed', 'completed_with_warnings') or (
                        summary.get('status') == 'running' and (folder / 'plot_axes.json').is_file()
                    )
                    if (
                        ready
                        and read_json(folder / 'config.json', {}).get('scenario') == scenario
                        and (folder / 'ego.csv').is_file()
                        and (folder / 'observations.csv').is_file()
                    ):
                        peers.append(folder)
                if peers:
                    make_plots(peers[0])
                else:
                    with axes_lock(RUNS):
                        registry = read_json(RUNS / 'plot-axes.json')
                        if registry and scenario in registry.get('scenarios', {}):
                            del registry['scenarios'][scenario]
                            write_json(RUNS / 'plot-axes.json', registry)
            except Exception as exc:
                warnings.append({'scenario': scenario, 'error': str(exc)})
        return {'deleted': idents, 'warnings': warnings}


def refresh_plot_scenarios(scenarios):
    from plot_axes import axes_lock
    from plots import make_plots

    for scenario in scenarios:
        peers = []
        for folder in sorted(RUNS.iterdir()):
            if not folder.is_dir() or folder.resolve() != folder:
                continue
            summary = read_json(folder / 'summary.json', {})
            if (
                summary.get('status') in ('completed', 'completed_with_warnings')
                and read_json(folder / 'config.json', {}).get('scenario') == scenario
                and (folder / 'ego.csv').is_file()
                and (folder / 'observations.csv').is_file()
            ):
                peers.append(folder)
        if peers:
            make_plots(peers[0])
        else:
            with axes_lock(RUNS):
                registry = read_json(RUNS / 'plot-axes.json')
                if registry and scenario in registry.get('scenarios', {}):
                    del registry['scenarios'][scenario]
                    write_json(RUNS / 'plot-axes.json', registry)


def start_run(raw, replace_id=None):
    c = validate(raw)
    with LOCK:
        if ACTIVE:
            raise RunConflict('已有实验运行中，请等待完成或停止。')
        out = run_dir(replace_id) if replace_id is not None else None
        previous = read_json(out / 'summary.json', {}) if out else {}
        previous_config = read_json(out / 'config.json', previous.get('config', {})) if out else {}
        if previous.get('status') in ('running', 'starting'):
            raise RunConflict('正在运行的记录不能覆盖。')
        execution_id = new_run_id(c['scenario'])
        ident = replace_id if replace_id is not None else execution_id
        out = out or RUNS / ident
        inputs = ROOT / 'work'
        inputs.mkdir(exist_ok=True)
        config = inputs / (execution_id + '.json')
        write_json(config, c)
        log = (inputs / (execution_id + '.log')).open('w', encoding='utf-8')
        backup = None
        try:
            command = [
                sys.executable,
                '-X',
                'utf8',
                str(ROOT / 'runner.py'),
                'run',
                '--config',
                str(config),
                '--output',
                str(out),
            ]
            if replace_id is not None:
                from plot_axes import axes_lock

                backup = (
                    ROOT.resolve()
                    / 'work/replaced-runs'
                    / f'{time.time_ns()}_{secrets.token_hex(4)}'
                    / ident
                )
                if backup.resolve() != backup:
                    raise ValueError('无效备份目录')
                backup.parent.mkdir(parents=True)
                with axes_lock(RUNS):
                    out.rename(backup)
                if (backup / 'plot_axes_policy.json').is_file():
                    command.extend(['--plot-axes-policy', str(backup / 'plot_axes_policy.json')])
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0,
            )
        except Exception:
            log.close()
            if backup and backup.exists() and not out.exists():
                with axes_lock(RUNS):
                    backup.rename(out)
            raise
        RUN_JOBS[ident] = {
            'run_id': ident,
            'config': c,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'backup': str(backup) if backup else None,
            'previous_scenario': previous_config.get('scenario'),
        }
        ACTIVE[ident] = process
        threading.Thread(target=monitor, args=(ident, process, log), daemon=True).start()
        return {'run_id': ident, 'config': c, 'overwritten': replace_id is not None}


def run_summary(ident, *, details=False):
    job = RUN_JOBS.get(ident) if ident in ACTIVE else None
    out = RUNS / ident if job else run_dir(ident)
    summary = read_json(out / 'summary.json', None)
    if summary is None:
        summary = {'status': 'starting', 'run_id': ident}
        if job:
            summary.update(config=job['config'], created_utc=job['created_utc'])
    # Workers may have finished while the monitor is still refreshing plots.
    if job and summary.get('status') not in ('running', 'starting'):
        summary = {**summary, 'status': 'running'}
    if details:
        summary['progress'] = read_json(out / 'progress.json', {'stage': 'starting', 'fraction': 0})
        summary['files'] = (
            [p.name for p in out.iterdir() if p.is_file() and not p.name.endswith('.tmp')]
            if out.exists()
            else []
        )
    return summary


def monitor(ident, process, log):
    code = process.wait()
    log.close()
    with LOCK:
        job = RUN_JOBS.get(ident, {})
        out = RUNS / ident
        summary = read_json(out / 'summary.json', {'run_id': ident})
        if summary.get('status') in ['running', 'starting', None]:
            summary.update(
                status='failed',
                error=f'运行进程异常退出 ({code})；请查看 worker.log。',
                collision=None,
            )
            if job:
                summary.update(config=job['config'], created_utc=job['created_utc'])
            out.mkdir(exist_ok=True)
            write_json(out / 'summary.json', summary)
        shutil.copy2(log.name, out / 'worker.log')
        if job.get('backup'):
            write_json(
                out / 'replacement.json',
                {'backup': job['backup'], 'replaced_utc': job['created_utc']},
            )
            if summary.get('status') in ('failed', 'cancelled'):
                note = f'旧模拟的完整结果已备份至：{job["backup"]}'
                key = 'error' if summary.get('status') == 'failed' else 'warning'
                summary[key] = '；'.join(filter(None, [summary.get(key), note]))
                write_json(out / 'summary.json', summary)
            try:
                refresh_plot_scenarios(
                    {job.get('previous_scenario'), job['config']['scenario']} - {None}
                )
            except Exception as exc:
                summary['warning'] = '；'.join(
                    filter(None, [summary.get('warning'), f'覆盖后的共享曲线更新失败：{exc}'])
                )
                if summary.get('status') == 'completed':
                    summary['status'] = 'completed_with_warnings'
                write_json(out / 'summary.json', summary)
        ACTIVE.pop(ident, None)
        RUN_JOBS.pop(ident, None)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_json(self, value, status=200):
        data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def allowed_host(self):
        return self.headers.get('Host', '').split(':')[0] in ['127.0.0.1', 'localhost']

    def do_GET(self):
        if not self.allowed_host():
            return self.send_json({'error': 'Local access only'}, 403)
        path = unquote(urlparse(self.path).path)
        try:
            if path == '/api/health':
                return self.send_json(
                    {
                        'app': 'tme180-simulation-studio',
                        'root': str(ROOT),
                        'version': VERSION,
                        'process_user': process_user(),
                    }
                )
            if path == '/api/bootstrap':
                cases = []
                for ident, s in SCENARIOS.items():
                    meta = read_json(REFERENCES / ident / 'metadata.json', {})
                    initial = meta['initial_ego']
                    cases.append(
                        {
                            'id': ident,
                            **s,
                            'speed': initial['ego_speed_kph'],
                            'heading_deg': initial['ego_heading_deg'],
                            'duration_s': original_duration(ident, meta),
                            'timing_policy': meta.get('timing_policy'),
                            'stop_trigger_s': meta.get('stop_trigger_s'),
                            'fixed_objects': [
                                {
                                    'name': o['name'],
                                    'type': object_label(o, meta),
                                    'initial_speed_kph': round(o['speed'] * 3.6, 2),
                                }
                                for o in meta.get('objects', [])
                                if o['id'] != meta['ego_id']
                            ],
                        }
                    )
                return self.send_json(
                    {
                        'token': TOKEN,
                        'default': validate(DEFAULT),
                        'scenarios': cases,
                        'version': VERSION,
                        'capabilities': {'overwrite_run': True, 'open_source': True},
                        'source_links': flow_source_links(),
                        'literature': paper_catalog(),
                    }
                )
            if path == '/api/runs':
                with LOCK:
                    ids = {
                        p.name
                        for p in RUNS.iterdir()
                        if p.is_dir() and (p / 'summary.json').is_file()
                    }
                    ids.update(ACTIVE)
                    runs = [run_summary(ident) for ident in sorted(ids, reverse=True)]
                return self.send_json(runs[:100])
            if path.startswith('/api/run/'):
                ident = path.split('/')[-1]
                with LOCK:
                    s = run_summary(ident, details=True)
                return self.send_json(s)
            if path.startswith('/runs/'):
                parts = path.split('/')
                if len(parts) != 4:
                    raise ValueError('无效文件路径')
                out = run_dir(parts[2])
                file = (out / parts[3]).resolve()
                if file.parent != out.resolve():
                    raise ValueError('无效文件路径')
                return self.send_file(file)
            if path == '/api/literature':
                return self.send_json(paper_catalog())
            if path.startswith('/literature/'):
                parts = path.split('/')
                if len(parts) != 4:
                    raise ValueError('无效参考资料路径')
                return self.send_file(paper_file(parts[2], parts[3]))
            if path == '/paper':
                return self.send_file(ROOT / 'web/paper.html')
            if path in ['/', '/index.html']:
                return self.send_file(ROOT / 'web/index.html')
            if path in ['/app.js', '/style.css', '/paper.js', '/paper.css']:
                return self.send_file(ROOT / 'web' / path[1:])
            self.send_json({'error': 'Not found'}, 404)
        except (ValueError, OSError) as e:
            self.send_json({'error': str(e)}, 404)

    def send_file(self, file):
        if not file.is_file():
            return self.send_json({'error': '文件尚未生成'}, 404)
        size = file.stat().st_size
        start, end = 0, size - 1
        status = 200
        range_header = self.headers.get('Range')
        if range_header:
            m = re.fullmatch(r'bytes=(\d+)-(\d*)', range_header)
            if not m:
                return self.send_json({'error': 'Invalid range'}, 416)
            start = int(m[1])
            end = min(int(m[2]) if m[2] else size - 1, size - 1)
            if start > end:
                return self.send_json({'error': 'Invalid range'}, 416)
            status = 206
        mime = mimetypes.guess_type(file.name)[0] or 'application/octet-stream'
        if file.suffix == '.js':
            mime = 'text/javascript'
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(end - start + 1))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if status == 206:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            with file.open('rb') as f:
                f.seek(start)
                remaining = end - start + 1
                while remaining:
                    block = f.read(min(262144, remaining))
                    if not block:
                        break
                    self.wfile.write(block)
                    remaining -= len(block)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def do_POST(self):
        origin = self.headers.get('Origin')
        if (
            not self.allowed_host()
            or self.headers.get('X-Studio-Token') != TOKEN
            or (origin and urlparse(origin).netloc != self.headers.get('Host'))
        ):
            return self.send_json({'error': '请求未授权；请刷新本地页面。'}, 403)
        try:
            n = int(self.headers.get('Content-Length', '0'))
            if not 0 < n <= 32768:
                raise ValueError('请求过大或为空')
            data = json.loads(self.rfile.read(n))
            if not isinstance(data, dict):
                raise ValueError('请求必须是 JSON 对象。')
            path = urlparse(self.path).path
            if path == '/api/run':
                return self.send_json(start_run(data), 202)
            if path == '/api/run/overwrite':
                if not isinstance(data.get('run_id'), str):
                    raise ValueError('无效运行编号')
                return self.send_json(start_run(data['config'], data['run_id']), 202)
            if path == '/api/cancel':
                ident = data['run_id']
                with LOCK:
                    process = ACTIVE.get(ident)
                    if not process:
                        raise ValueError('该实验没有运行')
                    subprocess.run(
                        ['taskkill', '/PID', str(process.pid), '/T', '/F'],
                        capture_output=True,
                        creationflags=subprocess.CREATE_NO_WINDOW,
                    )
                    out = RUNS / ident
                    out.mkdir(exist_ok=True)
                    s = read_json(out / 'summary.json', {'run_id': ident})
                    s.update(
                        status='cancelled', collision=None, warning='运行已停止；输出可能不完整。'
                    )
                    write_json(out / 'summary.json', s)
                return self.send_json({'ok': True})
            if path == '/api/runs/delete':
                return self.send_json(delete_runs(data['run_ids']))
            if path == '/api/open-folder':
                open_results_folder(data['run_id'])
                return self.send_json({'ok': True})
            if path == '/api/open-source':
                try:
                    source = open_source(data.get('source_id'))
                except RuntimeError as exc:
                    return self.send_json({'error': str(exc)}, 400)
                return self.send_json({'ok': True, 'source': source})
            if path == '/api/shutdown':
                if ACTIVE:
                    raise ValueError('请先停止正在运行的实验。')
                self.send_json({'ok': True})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            self.send_json({'error': 'Not found'}, 404)
        except RunConflict as e:
            self.send_json({'error': str(e)}, 409)
        except (ValueError, KeyError, TypeError, OSError) as e:
            self.send_json({'error': str(e)}, 400)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    RUNS.mkdir(exist_ok=True)
    for port in range(8976, 8990):
        url = f'http://127.0.0.1:{port}'
        try:
            with urlopen(url + '/api/health', timeout=0.3) as r:
                health = json.load(r)
        except (OSError, ValueError):
            health = {}
        # Do not swallow shutdown refusal and start a second service over the
        # same results while another account's experiments are still active.
        if reuse_service(url, health):
            if not args.no_browser:
                webbrowser.open(url)
            print(url, flush=True)
            return
        try:
            server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError('无法找到可用的本地端口')
    # A worker from a previously interrupted server cannot be reported as active.
    for p in RUNS.glob('*/summary.json'):
        s = read_json(p, {})
        if s.get('status') == 'running':
            s.update(
                status='interrupted',
                collision=None,
                warning='上一次应用运行被中断，请重新运行以获取完整结果。',
            )
            write_json(p, s)
    write_json(
        ROOT / 'server.json', {'url': url, 'pid': os.getpid(), 'process_user': process_user()}
    )
    if not args.no_browser:
        webbrowser.open(url)
    print(url, flush=True)
    server.serve_forever()
    server.server_close()


if __name__ == '__main__':
    main()
