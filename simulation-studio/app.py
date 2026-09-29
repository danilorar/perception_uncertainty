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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, unquote
from urllib.request import Request, urlopen
import webbrowser
from settings import ROOT, RUNS, DEFAULT, SCENARIOS, VERSION, validate, write_json, REFERENCES, original_duration
from runner import new_run_id
from scenario import object_label

TOKEN=secrets.token_urlsafe(24)
LOCK=threading.Lock()
ACTIVE={}

def process_user():
    # Environment variables can belong to the desktop user even when a helper
    # actually runs under a restricted account. Ask Windows for the token user.
    if os.name=='nt':
        name=ctypes.create_unicode_buffer(257)
        size=ctypes.c_ulong(len(name))
        if not ctypes.windll.advapi32.GetUserNameW(name,ctypes.byref(size)):
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
    if health.get('app')!='tme180-simulation-studio' or health.get('root')!=str(ROOT):
        return False
    if health.get('process_user')==process_user():
        return True
    try:
        with urlopen(url+'/api/bootstrap',timeout=2) as response:
            token=json.load(response)['token']
        request=Request(url+'/api/shutdown',data=b'{}',method='POST',
                        headers={'Content-Type':'application/json','X-Studio-Token':token})
        with urlopen(request,timeout=2) as response:
            if not json.load(response).get('ok'):
                raise RuntimeError('后台服务未确认退出')
    except Exception as exc:
        raise RuntimeError('旧后台服务无法退出；若实验正在运行，请等其完成后重新启动。') from exc
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        try:
            with urlopen(url+'/api/health',timeout=.3): pass
        except OSError:
            return False
        time.sleep(.1)
    raise RuntimeError('旧后台服务尚未退出，请稍后重新启动。')


def open_results_folder(ident):
    folder=run_dir(ident)
    try:
        os.startfile(str(folder),'explore')
    except PermissionError as exc:
        raise OSError('后台服务无法访问桌面，未能打开文件夹。请通过桌面快捷方式重新启动应用。'
                      f'结果仍保存在：{folder}') from exc

def read_json(path,default=None):
    try: return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError,ValueError): return default

def run_dir(ident):
    if not re.fullmatch(r'[A-Za-z0-9_\-]+',ident): raise ValueError('无效运行编号')
    p=RUNS/ident
    if not p.is_dir(): raise ValueError('运行记录不存在')
    return p

def monitor(ident,process,log):
    code=process.wait(); log.close()
    with LOCK:
        out=RUNS/ident
        if out.is_dir():
            shutil.copy2(log.name,out/'worker.log')
        summary=read_json(out/'summary.json',{'run_id':ident})
        if summary.get('status') in ['running',None]:
            summary.update(status='failed',error=f'运行进程异常退出 ({code})；请查看 worker.log。',collision=None)
            out.mkdir(exist_ok=True);write_json(out/'summary.json',summary)
        ACTIVE.pop(ident,None)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass

    def send_json(self,value,status=200):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
        self.end_headers();self.wfile.write(data)

    def allowed_host(self):
        return self.headers.get('Host','').split(':')[0] in ['127.0.0.1','localhost']

    def do_GET(self):
        if not self.allowed_host(): return self.send_json({'error':'Local access only'},403)
        path=unquote(urlparse(self.path).path)
        try:
            if path=='/api/health': return self.send_json({'app':'tme180-simulation-studio','root':str(ROOT),'version':VERSION,'process_user':process_user()})
            if path=='/api/bootstrap':
                cases=[]
                for ident,s in SCENARIOS.items():
                    meta=read_json(REFERENCES/ident/'metadata.json',{})
                    initial=meta['initial_ego']
                    cases.append({'id':ident,**s,'speed':initial['ego_speed_kph'],
                        'heading_deg':initial['ego_heading_deg'],
                        'duration_s':original_duration(ident,meta),
                        'fixed_objects':[{'name':o['name'],'type':object_label(o,meta),'initial_speed_kph':round(o['speed']*3.6,2)} for o in meta.get('objects',[]) if o['id']!=meta['ego_id']]})
                return self.send_json({'token':TOKEN,'default':validate(DEFAULT),'scenarios':cases,'version':VERSION})
            if path=='/api/runs':
                runs=[]
                for p in sorted(RUNS.glob('*'),reverse=True):
                    if p.is_dir():
                        s=read_json(p/'summary.json')
                        if s: runs.append(s)
                return self.send_json(runs[:100])
            if path.startswith('/api/run/'):
                ident=path.split('/')[-1];out=run_dir(ident)
                s=read_json(out/'summary.json',{'status':'starting','run_id':ident})
                s['progress']=read_json(out/'progress.json',{'stage':'starting','fraction':0})
                s['files']=[p.name for p in out.iterdir() if p.is_file() and not p.name.endswith('.tmp')]
                return self.send_json(s)
            if path.startswith('/runs/'):
                parts=path.split('/')
                if len(parts)!=4: raise ValueError('无效文件路径')
                out=run_dir(parts[2]);file=(out/parts[3]).resolve()
                if file.parent!=out.resolve(): raise ValueError('无效文件路径')
                return self.send_file(file)
            if path in ['/','/index.html']: return self.send_file(ROOT/'web/index.html')
            if path in ['/app.js','/style.css']: return self.send_file(ROOT/'web'/path[1:])
            self.send_json({'error':'Not found'},404)
        except (ValueError,OSError) as e: self.send_json({'error':str(e)},404)

    def send_file(self,file):
        if not file.is_file(): return self.send_json({'error':'文件尚未生成'},404)
        size=file.stat().st_size;start,end=0,size-1;status=200
        range_header=self.headers.get('Range')
        if range_header:
            m=re.fullmatch(r'bytes=(\d+)-(\d*)',range_header)
            if not m: return self.send_json({'error':'Invalid range'},416)
            start=int(m[1]);end=min(int(m[2]) if m[2] else size-1,size-1)
            if start>end: return self.send_json({'error':'Invalid range'},416)
            status=206
        mime=mimetypes.guess_type(file.name)[0] or 'application/octet-stream'
        if file.suffix=='.js': mime='text/javascript'
        self.send_response(status);self.send_header('Content-Type',mime)
        self.send_header('Content-Length',str(end-start+1));self.send_header('Accept-Ranges','bytes')
        self.send_header('Cache-Control','no-cache');self.send_header('X-Content-Type-Options','nosniff')
        if status==206: self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            with file.open('rb') as f:
                f.seek(start); remaining=end-start+1
                while remaining:
                    block=f.read(min(262144,remaining))
                    if not block: break
                    self.wfile.write(block);remaining-=len(block)
        except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError): pass

    def do_POST(self):
        origin=self.headers.get('Origin')
        if (not self.allowed_host() or self.headers.get('X-Studio-Token')!=TOKEN or
                (origin and urlparse(origin).netloc!=self.headers.get('Host'))):
            return self.send_json({'error':'请求未授权；请刷新本地页面。'},403)
        try:
            n=int(self.headers.get('Content-Length','0'))
            if not 0<n<=32768: raise ValueError('请求过大或为空')
            data=json.loads(self.rfile.read(n))
            path=urlparse(self.path).path
            if path=='/api/run':
                c=validate(data)
                with LOCK:
                    if ACTIVE: return self.send_json({'error':'已有实验运行中，请等待完成或停止。'},409)
                    ident=new_run_id(c['scenario'])
                    inputs=ROOT/'work'; inputs.mkdir(exist_ok=True)
                    config=inputs/(ident+'.json');write_json(config,c)
                    log=(inputs/(ident+'.log')).open('w',encoding='utf-8')
                    p=subprocess.Popen([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(config),'--output',str(RUNS/ident)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                    ACTIVE[ident]=p
                    threading.Thread(target=monitor,args=(ident,p,log),daemon=True).start()
                return self.send_json({'run_id':ident},202)
            if path=='/api/cancel':
                ident=data['run_id']
                with LOCK:
                    process=ACTIVE.get(ident)
                    if not process: raise ValueError('该实验没有运行')
                    subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
                    out=RUNS/ident;out.mkdir(exist_ok=True)
                    s=read_json(out/'summary.json',{'run_id':ident})
                    s.update(status='cancelled',collision=None,warning='运行已停止；输出可能不完整。')
                    write_json(out/'summary.json',s)
                return self.send_json({'ok':True})
            if path=='/api/open-folder':
                open_results_folder(data['run_id']);return self.send_json({'ok':True})
            if path=='/api/shutdown':
                if ACTIVE: raise ValueError('请先停止正在运行的实验。')
                self.send_json({'ok':True});threading.Thread(target=self.server.shutdown,daemon=True).start();return
            self.send_json({'error':'Not found'},404)
        except (ValueError,KeyError,TypeError,OSError) as e: self.send_json({'error':str(e)},400)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--no-browser',action='store_true');args=parser.parse_args()
    RUNS.mkdir(exist_ok=True)
    for port in range(8976,8990):
        url=f'http://127.0.0.1:{port}'
        try:
            with urlopen(url+'/api/health',timeout=.3) as r: health=json.load(r)
        except (OSError,ValueError): health={}
        # Do not swallow shutdown refusal and start a second service over the
        # same results while another account's experiments are still active.
        if reuse_service(url,health):
            if not args.no_browser: webbrowser.open(url)
            print(url,flush=True);return
        try: server=ThreadingHTTPServer(('127.0.0.1',port),Handler);break
        except OSError: continue
    else: raise RuntimeError('无法找到可用的本地端口')
    # A worker from a previously interrupted server cannot be reported as active.
    for p in RUNS.glob('*/summary.json'):
        s=read_json(p,{})
        if s.get('status')=='running':
            s.update(status='interrupted',collision=None,warning='上一次应用运行被中断，请重新运行以获取完整结果。');write_json(p,s)
    write_json(ROOT/'server.json',{'url':url,'pid':os.getpid(),'process_user':process_user()})
    if not args.no_browser: webbrowser.open(url)
    print(url,flush=True)
    server.serve_forever();server.server_close()

if __name__=='__main__': main()
