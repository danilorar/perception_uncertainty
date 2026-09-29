"""Render the actual saved run through esmini; encode an ordinary MP4."""
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import ctypes
import imageio_ffmpeg
from engine import Engine
from settings import write_json, sha256

def export(out):
    out=Path(out).resolve(); os.chdir(out)
    frames={}
    with (out/'truth.csv').open(encoding='utf-8') as f:
        for row in csv.DictReader(f):
            t=float(row['time_s'])
            frames.setdefault(t,[]).append({'id':int(row['object_id']), **{k:float(row[field]) for k,field in [('x','x_m'),('y','y_m'),('z','z_m'),('h','heading_rad'),('p','pitch_rad'),('r','roll_rad'),('speed','speed_mps'),('vx','vx_mps'),('vy','vy_mps')]}})
    times=sorted(frames); last=times[-1]; engine=Engine(); process=None
    # Sampling existing frames preserves the simulation time step and results.
    fps=25; sample_times=[min(times,key=lambda v:abs(v-i/fps)) for i in range(math.ceil(last*fps)+1)]
    positions=[s for states in frames.values() for s in states]
    xmin,xmax=min(s['x'] for s in positions)-5,max(s['x'] for s in positions)+7
    ymin,ymax=min(s['y'] for s in positions)-7,max(s['y'] for s in positions)+7
    # esmini 3.8.1 simulates orthographic top view with a 1-degree FOV.
    # See ViewerBase/viewer.cpp ORTHO_FOV, rather than a normal 30-degree lens.
    height=0.55*max(ymax-ymin,(xmax-xmin)/(960/540),20)/math.tan(math.radians(.5))
    camera=[(xmin+xmax)/2,(ymin+ymax)/2,height,1.5*math.pi]
    try:
        engine.init(out/'scene.xosc',out/'video-esmini.log',video=True,camera=camera)
        previous=0
        command=None
        with (out/'ffmpeg.log').open('wb') as log:
            for n,t in enumerate(sample_times):
                for s in frames[t]: engine.report(s)
                engine.step(t-previous)
                width,height,pix,data=engine.image()
                if process is None:
                    command=[imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','warning','-f','rawvideo','-pix_fmt',pix,'-s',f'{width}x{height}','-r',str(fps),'-i','pipe:0','-vf','vflip','-an','-c:v','libx264','-pix_fmt','yuv420p','-crf','21','-movflags','+faststart',str(out/'video.mp4')]
                    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log,creationflags=subprocess.CREATE_NO_WINDOW)
                process.stdin.write(data); previous=t
                if n%25==0: write_json(out/'progress.json',{'stage':'video','fraction':.8+.19*n/len(sample_times)})
            process.stdin.close()
            if process.wait(timeout=60)!=0: raise RuntimeError('FFmpeg 视频编码失败')
        write_json(out/'video_metadata.json',{'fps':fps,'width':width,'height':height,'frames':len(sample_times),'simulation_end_s':last,'camera':camera,'renderer_sha256':sha256(__file__),'method':'esmini offscreen render of saved truth; nearest saved simulation sample at 25 fps','ffmpeg_command':command})
    finally:
        if process and process.poll() is None:
            process.kill(); process.wait()
        engine.close()

if __name__=='__main__': export(sys.argv[1])
