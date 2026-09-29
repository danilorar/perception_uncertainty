"""Run all scenes and verify the eight added source variants end to end."""
import csv
import json
import math
from pathlib import Path
import re
import subprocess
import sys

import imageio_ffmpeg
from runner import new_run_id
from scenario import Reference, object_kind
from settings import ROOT, SCENARIOS, validate, write_json


def main():
    folder=ROOT/'verification'/new_run_id('eight_scenarios')
    folder.mkdir(parents=True)
    checks=[]
    original={'ccrs','cpna','cccscp'}
    variants=[]
    for case,scene in SCENARIOS.items():
        variants.append((case+'_aeb',{'scenario':case,'ego_speed_kph':scene['speed'],
                                    'export_video':case not in original}))
        if case not in original:
            variants.append((case+'_constant',{'scenario':case,'ego_speed_kph':scene['speed'],
                                              'controller':'constant_speed','export_video':False}))
    variants.append(('cbna_changed_ego',{'scenario':'cbna','ego_speed_kph':40,
                                       'ego_heading_offset_deg':10,'dt_s':.05,'export_video':False}))
    for name,overrides in variants:
        config=validate(overrides)
        cp=folder/(name+'.json');write_json(cp,config)
        out=folder/name
        with (folder/(name+'.log')).open('w',encoding='utf-8') as log:
            process=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run',
                                    '--config',str(cp),'--output',str(out)],stdout=log,
                                   stderr=subprocess.STDOUT,timeout=240,creationflags=subprocess.CREATE_NO_WINDOW)
        assert process.returncode==0,(name,folder/(name+'.log'))
        summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
        assert summary['status']=='completed',(name,summary)
        ref=Reference(config['scenario'])
        assert summary['end_time_s']==ref.duration
        assert summary['end_reason']=='original_reference_end'
        assert summary['target_max_state_error']<1e-5
        with (out/'truth.csv').open(encoding='utf-8') as source:
            truth=list(csv.DictReader(source))
        for row in truth:
            t=float(row['time_s']);ident=int(row['object_id'])
            assert 0<=t<=ref.duration
            if ident==ref.ego_id:continue
            target=next(s for s in ref.states(t) if s['id']==ident)
            for column,field in [('x_m','x'),('y_m','y'),('heading_rad','h'),('speed_mps','speed')]:
                assert abs(float(row[column])-target[field])<1e-7,(name,t,column)
        if config['controller']=='constant_speed':
            log=(ref.directory/'reference.log').read_text(encoding='utf-8')
            collisions=re.findall(r'\[([0-9.]+)\] \[warn\] Collision between Ego and ',log)
            assert collisions and summary['collision_details'],name
            assert abs(summary['collision_details']['time_s']-float(collisions[0]))<=.011,name
        if config['export_video']:
            video=json.loads((out/'video_metadata.json').read_text(encoding='utf-8'))
            assert video['simulation_end_s']==ref.duration
            decoded=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(out/'video.mp4'),
                                    '-f','null','-'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
            assert decoded.returncode==0,(name,decoded.stderr)
            instant=min(ref.duration*.8,(summary['first_brake_time_s'] or ref.duration*.6)+.3)
            frame=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-y','-ss',str(instant),
                                  '-i',str(out/'video.mp4'),'-frames:v','1',str(folder/(config['scenario']+'.png'))],
                                 capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
            assert frame.returncode==0,(name,frame.stderr)
        checks.append({'test':name,'passed':True,'duration_s':ref.duration,'collision':summary['collision'],
                       'video':summary['video_status'],'directory':str(out)})
        write_json(folder/'verification.json',{'passed':len(checks)==len(variants),'checks':checks})
        print(name,summary['collision'],ref.duration,flush=True)
    print('ALL PASSED:',folder,flush=True)


if __name__=='__main__':main()
