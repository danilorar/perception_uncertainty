"""Run headless acceptance checks, including one complete video export."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone
from scenario import Reference
from settings import ROOT, SCENARIOS, validate, write_json


def main():
    folder=ROOT/'verification'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_teacher_scenes')
    folder.mkdir()
    cases=[(case,{'scenario':case,'export_video':False}) for case in SCENARIOS]
    cases += [('replay_noisy',{'uncertainty':'gaussian_range','dt_s':.02,'export_video':False}),
              ('replay_coarse',{'scenario':'gidas_1554431','dt_s':.05,'export_video':False}),
              ('aeb',{'controller':'aeb','uncertainty':'gaussian_range','export_video':False}),
              ('video',{'export_video':True})]
    checks=[]
    for label,raw in cases:
        config=validate(raw)
        cp=folder/(label+'.json');write_json(cp,config)
        out=folder/label
        with (folder/(label+'.log')).open('w',encoding='utf-8') as log:
            p=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(cp),'--output',str(out)],
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW,timeout=300)
        summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
        assert p.returncode==0 and summary['status'] in ['completed','completed_with_warnings'],summary
        assert summary['end_time_s']==9.98
        ref=Reference(config['scenario'])
        with (out/'truth.csv').open() as f: rows=list(csv.DictReader(f))
        max_error=0.0
        for row in rows:
            expected=next(s for s in ref.states(float(row['time_s'])) if s['id']==int(row['object_id']))
            if config['controller']=='original_trajectory' or int(row['object_id'])!=ref.ego_id:
                max_error=max(max_error,*(abs(float(row[col])-expected[key]) for col,key in [('x_m','x'),('y_m','y'),('heading_rad','h'),('speed_mps','speed')]))
        assert max_error < 1e-9,max_error
        if config['controller']=='original_trajectory':
            assert summary['collision'] and summary['first_brake_time_s'] is None
            assert summary['ego_replay_max_state_error'] < 1e-5
            expected_hit={'gidas_1549178':4.91,'gidas_1554254':4.92,'gidas_1554431':5.08}[config['scenario']]
            assert abs(summary['collision_details']['time_s']-expected_hit)<=config['dt_s']+1e-8
        else:
            assert summary['first_brake_time_s'] is not None
        if config['export_video']:
            assert summary['video_status']=='ready',summary
            assert (out/'video.mp4').stat().st_size>10000
        result={'label':label,'status':summary['status'],'max_source_state_error':max_error,
                'collision':summary['collision'],'collision_time_s':(summary['collision_details'] or {}).get('time_s'),
                'brake_time_s':summary['first_brake_time_s'],'video_status':summary['video_status']}
        checks.append(result)
        print(json.dumps(result),flush=True)
    write_json(folder/'verification.json',{'checks':checks,'passed':True})
    print(str(folder),flush=True)


if __name__=='__main__':main()
