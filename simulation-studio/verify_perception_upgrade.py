"""Headless end-to-end checks; write fresh artifacts, preserve existing results."""
import csv
import json
import subprocess
import sys
from datetime import datetime,timezone
from settings import ROOT,DEFAULT,SCENARIOS,validate,write_json
from scenario import Reference


def main():
    folder=ROOT/'verification'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_perception_upgrade')
    folder.mkdir()
    cases=[(case,{'scenario':case}) for case in SCENARIOS]
    cases += [('ideal',{'uncertainty':'ideal'}),
              ('raw_single',{'observation_filter':'raw','aeb_trigger_mode':'instant'}),
              ('coarse',{'dt_s':.02}),
              ('original_replay',{'controller':'original_trajectory','uncertainty':'recorded_truth'}),
              ('video',{'uncertainty':'gaussian_range','range_sigma_m':2,'export_video':True})]
    checks=[]
    for label,extra in cases:
        c=validate(DEFAULT|{'controller':'aeb_follow','uncertainty':'distance_gaussian_range','export_video':False}|extra)
        cp=folder/(label+'.json');write_json(cp,c)
        out=folder/label
        with (folder/(label+'.log')).open('w',encoding='utf-8') as log:
            result=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(cp),'--output',str(out)],
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW,timeout=300)
        s=json.loads((out/'summary.json').read_text(encoding='utf-8'))
        assert result.returncode==0 and s['status'] in ['completed','completed_with_warnings'],s
        assert s['end_time_s']==9.98
        ref=Reference(c['scenario'])
        maximum=0.
        with (out/'truth.csv').open() as f:
            for row in csv.DictReader(f):
                time=float(row['time_s']);ident=int(row['object_id'])
                until_brake=s['first_brake_time_s'] is None or time<=s['first_brake_time_s']
                if ident!=ref.ego_id or until_brake:
                    expected=next(v for v in ref.states(time) if v['id']==ident)
                    maximum=max(maximum,*(abs(float(row[col])-expected[k]) for col,k in [('x_m','x'),('y_m','y'),('speed_mps','speed')]))
        assert maximum<1e-9,maximum
        with (out/'observations.csv').open() as f: obs=list(csv.DictReader(f))
        assert len({r['time_s'] for r in obs})==200
        assert all(float(r['time_s'])<=9.98 for r in obs)
        if c['controller']=='original_trajectory':
            assert s['collision'] and s['first_brake_time_s'] is None
        if label=='ideal':
            assert s['assessment']['brake_advance_vs_ideal_s']==0
        if c['export_video']:
            assert s['video_status']=='ready',s
            assert (out/'video.mp4').stat().st_size>10000
        check={'label':label,'status':s['status'],'max_reference_error':maximum,
            'collision':s['collision'],'brake_s':s['first_brake_time_s'],
            'advance_s':s['assessment']['brake_advance_vs_ideal_s'],
            'observed_rmse_m':s['assessment']['matched_observed_range_rmse_m'],
            'estimated_rmse_m':s['assessment']['estimated_range_rmse_m'],'video':s['video_status']}
        checks.append(check);print(json.dumps(check),flush=True)
    write_json(folder/'verification.json',{'passed':True,'checks':checks})
    print(str(folder),flush=True)


if __name__=='__main__':main()
