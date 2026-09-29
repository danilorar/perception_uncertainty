"""Native-sensor acceptance runs; all evidence goes into a fresh collection."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from models import range_error_parameters
from scenario import Reference
from settings import DEFAULT, ROOT, RUNTIME, SCENARIOS, sha256, validate, write_json


def rows(path):
    with path.open(encoding='utf-8',newline='') as f:
        return list(csv.DictReader(f))


def inspect(out):
    summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
    c=summary['config']
    assert summary['status'] in ['completed','completed_with_warnings'],summary
    assert summary['end_time_s']==9.98
    sensor=json.loads((out/'sensor.json').read_text(encoding='utf-8'))
    frames=[json.loads(line) for line in (out/'sensor_frames.jsonl').read_text().splitlines()]
    assert sensor['backend']=='esmini_ideal_object_sensor'
    assert sensor['fetch_count']==len(frames)==200
    frames={r['time_s']:r for r in frames}
    obs=rows(out/'observations.csv')
    detected=0; missed=0
    for r in obs:
        native=r['sensor_detected']=='True'
        assert native==(int(r['object_id']) in frames[float(r['time_s'])]['detected_ids'])
        assert native==(r['visible']=='True')
        assert int(r['sensor_id'])==sensor['sensor_id']
        if not native:
            missed+=1
            assert not r['raw_observed_range_m'] and not r['sampled_range_error_m']
            assert not r['controller_range_m'] and not r['ideal_range_m']
            continue
        detected+=1
        ideal=float(r['ideal_range_m'])
        assert abs(ideal-float(r['true_range_m']))<1e-8
        assert float(r['sensor_reference_range_m'])<=c['sensor_range_m']+1e-8
        assert abs(float(r['raw_observed_range_m'])-ideal-float(r['sampled_range_error_m']))<1e-8
        mean,sigma,mae=range_error_parameters(c,ideal)
        assert abs(float(r['range_error_sigma_m'])-sigma)<1e-8
        if c['uncertainty'] in ['ideal','recorded_truth'] or sigma==0 and mean==0:
            assert float(r['sampled_range_error_m'])==0
        if c['uncertainty']=='distance_gaussian_range' and float(r['raw_observed_range_m'])<0:
            assert r['observation_valid']=='False' and not r['controller_range_m']
    ref=Reference(c['scenario'])
    maximum=0.
    for r in rows(out/'truth.csv'):
        t=float(r['time_s']);ident=int(r['object_id'])
        unchanged=(ident!=ref.ego_id or c['controller']=='original_trajectory' or
                   c['controller']=='aeb_follow' and (summary['first_brake_time_s'] is None or t<=summary['first_brake_time_s']))
        if unchanged:
            expected=next(s for s in ref.states(t) if s['id']==ident)
            maximum=max(maximum,*(abs(float(r[col])-expected[key]) for col,key in
                                  [('x_m','x'),('y_m','y'),('speed_mps','speed')]))
    assert maximum<1e-9
    versions=json.loads((out/'versions.json').read_text(encoding='utf-8'))
    assert versions['esmini_runtime_directory']==str(RUNTIME)
    assert versions['esmini_dll_sha256']==sha256(RUNTIME/'bin/esminiLib.dll')
    if summary['ideal_reference']:
        baseline=out/'ideal_reference'
        check=inspect(baseline)
        a,b=rows(out/'ego.csv'),rows(baseline/'ego.csv')
        assert [(r['time_s'],r['ideal_ego_speed_kph'],r['ideal_aeb_state']) for r in a]==[
            (r['time_s'],r['speed_kph'],r['aeb_state']) for r in b]
        assert not (baseline/'ideal_reference').exists()
        if c['uncertainty']=='ideal':
            assert [r['speed_kph'] for r in a]==[r['speed_kph'] for r in b]
            assert summary['assessment']['brake_advance_vs_ideal_s']==0
    if c['export_video']:
        assert summary['video_status']=='ready',summary
        assert (out/'video.mp4').stat().st_size>10000
    return {'run':out.name,'native_queries':sensor['fetch_count'],'detected':detected,
            'missed':missed,'source_max_error':maximum,'brake_s':summary['first_brake_time_s'],
            'video':summary['video_status']}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--existing',type=Path,help='Recheck saved runs without rerunning simulations')
    args=parser.parse_args()
    folder=args.existing.resolve() if args.existing else ROOT/'verification'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_native_sensor')
    if not args.existing: folder.mkdir()
    print(str(folder),flush=True)
    cases=[(scene,{'scenario':scene,'export_video':scene=='gidas_1549178'}) for scene in SCENARIOS]
    cases += [('ideal',{'uncertainty':'ideal'}),
              ('replay',{'controller':'original_trajectory','uncertainty':'recorded_truth'}),
              ('replay_noisy',{'controller':'original_trajectory'}),
              ('repeat',{'controller':'original_trajectory'}),
              ('other_seed',{'controller':'original_trajectory','seed':42}),
              ('short_range',{'sensor_range_m':1}),
              ('coarse',{'dt_s':.02}),
              ('fixed_gaussian',{'uncertainty':'gaussian_range','range_sigma_m':2})]
    checks=[]
    for label,extra in cases:
        c=validate(DEFAULT|{'controller':'aeb_follow','uncertainty':'distance_gaussian_range','export_video':False}|extra)
        if not args.existing:
            cp=folder/(label+'.json');write_json(cp,c)
            with (folder/(label+'.log')).open('w',encoding='utf-8') as log:
                result=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run',
                                       '--config',str(cp),'--output',str(folder/label)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=300,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            assert result.returncode==0,(folder/(label+'.log')).read_text(encoding='utf-8')
        check=inspect(folder/label)
        checks.append(check);print(json.dumps(check),flush=True)
    for filename in ['observations.csv','ego.csv','truth.csv','sensor_frames.jsonl']:
        assert sha256(folder/'replay_noisy'/filename)==sha256(folder/'repeat'/filename),filename
    assert sha256(folder/'other_seed'/'observations.csv')!=sha256(folder/'repeat'/'observations.csv')
    assert sha256(folder/'other_seed'/'sensor_frames.jsonl')==sha256(folder/'repeat'/'sensor_frames.jsonl')
    # Closed-loop numerical integration can differ at floating-point precision;
    # compare timestamps/flags exactly and numeric observations to 1 nanometre.
    coarse,fine=rows(folder/'coarse'/'observations.csv'),rows(folder/'gidas_1549178'/'observations.csv')
    assert len(coarse)==len(fine)
    max_step_difference=0.
    for a,b in zip(coarse,fine):
        for key in a:
            if a[key]==b[key]: continue
            difference=abs(float(a[key])-float(b[key]))
            max_step_difference=max(max_step_difference,difference)
            assert difference<1e-9,(key,a[key],b[key])
    first=rows(folder/'short_range'/'observations.csv')[0]
    assert first['sensor_detected']=='False' and not first['controller_range_m']
    scales={}
    for label,_ in cases:
        out=folder/label
        scene=json.loads((out/'config.json').read_text())['scenario']
        axes=json.loads((out/'plot_axes.json').read_text())
        if scene in scales: assert scales[scene]==axes
        else: scales[scene]=axes
    write_json(folder/'verification.json',{'passed':True,'checks':checks,
        'max_integration_step_observation_difference':max_step_difference,
        'verification_script_sha256':sha256(__file__),
        'assertions':['Native DLL detection feeds every observation mode',
                      'Uncertainty follows detection with unchanged centre-range calibration',
                      'No detection produces no noise draw or controller observation',
                      'Same seed reproduces CSV; seed cannot affect replay detection',
                      'Observation clock independent of .01/.02 integration steps',
                      'Noise-free AEB matches independent native-sensor reference',
                      'Source trajectories preserved; all runs use full DLL',
                      'Shared plot axes and MP4 export verified']})
    print('PASS '+str(folder/'verification.json'),flush=True)


if __name__=='__main__':main()
