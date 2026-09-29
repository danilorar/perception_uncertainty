"""Integration checks with actual esmini. Outputs retained as evidence."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
from settings import ROOT, DEFAULT, write_json, sha256
from runner import new_run_id
from scenario import Reference

def main():
    folder=ROOT/'verification'/new_run_id('checks');folder.mkdir(parents=True)
    checks=[];outputs={}
    variants={
        'ideal':{}, 'no_brake':{'controller':'constant_speed'},
        'zero_noise':{'uncertainty':'gaussian_range','range_sigma_m':0},
        'noise1':{'uncertainty':'gaussian_range','range_sigma_m':2,'seed':42},
        'noise_repeat':{'uncertainty':'gaussian_range','range_sigma_m':2,'seed':42},
        'noise_other_seed':{'uncertainty':'gaussian_range','range_sigma_m':2,'seed':43},
        'heading':{'ego_heading_offset_deg':10,'ego_speed_kph':40},
        'pedestrian':{'scenario':'cpna','ego_speed_kph':30,'export_video':True},
        'pedestrian_reference':{'scenario':'cpna','ego_speed_kph':30,'controller':'constant_speed'},
        'pedestrian_coarse':{'scenario':'cpna','ego_speed_kph':30,'dt_s':.05,'duration_s':20},
        'pedestrian_short_config':{'scenario':'cpna','ego_speed_kph':30,'duration_s':1},
        'junction':{'scenario':'cccscp','ego_speed_kph':20,'export_video':True},
        'coarse_step':{'dt_s':.05}, 'medium_step':{'dt_s':.02},
    }
    for name,overrides in variants.items():
        c=DEFAULT|{'export_video':False}|overrides
        config=folder/(name+'.json');write_json(config,c);out=folder/name
        with (folder/(name+'.log')).open('w',encoding='utf-8') as log:
            p=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(config),'--output',str(out)],stdout=log,stderr=subprocess.STDOUT,timeout=240,creationflags=subprocess.CREATE_NO_WINDOW)
        if p.returncode: raise RuntimeError(f'{name} failed: {folder/(name+".log")}')
        s=json.loads((out/'summary.json').read_text(encoding='utf-8'));outputs[name]=(out,s)
        assert s['status']=='completed',(name,s)
        assert s['target_max_state_error']<1e-5
        ref=Reference(c['scenario'])
        assert s['end_reason']=='original_reference_end'
        assert s['end_time_s']==s['config']['duration_s']==ref.duration
        checks.append({'test':name,'passed':True,'collision':s['collision'],'end_time_s':s['end_time_s'],'brake_time_s':s['first_brake_time_s'],'directory':str(out)})
        print(name,s['collision'],s['first_brake_time_s'],flush=True)
    assert not outputs['ideal'][1]['collision']
    collision=outputs['no_brake'][1]['collision_details']
    assert collision is not None and abs(collision['time_s']-4.70)<1e-8
    assert abs(collision['ego_speed_kph']-50)<1e-8
    assert outputs['pedestrian_reference'][1]['collision']
    assert outputs['pedestrian_reference'][1]['end_time_s']>outputs['pedestrian_reference'][1]['collision_details']['time_s']
    assert not outputs['pedestrian'][1]['collision']
    for filename in ['ego.csv','truth.csv','observations.csv']:
        assert sha256(outputs['ideal'][0]/filename)==sha256(outputs['zero_noise'][0]/filename),filename
        assert sha256(outputs['noise1'][0]/filename)==sha256(outputs['noise_repeat'][0]/filename),filename
    assert sha256(outputs['noise1'][0]/'observations.csv')!=sha256(outputs['noise_other_seed'][0]/'observations.csv')
    assert outputs['ideal'][1]['reference_sha256']==outputs['heading'][1]['reference_sha256']
    with (outputs['heading'][0]/'ego.csv').open() as f: rows=list(csv.DictReader(f))
    assert abs(float(rows[-1]['y_m'])-float(rows[0]['y_m']))>1
    for name,(out,s) in outputs.items():
        with (out/'ego.csv').open() as f: data=list(csv.DictReader(f))
        times=[float(r['time_s']) for r in data]
        dt=s['config']['dt_s']
        assert times[0]==0 and times[-1]==s['config']['duration_s']
        assert all(abs(b-a-dt)<1e-7 for a,b in zip(times[:-2],times[1:-1]))
        assert 0<times[-1]-times[-2]<=dt+1e-7
        assert all(math.isfinite(float(r['speed_mps'])) and float(r['speed_mps'])>=0 for r in data)
        if s['config']['scenario']=='cpna':
            with (out/'truth.csv').open() as f: pedestrian=[r for r in csv.DictReader(f) if r['object_name']=='VRU']
            assert all(abs(float(r['x_m'])-150)<1e-6 for r in pedestrian)
            assert all(float(b['y_m'])>=float(a['y_m'])-1e-7 for a,b in zip(pedestrian,pedestrian[1:]))
        if s['config']['export_video']:
            video=json.loads((out/'video_metadata.json').read_text(encoding='utf-8'))
            assert video['simulation_end_s']==s['end_time_s']
    write_json(folder/'verification.json',{'passed':True,'checks':checks,'assertions':['analytic constant-speed CCRs collision at 4.70s / 50kmh','AEB avoids CCRs and CPNA collision under these settings','zero noise equals ideal data byte-for-byte','same seed reproduces all CSV data byte-for-byte','different seed changes observations','fixed target truth unchanged under ego speed and heading changes','heading affects motion','all runs end exactly at original baseline endpoint, including collisions and shortened final steps','legacy short and long duration settings cannot change original duration','pedestrian follows straight crossing throughout every saved run','pedestrian and intersection video endpoints match original duration','time steps and nonnegative finite speeds']})
    print('ALL PASSED:',folder,flush=True)

if __name__=='__main__': main()
