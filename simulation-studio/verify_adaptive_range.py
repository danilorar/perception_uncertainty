"""Paired fixed/adaptive checks in fresh folders; no UI or existing runs touched."""
import csv
import json
import os
import statistics
import subprocess
import sys
from datetime import datetime, timezone

from models import Perception
from scenario import Reference, build_scene
from engine import Engine
from settings import DEFAULT, ROOT, SCENARIOS, validate, write_json
from tracking import RangeTracker, observation_ticks


def native_replay(ref, c, folder):
    out=folder/c['scenario']; out.mkdir()
    states=ref.states(0)
    ego=next(s for s in states if s['id']==ref.ego_id)
    build_scene(out/'scene.xosc',ref.meta,states,c['duration_s'])
    engine=Engine()
    frames=[]
    try:
        engine.init(out/'scene.xosc',out/'esmini.log')
        sensor=engine.add_object_sensor(ego,c)
        previous=0.0
        for t in observation_ticks(c['duration_s'],c['observation_period_s']):
            states=ref.states(t)
            for s in states: engine.report(s)
            engine.step(round(t-previous,8)); previous=t
            frames.append((t,next(s for s in states if s['id']==ref.ego_id),
                           [s for s in states if s['id']!=ref.ego_id],engine.detect(sensor)))
        write_json(out/'sensor.json',engine.sensors[sensor])
    finally:
        engine.close()
    return frames


def main():
    folder=ROOT/'verification'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_adaptive_range')
    folder.mkdir()
    replay=[]
    # Exactly the same observations go to both estimators; truth is used only
    # outside the trackers to score their output. Twenty seeds per scene.
    for scene in SCENARIOS:
        ref=Reference(scene)
        frames=native_replay(ref,validate(DEFAULT|{'scenario':scene}),folder)
        for seed in range(1,21):
            c=validate(DEFAULT|{'scenario':scene,'seed':seed,'uncertainty':'distance_gaussian_range','export_video':False})
            perception=Perception(c)
            trackers={m:RangeTracker(c|{'kalman_noise_mode':m}) for m in ['fixed','predicted_range']}
            errors={m:[] for m in trackers}
            for t,ego,targets,detections in frames:
                obs,records=perception.observe(ego,targets,detections=detections)
                truths={r['object_id']:r['true_range_m'] for r in records}
                diags={m:tracker.update(t,ego,obs)[1] for m,tracker in trackers.items()}
                for ident in truths:
                    if all(diags[m].get(ident,{}).get('track_ready') for m in trackers):
                        for m in trackers: errors[m].append(diags[m][ident]['estimated_range_m']-truths[ident])
            replay.append({'scenario':scene,'seed':seed,**{m:(sum(e*e for e in v)/len(v))**.5 for m,v in errors.items()}})
    write_json(folder/'paired_replay.json',replay)
    checks=[]
    for scene in SCENARIOS:
        for seed in [1,7,42]:
            for mode in ['fixed','predicted_range']:
                label=f'{scene}_{seed}_{mode}'
                c=validate(DEFAULT|{'scenario':scene,'seed':seed,'controller':'aeb_follow','uncertainty':'distance_gaussian_range','kalman_noise_mode':mode,'export_video':False})
                cp=folder/(label+'.json'); write_json(cp,c)
                out=folder/label
                with (folder/(label+'.log')).open('w',encoding='utf-8') as log:
                    result=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(cp),'--output',str(out)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0,timeout=180)
                s=json.loads((out/'summary.json').read_text(encoding='utf-8'))
                assert result.returncode==0 and s['status'] in ['completed','completed_with_warnings'],s
                assert s['target_max_state_error'] < 1e-5
                with (out/'observations.csv').open() as f:
                    rows=list(csv.DictReader(f))
                assert any(r['assumed_range_sigma_m'] for r in rows)
                checks.append({'scenario':scene,'seed':seed,'mode':mode,'collision':s['collision'],'brake_s':s['first_brake_time_s'],'advance_s':s['assessment']['brake_advance_vs_ideal_s'],'rmse_m':s['assessment']['estimated_range_rmse_m']})
    aggregate={scene:{m:statistics.mean(r[m] for r in replay if r['scenario']==scene) for m in ['fixed','predicted_range']} for scene in SCENARIOS}
    report={'passed':True,'replay_mean_run_rmse_m':aggregate,'closed_loop':checks,'note':'Replay uses identical observations, 20 seeds/scene; closed-loop uses 3 seeds/scene and paths can diverge. Mean of per-run RMSE, not pooled RMSE. No claim of universal improvement or corrected rejection bias.'}
    write_json(folder/'verification.json',report)
    print(json.dumps(report,indent=2),flush=True)
    print(folder,flush=True)


if __name__=='__main__': main()
