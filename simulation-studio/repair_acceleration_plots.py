"""Correct only acceleration postprocessing of completed teacher replay results.

Keep original derived outputs in each result's postprocessing-backups folder.
Simulation provenance, input data, observations, video and collision metrics stay
unchanged. This is a local, explicit maintenance operation, not a startup hook.
"""
import csv
from datetime import datetime,timezone
import json
from pathlib import Path
import shutil
from acceleration import TrajectoryAcceleration
from plots import make_plots
from settings import ROOT, RUNS, sha256, write_json


def repair(out):
    summary_path=out/'summary.json'
    summary=json.loads(summary_path.read_text(encoding='utf-8'))
    if (summary.get('status') not in ['completed','completed_with_warnings']
            or summary.get('config',{}).get('controller')!='original_trajectory'):
        return None
    meta=json.loads((out/'reference.json').read_text(encoding='utf-8'))
    if meta.get('source_kind')!='teacher_trajectory':
        return None
    series=TrajectoryAcceleration(out/'fixed_reference.csv',meta['ego_id'])
    if summary.get('acceleration_estimate')==series.metadata:
        return None
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    backup=out/'postprocessing-backups'/stamp
    backup.mkdir(parents=True)
    for name in ['ego.csv','speed.png','speed.svg','summary.json','acceleration_estimate.json']:
        if (out/name).exists():shutil.copy2(out/name,backup/name)
    protected={p.name:sha256(p) for p in out.iterdir() if p.name in
               ['truth.csv','observations.csv','video.mp4','replay.dat','config.json','reference.json','versions.json']}
    with (out/'ego.csv').open(encoding='utf-8',newline='') as f:
        reader=csv.DictReader(f);fields=reader.fieldnames;rows=list(reader)
    original=[r.copy() for r in rows]
    for row in rows:row['trajectory_accel_mps2']=series.at(float(row['time_s']))
    temporary=out/'ego.csv.tmp'
    with temporary.open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    temporary.replace(out/'ego.csv')
    write_json(out/'acceleration_estimate.json',series.metadata)
    make_plots(out,speed_only=True)
    summary['acceleration_estimate']=series.metadata
    summary['acceleration_postprocessing']={
        'updated_utc':stamp,'backup_directory':str(backup.relative_to(out)),
        'implementation_sha256':{name:sha256(ROOT/name) for name in ['acceleration.py','plots.py']},
        'simulation_unchanged':True,
    }
    write_json(summary_path,summary)
    with (out/'ego.csv').open(encoding='utf-8',newline='') as f:updated=list(csv.DictReader(f))
    assert len(original)==len(updated)
    for before,after in zip(original,updated):
        assert {k:v for k,v in before.items() if k!='trajectory_accel_mps2'}=={k:v for k,v in after.items() if k!='trajectory_accel_mps2'}
    assert all(sha256(out/name)==digest for name,digest in protected.items())
    return {'run_id':out.name,'corrected_rows':len(rows),'simulation_files_unchanged':True,'backup':str(backup)}


if __name__=='__main__':
    results=[]
    for directory in sorted(RUNS.iterdir()):
        if directory.is_dir() and (directory/'summary.json').exists():
            result=repair(directory)
            if result:
                results.append(result)
                print(json.dumps(result,ensure_ascii=False),flush=True)
    write_json(ROOT/'work/acceleration-repair.json',{'corrected_runs':results})
