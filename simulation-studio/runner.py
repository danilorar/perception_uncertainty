"""Single-run and batch CLI. Each experiment has its own directory/process."""
import argparse
import csv
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback
import uuid
from settings import ROOT, RUNS, DEFAULT, RUNTIME, VERSION, validate, write_json, sha256
from scenario import Reference, build_scene
from engine import Engine
from models import Perception, AEB, advance_ego, center
from acceleration import TrajectoryAcceleration
from tracking import RangeTracker, simulation_times, observation_ticks
from ego_path import RecordedEgoPath

def csv_writer(out,name,fields):
    f=(out/name).open('w',encoding='utf-8',newline='')
    w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
    return f,w

def new_run_id(case):
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+case+'_'+uuid.uuid4().hex[:6]

def ideal_reference(config, out):
    """Run the noise-free counterfactual in its own process/DLL instance."""
    c = config | {'uncertainty': 'ideal', 'export_video': False}
    cp = out/'ideal_reference_config.json'
    write_json(cp, c)
    destination = out/'ideal_reference'
    with (out/'ideal_reference.log').open('w', encoding='utf-8') as log:
        result = subprocess.run([sys.executable, '-X', 'utf8', str(ROOT/'runner.py'),
            'run', '--config', str(cp), '--output', str(destination), '--diagnostic-only'],
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=180,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
    if result.returncode:
        raise RuntimeError('独立 esmini 理想对照失败；详见 ideal_reference.log。')
    summary = json.loads((destination/'summary.json').read_text(encoding='utf-8'))
    if summary['status'] not in ['completed', 'completed_with_warnings']:
        raise RuntimeError('独立 esmini 理想对照未完成')
    with (destination/'ego.csv').open(encoding='utf-8', newline='') as stream:
        frames = {float(row['time_s']): row for row in csv.DictReader(stream)}
    return frames, summary['first_brake_time_s']


def run(config,out, *, diagnostic_only=False):
    c=validate(config); out=Path(out).resolve(); out.mkdir(parents=True,exist_ok=False)
    os.chdir(out)
    write_json(out/'config.json',c)
    command=[sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(out/'config.json')]
    if diagnostic_only: command.append('--diagnostic-only')
    (out/'reproduce.ps1').write_text('& '+' '.join("'"+v.replace("'","''")+"'" for v in command)+'\n',encoding='utf-8-sig')
    summary={'status':'running','run_id':out.name,'config':c,'created_utc':datetime.now(timezone.utc).isoformat(),'collision':None,'video_status':'pending' if c['export_video'] else 'not_requested'}
    write_json(out/'summary.json',summary)
    files=[]; engine=None
    try:
        ref=Reference(c['scenario'])
        if c['duration_s'] != ref.duration:
            raise ValueError('原版基准在启动期间发生变化，请重新运行。')
        initial=ref.states(0)
        ego=next(s for s in initial if s['id']==ref.ego_id)
        replay=c['controller']=='original_trajectory'
        follow=c['controller']=='aeb_follow'
        has_aeb=c['controller'] in ['aeb','aeb_follow']
        ideal_frames={}; ideal_brake_time=None
        if has_aeb and not diagnostic_only:
            write_json(out/'progress.json',{'stage':'ideal_reference','fraction':.02})
            ideal_frames,ideal_brake_time=ideal_reference(c,out)
        if not replay and not follow:
            ego['speed']=c['ego_speed_kph']/3.6
            ego['h']=math.radians(c['ego_heading_deg'])
            ego['vx'],ego['vy']=ego['speed']*math.cos(ego['h']),ego['speed']*math.sin(ego['h'])
        build_scene(out/'scene.xosc',ref.meta,initial,c['duration_s'])
        shutil.copy2(ref.directory/'truth.csv',out/'fixed_reference.csv')
        write_json(out/'reference.json',ref.meta)
        acceleration_estimate=TrajectoryAcceleration(out/'fixed_reference.csv',ref.ego_id) if replay or follow else None
        if acceleration_estimate:
            write_json(out/'acceleration_estimate.json',acceleration_estimate.metadata)
        versions={'studio':VERSION,'python':sys.version,'esmini':(RUNTIME/'version.txt').read_text(),
            'esmini_runtime_directory':str(RUNTIME),
            'esmini_installation':json.loads((RUNTIME/'installation.json').read_text(encoding='utf-8')) if (RUNTIME/'installation.json').exists() else None,
            'esmini_dll_sha256':sha256(RUNTIME/'bin/esminiLib.dll'),
            'perception_backend':'esmini_ideal_object_sensor',
            'packages':{n:importlib.metadata.version(n) for n in ['matplotlib','imageio-ffmpeg','numpy','pillow']},
            'implementation_sha256':{p.name:sha256(p) for p in ROOT.glob('*.py')}}
        write_json(out/'versions.json',versions)
        engine=Engine(); engine.init(out/'scene.xosc',out/'esmini.log',dat=None if diagnostic_only else out/'replay.dat')
        sensor_id=engine.add_object_sensor(ego,c)
        write_json(out/'sensor.json',engine.sensors[sensor_id])
        for s in initial: engine.report(s)
        engine.step(0)
        write_json(out/'engine_args.json',engine.args)
        truth_fields=['time_s','object_id','object_name','object_type','x_m','y_m','z_m','heading_rad','pitch_rad','roll_rad','speed_mps','vx_mps','vy_mps','length_m','width_m','height_m','center_offset_x_m','center_offset_y_m','center_offset_z_m']
        ef,ew=csv_writer(out,'ego.csv',['time_s','x_m','y_m','heading_rad','speed_mps','speed_kph','command_accel_mps2','trajectory_accel_mps2','predicted_contact_time_s','brake_active','collision_ids','aeb_state','fresh_observation','ideal_ego_speed_kph','ideal_aeb_state'])
        tf,tw=csv_writer(out,'truth.csv',truth_fields)
        obsfields=['time_s','object_id','object_name','visible','observation_valid','true_range_m','observed_range_m','raw_observed_range_m','range_error_mean_m','range_error_sigma_m','range_error_expected_abs_m','sampled_range_error_m','actual_range_error_m','bearing_deg','observed_center_x_m','observed_center_y_m']
        obsfields+=['estimated_range_m','estimated_range_rate_mps','estimate_std_m','track_ready','track_samples','controller_range_m','innovation_m','filter_status']
        obsfields+=['assumed_range_sigma_m']
        obsfields+=['sensor_id','sensor_detected','ideal_range_m','sensor_reference_range_m']
        of,ow=csv_writer(out,'observations.csv',obsfields); files=[ef,tf,of]
        sf=(out/'sensor_frames.jsonl').open('w',encoding='utf-8'); files.append(sf)
        perception,controller=Perception(c),AEB(c)
        tracker=RangeTracker(c)
        path=RecordedEgoPath(ref) if follow else None
        # Counterfactual data is used only for exported diagnostics, never for
        # detection, tracking, or commands on the actual trajectory.
        measured_errors=[]; estimated_errors=[]; matched_observed_errors=[]; observation_samples=0
        min_range=None; brake_range=None; stop_time=None
        collision=None; target_error=0.0; ego_replay_error=0.0; rows=0; invalid_observations=0
        times=simulation_times(c['duration_s'],c['dt_s'],c['observation_period_s'])
        ticks=set(observation_ticks(c['duration_s'],c['observation_period_s'] or c['dt_s']))
        for i,t in enumerate(times):
            targets=[s for s in ref.states(t) if s['id']!=ref.ego_id]
            # Values fed to esmini must agree with the fixed target data.
            actual={s['id']:s for s in engine.states()}
            for s in targets:
                target_error=max(target_error, *(abs(actual[s['id']][k]-s[k]) for k in ['x','y','speed']))
            if target_error > 1e-5:
                raise RuntimeError(f'固定对象轨迹偏差超过容差: {target_error}')
            if replay:
                expected=next(s for s in ref.states(t) if s['id']==ref.ego_id)
                ego_replay_error=max(ego_replay_error,*(abs(actual[ref.ego_id][k]-expected[k]) for k in ['x','y','speed']))
                if ego_replay_error > 1e-5:
                    raise RuntimeError(f'自车原始回放偏差超过容差: {ego_replay_error}')
            fresh=t in ticks
            records=[]; observations=[]
            if fresh:
                detections=engine.detect(sensor_id)
                sf.write(json.dumps({'time_s':t,'sensor_id':sensor_id,
                    'detected_ids':[s['id'] for s in detections]},allow_nan=False)+'\n')
                raw_observations,records=perception.observe(ego,targets,detections=detections)
                observations,diagnostics=tracker.update(t,ego,raw_observations)
                for record in records:
                    record['sensor_id']=sensor_id
                    record.update(diagnostics.get(record['object_id'],dict(estimated_range_m=None,
                        estimated_range_rate_mps=None,estimate_std_m=None,track_ready=False,track_samples=0,
                        controller_range_m=None,innovation_m=None,filter_status='no_valid_observation')))
                    if record['observation_valid']:
                        measured_errors.append(record['observed_range_m']-record['true_range_m'])
                    if record['track_ready']:
                        estimated_errors.append(record['estimated_range_m']-record['true_range_m'])
                        matched_observed_errors.append(record['observed_range_m']-record['true_range_m'])
                observation_samples+=1
            invalid_observations+=sum(r['visible'] and not r['observation_valid'] for r in records)
            acceleration,ttc=(0.0,None) if replay else controller.command(t,ego,observations,fresh=fresh)
            distances={s['id']:math.dist(center(ego),center(s)) for s in targets}
            if distances:
                min_range=min([*distances.values()]+([min_range] if min_range is not None else []))
            if controller.brake_time==t and brake_range is None:
                brake_range=distances.get(controller.pending_object,min(distances.values(),default=None))
            if has_aeb and controller.brake_time is not None and stop_time is None and ego['speed']<=1e-6:
                stop_time=t
            trajectory_accel=acceleration_estimate.at(t) if replay or (follow and controller.brake_time is None) else None
            hits=engine.collisions(ref.ego_id)
            if hits and collision is None:
                collision={'time_s':t,'ego_speed_mps':ego['speed'],'ego_speed_kph':ego['speed']*3.6,'object_ids':hits,
                    'relative_speed_kph':max(math.hypot(ego['vx']-s['vx'],ego['vy']-s['vy'])*3.6 for s in targets if s['id'] in hits)}
            ew.writerow({'time_s':t,'x_m':ego['x'],'y_m':ego['y'],'heading_rad':ego['h'],
                'speed_mps':ego['speed'],'speed_kph':ego['speed']*3.6,'command_accel_mps2':None if replay or (follow and controller.brake_time is None) else acceleration,
                'trajectory_accel_mps2':trajectory_accel,
                'predicted_contact_time_s':ttc,'brake_active':controller.brake_time is not None,'collision_ids':';'.join(map(str,hits)),
                'aeb_state':controller.state if has_aeb else 'not_applicable','fresh_observation':fresh,
                'ideal_ego_speed_kph':ideal_frames[t]['speed_kph'] if ideal_frames else None,
                'ideal_aeb_state':ideal_frames[t]['aeb_state'] if ideal_frames else 'not_applicable'})
            for s in [ego]+targets:
                tw.writerow(dict(zip(truth_fields,[t,s['id'],s['name'],s['objectType'],s['x'],s['y'],s['z'],s['h'],s['p'],s['r'],s['speed'],s['vx'],s['vy'],s['length'],s['width'],s['height'],s['centerOffsetX'],s['centerOffsetY'],s['centerOffsetZ']])))
            for r in records: ow.writerow({'time_s':t,**r})
            rows+=1
            if i%50==0:
                write_json(out/'progress.json',{'stage':'simulation','time_s':t,'duration_s':c['duration_s'],'fraction':t/c['duration_s']*.65})
            if i==len(times)-1: break
            # A shortened final step reaches the exact original endpoint, even
            # when the selected timestep does not divide its duration.
            next_t=times[i+1]
            step_dt=round(next_t-t,8)
            if replay or (follow and controller.brake_time is None):
                ego=next(s for s in ref.states(next_t) if s['id']==ref.ego_id)
            elif follow:
                ego=path.advance(ego,acceleration,step_dt,t)
            else:
                ego=advance_ego(ego,acceleration,step_dt)
            next_targets=[s for s in ref.states(next_t) if s['id']!=ref.ego_id]
            for s in [ego]+next_targets: engine.report(s)
            engine.step(step_dt)
        sensor_metadata=engine.sensors[sensor_id].copy()
        write_json(out/'sensor.json',sensor_metadata)
        engine.close(); engine=None
        for f in files: f.close()
        files=[]
        summary.update(collision=collision is not None,collision_details=collision,first_trigger_time_s=controller.trigger_time,
            first_brake_time_s=controller.brake_time,final_speed_kph=ego['speed']*3.6,end_time_s=t,
            samples=rows,end_reason='recorded_trajectory_end',timing_policy=ref.meta['timing_policy'],
            target_max_state_error=target_error,reference_sha256=ref.meta['truth_sha256'],
            ego_replay_max_state_error=ego_replay_error if replay else None,
            acceleration_estimate=acceleration_estimate.metadata if acceleration_estimate else None,
            invalid_range_observations=invalid_observations,
            sensor=sensor_metadata,
            ideal_reference={'directory':'ideal_reference','backend':'esmini_ideal_object_sensor','independent_process':True} if ideal_frames else None,
            diagnostic_only=diagnostic_only,
            assessment={'ideal_first_brake_time_s':ideal_brake_time,
                'brake_advance_vs_ideal_s':(ideal_brake_time-controller.brake_time if ideal_brake_time is not None and controller.brake_time is not None else None),
                'braked_without_ideal_intervention':bool(ideal_frames) and controller.brake_time is not None and ideal_brake_time is None,
                'true_center_range_at_brake_m':brake_range,'minimum_center_range_m':min_range,'stop_time_s':stop_time,
                'cancelled_confirmations':controller.cancelled_confirmations,'observation_frames':observation_samples,
                'observed_range_rmse_m':math.sqrt(sum(e*e for e in measured_errors)/len(measured_errors)) if measured_errors else None,
                'estimated_range_rmse_m':math.sqrt(sum(e*e for e in estimated_errors)/len(estimated_errors)) if estimated_errors else None,
                'matched_observed_range_rmse_m':math.sqrt(sum(e*e for e in matched_observed_errors)/len(matched_observed_errors)) if matched_observed_errors else None,
                'observed_error_samples':len(measured_errors),'estimated_error_samples':len(estimated_errors),
                'meaning':'Positive brake advance means earlier than an independent ideal-range counterfactual with the same driver, gates, observation clock and AEB logic. Not a false-positive label. RMSE is against each run truth; observed uses valid readings, estimated uses ready tracks.'},
            source_warnings=ref.meta.get('source_warnings',[]),
            interpretation='Geometric overlap of truth bounding boxes at sampled timesteps. No physical crash response. No collision means no collision within this run horizon.',
            model=('Original trajectory replay for both vehicles; derived velocities; no controller commands and no HiDriveController implementation.' if replay else 'Original ego motion until actual AEB brake onset, then AEB replaces driver speed along recorded ego path.' if follow else 'Fixed-heading AEB or constant-speed ego. Teacher target trajectory retained.'),
            perception='Native esmini ideal object sensor in every uncertainty mode. Only detected IDs are read via esmini state/velocity APIs; centre range then passes through the configured uncertainty unit. No occlusion or velocity error.',
            tracking={'mode':c['observation_filter'],'period_s':c['observation_period_s'] or c['dt_s'],
                'state':'centre range and radial range rate','range_sigma_m':c['kalman_range_sigma_m'],
                'noise_mode':c['kalman_noise_mode'],
                'range_sigma_policy':'Fixed mode: configured sigma. Predicted-range mode with distance noise: bootstrap max(configured sigma, sigma(first measurement)); then R=max(0.01, slope^2*(max(0, predicted range-zero)^2+predicted range variance)). No true range input; negative-reading selection bias is not corrected.',
                'velocity_assumption':'Target velocity and bearing are assumed accurate, including radial-rate auxiliary updates; not a noisy-velocity benchmark.',
                'warmup_s':c['kalman_warmup_s'],'missing':'No AEB evidence from missing/invalid observations; reacquire after > max(0.2 s, 2.5 observation periods).',
                'exact_mode':'Noise-free configurations bypass filtering and warmup.'},
            aeb_logic={'mode':c['aeb_trigger_mode'],'confirmation_s':c['aeb_confirmation_s'],
                'policy':'Same target must meet the contact threshold at each fresh observation through confirmation and response delay. Safe/missing evidence cancels before braking. Once braking starts, hold to stop. Instant mode retains legacy single-hit latching.'})
        if c['uncertainty']=='distance_gaussian_range':
            summary['range_error_model']={
                'distribution':'N(0, sigma(r)^2)', 'distance':'true bounding-box centre-to-centre range in metres',
                'zero_error_range_m':c['noise_zero_range_m'], 'reference_range_m':c['noise_reference_range_m'],
                'reference_mean_absolute_error_pct':c['noise_reference_mae_pct'],
                'mean_absolute_error_m':'reference_range * reference_mae_pct / 100 * max(0, r - zero_range) / (reference_range - zero_range)',
                'sigma_m':'mean_absolute_error_m * sqrt(pi/2)',
                'beyond_reference':'Continue the same linear MAE slope; no cap.',
                'invalid_measurements':'Negative raw ranges are saved and marked invalid, not clamped or resampled; excluded from controller observations. Calibration is for all sampled errors, not only valid measurements.',
                'sampling':'Independent on each visible-object observation tick. Period 0 uses legacy simulation ticks; otherwise observation clock is independent of integration dt.',
            }
        write_json(out/'summary.json',summary)
        if not diagnostic_only:
            write_json(out/'progress.json',{'stage':'plots','fraction':.7})
            from plots import make_plots
            make_plots(out)
        if c['export_video'] and not diagnostic_only:
            write_json(out/'progress.json',{'stage':'video','fraction':.8})
            with (out/'video.log').open('w',encoding='utf-8') as log:
                try:
                    result=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'video.py'),str(out)],stdout=log,stderr=subprocess.STDOUT,timeout=240,creationflags=subprocess.CREATE_NO_WINDOW)
                    video_failed=result.returncode != 0
                except subprocess.TimeoutExpired:
                    log.write('Video export exceeded the 240-second limit.\n')
                    video_failed=True
            if video_failed or not (out/'video.mp4').exists():
                summary['video_status']='failed'; summary['status']='completed_with_warnings'
                summary['warning']='仿真与数据已完成，但视频导出失败；详见 video.log。'
            else:
                summary['video_status']='ready'; summary['status']='completed'
        else: summary['status']='completed'
        log=(out/'esmini.log').read_text(encoding='utf-8',errors='replace')
        summary['esmini_error_lines']=[line for line in log.splitlines() if '[error]' in line]
        if summary['esmini_error_lines']:
            summary['status']='completed_with_warnings'
            summary['warning']='；'.join(ref.meta.get('source_warnings',[])) or '仿真完成，esmini 有提示；详见 esmini.log。'
        write_json(out/'summary.json',summary)
        write_json(out/'progress.json',{'stage':'done','fraction':1})
        print(json.dumps({'directory':str(out),'status':summary['status'],'collision':summary['collision']},ensure_ascii=False),flush=True)
        return summary
    except Exception as exc:
        summary.update(status='failed',error=str(exc))
        write_json(out/'summary.json',summary)
        (out/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
        raise
    finally:
        for f in files: f.close()
        if engine: engine.close()

def main():
    parser=argparse.ArgumentParser(description='TME180 Simulation Studio')
    subs=parser.add_subparsers(dest='command',required=True)
    single=subs.add_parser('run'); single.add_argument('--config',required=True); single.add_argument('--output')
    single.add_argument('--diagnostic-only',action='store_true',help=argparse.SUPPRESS)
    batch=subs.add_parser('batch'); batch.add_argument('--matrix',required=True)
    args=parser.parse_args()
    if args.command=='run':
        c=validate(json.loads(Path(args.config).read_text(encoding='utf-8-sig')))
        return run(c,Path(args.output) if args.output else RUNS/new_run_id(c['scenario']),diagnostic_only=args.diagnostic_only)
    matrix=json.loads(Path(args.matrix).read_text(encoding='utf-8-sig'))
    base=validate(matrix.get('base',{})); variants=matrix.get('runs',[])
    if not isinstance(variants,list) or not 1<=len(variants)<=1000:
        raise ValueError('runs 必须是包含 1–1000 组配置的列表')
    configs=[validate(base | item) for item in variants]
    folder=ROOT/'batches'/new_run_id('batch'); folder.mkdir(parents=True)
    shutil.copy2(args.matrix,folder/'matrix.json')
    results=[]
    for i,c in enumerate(configs):
        cp=folder/f'config_{i+1:04}.json'; write_json(cp,c)
        out=RUNS/new_run_id(c['scenario'])
        with (folder/f'run_{i+1:04}.log').open('w',encoding='utf-8') as log:
            p=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'runner.py'),'run','--config',str(cp),'--output',str(out)],stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        s=json.loads((out/'summary.json').read_text(encoding='utf-8')) if (out/'summary.json').exists() else {'status':'failed'}
        results.append({'run_id':out.name,'scenario':c['scenario'],'seed':c['seed'],'uncertainty':c['uncertainty'],
            'sigma_m':c['range_sigma_m'] if c['uncertainty']=='gaussian_range' else None,
            'zero_error_range_m':c['noise_zero_range_m'] if c['uncertainty']=='distance_gaussian_range' else None,
            'reference_range_m':c['noise_reference_range_m'] if c['uncertainty']=='distance_gaussian_range' else None,
            'reference_mae_pct':c['noise_reference_mae_pct'] if c['uncertainty']=='distance_gaussian_range' else None,
            'observation_filter':c['observation_filter'],'aeb_trigger_mode':c['aeb_trigger_mode'],'observation_period_s':c['observation_period_s'],
            'brake_advance_vs_ideal_s':s.get('assessment',{}).get('brake_advance_vs_ideal_s'),
            'true_center_range_at_brake_m':s.get('assessment',{}).get('true_center_range_at_brake_m'),
            'status':s['status'],'exit_code':p.returncode,'collision':s.get('collision'),'impact_speed_kph':(s.get('collision_details') or {}).get('ego_speed_kph'),'brake_time_s':s.get('first_brake_time_s')})
        with (folder/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,results[0].keys());w.writeheader();w.writerows(results)
        print(f'[{i+1}/{len(configs)}] {out.name}: {s["status"]}',flush=True)
    print(str(folder),flush=True)

if __name__=='__main__': main()
