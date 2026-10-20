"""Studio outputs around the supplied HiDrive AEB controller.

The two controller sources are opaque compiler inputs: Studio never opens,
parses or hashes them. It only compares file size/mtime with the build copy,
starts the isolated esmini 3.8.1 executable (controller ported to the esmini 3
API) on the teacher scenario (which
already declares HiDriveController for object_1), and converts the engine's
public outputs (CSV logger and LOG_VALUE lines) into the usual Studio files.

Studio range errors reach the controller through the public engine hook in
hidrive/PerceptionNoise.hpp: Studio writes unit OU samples per target and step,
esmini moves each target to its perceived centre only during the controller
step and restores the truth afterwards, and logs every applied error.
"""
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

from models import CorrelatedRangeNoise, Observation, center, longitudinal_risk
from scenario import Reference
from settings import HIDRIVE_RUNTIME, HIDRIVE_SOURCE, ROOT, RUNTIME, VERSION, sha256, write_json

TRUTH_FIELDS = ['time_s', 'object_id', 'object_name', 'object_type', 'x_m', 'y_m', 'z_m',
                'heading_rad', 'pitch_rad', 'roll_rad', 'speed_mps', 'vx_mps', 'vy_mps',
                'length_m', 'width_m', 'height_m', 'center_offset_x_m', 'center_offset_y_m',
                'center_offset_z_m', 'object_category']
EGO_FIELDS = ['time_s', 'x_m', 'y_m', 'heading_rad', 'speed_mps', 'speed_kph',
              'command_accel_mps2', 'trajectory_accel_mps2', 'predicted_contact_time_s',
              'brake_active', 'collision_ids', 'aeb_state', 'fresh_observation',
              'ideal_ego_speed_kph', 'ideal_aeb_state', 'controller_state']
OBS_FIELDS = ['time_s', 'object_id', 'object_name', 'visible', 'observation_valid',
              'true_range_m', 'observed_range_m', 'raw_observed_range_m', 'range_error_mean_m',
              'range_error_sigma_m', 'range_error_expected_abs_m', 'range_error_normalized',
              'range_error_transition_rho', 'range_error_elapsed_s', 'sampled_range_error_m',
              'actual_range_error_m', 'bearing_deg', 'observed_center_x_m', 'observed_center_y_m',
              'controller_range_m', 'sensor_id', 'sensor_detected', 'ideal_range_m',
              'sensor_reference_range_m']
EXECUTABLE = HIDRIVE_RUNTIME / 'bin/esmini.exe'
CONTROLLERS = HIDRIVE_RUNTIME / 'EnvironmentSimulator/Modules/Controllers'
PRIVATE_NAMES = ('ControllerHiDrive.cpp', 'ControllerHiDrive.hpp')
NOISE_HEADER = ROOT / 'hidrive/PerceptionNoise.hpp'
# Invalid (negative) measured ranges move the target this far behind the ego.
HIDE_DISTANCE_M = 1000.0
NOISE_MARGIN_S = 2.0
# esmini 3.x log line: "[12.340] [info] LOG_VALUE:name:value" (optional [file::func::line] metadata).
LOG_VALUE = re.compile(r'\[([0-9.]+)\] \[\w+\] (?:\[[^\]]*\] )?LOG_VALUE:(\w+):(\S+)')


def controller_metadata(directory):
    """Filesystem metadata only; never read the private controller sources."""
    return {name: {'size': (Path(directory) / name).stat().st_size,
                   'mtime_ns': (Path(directory) / name).stat().st_mtime_ns}
            for name in PRIVATE_NAMES if (Path(directory) / name).is_file()}


def check_build():
    if not EXECUTABLE.is_file():
        raise RuntimeError('未找到 HiDrive 版 esmini；请先运行 python hidrive/build.py。')
    built = controller_metadata(CONTROLLERS)
    supplied = controller_metadata(HIDRIVE_SOURCE)
    if supplied and supplied != built:
        raise RuntimeError('“AEB Controler” 中的控制器文件已更新；请先运行 python hidrive/build.py 重新编译。')
    info_path = HIDRIVE_RUNTIME / 'hidrive-build.json'
    info = json.loads(info_path.read_text(encoding='utf-8')) if info_path.is_file() else {}
    if info.get('perception_noise_sha256') != sha256(NOISE_HEADER):
        raise RuntimeError('HiDrive 版 esmini 的感知误差接口不是最新；请先运行 python hidrive/build.py 重新编译。')
    return built, supplied, info


def build_hidrive_scene(path, metadata):
    """Teacher scenario unchanged except absolute road/catalog paths."""
    source = Path(metadata['source_file'])
    tree = ET.parse(source)
    root = tree.getroot()
    root.find('./RoadNetwork/LogicFile').set('filepath', metadata['road_file'])
    for directory in root.findall('./CatalogLocations/*/Directory'):
        directory.set('path', str((source.parent / directory.get('path')).resolve()))
    controller = root.find('./Entities/ScenarioObject[@name="object_1"]/ObjectController/Controller')
    if controller is None or controller.get('name') != 'HiDriveController':
        raise RuntimeError('源场景的 object_1 未声明 HiDriveController。')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return {p.get('name'): p.get('value') for p in controller.findall('./Properties/Property')}


def write_noise_file(path, c, ego_name, targets, log_path, duration_s):
    """Unit OU samples per target at every integration step.

    One independent stream per object (seed:range-ou:id); the distance-dependent
    amplitude is applied inside esmini on the live true centre range.
    """
    noise = CorrelatedRangeNoise(c['seed'], c['noise_correlation_time_s'])
    steps = round((duration_s + NOISE_MARGIN_S) / c['dt_s']) + 1
    lines = [f'observer\t{ego_name}', f'zero_range_m\t{c["noise_zero_range_m"]!r}',
             f'reference_range_m\t{c["noise_reference_range_m"]!r}',
             f'reference_mae_pct\t{c["noise_reference_mae_pct"]!r}', f'dt_s\t{c["dt_s"]!r}',
             f'hide_distance_m\t{HIDE_DISTANCE_M!r}', f'log\t{log_path}']
    for target in sorted(targets, key=lambda s: s['id']):
        samples = [noise.sample(target['id'], round(k * c['dt_s'], 8))[0] for k in range(steps)]
        lines.append('\t'.join(['z', target['name'], *map(repr, samples)]))
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def read_perception_log(path):
    """Errors esmini applied during controller steps: {(time, name): row}."""
    with Path(path).open(encoding='utf-8') as stream:
        return {(round(float(r['time_s']), 8), r['object_name']): r
                for r in csv.DictReader(stream, delimiter='\t')}


def read_esmini_csv(path):
    """Parse the esmini --csv_logger file into {time: {name: fields}}."""
    frames = {}
    header = None
    with Path(path).open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            cells = [cell.strip() for cell in line.split(',')]
            if cells[0].startswith('Index'):
                header = cells
                continue
            if header is None or not cells[0].isdigit():
                continue
            t = round(float(cells[1]), 8)
            entities = {}
            for name, value in zip(header[2:], cells[2:]):
                match = re.match(r'#(\d+) (.+?)(?: \[.*\])?$', name)
                if match:
                    entities.setdefault(int(match[1]), {})[match[2].strip()] = value
            frames[t] = {e['Entity_Name']: e for e in entities.values()}
    if not frames:
        raise RuntimeError('HiDrive esmini 没有写出 CSV 数据。')
    return frames


def read_log_values(path):
    values = {}
    with Path(path).open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            match = LOG_VALUE.match(line)
            if match:
                values.setdefault(round(float(match[1]), 8), {})[match[2]] = match[3]
    return values


def finite(value):
    return value if math.isfinite(value) else None


def _truth_observation(state):
    """Truth-only evaluation helper (TTC/gap diagnostics), never given to the controller."""
    x, y = center(state)
    return Observation(state['id'], x, y, state['vx'], state['vy'], state['h'],
                       state['length'], state['width'], state['speed'])


def number(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def to_state(row, base):
    speed = float(row['Current_Speed'])
    state = dict(base)
    state.update(id=base['id'], name=base['name'],
                 x=float(row['World_Position_X']), y=float(row['World_Position_Y']),
                 z=float(row['World_Position_Z']), h=float(row['World_Heading_Angle']),
                 p=float(row['World_Pitch_Angle']), r=0.0, speed=speed,
                 vx=float(row['Vel_X']), vy=float(row['Vel_Y']),
                 length=float(row['bb_length']), width=float(row['bb_width']),
                 height=float(row['bb_height']), centerOffsetX=float(row['bb_x']),
                 centerOffsetY=float(row['bb_y']), centerOffsetZ=float(row['bb_z']))
    return state


def run_hidrive(c, out, *, diagnostic_only=False, plot_axes_policy=None):
    from runner import csv_writer, finish_run
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.chdir(out)
    write_json(out / 'config.json', c)
    if plot_axes_policy is not None:
        shutil.copy2(plot_axes_policy, out / 'plot_axes_policy.json')
    command = [sys.executable, '-X', 'utf8', str(ROOT / 'runner.py'), 'run', '--config', str(out / 'config.json')]
    (out / 'reproduce.ps1').write_text('& ' + ' '.join("'" + p.replace("'", "''") + "'" for p in command) + '\n', encoding='utf-8-sig')
    summary = dict(status='running', run_id=out.name, config=c, collision=None,
                   created_utc=datetime.now(timezone.utc).isoformat(),
                   video_status='pending' if c['export_video'] else 'not_requested')
    write_json(out / 'summary.json', summary)
    files = []
    try:
        built, supplied, build_info = check_build()
        ref = Reference(c['scenario'])
        write_json(out / 'reference.json', ref.meta)
        shutil.copy2(ref.directory / 'truth.csv', out / 'fixed_reference.csv')
        properties = build_hidrive_scene(out / 'scene.xosc', ref.meta)
        write_json(out / 'versions.json', {
            'studio': VERSION, 'python': sys.version,
            'controller': 'HiDriveController (supplied source, not inspected)',
            'engine': f"esmini {build_info.get('engine_tag', '?')} (official source archive, isolated build)",
            'engine_archive_sha256': build_info.get('engine_archive_sha256'),
            'executable': str(EXECUTABLE), 'executable_sha256': sha256(EXECUTABLE),
            'controller_build_metadata': built, 'controller_supplied_metadata': supplied,
            'perception_noise_sha256': sha256(NOISE_HEADER),
            'implementation_sha256': {p.name: sha256(p) for p in ROOT.glob('*.py')}})
        base = {s['name']: s for s in ref.meta['objects']}
        ego_name = ref.meta['ego_name']
        noisy = c['uncertainty'] == 'distance_ou_range'
        noise_args = []
        if noisy:
            duration = max(c['duration_s'], float(ref.meta.get('stop_trigger_s') or 0.0))
            write_noise_file(out / 'perception_noise.tsv', c, ego_name,
                             [s for s in base.values() if s['name'] != ego_name],
                             out / 'perception_applied.tsv', duration)
            noise_args = ['--perception_noise', out / 'perception_noise.tsv']
        write_json(out / 'progress.json', {'stage': 'simulation', 'fraction': .05})
        # The controller needs a viewer object; --headless with --window gives
        # an off-screen pbuffer viewer, so no window appears.
        args = [EXECUTABLE, '--osc', out / 'scene.xosc', '--headless', '--window', '0', '0', '320', '180',
                '--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',
                '--csv_logger', out / 'hidrive_esmini.csv', '--record', out / 'hidrive_replay.dat',
                '--logfile_path', out / 'esmini.log', '--disable_stdout', '--path', RUNTIME / 'resources',
                # Teacher scenes name '../models/<car>.osgb'; resolve it inside the full model set.
                '--path', RUNTIME / 'resources/models',
                *noise_args]
        with (out / 'hidrive_stdout.log').open('w', encoding='utf-8') as log:
            result = subprocess.run([str(a) for a in args], cwd=out, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=300, creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        if result.returncode != 0:
            raise RuntimeError(f'HiDrive esmini 异常退出（代码 {result.returncode}）；详见 esmini.log。')
        frames = read_esmini_csv(out / 'hidrive_esmini.csv')
        logged = read_log_values(out / 'esmini.log')
        applied = read_perception_log(out / 'perception_applied.tsv') if noisy else {}
        rho = math.exp(-c['dt_s'] / c['noise_correlation_time_s'])
        errors, invalid, seen = [], 0, set()
        writers = {}
        for name, fields in [('ego', EGO_FIELDS), ('truth', TRUTH_FIELDS), ('observations', OBS_FIELDS),
                              ('metrics', ['time_s', 'collision', 'engine_collision', 'minimum_distance_m', 'minimum_gap_m', 'ttc_s'])]:
            file, writers[name] = csv_writer(out, name + '.csv', fields)
            files.append(file)
        first_trigger = first_brake = stop_time = brake_range = collision = None
        min_range = min_gap = min_ttc = math.inf
        min_accel = 0.0
        times = sorted(frames)
        for index, t in enumerate(times):
            rows = frames[t]
            states = [to_state(rows[name], base[name]) for name in base if name in rows]
            ego = next(s for s in states if s['name'] == ego_name)
            targets = [s for s in states if s['name'] != ego_name]
            for s in states:
                writers['truth'].writerow(dict(zip(TRUTH_FIELDS, [t, s['id'], s['name'], s['objectType'],
                    s['x'], s['y'], s['z'], s['h'], s['p'], s['r'], s['speed'], s['vx'], s['vy'],
                    s['length'], s['width'], s['height'], s['centerOffsetX'], s['centerOffsetY'],
                    s['centerOffsetZ'], s['objectCategory']])))
            distances = {s['id']: math.dist(center(ego), center(s)) for s in targets}
            gaps, ttcs = [], []
            for s in targets:
                gap, closing, in_path = longitudinal_risk(ego, _truth_observation(s))
                if in_path:
                    gaps.append(gap)
                    if closing > 1e-3:
                        ttcs.append(max(gap, 0.0) / closing)
                record = applied.get((t, s['name']))
                if record is None:
                    writers['observations'].writerow({'time_s': t, 'object_id': s['id'], 'object_name': s['name'],
                        'visible': '', 'observation_valid': '', 'true_range_m': distances[s['id']]})
                    continue
                # Range the controller step saw (the error is relative to it); the
                # CSV row is logged after the step, so it can differ by one move.
                true_range = float(record['true_range_m'])
                valid = record['valid'] == '1'
                raw, sigma = float(record['raw_range_m']), float(record['sigma_m'])
                observed = raw if valid else None
                if valid:
                    errors.append(raw - true_range)
                invalid += not valid
                first = s['id'] not in seen
                seen.add(s['id'])
                ex, ey = center(ego)
                tx, ty = center(s)
                bearing = math.atan2(ty - ey, tx - ex) - ego['h']
                writers['observations'].writerow({'time_s': t, 'object_id': s['id'], 'object_name': s['name'],
                    'visible': True, 'observation_valid': valid, 'true_range_m': true_range,
                    'observed_range_m': observed, 'raw_observed_range_m': raw, 'range_error_mean_m': 0.0,
                    'range_error_sigma_m': sigma, 'range_error_expected_abs_m': sigma * math.sqrt(2 / math.pi),
                    'range_error_normalized': float(record['z']),
                    'range_error_transition_rho': None if first else rho,
                    'range_error_elapsed_s': None if first else c['dt_s'],
                    'sampled_range_error_m': float(record['error_m']),
                    'actual_range_error_m': None if observed is None else observed - true_range,
                    'bearing_deg': math.degrees(math.atan2(math.sin(bearing), math.cos(bearing))),
                    'observed_center_x_m': float(record['observed_center_x_m']) if valid else None,
                    'observed_center_y_m': float(record['observed_center_y_m']) if valid else None,
                    'controller_range_m': observed, 'ideal_range_m': true_range})
            gap, ttc = min(gaps, default=None), min(ttcs, default=None)
            min_range = min(min_range, *distances.values())
            min_gap = min(min_gap, gap if gap is not None else math.inf)
            min_ttc = min(min_ttc, ttc if ttc is not None else math.inf)
            hits = [int(v) for v in rows[ego_name].get('collision_ids', '').split() if v.strip().isdigit()]
            hit = bool(hits)
            writers['metrics'].writerow({'time_s': t, 'collision': int(hit), 'engine_collision': int(hit),
                'minimum_distance_m': min(distances.values(), default=None), 'minimum_gap_m': gap, 'ttc_s': ttc})
            values = logged.get(t, {})
            active = values.get('AEB_active') == '1'
            accel = number(values.get('Acceleration'))
            if active and first_trigger is None:
                first_trigger = t
            if accel is not None:
                min_accel = min(min_accel, accel)
                if accel < -.01 and first_brake is None:
                    first_brake, brake_range = t, min(distances.values(), default=None)
            if first_brake is not None and ego['speed'] <= 1e-6 and stop_time is None:
                stop_time = t
            if hit and collision is None:
                collision = dict(time_s=t, ego_speed_mps=ego['speed'], ego_speed_kph=ego['speed'] * 3.6,
                    object_ids=hits,
                    relative_speed_kph=max((math.hypot(ego['vx'] - s['vx'], ego['vy'] - s['vy']) * 3.6
                                            for s in targets), default=None))
            state = 'collision' if hit else 'braking' if active else 'monitoring'
            writers['ego'].writerow(dict(zip(EGO_FIELDS, [t, ego['x'], ego['y'], ego['h'], ego['speed'],
                ego['speed'] * 3.6, accel, None, ttc, active, ';'.join(map(str, hits)), state, '', None,
                None, state])))
            # Stop at first contact; esmini
            # keeps moving the bodies afterwards without collision dynamics.
            if hit:
                break
        for file in files:
            file.close()
        files = []
        end_reason = 'collision' if collision else 'scenario_stop'
        summary.update(collision=collision is not None, collision_details=collision,
            first_trigger_time_s=first_trigger, first_brake_time_s=first_brake,
            final_speed_kph=ego['speed'] * 3.6, end_time_s=t, samples=index + 1,
            end_reason=end_reason, timing_policy='hidrive_source_stop_trigger_or_first_contact',
            reference_sha256=ref.meta['truth_sha256'], diagnostic_only=diagnostic_only,
            minimum_gap_m=finite(min_gap), minimum_ttc_s=finite(min_ttc),
            minimum_acceleration_mps2=min_accel, source_warnings=ref.meta.get('source_warnings', []),
            ideal_reference=None, invalid_range_observations=invalid,
            assessment=dict(ideal_first_brake_time_s=None, brake_advance_vs_ideal_s=None,
                braked_without_ideal_intervention=False, true_center_range_at_brake_m=brake_range,
                minimum_center_range_m=finite(min_range), stop_time_s=stop_time,
                observed_range_rmse_m=math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
                observed_error_samples=len(errors),
                meaning='Trigger = first AEB_active=1; first brake = first controller-reported acceleration below -0.01 m/s². Both read from the esmini log. TTC is truth-based and diagnostic only.'),
            controller_implementation=dict(name='HiDriveController', language='C++',
                source_inspected=False, engine=f"esmini {build_info.get('engine_tag', '?')} (isolated build)",
                scenario_properties=properties),
            model='Supplied HiDrive controller on object_1 of the unchanged teacher scenario (mode from scenario properties).',
            interpretation='Truth from the esmini CSV logger; stop at first contact.',
            perception=("HiDrive controller's own target selection; Studio OU range error injected by the public "
                        "esmini hook (targets moved to their perceived centre during each controller step, truth "
                        "restored afterwards). Studio sensor range/FOV settings are not used." if noisy else
                        "HiDrive controller's own internal perception with ideal object states; Studio sensor "
                        "settings are not used."),
            observation_processing=dict(mode='direct', period_s=c['dt_s'],
                policy='Fresh range error at every controller step; no filter or tracker.',
                missing=f'A negative measured range is invalid: the target is placed {HIDE_DISTANCE_M:g} m behind the ego for that step.',
                velocity_assumption='Target speed, heading and box size are ideal; only the centre position is perturbed.')
                if noisy else None)
        return finish_run(c, out, summary, diagnostic_only=diagnostic_only)
    except Exception as exc:
        summary.update(status='failed', error=str(exc))
        write_json(out / 'summary.json', summary)
        (out / 'error.log').write_text(traceback.format_exc(), encoding='utf-8')
        raise
    finally:
        for file in files:
            file.close()
