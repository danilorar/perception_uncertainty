"""Turn esmini's outputs into the result tables and metrics of one run.

Truth comes from the CSV logger. Controller signals come from LOG_VALUE lines in the esmini log.
TTC and gap use the truth and are diagnostic only: the controller never sees them.
"""
import csv
import math
from pathlib import Path
import re

TRUTH_FIELDS = ['time_s', 'object_id', 'object_name', 'x_m', 'y_m', 'z_m', 'heading_rad', 'pitch_rad', 'speed_mps',
                'vx_mps', 'vy_mps', 'length_m', 'width_m', 'height_m', 'center_offset_x_m', 'center_offset_y_m',
                'center_offset_z_m']
EGO_FIELDS = ['time_s', 'x_m', 'y_m', 'heading_rad', 'speed_mps', 'speed_kph', 'command_accel_mps2', 'brake_active',
              'collision_ids', 'controller_state']
OBSERVATION_FIELDS = ['time_s', 'object_id', 'object_name', 'observation_valid', 'true_range_m', 'observed_range_m',
                      'raw_observed_range_m', 'range_error_mean_m', 'range_error_sigma_m', 'range_error_normalized',
                      'sampled_range_error_m', 'actual_range_error_m', 'bearing_deg', 'observed_center_x_m',
                      'observed_center_y_m']
METRIC_FIELDS = ['time_s', 'collision', 'minimum_distance_m', 'minimum_gap_m', 'ttc_s']
# esmini log line: "[12.340] [info] LOG_VALUE:name:value" (optional [file::func::line] metadata).
LOG_VALUE = re.compile(r'\[([0-9.]+)\] \[\w+\] (?:\[[^\]]*\] )?LOG_VALUE:(\w+):(\S+)')
# First brake: first controller acceleration below this value.
BRAKE_THRESHOLD_MPS2 = -0.01


def read_perception_log(path):
    """Errors esmini applied during controller steps: {(time, name): row}."""
    with Path(path).open(encoding='utf-8') as stream:
        return {(round(float(row['time_s']), 8), row['object_name']): row
                for row in csv.DictReader(stream, delimiter='\t')}


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
            entities = {}
            for name, value in zip(header[2:], cells[2:]):
                match = re.match(r'#(\d+) (.+?)(?: \[.*\])?$', name)
                if match:
                    entities.setdefault(int(match[1]), {})[match[2].strip()] = value
            frames[round(float(cells[1]), 8)] = {entity['Entity_Name']: entity for entity in entities.values()}
    if not frames:
        raise RuntimeError('esmini wrote no CSV data.')
    return frames


def read_log_values(path):
    values = {}
    with Path(path).open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            match = LOG_VALUE.match(line)
            if match:
                values.setdefault(round(float(match[1]), 8), {})[match[2]] = match[3]
    return values


def to_state(row, item):
    return dict(id=item['id'], name=item['name'], x=float(row['World_Position_X']), y=float(row['World_Position_Y']),
                z=float(row['World_Position_Z']), h=float(row['World_Heading_Angle']),
                p=float(row['World_Pitch_Angle']), speed=float(row['Current_Speed']), vx=float(row['Vel_X']),
                vy=float(row['Vel_Y']), length=float(row['bb_length']), width=float(row['bb_width']),
                height=float(row['bb_height']), centerOffsetX=float(row['bb_x']), centerOffsetY=float(row['bb_y']),
                centerOffsetZ=float(row['bb_z']))


def center(state):
    cos_h, sin_h = math.cos(state['h']), math.sin(state['h'])
    return (state['x'] + cos_h * state['centerOffsetX'] - sin_h * state['centerOffsetY'],
            state['y'] + sin_h * state['centerOffsetX'] + cos_h * state['centerOffsetY'])


def longitudinal_risk(ego, target):
    """Rear-end gap, closing speed and in-path flag. Adapted from Zhuo Ma's risk metrics."""
    ex, ey = center(ego)
    tx, ty = center(target)
    dx, dy = tx - ex, ty - ey
    longitudinal = dx * math.cos(ego['h']) + dy * math.sin(ego['h'])
    lateral = -dx * math.sin(ego['h']) + dy * math.cos(ego['h'])
    angle = target['h'] - ego['h']
    cos_a, sin_a = abs(math.cos(angle)), abs(math.sin(angle))
    half_length = (target['length'] * cos_a + target['width'] * sin_a) / 2
    half_width = (target['width'] * cos_a + target['length'] * sin_a) / 2
    gap = longitudinal - ego['length'] / 2 - half_length
    closing = ego['speed'] - target['speed'] * math.cos(angle)
    in_path = longitudinal > 0 and abs(lateral) <= ego['width'] / 2 + half_width
    return gap, closing, in_path


def number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def finite(value):
    return value if math.isfinite(value) else None


class Tables:
    """The four result tables, written row by row."""

    def __init__(self, out):
        self.files, self.writers = [], {}
        for name, fields in (('ego', EGO_FIELDS), ('truth', TRUTH_FIELDS), ('observations', OBSERVATION_FIELDS),
                             ('metrics', METRIC_FIELDS)):
            stream = (out / f'{name}.csv').open('w', encoding='utf-8', newline='')
            self.files.append(stream)
            self.writers[name] = csv.DictWriter(stream, fieldnames=fields)
            self.writers[name].writeheader()

    def __enter__(self):
        return self.writers

    def __exit__(self, *_):
        for stream in self.files:
            stream.close()


def observation_row(t, state, ego, distance, record):
    """One row of observations.csv; record is what the hook applied on this step (None without a hook)."""
    row = {'time_s': t, 'object_id': state['id'], 'object_name': state['name'], 'true_range_m': distance}
    if record is None:
        return row, None, False
    # Range the controller step saw (the error is relative to it); the CSV row is logged after the step.
    true_range = float(record['true_range_m'])
    valid = record['valid'] == '1'
    raw = float(record['raw_range_m'])
    observed = raw if valid else None
    ex, ey = center(ego)
    tx, ty = center(state)
    bearing = math.atan2(ty - ey, tx - ex) - ego['h']
    row.update(observation_valid=valid, true_range_m=true_range, observed_range_m=observed, raw_observed_range_m=raw,
               range_error_mean_m=float(record['mean_error_m']), range_error_sigma_m=float(record['sigma_m']),
               range_error_normalized=float(record['z']), sampled_range_error_m=float(record['error_m']),
               actual_range_error_m=None if observed is None else observed - true_range,
               bearing_deg=math.degrees(math.atan2(math.sin(bearing), math.cos(bearing))),
               observed_center_x_m=float(record['observed_center_x_m']) if valid else None,
               observed_center_y_m=float(record['observed_center_y_m']) if valid else None)
    return row, (raw - true_range if valid else None), not valid


def collect(out, scenario, controller, injected):
    """Write ego, truth, observations and metrics tables; return the summary values."""
    frames = read_esmini_csv(out / 'esmini.csv')
    logged = read_log_values(out / 'esmini.log')
    applied = read_perception_log(out / 'perception_applied.tsv') if injected else {}
    active_name, accel_name = controller.signals.get('active'), controller.signals.get('acceleration')
    errors, invalid = [], 0
    first_trigger = first_brake = stop_time = brake_range = collision = None
    min_range = min_gap = min_ttc = math.inf
    min_accel = 0.0
    with Tables(out) as writers:
        for index, t in enumerate(sorted(frames)):
            rows = frames[t]
            states = [to_state(rows[item['name']], item) for item in scenario.objects if item['name'] in rows]
            ego = next(state for state in states if state['name'] == scenario.ego)
            targets = [state for state in states if state['name'] != scenario.ego]
            for s in states:
                writers['truth'].writerow(dict(zip(TRUTH_FIELDS, [
                    t, s['id'], s['name'], s['x'], s['y'], s['z'], s['h'], s['p'], s['speed'], s['vx'], s['vy'],
                    s['length'], s['width'], s['height'], s['centerOffsetX'], s['centerOffsetY'], s['centerOffsetZ']])))
            distances = {s['id']: math.dist(center(ego), center(s)) for s in targets}
            gaps, ttcs = [], []
            for s in targets:
                gap, closing, in_path = longitudinal_risk(ego, s)
                if in_path:
                    gaps.append(gap)
                    if closing > 1e-3:
                        ttcs.append(max(gap, 0.0) / closing)
                row, error, is_invalid = observation_row(t, s, ego, distances[s['id']], applied.get((t, s['name'])))
                writers['observations'].writerow(row)
                invalid += is_invalid
                if error is not None:
                    errors.append(error)
            gap, ttc = min(gaps, default=None), min(ttcs, default=None)
            min_range = min(min_range, *distances.values())
            min_gap = min(min_gap, gap if gap is not None else math.inf)
            min_ttc = min(min_ttc, ttc if ttc is not None else math.inf)
            hits = [int(v) for v in rows[scenario.ego].get('collision_ids', '').split() if v.strip().isdigit()]
            hit = bool(hits)
            writers['metrics'].writerow({'time_s': t, 'collision': int(hit),
                                         'minimum_distance_m': min(distances.values(), default=None),
                                         'minimum_gap_m': gap, 'ttc_s': ttc})
            values = logged.get(t, {})
            active = values.get(active_name) == '1'
            accel = number(values.get(accel_name))
            if active and first_trigger is None:
                first_trigger = t
            if accel is not None:
                min_accel = min(min_accel, accel)
                if accel < BRAKE_THRESHOLD_MPS2 and first_brake is None:
                    first_brake, brake_range = t, min(distances.values(), default=None)
            if first_brake is not None and ego['speed'] <= 1e-6 and stop_time is None:
                stop_time = t
            if hit and collision is None:
                collision = dict(time_s=t, ego_speed_mps=ego['speed'], ego_speed_kph=ego['speed'] * 3.6, object_ids=hits,
                                 relative_speed_kph=max((math.hypot(ego['vx'] - s['vx'], ego['vy'] - s['vy']) * 3.6
                                                         for s in targets), default=None))
            state = 'collision' if hit else 'braking' if active else 'monitoring'
            writers['ego'].writerow(dict(zip(EGO_FIELDS, [t, ego['x'], ego['y'], ego['h'], ego['speed'], ego['speed'] * 3.6,
                                                          accel, active, ';'.join(map(str, hits)), state])))
            # Stop at first contact: esmini keeps moving the bodies afterwards without collision dynamics.
            if hit:
                break
    return dict(collision=collision is not None, collision_details=collision, first_trigger_time_s=first_trigger,
                first_brake_time_s=first_brake, true_center_range_at_brake_m=brake_range, stop_time_s=stop_time,
                final_speed_kph=ego['speed'] * 3.6, end_time_s=t, samples=index + 1,
                end_reason='collision' if collision else 'scenario_stop', minimum_gap_m=finite(min_gap),
                minimum_ttc_s=finite(min_ttc), minimum_acceleration_mps2=min_accel,
                minimum_center_range_m=finite(min_range), invalid_range_observations=invalid,
                observed_range_rmse_m=math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
                observed_error_samples=len(errors))
