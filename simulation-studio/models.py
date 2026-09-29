"""Explicit research prototypes, not supplied or calibrated supervisor models.

No target ground truth is exposed to the controller. Its target inputs are
Observation instances produced by Perception. Object dimensions, orientation
and velocity are ideal in this first prototype; range alone is perturbed.
"""
from dataclasses import dataclass
import math
import random

def center(d):
    co, si = math.cos(d['h']), math.sin(d['h'])
    return (d['x'] + co*d['centerOffsetX'] - si*d['centerOffsetY'],
            d['y'] + si*d['centerOffsetX'] + co*d['centerOffsetY'])

def axes(h):
    return [(math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))]

def projected_radius(length, width, h, axis):
    a, b = axes(h)
    return abs(a[0]*axis[0]+a[1]*axis[1])*length/2 + abs(b[0]*axis[0]+b[1]*axis[1])*width/2


def range_error_parameters(config, distance):
    """Return signed-error mean, standard deviation, and mean absolute error.

    For the distance-dependent model, the *unconditional sampled error* is a
    zero-mean Gaussian. E|error| = sigma * sqrt(2/pi). MAE in metres grows
    linearly from the zero-error boundary through the reference anchor, and
    continues at the same slope beyond it. Distance is centre-to-centre.
    """
    mode = config['uncertainty']
    if mode in ['ideal', 'recorded_truth']:
        return 0.0, 0.0, 0.0
    if mode == 'distance_gaussian_range':
        zero, reference = config['noise_zero_range_m'], config['noise_reference_range_m']
        mae = reference * config['noise_reference_mae_pct']/100 * max(0.0, distance-zero)/(reference-zero)
        return 0.0, mae*math.sqrt(math.pi/2), mae
    # Keep previous fixed-parameter configurations reproducible.
    mean, sigma = config['range_bias_m'], config['range_sigma_m']
    if sigma == 0:
        mae = abs(mean)
    else:
        mae = sigma*math.sqrt(2/math.pi)*math.exp(-mean*mean/(2*sigma*sigma)) + mean*math.erf(mean/(sigma*math.sqrt(2)))
    return mean, sigma, mae

@dataclass(frozen=True)
class Observation:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    heading: float
    length: float
    width: float

class Perception:
    def __init__(self, config):
        self.config = config
        self.rng = random.Random(config['seed'])

    def observe(self, ego, targets, *, detections):
        """Apply uncertainty to native detections; targets are audit truth only.

        No fallback range/FOV gate lives here. Even zero-error modes require
        the detected object states supplied by Engine.detect().
        """
        c = self.config
        ex, ey = center(ego)
        detected = {s['id']: s for s in detections}
        observations, records = [], []
        for target in sorted(targets, key=lambda s: s['id']):
            tx, ty = center(target)
            true_distance = math.hypot(tx-ex, ty-ey)
            measured = detected.get(target['id'])
            visible = measured is not None
            if visible:
                tx, ty = center(measured)
            dx, dy = tx-ex, ty-ey
            distance = math.hypot(dx, dy)
            bearing = math.atan2(math.sin(math.atan2(dy, dx)-ego['h']), math.cos(math.atan2(dy, dx)-ego['h']))
            error_mean, error_sigma, error_mae = range_error_parameters(c, distance)
            observed_range = error = None
            raw_observed_range = None
            valid = False
            ox = oy = None
            if visible:
                if c['uncertainty'] in ['ideal','recorded_truth'] or (c['uncertainty']=='distance_gaussian_range' and error_sigma==0):
                    error = 0.0
                else:
                    error = error_mean + self.rng.gauss(0, error_sigma)
                raw_observed_range = distance + error
                # Reject nonphysical measurements rather than clamping or redrawing
                # the new Gaussian error. Preserve every draw for distribution QA.
                valid = c['uncertainty'] != 'distance_gaussian_range' or raw_observed_range >= 0
                if valid:
                    observed_range = max(0.0, raw_observed_range)
                    scale = observed_range/distance if distance > 1e-12 else 0
                    ox, oy = ex+dx*scale, ey+dy*scale
                    observations.append(Observation(measured['id'], ox, oy, measured['vx'], measured['vy'], measured['h'], measured['length'], measured['width']))
            records.append({'object_id': target['id'], 'object_name': target['name'], 'visible': visible,
                'sensor_detected': visible, 'ideal_range_m': distance if visible else None,
                'sensor_reference_range_m': math.hypot(measured['x']-ex, measured['y']-ey) if visible else None,
                'true_range_m': true_distance, 'observed_range_m': observed_range,
                'observation_valid':valid, 'raw_observed_range_m':raw_observed_range,
                'range_error_mean_m':error_mean, 'range_error_sigma_m':error_sigma,
                'range_error_expected_abs_m':error_mae,
                'sampled_range_error_m': error, 'actual_range_error_m': None if observed_range is None else observed_range-true_distance,
                'bearing_deg': math.degrees(bearing), 'observed_center_x_m': ox, 'observed_center_y_m': oy})
        return observations, records

def predicted_contact_time(ego, obs, horizon):
    """Continuous SAT for translating oriented rectangles at constant velocity.

    Current headings stay fixed during this prediction. All target values come
    from observations; no future recorded target trajectory is used.
    """
    ex, ey = center(ego)
    dx, dy = obs.x-ex, obs.y-ey
    dvx, dvy = obs.vx-ego['vx'], obs.vy-ego['vy']
    entry, leave = 0.0, horizon
    for axis in axes(ego['h']) + axes(obs.heading):
        radius = projected_radius(ego['length'], ego['width'], ego['h'], axis) + projected_radius(obs.length, obs.width, obs.heading, axis)
        dist = dx*axis[0]+dy*axis[1]
        velocity = dvx*axis[0]+dvy*axis[1]
        if abs(velocity) < 1e-12:
            if abs(dist) > radius:
                return None
        else:
            a, b = sorted(((-radius-dist)/velocity, (radius-dist)/velocity))
            entry, leave = max(entry, a), min(leave, b)
            if entry > leave:
                return None
    return entry if leave >= 0 and entry <= horizon else None

class AEB:
    def __init__(self, config):
        self.c = config
        self.trigger_time = None
        self.brake_time = None
        self.pending_since = None
        self.pending_object = None
        self.armed_time = None
        self.state = 'monitoring'
        self.cancelled_confirmations = 0
        self.last_ttc = None

    def command(self, time, ego, observations, *, fresh=True):
        candidates = [(predicted_contact_time(ego, o, self.c['ttc_threshold_s']),o.id) for o in observations] if fresh else []
        hazard = min(((v,ident) for v,ident in candidates if v is not None),default=None)
        if fresh:
            self.last_ttc = hazard[0] if hazard else None
        ttc = self.last_ttc
        if self.c['controller'] in ['constant_speed','original_trajectory']:
            return 0.0, ttc
        confirmed = self.c.get('aeb_trigger_mode','instant') == 'confirmed'
        if fresh and self.brake_time is None:
            if hazard is None or ego['speed']<=0:
                if confirmed:
                    self.cancelled_confirmations += int(self.pending_since is not None)
                    self.pending_since = self.pending_object = self.armed_time = None
                    self.state = 'monitoring'
            elif not confirmed:
                if self.armed_time is None:
                    self.armed_time = time
                    self.trigger_time = time
            else:
                if self.pending_object != hazard[1]:
                    self.pending_since, self.pending_object = time,hazard[1]
                    self.armed_time = None
                self.state = 'confirming'
                if self.armed_time is None and time-self.pending_since+1e-9 >= self.c['aeb_confirmation_s']:
                    self.armed_time = time
                    if self.trigger_time is None:
                        self.trigger_time = time
        if self.armed_time is not None:
            self.state = 'response_delay'
        if self.brake_time is not None or (self.armed_time is not None and time + 1e-9 >= self.armed_time + self.c['reaction_delay_s']):
            if self.brake_time is None:
                self.brake_time = time
            self.state = 'braking' if ego['speed']>0 else 'stopped'
            return (-self.c['brake_decel_mps2'] if ego['speed'] > 0 else 0), ttc
        return 0.0, ttc

def advance_ego(ego, acceleration, dt):
    updated = ego.copy()
    moving_dt = min(dt, ego['speed'] / -acceleration) if acceleration < 0 else dt
    distance = ego['speed']*moving_dt + 0.5*acceleration*moving_dt*moving_dt
    updated['speed'] = max(0.0, ego['speed'] + acceleration*dt)
    updated['x'] += math.cos(ego['h'])*distance
    updated['y'] += math.sin(ego['h'])*distance
    updated['vx'], updated['vy'] = updated['speed']*math.cos(ego['h']), updated['speed']*math.sin(ego['h'])
    return updated
