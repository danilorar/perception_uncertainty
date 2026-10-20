"""Range-error model and truth geometry used around the HiDrive AEB.

The OU samples generated here are injected into the controller's view by the
esmini hook (hidrive/PerceptionNoise.hpp); range_error_parameters documents the
same distance law that the hook applies. longitudinal_risk is truth-only TTC /
gap evaluation for the saved results.
"""

from dataclasses import dataclass
import math
import random


def center(d):
    co, si = math.cos(d['h']), math.sin(d['h'])
    return (
        d['x'] + co * d['centerOffsetX'] - si * d['centerOffsetY'],
        d['y'] + si * d['centerOffsetX'] + co * d['centerOffsetY'],
    )


def range_error_parameters(config, distance):
    """Return signed-error mean, standard deviation, and mean absolute error.

    This is the marginal amplitude envelope of the OU error, not independent
    per-frame sampling. At fixed range under stationary initialization,
    E|error| = sigma * sqrt(2/pi). The linear distance law is an uncalibrated
    project assumption, not a parameter fit from the cited literature.
    """
    mode = config['uncertainty']
    if mode in ['ideal', 'recorded_truth']:
        return 0.0, 0.0, 0.0
    if mode == 'distance_ou_range':
        zero, reference = config['noise_zero_range_m'], config['noise_reference_range_m']
        mae = (
            reference
            * config['noise_reference_mae_pct']
            / 100
            * max(0.0, distance - zero)
            / (reference - zero)
        )
        return 0.0, mae * math.sqrt(math.pi / 2), mae
    raise ValueError('Unsupported range error model; use distance_ou_range or ideal observations.')


class CorrelatedRangeNoise:
    """Unit-variance OU process per object, sampled at actual observation times.

    Turlej (Electronics 2023, 12, 71), p. 6: temporally correlated perception.
    Yang et al. (JAIF 2020, 15(2)), p. 93, eqs. (3), (4), (7): exact
    discrete Gauss-Markov transition and stationary variance normalization.
    This generates errors; no observation filtering or target estimation.
    """

    def __init__(self, seed, tau_s):
        if not math.isfinite(tau_s) or tau_s <= 0:
            raise ValueError('OU correlation time must be finite and positive.')
        self.seed, self.tau_s = seed, tau_s
        self.states = {}
        self.streams = {}

    def sample(self, object_id, time_s):
        if not math.isfinite(time_s) or time_s < 0:
            raise ValueError('Observation time must be finite and non-negative.')
        previous = self.states.get(object_id)
        if previous is None:
            # One independent stream per object: unrelated detections cannot
            # change this object's error sequence. Start in the stationary law.
            rng = random.Random(f'{self.seed}:range-ou:{object_id}')
            self.streams[object_id] = rng
            z = rng.gauss(0, 1)
            elapsed = rho = None
        else:
            last_time, z = previous
            elapsed = time_s - last_time
            if elapsed < -1e-9:
                raise ValueError('Observation times must not move backwards.')
            elapsed = max(0.0, elapsed)
            rho = math.exp(-elapsed / self.tau_s)
            if elapsed > 0:
                innovation_scale = math.sqrt(-math.expm1(-2 * elapsed / self.tau_s))
                z = rho * z + innovation_scale * self.streams[object_id].gauss(0, 1)
        self.states[object_id] = (time_s, z)
        return z, rho, elapsed


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
    speed: float | None = None
    z: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    height: float = 1.4
    center_offset_x: float = 0.0
    center_offset_y: float = 0.0
    center_offset_z: float = 0.7
    object_type: int = 1
    object_category: int = 0


def longitudinal_risk(ego, obs):
    """Rear-end gap, closing speed and in-path flag between ego and a box.

    Adapted from Zhuo Ma's risk_metrics, feature/esmini-perception @ cdb9639.
    Used only for truth-based diagnostics of saved runs.
    """
    ex, ey = center(ego)
    dx, dy = obs.x - ex, obs.y - ey
    longitudinal = dx * math.cos(ego['h']) + dy * math.sin(ego['h'])
    lateral = -dx * math.sin(ego['h']) + dy * math.cos(ego['h'])
    angle = obs.heading - ego['h']
    co, si = abs(math.cos(angle)), abs(math.sin(angle))
    half_length = (obs.length * co + obs.width * si) / 2
    half_width = (obs.width * co + obs.length * si) / 2
    gap = longitudinal - ego['length'] / 2 - half_length
    target_speed = obs.speed if obs.speed is not None else math.hypot(obs.vx, obs.vy)
    closing = ego['speed'] - target_speed * math.cos(angle)
    in_path = longitudinal > 0 and abs(lateral) <= ego['width'] / 2 + half_width
    return gap, closing, in_path
