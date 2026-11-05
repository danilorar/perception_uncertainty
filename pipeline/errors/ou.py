"""Temporally correlated range error (Ornstein-Uhlenbeck) that grows with distance.

The distance law and correlation time are assumptions for sensitivity studies, not calibrated sensor values.
"""
import math
import random

INJECTS = True


class CorrelatedRangeNoise:
    """Unit-variance OU process per object, sampled at observation times.

    Turlej (Electronics 2023, 12, 71), p. 6: temporally correlated perception.
    Yang et al. (JAIF 2020, 15(2)), p. 93, eqs. (3), (4), (7): exact discrete Gauss-Markov
    transition and stationary variance normalization.
    """

    def __init__(self, seed, tau_s):
        self.seed, self.tau_s = seed, tau_s
        self.states = {}
        self.streams = {}

    def sample(self, object_id, time_s):
        previous = self.states.get(object_id)
        if previous is None:
            # One independent stream per object: unrelated objects cannot change this object's errors.
            rng = random.Random(f'{self.seed}:range-ou:{object_id}')
            self.streams[object_id] = rng
            z = rng.gauss(0, 1)
        else:
            last_time, z = previous
            elapsed = max(0.0, time_s - last_time)
            if elapsed > 0:
                rho = math.exp(-elapsed / self.tau_s)
                z = rho * z + math.sqrt(-math.expm1(-2 * elapsed / self.tau_s)) * self.streams[object_id].gauss(0, 1)
        self.states[object_id] = (time_s, z)
        return z


def unit_samples(c, target_id, steps):
    noise = CorrelatedRangeNoise(c['seed'], c['noise_correlation_time_s'])
    return [noise.sample(target_id, round(k * c['dt_s'], 8)) for k in range(steps)]


def validate(c):
    if c['range_factors'] is not None:
        raise ValueError('range_factors can only be set with uncertainty "range_factors".')
    if c['noise_reference_range_m'] <= c['noise_zero_range_m']:
        raise ValueError('noise_reference_range_m must be greater than noise_zero_range_m.')


def mean_error_pct(c):
    return c['noise_reference_mae_pct']


def settings(c):
    return []


def tracks(c, target_id, steps):
    return {'z': unit_samples(c, target_id, steps)}


def describe(c):
    return {'model': 'distance_scaled_stationary_ou_v1',
            'distance': 'true bounding-box centre-to-centre range r, in metres',
            'mean_absolute_error_m': 'reference_range * reference_mae_pct / 100 * max(0, r - zero_range) / (reference_range - zero_range)',
            'sigma_m': 'mean_absolute_error_m * sqrt(pi/2); the same slope continues beyond the reference range',
            'transition': 'z_next = exp(-dt/tau) * z + sqrt(1 - exp(-2*dt/tau)) * N(0,1), starting from N(0,1), one stream per object',
            'correlation_time_s': c['noise_correlation_time_s'],
            'zero_error_range_m': c['noise_zero_range_m'], 'reference_range_m': c['noise_reference_range_m'],
            'reference_mean_absolute_error_pct': c['noise_reference_mae_pct'],
            'invalid_measurements': 'A negative range hides the target for that controller step.',
            'calibration': 'Assumed values; a sensitivity model, not measured sensor risk.'}
