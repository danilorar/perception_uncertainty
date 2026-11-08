"""OU: a temporally correlated range error (Ornstein-Uhlenbeck) that grows with distance.

    mean absolute error = reference_range_m * mae_pct / 100 * max(0, r - zero_range_m) / (reference_range_m - zero_range_m)
    bias                = reference_range_m * bias_pct / 100 * max(0, r - zero_range_m) / (reference_range_m - zero_range_m)
    range += bias + mean absolute error * sqrt(pi / 2) * z,   z: unit OU process with correlation time correlation_time_s

r is the true range. bias_pct > 0 reads the target too far, < 0 too near; it follows the same distance law, so
mae_pct = bias_pct gives a random and a constant error of the same mean size. With a bias, mae_pct is the size of the
random part only. The distance law and correlation time are assumptions for sensitivity studies, not
calibrated sensor values.
"""
import math
import random

SETTINGS = {'mae_pct': 5.0, 'bias_pct': 0.0, 'correlation_time_s': 0.5, 'zero_range_m': 3.0, 'reference_range_m': 100.0}
RANGES = {'mae_pct': (0, 100), 'bias_pct': (-100, 100), 'correlation_time_s': (0.01, 20), 'zero_range_m': (0, 499),
          'reference_range_m': (1, 500)}
LOG = ('z', 'sigma_m', 'error_m')
SIGMA_PER_MAE = math.sqrt(math.pi / 2.0)


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


def unit_samples(seed, tau_s, object_id, dt_s, steps):
    noise = CorrelatedRangeNoise(seed, tau_s)
    return [noise.sample(object_id, round(k * dt_s, 8)) for k in range(steps)]


def check(settings, run):
    if settings['reference_range_m'] <= settings['zero_range_m']:
        raise ValueError('reference_range_m must be greater than zero_range_m')


class Error:
    def __init__(self, settings, run):
        self.settings = settings
        self.pct, self.zero, self.reference = settings['mae_pct'], settings['zero_range_m'], settings['reference_range_m']
        self.bias = settings['bias_pct']
        self.z = {target['name']: unit_samples(run.seed, settings['correlation_time_s'], target['id'], run.dt_s, run.steps)
                  for target in run.targets}

    def perceive(self, step, t, ego, targets):
        for target in targets:
            r = target['true_range']
            mae = self.reference * self.pct / 100.0 * max(0.0, r - self.zero) / (self.reference - self.zero)
            sigma = mae * SIGMA_PER_MAE
            z = self.z[target['name']][step]
            error = sigma * z
            if self.bias:  # skipped at 0, so runs without a bias stay identical to the ones before bias_pct existed
                error = self.reference * self.bias / 100.0 * max(0.0, r - self.zero) / (self.reference - self.zero) + error
            target['range'] = target['range'] + error
            target['log'].update(z=z, sigma_m=sigma, error_m=error)

    def describe(self):
        return {'transition': 'z_next = exp(-dt/tau) * z + sqrt(1 - exp(-2*dt/tau)) * N(0,1), from N(0,1), one stream per object',
                'calibration': 'Assumed values; a sensitivity model, not measured sensor risk.', **self.settings}
