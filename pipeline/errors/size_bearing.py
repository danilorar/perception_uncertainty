"""Physical length/width error conditional on sensor distance and azimuth.

For each dimension k, phi = (1, log1p(r/r_ref), v, log1p(r/r_ref)*v),
v = (sin(bearing)/sin(bearing_ref))**2:
    mu_k = dot(k_mu, phi)
    sigma_k = k_sigma_ref * exp(dot(k_sigma_log, phi))
    output_k = input_k * exp(clip(mu_k + sigma_k*z_k, -limit, limit))

The paired, unit-variance Gaussian processes z have an exact discrete OU
transition. Coefficients are assumptions until fitted to dimension estimates
and ground truth. This is not a camera renderer or a calibrated sensor model.
Only perceived length/width are changed, using the existing SIZE bridge flag.
See docs/SIZE_BEARING.md for units, domain, clipping and calibration.
"""
import copy
import math
import random

SETTINGS = {
    'reference_distance_m': 50.0,
    'reference_bearing_deg': 60.0,
    'min_distance_m': 0.1,
    'max_distance_m': 150.0,
    'max_abs_bearing_deg': 60.0,
    'sensor_forward_m': 0.0,
    'sensor_left_m': 0.0,
    'sensor_yaw_deg': 0.0,
    'length_mu': [0.0, 0.0, 0.0, 0.0],
    'length_sigma_ref': 0.0,
    'length_sigma_log': [0.0, 0.0, 0.0, 0.0],
    'width_mu': [0.0, 0.0, 0.0, 0.0],
    'width_sigma_ref': 0.0,
    'width_sigma_log': [0.0, 0.0, 0.0, 0.0],
    'correlation_time_s': 0.5,
    'length_width_correlation': 0.0,
    'max_abs_log_error': math.log(4.0),
    'only_correct_class': True,
    'parameter_source': 'assumed sensitivity parameters; not calibrated',
}
RANGES = {
    'reference_distance_m': (0.1, 10000),
    'reference_bearing_deg': (1, 89),
    'min_distance_m': (0, 10000),
    'max_distance_m': (0.1, 10000),
    'max_abs_bearing_deg': (1, 89),
    'sensor_forward_m': (-20, 20),
    'sensor_left_m': (-20, 20),
    'sensor_yaw_deg': (-180, 180),
    'length_sigma_ref': (0, 1),
    'width_sigma_ref': (0, 1),
    'correlation_time_s': (0, 30),
    'length_width_correlation': (-1, 1),
    'max_abs_log_error': (0.01, 5),
}
LOG = (
    'distance_m', 'bearing_deg', 'in_domain', 'applied', 'skip_reason',
    'true_length_m', 'true_width_m', 'input_length_m', 'input_width_m',
    'estimated_length_m', 'estimated_width_m',
    'length_mu', 'width_mu', 'length_sigma', 'width_sigma',
    'length_z', 'width_z', 'length_raw_log_error', 'width_raw_log_error',
    'length_clipped', 'width_clipped', 'length_error_m', 'width_error_m',
)


def features(settings, distance_m, bearing_rad):
    u = math.log1p(distance_m / settings['reference_distance_m'])
    v = (math.sin(bearing_rad) / math.sin(math.radians(settings['reference_bearing_deg']))) ** 2
    return (1.0, u, v, u * v)


def parameters(settings, distance_m, bearing_rad, dimension):
    """Return mean/std of the UNCLIPPED log-ratio, not a percentage error."""
    phi = features(settings, distance_m, bearing_rad)
    mu = sum(a * x for a, x in zip(settings[dimension + '_mu'], phi))
    reference = settings[dimension + '_sigma_ref']
    sigma = 0.0 if reference == 0 else math.exp(
        math.log(reference) + sum(b * x for b, x in zip(settings[dimension + '_sigma_log'], phi)))
    return mu, sigma


def check(settings, run):
    if settings['max_distance_m'] <= settings['min_distance_m']:
        raise ValueError('max_distance_m must exceed min_distance_m')
    if not isinstance(settings['parameter_source'], str) or not settings['parameter_source'].strip():
        raise ValueError('parameter_source must be nonempty text')
    for key in ('length_mu', 'width_mu', 'length_sigma_log', 'width_sigma_log'):
        values = settings[key]
        if (not isinstance(values, list) or len(values) != 4 or
                any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values)):
            raise ValueError(f'{key} must contain four finite numbers: [constant, distance, bearing, interaction]')
    # Bilinear surfaces attain their extrema at domain corners. Reject extreme
    # configurations before exponentiation; actual random draws are clipped/logged.
    for distance in (settings['min_distance_m'], settings['max_distance_m']):
        for bearing in (0.0, math.radians(settings['max_abs_bearing_deg'])):
            phi = features(settings, distance, bearing)
            for dimension in ('length', 'width'):
                mu = sum(a * x for a, x in zip(settings[dimension + '_mu'], phi))
                if not math.isfinite(mu) or abs(mu) > settings['max_abs_log_error']:
                    raise ValueError(f'{dimension}_mu exceeds max_abs_log_error inside the domain')
                reference = settings[dimension + '_sigma_ref']
                if reference > 0:
                    log_sigma = math.log(reference) + sum(
                        b * x for b, x in zip(settings[dimension + '_sigma_log'], phi))
                    if not math.isfinite(log_sigma) or log_sigma > 0:
                        raise ValueError(f'{dimension} log-ratio standard deviation must be <= 1 throughout the domain')


def sensor_geometry(settings, ego, truth):
    """Planar centre distance and azimuth from the configured sensor mounting."""
    c, s = math.cos(ego['heading']), math.sin(ego['heading'])
    sx = ego['x'] + c * settings['sensor_forward_m'] - s * settings['sensor_left_m']
    sy = ego['y'] + s * settings['sensor_forward_m'] + c * settings['sensor_left_m']
    dx, dy = truth['x'] - sx, truth['y'] - sy
    angle = math.atan2(dy, dx) - ego['heading'] - math.radians(settings['sensor_yaw_deg'])
    return math.hypot(dx, dy), math.atan2(math.sin(angle), math.cos(angle))


class Error:
    def __init__(self, settings, run):
        self.settings = copy.deepcopy(settings)
        self.noise = {}
        tau = settings['correlation_time_s']
        rho_t = math.exp(-run.dt_s / tau) if tau else 0.0
        innovation_scale = math.sqrt(-math.expm1(-2 * run.dt_s / tau)) if tau else 1.0
        rho_lw = settings['length_width_correlation']
        independent_scale = math.sqrt(max(0.0, 1.0 - rho_lw * rho_lw))
        for target in run.targets:
            prefix = f'{run.seed}:{run.label}:{target["id"]}:size-bearing'
            length_rng = random.Random(prefix + ':length')
            width_rng = random.Random(prefix + ':width-independent')
            samples = []
            zl = zw = 0.0
            for index in range(run.steps):
                il = length_rng.gauss(0, 1)
                iw = rho_lw * il + independent_scale * width_rng.gauss(0, 1)
                if index == 0:
                    zl, zw = il, iw  # stationary initialization
                else:
                    zl = rho_t * zl + innovation_scale * il
                    zw = rho_t * zw + innovation_scale * iw
                samples.append((zl, zw))
            self.noise[target['name']] = samples

    def perceive(self, step, t, ego, targets):
        settings = self.settings
        for target in targets:
            truth = target['true']
            distance, bearing = sensor_geometry(settings, ego, truth)
            in_domain = (settings['min_distance_m'] <= distance <= settings['max_distance_m'] and
                         abs(math.degrees(bearing)) <= settings['max_abs_bearing_deg'] + 1e-10)
            reason = ('outside_domain' if not in_domain else
                      'wrong_class' if settings['only_correct_class'] and not target['class_correct'] else '')
            before = {k: target[k] for k in ('length', 'width')}
            note = dict(distance_m=distance, bearing_deg=math.degrees(bearing), in_domain=int(in_domain),
                        applied=int(not reason), skip_reason=reason,
                        true_length_m=truth['length'], true_width_m=truth['width'],
                        input_length_m=before['length'], input_width_m=before['width'])
            if not reason:
                noise = self.noise[target['name']][step]
                for dimension, z in zip(('length', 'width'), noise):
                    if not math.isfinite(before[dimension]) or before[dimension] <= 0:
                        raise ValueError(f'{target["name"]}: perceived {dimension} must be finite and positive')
                    mu, sigma = parameters(settings, distance, bearing, dimension)
                    raw = mu + sigma * z
                    limit = settings['max_abs_log_error']
                    used = max(-limit, min(limit, raw))
                    target[dimension] = before[dimension] * math.exp(used)
                    note.update({dimension + '_mu': mu, dimension + '_sigma': sigma, dimension + '_z': z,
                                 dimension + '_raw_log_error': raw, dimension + '_clipped': int(used != raw)})
            for dimension in ('length', 'width'):
                note[dimension + '_error_m'] = target[dimension] - truth[dimension]
                note['estimated_' + dimension + '_m'] = target[dimension]
            target['log'].update(note)

    def describe(self):
        return {
            'model': 'clipped lognormal physical length/width error conditional on planar sensor distance and azimuth',
            'features': '[1, log1p(r/reference_distance_m), (sin(beta)/sin(reference_bearing_deg))^2, interaction]',
            'distribution': 'log(output/input) = clip(mu + sigma*z, -max_abs_log_error, max_abs_log_error)',
            'noise': 'stationary Gaussian OU per target, plugin label and seed; optional length/width correlation',
            'scope': 'perceived length/width only; outside-domain targets pass through without changing visibility',
            'symmetry': 'equal surfaces for positive and negative azimuth',
            **copy.deepcopy(self.settings),
        }
