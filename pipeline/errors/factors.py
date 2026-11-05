"""Size, Identification and Detection range errors, with an optional OU term while the target is seen."""
import math

from .frames import missed_steps, wrong_class_steps
from .ou import unit_samples

INJECTS = True
DEFAULTS = {
    'scale': 1.0,                   # Size: measured range = scale * r + bias_m
    'bias_m': 0.0,
    'wrong_rate': 0.0,              # Identification: share of frames with the wrong class
    'wrong_scale': 1.0,             # replaces scale and bias_m while the class is wrong
    'wrong_bias_m': 0.0,
    'wrong_mean_s': 0.5,            # mean duration of a wrong-class episode
    'miss_rate': 0.0,               # Detection: share of lost frames
    'miss_model': 'markov',         # 'markov' or 'iid'
    'miss_mean_s': 0.5,             # mean duration of a loss (markov)
    'sensor_period_s': 0.05,        # a frame's state holds for every step inside the frame
    'lost_target': 'vanish',        # missed step: 'vanish' hides the target, 'predict' continues from the last seen step
    'lost_target_motion': 'live',   # predicted step: 'live' true target speed, 'frozen' speed of the last seen step
    # predicted distance: 'frozen_difference' holds both speeds at the last seen step, 'frozen_object' only the target's
    'lost_target_distance': 'frozen_difference',
    'ou_mae_pct': 0.0,              # OU error while seen: mean error at noise_reference_range_m, in percent
}
NUMBERS = {
    'scale': (0.1, 10), 'bias_m': (-50, 50), 'wrong_rate': (0, 0.5), 'wrong_scale': (0.1, 10),
    'wrong_bias_m': (-50, 50), 'wrong_mean_s': (0.01, 20), 'miss_rate': (0, 0.5), 'miss_mean_s': (0.01, 20),
    'sensor_period_s': (0.01, 1), 'ou_mae_pct': (0, 100),
}
CHOICES = {
    'miss_model': ('markov', 'iid'), 'lost_target': ('vanish', 'predict'), 'lost_target_motion': ('live', 'frozen'),
    'lost_target_distance': ('frozen_difference', 'frozen_object'),
}


def validate(c):
    raw = c['range_factors']
    if not isinstance(raw, dict):
        raise ValueError('uncertainty "range_factors" needs a range_factors object (it can be empty).')
    extra = set(raw) - set(DEFAULTS)
    if extra:
        raise ValueError('Unknown range_factors settings: ' + ', '.join(sorted(extra)))
    factors = DEFAULTS | raw
    for key, choices in CHOICES.items():
        if factors[key] not in choices:
            raise ValueError(f'range_factors.{key}: must be one of {", ".join(choices)}')
    if factors['lost_target_motion'] == 'frozen' and factors['lost_target'] != 'predict':
        raise ValueError('lost_target_motion "frozen" needs lost_target "predict".')
    if factors['lost_target_distance'] == 'frozen_object' and factors['lost_target_motion'] != 'frozen':
        raise ValueError('lost_target_distance "frozen_object" needs lost_target_motion "frozen".')
    for key, (low, high) in NUMBERS.items():
        value = factors[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'range_factors.{key}: must be a number')
        factors[key] = float(value)
        if not math.isfinite(factors[key]) or not low <= factors[key] <= high:
            raise ValueError(f'range_factors.{key}: must be between {low} and {high}')
    period = factors['sensor_period_s']
    if abs(period / c['dt_s'] - round(period / c['dt_s'])) > 1e-9:
        raise ValueError('range_factors.sensor_period_s must be a whole number of time steps.')
    if factors['wrong_mean_s'] < period or factors['miss_mean_s'] < period:
        raise ValueError('Mean durations cannot be shorter than sensor_period_s.')
    c['range_factors'] = factors


def mean_error_pct(c):
    return c['range_factors']['ou_mae_pct']


def settings(c):
    factors = c['range_factors']
    items = [(key, factors[key]) for key in ('scale', 'bias_m', 'wrong_scale', 'wrong_bias_m')]
    if factors['lost_target'] == 'predict':
        items.append(('predict_when_lost', 1))
    if factors['lost_target_motion'] == 'frozen':
        items.append(('freeze_motion_when_lost', 1))
    if factors['lost_target_distance'] == 'frozen_object':
        items.append(('live_observer_when_lost', 1))
    return items


def tracks(c, target_id, steps):
    factors = c['range_factors']
    return {'z': unit_samples(c, target_id, steps) if factors['ou_mae_pct'] else [0.0] * steps,
            'missed': missed_steps(factors, c['seed'], target_id, c['dt_s'], steps),
            'wrong': wrong_class_steps(factors, c['seed'], target_id, c['dt_s'], steps)}


def describe(c):
    factors = c['range_factors']
    if factors['lost_target'] != 'predict':
        missing = 'A missed detection or negative range hides the target for that controller step.'
    elif factors['lost_target_distance'] == 'frozen_object':
        missing = ("A missed detection reports the last measured range, stepped on by the ego's live speed minus the "
                   'target speed of the last seen step, no lower than the boxes touching.')
    else:
        missing = ('A missed detection reports the last measured range minus the last speed difference times the '
                   'elapsed time, no lower than the boxes touching.')
    return {'model': 'range_factors_v1',
            'measured_range': 's * r + b, with s and b replaced by wrong_scale and wrong_bias_m while the class is wrong',
            'distance': 'true bounding-box centre-to-centre range r, in metres',
            'factors': factors,
            'detection': "Zhuo Ma's two-state Markov or per-frame IID loss: enter = p*recover/(1-p), recover = period/mean duration, stationary start.",
            'identification': 'The same two-state chain on a separate stream, with wrong_rate and wrong_mean_s.',
            'missing': missing,
            'calibration': 'Assumed values; a sensitivity model, not measured sensor risk.'}
