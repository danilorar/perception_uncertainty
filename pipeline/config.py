"""Run configuration: defaults and checks. A configuration file only lists what differs from DEFAULT."""
import math

from . import controllers, errors, scenarios

DEFAULT = {
    'scenario': 'gidas_1549178',
    'controller': 'hidrive',
    'uncertainty': 'recorded_truth',    # error model, see errors/__init__.py
    # Range error of distance_ou_range: mean absolute error grows linearly from zero_range to reference_range.
    'noise_zero_range_m': 3.0,
    'noise_reference_range_m': 100.0,
    'noise_reference_mae_pct': 50.0,
    'noise_correlation_time_s': 0.5,
    'range_factors': None,              # settings of the range_factors model, see errors/factors.py
    'seed': 42,
    'dt_s': 0.01,
}
NUMBERS = {
    'noise_zero_range_m': (0, 499),
    'noise_reference_range_m': (1, 500),
    'noise_reference_mae_pct': (0, 100),
    'noise_correlation_time_s': (0.01, 20),
}


def validate(raw):
    """Return the complete configuration, or raise ValueError."""
    if not isinstance(raw, dict):
        raise ValueError('A configuration must be a JSON object.')
    # duration_s comes from the scenario; saved config.json files contain it.
    raw = {key: value for key, value in raw.items() if key != 'duration_s'}
    extra = set(raw) - set(DEFAULT)
    if extra:
        raise ValueError('Unknown settings: ' + ', '.join(sorted(extra)))
    c = DEFAULT | raw
    scenario = scenarios.get(c['scenario'])
    controllers.get(c['controller'])
    if c['uncertainty'] not in errors.MODELS:
        raise ValueError(f'Unknown error model {c["uncertainty"]!r}. Available: {", ".join(errors.MODELS)}')
    for key, (low, high) in NUMBERS.items():
        if isinstance(c[key], bool) or not isinstance(c[key], (int, float)):
            raise ValueError(f'{key}: must be a number')
        c[key] = float(c[key])
        if not math.isfinite(c[key]) or not low <= c[key] <= high:
            raise ValueError(f'{key}: must be between {low} and {high}')
    if c['dt_s'] not in (0.01, 0.02, 0.05):
        raise ValueError('dt_s: must be 0.01, 0.02 or 0.05')
    if isinstance(c['seed'], bool) or not isinstance(c['seed'], int) or not 0 <= c['seed'] <= 2147483647:
        raise ValueError('seed: must be an integer from 0 to 2147483647')
    errors.MODELS[c['uncertainty']].validate(c)
    c['duration_s'] = scenario.duration(c['dt_s'])
    return c
