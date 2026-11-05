"""Run configuration: defaults and checks. A configuration file only lists what differs from DEFAULT."""
import copy

from . import controllers, errors, scenarios

DEFAULT = {
    'scenario': 'gidas_1549178',
    'controller': 'hidrive',
    'errors': [],                       # error plugins in the order they run; see errors/__init__.py
    'seed': 42,
    'dt_s': 0.01,
}
# Settings of the first version. They now live in error plugins.
OLD = {'uncertainty', 'range_factors', 'noise_zero_range_m', 'noise_reference_range_m', 'noise_reference_mae_pct',
       'noise_correlation_time_s'}


def validate(raw):
    """Return the complete configuration, or raise ValueError."""
    if not isinstance(raw, dict):
        raise ValueError('A configuration must be a JSON object.')
    # duration_s comes from the scenario; saved config.json files contain it.
    raw = {key: value for key, value in raw.items() if key != 'duration_s'}
    extra = set(raw) - set(DEFAULT)
    if extra & OLD:
        raise ValueError(f'{", ".join(sorted(extra & OLD))}: errors are now set with "errors", '
                         'such as "errors": [{"type": "ou", "mae_pct": 5.0}]. See README.md.')
    if extra:
        raise ValueError('Unknown settings: ' + ', '.join(sorted(extra)))
    c = copy.deepcopy(DEFAULT | raw)
    scenario = scenarios.get(c['scenario'])
    controllers.get(c['controller'])
    if c['dt_s'] not in (0.01, 0.02, 0.05):
        raise ValueError('dt_s: must be 0.01, 0.02 or 0.05')
    if isinstance(c['seed'], bool) or not isinstance(c['seed'], int) or not 0 <= c['seed'] <= 2147483647:
        raise ValueError('seed: must be an integer from 0 to 2147483647')
    c['errors'] = errors.validate(c['errors'], c)
    c['duration_s'] = scenario.duration(c['dt_s'])
    return c
