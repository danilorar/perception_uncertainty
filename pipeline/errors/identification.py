"""Identification: in some sensor frames the class is wrong, so the size assumption and the range are wrong.

On a wrong frame:  range += (wrong_scale - 1) * true range + wrong_bias_m, and class_correct becomes False
(size then skips that step). Wrong frames come from a two-state Markov chain with share wrong_rate and mean
episode length wrong_mean_s; a frame's state holds for every step inside it.
"""
from .frames import check_period, wrong_class_steps

SETTINGS = {'wrong_rate': 0.0, 'wrong_mean_s': 0.5, 'wrong_scale': 1.0, 'wrong_bias_m': 0.0, 'sensor_period_s': 0.05}
RANGES = {'wrong_rate': (0, 0.5), 'wrong_mean_s': (0.01, 20), 'wrong_scale': (0.1, 10), 'wrong_bias_m': (-50, 50),
          'sensor_period_s': (0.01, 1)}
LOG = ('wrong', 'error_m')


def check(settings, run):
    check_period(settings['sensor_period_s'], settings['wrong_mean_s'], run.dt_s)


class Error:
    def __init__(self, settings, run):
        self.settings = settings
        self.scale, self.bias = settings['wrong_scale'], settings['wrong_bias_m']
        self.wrong = {target['name']: wrong_class_steps(settings, run.seed, target['id'], run.dt_s, run.steps)
                      for target in run.targets}

    def perceive(self, step, t, ego, targets):
        for target in targets:
            wrong = self.wrong[target['name']][step]
            target['log']['wrong'] = int(wrong)
            if wrong:
                error = (self.scale - 1.0) * target['true_range'] + self.bias
                target['range'] = target['range'] + error
                target['class_correct'] = False
                target['log']['error_m'] = error

    def describe(self):
        return {'frames': "Zhuo Ma's two-state Markov chain on its own stream (see frames.py)", **self.settings}
