"""Size: a wrong assumed object size gives a scaled and shifted range.

    range += (scale - 1) * true range + bias_m

Applies while the class is correct. While identification marks the class wrong, its own scale and offset
apply instead, so put identification before size.
"""
SETTINGS = {'scale': 1.0, 'bias_m': 0.0}
RANGES = {'scale': (0.1, 10), 'bias_m': (-50, 50)}
LOG = ('error_m',)


class Error:
    def __init__(self, settings, run):
        self.scale, self.bias = settings['scale'], settings['bias_m']

    def perceive(self, step, t, ego, targets):
        for target in targets:
            if target['class_correct']:
                error = (self.scale - 1.0) * target['true_range'] + self.bias
                target['range'] = target['range'] + error
                target['log']['error_m'] = error

    def describe(self):
        return {'range': '+= (scale - 1) * true range + bias_m while the class is correct',
                'scale': self.scale, 'bias_m': self.bias}
