"""Detection: in some sensor frames the target is lost.

Lost frames come from Zhuo Ma's two-state Markov chain (miss_model "markov": share miss_rate, mean loss
miss_mean_s) or from independent frames ("iid"). A frame's state holds for every step inside it.
What the controller gets on a lost step (lost_target):
    vanish    nothing: the target is hidden
    predict   the last measured range, moved on by
                  lost_target_distance "frozen_difference": (ego speed - target speed), both of the last seen step
                  lost_target_distance "frozen_object": (live ego speed - target speed of the last seen step)
              times the elapsed time, no lower than the two boxes touching. With lost_target_motion "frozen" the
              target's speed, velocity and acceleration of the last seen step are reported as well.
Put detection last, so the range it remembers includes the other errors.
"""
from .frames import check_period, missed_steps

SETTINGS = {'miss_rate': 0.0, 'miss_model': 'markov', 'miss_mean_s': 0.5, 'sensor_period_s': 0.05,
            'lost_target': 'vanish', 'lost_target_motion': 'live', 'lost_target_distance': 'frozen_difference'}
RANGES = {'miss_rate': (0, 0.5), 'miss_mean_s': (0.01, 20), 'sensor_period_s': (0.01, 1)}
CHOICES = {'miss_model': ('markov', 'iid'), 'lost_target': ('vanish', 'predict'), 'lost_target_motion': ('live', 'frozen'),
           'lost_target_distance': ('frozen_difference', 'frozen_object')}
LOG = ('missed',)
HELD = ('speed', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')


def check(settings, run):
    if settings['lost_target_motion'] == 'frozen' and settings['lost_target'] != 'predict':
        raise ValueError('lost_target_motion "frozen" needs lost_target "predict"')
    if settings['lost_target_distance'] == 'frozen_object' and settings['lost_target_motion'] != 'frozen':
        raise ValueError('lost_target_distance "frozen_object" needs lost_target_motion "frozen"')
    check_period(settings['sensor_period_s'], settings['miss_mean_s'], run.dt_s)


class Error:
    def __init__(self, settings, run):
        self.settings = settings
        self.predict = settings['lost_target'] == 'predict'
        self.freeze = settings['lost_target_motion'] == 'frozen'
        self.live_ego = settings['lost_target_distance'] == 'frozen_object'
        self.missed = {target['name']: missed_steps(settings, run.seed, target['id'], run.dt_s, run.steps)
                       for target in run.targets}
        self.last = {}

    def perceive(self, step, t, ego, targets):
        for target in targets:
            name, true = target['name'], target['true']
            missed = self.missed[name][step]
            target['log']['missed'] = int(missed)
            if not self.predict:
                if missed:
                    target['seen'] = False
                continue
            if not missed:
                if target['seen'] and target['range'] >= 0.0:
                    self.last[name] = {'range': target['range'], 'closing': ego['speed'] - true['speed'], 'time': t,
                                       **{key: true[key] for key in HELD}}
                continue
            last = self.last.get(name)
            if last is None:
                target['seen'] = False  # never seen yet: nothing to predict from
                continue
            contact = (ego['length'] + true['length']) / 2.0
            if self.live_ego:
                # The held range stays unclamped: only the reported distance stops at contact.
                last['range'] = last['range'] - (ego['speed'] - last['speed']) * (t - last['time'])
                last['time'] = t
                distance = last['range']
            else:
                distance = last['range'] - last['closing'] * (t - last['time'])
            target['range'] = max(contact, distance)
            target['seen'] = target['predicted'] = True
            if self.freeze:
                target.update({key: last[key] for key in HELD})

    def describe(self):
        return {'frames': "Zhuo Ma's two-state Markov or per-frame IID loss: enter = p*recover/(1-p), "
                          'recover = period/mean duration, stationary start.', **self.settings}
