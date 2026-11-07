"""Template plugin: white Gaussian range noise. Copy this file to start a new error plugin.

1. Copy it to pipeline/errors/<your_name>.py; the file name becomes the "type".
2. Change SETTINGS and perceive(). pipeline/errors/__init__.py lists everything a target offers.
3. Use it: "errors": [{"type": "<your_name>", ...your settings}]. No compile is needed.
"""
import random

SETTINGS = {'sigma_m': 0.0}        # every setting with its default
RANGES = {'sigma_m': (0, 50)}      # allowed (low, high) for number settings
LOG = ('error_m',)                 # values written to perception_applied.tsv as "<name>.error_m"


class Error:
    def __init__(self, settings, run):
        """Called once per run. run has seed, dt_s, steps, duration_s, scenario, ego, targets and label."""
        self.sigma = settings['sigma_m']
        # One random stream per plugin and target: adding a target or a plugin changes no other errors.
        self.random = {target['name']: random.Random(f'{run.seed}:{run.label}:{target["id"]}') for target in run.targets}

    def perceive(self, step, t, ego, targets):
        """Called before every controller step. Change what the controller sees of each target."""
        for target in targets:
            error = self.random[target['name']].gauss(0.0, self.sigma)
            target['range'] += error
            target['log']['error_m'] = error

    def describe(self):
        """Saved in summary.json."""
        return {'range': '+= N(0, sigma_m^2), independent every step', 'sigma_m': self.sigma}
