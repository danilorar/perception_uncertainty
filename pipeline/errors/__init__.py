"""Range-error models. To add one: write a module like ou.py and list it in MODELS.

A model that injects errors (INJECTS = True) provides:
  validate(c)                     check its settings; may complete them in c
  mean_error_pct(c)               amplitude of z: mean absolute error at noise_reference_range_m, in percent
  settings(c)                     extra (key, value) pairs for the noise file
  tracks(c, target_id, steps)     one list per step: 'z' (required, unit error), 'missed', 'wrong' (optional, bool)
  describe(c)                     short description saved in summary.json

esmini reads the noise file and applies, on every controller step (engine/PerceptionNoise.hpp):
  measured range = scale * r + bias_m + sigma(r) * z,  with wrong_scale, wrong_bias_m on 'wrong' steps
  a 'missed' step or a negative range hides the target, or predicts it if predict_when_lost is set
"""
from pathlib import Path

from . import factors, none, ou

MODELS = {
    'recorded_truth': none,         # no error
    'distance_ou_range': ou,        # correlated range error that grows with distance
    'range_factors': factors,       # Size, Identification, Detection, optional OU
}
# Invalid (negative) measured ranges move the target this far behind the ego.
HIDE_DISTANCE_M = 1000.0


def write_noise_file(path, c, observer, targets, log_path, steps):
    model = MODELS[c['uncertainty']]
    lines = [f'observer\t{observer}', f'zero_range_m\t{c["noise_zero_range_m"]!r}',
             f'reference_range_m\t{c["noise_reference_range_m"]!r}',
             f'reference_mae_pct\t{model.mean_error_pct(c)!r}', f'dt_s\t{c["dt_s"]!r}',
             f'hide_distance_m\t{HIDE_DISTANCE_M!r}', f'log\t{log_path}']
    lines += [f'{key}\t{value!r}' for key, value in model.settings(c)]
    for target in sorted(targets, key=lambda t: t['id']):
        tracks = model.tracks(c, target['id'], steps)
        lines.append('\t'.join(['z', target['name'], *map(repr, tracks['z'])]))
        for key in ('missed', 'wrong'):
            if key in tracks:
                lines.append('\t'.join([key, target['name'], *('1' if value else '0' for value in tracks[key])]))
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')
