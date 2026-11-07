"""Two pictures for one run: speed.png and range.png."""
import csv
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .common import read_json

SPEED, BRAKE = '#197f7a', '#bc632d'


def read_table(path):
    with Path(path).open(encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def values(rows, key):
    return [float(row[key]) if row[key] else float('nan') for row in rows]


def speed_plot(out, ego, summary):
    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    t = values(ego, 'time_s')
    axes[0].plot(t, values(ego, 'speed_kph'), color=SPEED, lw=2)
    axes[0].set(ylabel='Ego speed (km/h)', title='Ego response')
    axes[1].step(t, values(ego, 'command_accel_mps2'), where='post', color=BRAKE)
    axes[1].set(xlabel='Simulation time (s)', ylabel='Controller acceleration (m/s²)')
    for axis in axes:
        if summary.get('first_brake_time_s') is not None:
            axis.axvline(summary['first_brake_time_s'], color=BRAKE, ls=':', lw=1)
        axis.grid(alpha=0.2)
    figure.savefig(out / 'speed.png', dpi=150, bbox_inches='tight')
    plt.close(figure)


def range_plot(out, observations):
    figure, (axis, error_axis) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for name in dict.fromkeys(row['object_name'] for row in observations):
        rows = [row for row in observations if row['object_name'] == name]
        t = values(rows, 'time_s')
        truth = values(rows, 'true_range_m')
        observed = values(rows, 'observed_range_m')
        line, = axis.plot(t, truth, lw=1.8, label=f'{name}: truth')
        axis.plot(t, observed, ls='--', alpha=0.65, color=line.get_color(), label=f'{name}: observed')
        error_axis.plot(t, [o - r for o, r in zip(observed, truth)], color=line.get_color(), alpha=0.6, label=name)
    axis.set(ylabel='Centre-to-centre range (m)', title='Truth and observed range')
    axis.legend(fontsize=8)
    error_axis.axhline(0, color='#7b8793', lw=0.7)
    error_axis.set(xlabel='Simulation time (s)', ylabel='Range error (m)')
    for item in (axis, error_axis):
        item.grid(alpha=0.2)
    figure.savefig(out / 'range.png', dpi=150, bbox_inches='tight')
    plt.close(figure)


def make_plots(out):
    out = Path(out)
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    speed_plot(out, read_table(out / 'ego.csv'), read_json(out / 'summary.json'))
    range_plot(out, read_table(out / 'observations.csv'))
