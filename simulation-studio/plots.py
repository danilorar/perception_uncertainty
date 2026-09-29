import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_axes import apply_axes, save_figure, sync_plots

def make_plots(out, *, speed_only=False):
    sync_plots(out, _render_plots, speed_only=speed_only)


def _render_plots(out, specs, changed):
    out=Path(out)
    with (out/'ego.csv').open(encoding='utf-8') as f:
        ego=list(csv.DictReader(f))
    plt.rcParams.update({'font.size':10, 'axes.spines.top':False,'axes.spines.right':False})
    t=[float(r['time_s']) for r in ego]
    config=json.loads((out/'config.json').read_text(encoding='utf-8'))
    replay=config.get('controller')=='original_trajectory'
    summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
    if 'speed' in changed:
        _speed_plot(out, ego, t, config, summary, replay, specs['speed'])
    if 'range' in changed:
        with (out/'observations.csv').open(encoding='utf-8') as f:
            observations=list(csv.DictReader(f))
        _range_plot(out, observations, specs['range'])


def _speed_plot(out, ego, t, config, summary, replay, spec):
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    axes[0].plot(t,[float(r['speed_kph']) for r in ego],color='#197f7a',lw=2,label='This run')
    if any(r.get('ideal_ego_speed_kph') for r in ego):
        axes[0].plot(t,[float(r['ideal_ego_speed_kph']) if r.get('ideal_ego_speed_kph') else float('nan') for r in ego],color='#7b8793',ls='--',label='Independent ideal-range baseline')
        axes[0].legend(fontsize=8)
    axes[0].set(ylabel='Ego speed (km/h)',title='Original ego trajectory (derived speed)' if replay else 'Ego response')
    field='trajectory_accel_mps2' if replay else 'command_accel_mps2'
    if replay:
        axes[1].plot(t,[float(r[field]) for r in ego],color='#bc632d',lw=1.5)
        metadata_path=out/'acceleration_estimate.json'
        metadata=json.loads(metadata_path.read_text(encoding='utf-8')) if metadata_path.exists() else None
        if metadata:
            axes[1].set_title(f"Estimated from position: {metadata['window_s']:.2f} s quadratic fit (not measured)",fontsize=9)
    else:
        if config.get('controller')=='aeb_follow':
            axes[1].plot(t,[float(r['trajectory_accel_mps2']) if r.get('trajectory_accel_mps2') else float('nan') for r in ego],color='#8c9ea7',label='Original driver (offline estimate)')
        axes[1].step(t,[float(r[field]) if r[field] else float('nan') for r in ego],where='post',color='#bc632d',label='AEB / control command')
        axes[1].legend(fontsize=8)
    for ax in axes:
        if summary.get('first_brake_time_s') is not None:
            ax.axvline(summary['first_brake_time_s'],color='#bc632d',ls=':',lw=1)
    axes[1].set(xlabel='Simulation time (s)',ylabel='Estimated acceleration (m/s²)' if replay else 'Command acceleration (m/s²)')
    for ax in axes: ax.grid(alpha=.2)
    apply_axes(axes, spec)
    save_figure(fig, out, 'speed'); plt.close(fig)


def _range_plot(out, observations, spec):
    fig,(ax,error_ax)=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for name in dict.fromkeys(r['object_name'] for r in observations):
        rows=[r for r in observations if r['object_name']==name]
        ts=[float(r['time_s']) for r in rows]
        line,=ax.plot(ts,[float(r['true_range_m']) for r in rows],label=f'{name}: truth',lw=1.8)
        ax.plot(ts,[float(r['observed_range_m']) if r['observed_range_m'] else float('nan') for r in rows],ls='--',alpha=.65,color=line.get_color(),label=f'{name}: observed')
        estimated=[float(r['estimated_range_m']) if r.get('estimated_range_m') else float('nan') for r in rows]
        if any(r.get('estimated_range_m') for r in rows):
            ax.plot(ts,estimated,color='#bc632d',lw=1.8,label=f'{name}: estimated')
            error_ax.plot(ts,[e-float(r['true_range_m']) for e,r in zip(estimated,rows)],color='#bc632d',label='Estimate error')
        error_ax.plot(ts,[float(r['observed_range_m'])-float(r['true_range_m']) if r['observed_range_m'] else float('nan') for r in rows],color=line.get_color(),alpha=.45,label='Valid observation error')
    ax.set(ylabel='Centre-to-centre range (m)',title='Truth, noisy observations and causal estimate')
    ax.grid(alpha=.2); ax.legend(fontsize=8,ncol=2)
    error_ax.axhline(0,color='#7b8793',lw=.7)
    error_ax.set(xlabel='Simulation time (s)',ylabel='Range error (m)')
    error_ax.grid(alpha=.2); error_ax.legend(fontsize=8,ncol=2)
    apply_axes((ax,error_ax), spec)
    save_figure(fig, out, 'range'); plt.close(fig)
