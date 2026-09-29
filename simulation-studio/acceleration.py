"""Offline acceleration estimates from recorded positions; never controller input.

A local quadratic fit suppresses quantization before differentiation. Projecting
the fitted acceleration onto fitted velocity estimates signed tangential
acceleration (d|v|/dt), not lateral acceleration or a braking command.
"""
import csv
from pathlib import Path
import numpy as np
from settings import sha256

WINDOW_S = .30


def estimate_acceleration(times, positions, window_s=WINDOW_S):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if (times.ndim != 1 or len(times) < 3 or positions.shape != (len(times), 2)
            or not np.isfinite(times).all() or not np.isfinite(positions).all()
            or not np.all(np.diff(times) > 0) or not np.isfinite(window_s) or window_s <= 0):
        raise ValueError('Acceleration estimation needs finite ordered times and 2D positions.')
    # Source references are sampled at 0.01 s, independently of the run step.
    dt = float(np.median(np.diff(times)))
    count = min(len(times), max(3, 2*round(window_s/(2*dt))+1))
    result = np.empty(len(times))
    for i, time in enumerate(times):
        start = min(max(i-count//2, 0), len(times)-count)
        section = slice(start, start+count)
        offsets = times[section]-time
        design = np.column_stack([np.ones(count), offsets, offsets**2])
        # Subtract the local origin to improve conditioning for large coordinates.
        coefficients = np.linalg.lstsq(design, positions[section]-positions[i], rcond=None)[0]
        velocity, acceleration = coefficients[1], 2*coefficients[2]
        speed = np.linalg.norm(velocity)
        result[i] = float(velocity @ acceleration / speed) if speed > 1e-8 else 0.0
    return result


class TrajectoryAcceleration:
    def __init__(self, source_csv, ego_id):
        source_csv = Path(source_csv)
        with source_csv.open(encoding='utf-8') as f:
            rows = [r for r in csv.DictReader(f) if int(r['id']) == ego_id]
        self.times = np.array([float(r['time_s']) for r in rows])
        positions = np.array([[float(r['x']), float(r['y'])] for r in rows])
        self.values = estimate_acceleration(self.times, positions)
        self.metadata = {
            'kind':'estimated_tangential_acceleration', 'unit':'m/s^2',
            'method':'Local least-squares quadratic fits to source world x/y positions; signed tangential component v dot a / |v|.',
            'window_s':WINDOW_S, 'polynomial_degree':2,
            'endpoints':'Shift the full fitting window at the start/end; no padding or invented samples.',
            'source_file':source_csv.name, 'source_sha256':sha256(source_csv),
            'source_sample_interval_s':float(np.median(np.diff(self.times))),
            'measured':False, 'controller_input':False,
            'limitation':'Offline estimate using neighbouring past/future samples. Transitions are broadened by the fitting window; residual quantization uncertainty remains. Source positions, speeds and control commands are unchanged.',
        }

    def at(self, time):
        if time < self.times[0] or time > self.times[-1]:
            raise ValueError('Acceleration requested outside the recorded trajectory.')
        return float(np.interp(time, self.times, self.values))
