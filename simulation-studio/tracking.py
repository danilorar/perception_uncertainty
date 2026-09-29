"""Causal range tracking. This module receives observations, never target truth.

State: [centre range, range rate]. Radial velocity is an auxiliary measurement
under the existing ideal target velocity/bearing assumption. Only range is noisy.
R is fixed or estimated from the prior range, never hidden truth/noise samples.
"""
from dataclasses import replace
import math
import numpy as np
from models import center


def observation_ticks(duration, period):
    return [round(i*period,8) for i in range(math.floor((duration+1e-8)/period)+1)]


def simulation_times(duration, dt, period):
    """Split integration steps at measurement times, including incommensurate grids."""
    return sorted(set(observation_ticks(duration,dt) + observation_ticks(duration,period or dt) + [duration]))


class RangeTracker:
    def __init__(self, config):
        self.c = config
        self.tracks = {}

    def range_variance(self, predicted_range, predicted_variance=0.0):
        """Approximate heteroscedastic R using only the prior state.

        The P allowance avoids treating an uncertain near-zero prediction as an
        exact measurement. It is a conservative plug-in approximation, not an
        exact treatment of the rejected negative-range distribution.
        """
        if (self.c.get('kalman_noise_mode', 'fixed') != 'predicted_range' or
                self.c['uncertainty'] != 'distance_gaussian_range'):
            return self.c['kalman_range_sigma_m']**2
        zero = self.c['noise_zero_range_m']
        reference = self.c['noise_reference_range_m']
        slope = reference*self.c['noise_reference_mae_pct']/100/(reference-zero)*math.sqrt(math.pi/2)
        return max(0.1**2, slope**2*(max(0.0, predicted_range-zero)**2 + max(0.0, predicted_variance)))

    def update(self, time, ego, observations):
        result, diagnostics = [], {}
        ex, ey = center(ego)
        period = self.c['observation_period_s'] or self.c['dt_s']
        for ident in list(self.tracks):
            if time-self.tracks[ident]['time'] > max(0.2,2.5*period):
                del self.tracks[ident]
        for obs in observations:
            dx, dy = obs.x-ex, obs.y-ey
            measured = math.hypot(dx,dy)
            nx, ny = (dx/measured,dy/measured) if measured > 1e-9 else (math.cos(ego['h']),math.sin(ego['h']))
            rate = (obs.vx-ego['vx'])*nx+(obs.vy-ego['vy'])*ny
            exact = (self.c['uncertainty'] in ['ideal','recorded_truth'] or
                     (self.c['uncertainty']=='gaussian_range' and self.c['range_sigma_m']==0 and self.c['range_bias_m']==0) or
                     (self.c['uncertainty']=='distance_gaussian_range' and self.c['noise_reference_mae_pct']==0))
            if self.c['observation_filter']=='raw' or exact:
                result.append(obs)
                diagnostics[obs.id] = dict(estimated_range_m=measured,estimated_range_rate_mps=rate,
                    estimate_std_m=0.0 if exact else None,track_ready=True,track_samples=1,
                    controller_range_m=measured,innovation_m=None,filter_status='exact' if exact else 'raw',
                    assumed_range_sigma_m=None)
                continue
            variance = self.c['kalman_range_sigma_m']**2
            track = self.tracks.get(obs.id)
            innovation = None
            if track is None:
                # Bootstrap from the first measurement, with the configured
                # fixed sigma as a minimum initial uncertainty (not a later floor).
                variance = max(variance, self.range_variance(measured))
                track = dict(x=np.array([measured,rate]),P=np.diag([variance,1e-4]),
                    time=time,birth=time,count=1)
                self.tracks[obs.id] = track
            else:
                dt = time-track['time']
                A = np.array([[1.,dt],[0.,1.]])
                G = np.array([dt*dt/2,dt])
                predicted = A@track['x']
                P = A@track['P']@A.T + np.outer(G,G)*self.c['kalman_accel_sigma_mps2']**2
                variance = self.range_variance(float(predicted[0]), float(P[0,0]))
                # Accurate velocity is an explicit inherited model assumption,
                # not a distance-truth input. Small variance avoids singular matrices.
                R = np.diag([variance,1e-4])
                residual = np.array([measured,rate])-predicted
                innovation = float(residual[0])
                K = np.linalg.solve((P+R).T,P.T).T
                I = np.eye(2)-K
                track.update(x=predicted+K@residual,P=I@P@I.T+K@R@K.T,
                             time=time,count=track['count']+1)
            estimate, estimate_rate = map(float,track['x'])
            ready = (track['count']>=2 and time-track['birth']+1e-8>=self.c['kalman_warmup_s'] and estimate>=0)
            if ready:
                result.append(replace(obs,x=ex+estimate*nx,y=ey+estimate*ny))
            diagnostics[obs.id] = dict(estimated_range_m=estimate,estimated_range_rate_mps=estimate_rate,
                estimate_std_m=math.sqrt(max(0,float(track['P'][0,0]))),track_ready=ready,
                track_samples=track['count'],controller_range_m=estimate if ready else None,
                innovation_m=innovation,filter_status='tracking' if ready else 'warming_up',
                assumed_range_sigma_m=math.sqrt(variance))
        return result, diagnostics
