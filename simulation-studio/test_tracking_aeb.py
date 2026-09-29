"""Behavioral regressions for measurement clocks, causal tracking and braking."""
import math
import random
import unittest
from dataclasses import replace
from models import AEB, Observation, Perception
from settings import DEFAULT,validate
from tracking import RangeTracker,simulation_times,observation_ticks
from scenario import Reference
from ego_path import RecordedEgoPath
from test_models import vehicle


class TrackingTests(unittest.TestCase):
    def config(self,**changes):
        return DEFAULT|{'controller':'aeb_follow','uncertainty':'gaussian_range',**changes}

    def test_filter_reduces_stationary_noise_without_truth_input(self):
        c=self.config(kalman_range_sigma_m=5)
        tracker=RangeTracker(c)
        rng=random.Random(44)
        raw=[];filtered=[]
        for i in range(200):
            z=50+rng.gauss(0,5)
            obs=Observation(1,z,0,0,0,0,4,2)
            ready,diag=tracker.update(i*.05,vehicle(speed=0),[obs])
            if ready:
                raw.append((z-50)**2);filtered.append((diag[1]['estimated_range_m']-50)**2)
        self.assertLess(sum(filtered)/len(filtered),sum(raw)/len(raw)*.25)

    def test_causal_prefix_and_truth_not_in_interface(self):
        a,b=RangeTracker(self.config()),RangeTracker(self.config())
        for i in range(10):
            obs=Observation(1,40-i,0,5,0,0,4,2)
            self.assertEqual(a.update(i*.05,vehicle(),[obs]),b.update(i*.05,vehicle(),[obs]))
        before=a.tracks[1]['x'].copy()
        b.update(.5,vehicle(),[replace(obs,x=100)])
        self.assertEqual(list(a.tracks[1]['x']),list(before))

    def test_warmup_missing_and_reacquisition(self):
        model=RangeTracker(self.config(kalman_warmup_s=.15))
        obs=Observation(1,40,0,10,0,0,4,2)
        for t in [0,.05,.1]: self.assertEqual(model.update(t,vehicle(),[obs])[0],[])
        self.assertEqual(len(model.update(.15,vehicle(),[obs])[0]),1)
        self.assertEqual(model.update(.2,vehicle(),[])[0],[])
        self.assertEqual(model.update(.5,vehicle(),[obs])[0],[])

    def test_exact_and_raw_modes_do_not_modify_observations(self):
        obs=Observation(1,40,0,10,0,0,4,2)
        for c in [self.config(observation_filter='raw'),self.config(uncertainty='ideal'),self.config(range_sigma_m=0)]:
            self.assertEqual(RangeTracker(c).update(0,vehicle(),[obs])[0],[obs])

    def test_sample_clock_independent_of_integrator(self):
        values=[]
        target=vehicle(40,0,0)|{'id':1,'name':'Target'}
        for dt in [.01,.02,.05]:
            c=self.config(dt_s=dt,observation_period_s=.05)
            p=Perception(c);ticks=set(observation_ticks(.98,.05))
            ts=simulation_times(.98,dt,.05)
            self.assertEqual(ts[-1],.98)
            self.assertTrue(all(0<b-a<=dt+1e-8 for a,b in zip(ts,ts[1:])))
            values.append([(t,p.observe(vehicle(),[target],detections=[target])[1][0]['sampled_range_error_m']) for t in ts if t in ticks])
        self.assertEqual(values[0],values[1]);self.assertEqual(values[1],values[2])

    def test_legacy_configuration_and_invalid_inputs(self):
        c=validate({'controller':'aeb'})
        self.assertEqual((c['observation_filter'],c['observation_period_s'],c['aeb_trigger_mode']),('raw',0,'instant'))
        self.assertEqual(c['kalman_noise_mode'], 'fixed')
        self.assertEqual(validate({})['kalman_noise_mode'], 'fixed')
        self.assertEqual(validate(DEFAULT|{'kalman_noise_mode':'predicted_range'})['kalman_noise_mode'], 'predicted_range')
        with self.assertRaises(ValueError):
            validate(DEFAULT|{'kalman_noise_mode':'truth'})
        for field,value in [('kalman_range_sigma_m',0),('kalman_accel_sigma_mps2',float('nan')),('observation_period_s',True),('aeb_confirmation_s',-1)]:
            with self.subTest(field=field),self.assertRaises(ValueError):validate(DEFAULT|{field:value})

    def test_adaptive_weights_use_prior_not_current_measurement(self):
        c=self.config(uncertainty='distance_gaussian_range',kalman_noise_mode='predicted_range')
        a,b=RangeTracker(c),RangeTracker(c)
        first=Observation(1,100,0,0,0,0,4,2)
        for tracker in [a,b]: tracker.update(0,vehicle(speed=0),[first])
        da=a.update(.05,vehicle(speed=0),[replace(first,x=10)])[1][1]
        db=b.update(.05,vehicle(speed=0),[replace(first,x=200)])[1][1]
        self.assertEqual(da['assumed_range_sigma_m'],db['assumed_range_sigma_m'])
        self.assertNotEqual(da['estimated_range_m'],db['estimated_range_m'])

    def test_adaptive_noise_tracks_range_and_uncertainty(self):
        tracker=RangeTracker(self.config(uncertainty='distance_gaussian_range',kalman_noise_mode='predicted_range'))
        self.assertAlmostEqual(math.sqrt(tracker.range_variance(100)),50*math.sqrt(math.pi/2))
        self.assertLess(tracker.range_variance(10),tracker.range_variance(100))
        self.assertGreater(tracker.range_variance(3,100),tracker.range_variance(3,0))
        self.assertGreater(tracker.range_variance(-10),0)

    def test_small_first_measurement_does_not_lock_track_at_zero(self):
        tracker=RangeTracker(self.config(uncertainty='distance_gaussian_range',kalman_noise_mode='predicted_range'))
        obs=Observation(1,.01,0,0,0,0,4,2)
        tracker.update(0,vehicle(speed=0),[obs])
        self.assertGreaterEqual(tracker.tracks[1]['P'][0,0],100)
        for i in range(1,21):
            diag=tracker.update(i*.05,vehicle(speed=0),[replace(obs,x=100)])[1][1]
        self.assertGreater(diag['estimated_range_m'],50)
        self.assertTrue(math.isfinite(diag['estimate_std_m']))

    def test_missing_mode_retains_fixed_filter_results(self):
        c=self.config(uncertainty='distance_gaussian_range',kalman_noise_mode='fixed')
        legacy={k:v for k,v in c.items() if k!='kalman_noise_mode'}
        a,b=RangeTracker(c),RangeTracker(legacy)
        for i,z in enumerate([100,50,130,80,90]):
            obs=Observation(1,z,0,0,0,0,4,2)
            self.assertEqual(a.update(i*.05,vehicle(speed=0),[obs]),b.update(i*.05,vehicle(speed=0),[obs]))


class ConfirmationTests(unittest.TestCase):
    def controller(self,**changes):
        return AEB(DEFAULT|{'controller':'aeb','ttc_threshold_s':2,'aeb_confirmation_s':.15,'reaction_delay_s':.1,**changes})

    def hazard(self,ident=1): return Observation(ident,20,0,0,0,0,4,2)

    def test_single_spike_does_not_latch(self):
        a=self.controller()
        a.command(0,vehicle(),[self.hazard()])
        a.command(.05,vehicle(),[])
        for t in [.1,.2,.5]: self.assertEqual(a.command(t,vehicle(),[])[0],0)
        self.assertIsNone(a.brake_time)
        self.assertEqual(a.cancelled_confirmations,1)

    def test_confirmation_uses_new_measurements_and_same_target(self):
        a=self.controller()
        a.command(0,vehicle(),[self.hazard()])
        a.command(.3,vehicle(),[],fresh=False)
        self.assertIsNone(a.trigger_time)
        a.command(.31,vehicle(),[self.hazard(2)])
        self.assertIsNone(a.trigger_time)
        a.command(.36,vehicle(),[self.hazard(2)])
        a.command(.46,vehicle(),[self.hazard(2)])
        self.assertAlmostEqual(a.trigger_time,.46)

    def test_clear_during_response_delay_cancels_then_real_hazard_brakes(self):
        a=self.controller()
        for t in [0,.05,.1,.15]: a.command(t,vehicle(),[self.hazard()])
        self.assertAlmostEqual(a.trigger_time,.15)
        a.command(.2,vehicle(),[])
        self.assertEqual(a.command(.25,vehicle(),[],fresh=False)[0],0)
        for t in [.3,.35,.4,.45,.5]: a.command(t,vehicle(),[self.hazard()])
        self.assertEqual(a.command(.55,vehicle(),[self.hazard()])[0],-8)
        self.assertEqual(a.command(.6,vehicle(),[])[0],-8)
        self.assertEqual(a.command(2,vehicle(speed=0),[])[0],0)
        self.assertEqual(a.state,'stopped')

    def test_no_threat_parallel_motion_does_not_brake(self):
        a=self.controller()
        safe=Observation(1,30,0,10,0,0,4,2)
        for i in range(100): self.assertEqual(a.command(i*.05,vehicle(),[safe])[0],0)
        self.assertIsNone(a.brake_time)


class EgoPathTests(unittest.TestCase):
    def test_takeover_is_continuous_and_follows_path_until_stop(self):
        ref=Reference('gidas_1554431')
        t=4.;ego=next(s for s in ref.states(t) if s['id']==ref.ego_id)
        path=RecordedEgoPath(ref)
        start=path.advance(ego,-8,0,t)
        self.assertAlmostEqual(start['x'],ego['x'],places=8)
        self.assertAlmostEqual(start['y'],ego['y'],places=8)
        first=path.distance
        for _ in range(500): ego=path.advance(ego,-8,.01,t)
        self.assertEqual(ego['speed'],0)
        source=next(s for s in ref.states(t) if s['id']==ref.ego_id)
        self.assertAlmostEqual(path.distance-first,source['speed']**2/16,places=7)
        self.assertTrue(math.isfinite(ego['h']))


if __name__=='__main__':unittest.main(verbosity=2)
