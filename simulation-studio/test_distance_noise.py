"""Distance-dependent Gaussian calibration and safe measurement handling."""
import math
import unittest
from unittest.mock import patch
from models import Perception, range_error_parameters
from settings import validate
from test_models import vehicle


class DistanceNoiseTests(unittest.TestCase):
    def config(self, **extra):
        return validate({'uncertainty':'distance_gaussian_range','sensor_range_m':500,**extra})

    def target(self, distance):
        return vehicle(distance,0,0)|{'id':1,'name':'Target'}

    def test_zero_zone_and_continuous_growth_through_anchor(self):
        c=self.config()
        for r in [0,1,2.99,3]:
            self.assertEqual(range_error_parameters(c,r),(0,0,0))
            model=Perception(c)
            with patch.object(model.rng,'gauss',side_effect=AssertionError('Zero zone must not draw noise')):
                obs,rows=model.observe(vehicle(),[self.target(r)],detections=[self.target(r)])
            self.assertEqual(rows[0]['sampled_range_error_m'],0)
            self.assertEqual(rows[0]['observed_range_m'],r)
            self.assertTrue(rows[0]['observation_valid'])
        for r,mae in [(3.000001,50*.000001/97),(50,50*47/97),(100,50),(200,50*197/97)]:
            mean,sigma,actual_mae=range_error_parameters(c,r)
            self.assertEqual(mean,0)
            self.assertAlmostEqual(actual_mae,mae)
            self.assertAlmostEqual(sigma*math.sqrt(2/math.pi),mae)

    def test_anchor_sample_distribution_has_requested_absolute_mean_and_symmetry(self):
        model=Perception(self.config(seed=817))
        errors=[];invalid=0
        ego,target=vehicle(),self.target(100)
        for _ in range(60000):
            _,rows=model.observe(ego,[target],detections=[target])
            errors.append(rows[0]['sampled_range_error_m'])
            invalid+=not rows[0]['observation_valid']
        self.assertLess(abs(sum(errors)/len(errors)),1.0)
        self.assertAlmostEqual(sum(map(abs,errors))/len(errors),50,delta=1.0)
        self.assertAlmostEqual(sum(e>0 for e in errors)/len(errors),.5,delta=.01)
        self.assertGreater(invalid,0)

    def test_negative_range_is_logged_without_clamping_redrawing_or_controller_input(self):
        model=Perception(self.config())
        with patch.object(model.rng,'gauss',return_value=-150) as draw:
            obs,rows=model.observe(vehicle(),[self.target(100)],detections=[self.target(100)])
        draw.assert_called_once()
        self.assertEqual(obs,[])
        self.assertTrue(rows[0]['visible'])
        self.assertFalse(rows[0]['observation_valid'])
        self.assertEqual(rows[0]['sampled_range_error_m'],-150)
        self.assertEqual(rows[0]['raw_observed_range_m'],-50)
        self.assertIsNone(rows[0]['observed_range_m'])
        self.assertIsNone(rows[0]['actual_range_error_m'])

    def test_seed_reproduces_but_repeated_measurements_vary(self):
        a,b=Perception(self.config(seed=4)),Perception(self.config(seed=4))
        ego,target=vehicle(),self.target(50)
        values=[a.observe(ego,[target],detections=[target]) for _ in range(20)]
        self.assertEqual(values,[b.observe(ego,[target],detections=[target]) for _ in range(20)])
        self.assertGreater(len({x[1][0]['sampled_range_error_m'] for x in values}),1)

    def test_missed_detection_does_not_draw_noise(self):
        model=Perception(self.config())
        with patch.object(model.rng,'gauss',side_effect=AssertionError('Undetected target cannot draw noise')):
            obs,rows=model.observe(vehicle(),[self.target(20)],detections=[])
        self.assertEqual(obs,[])
        self.assertIsNone(rows[0]['sampled_range_error_m'])
        self.assertFalse(rows[0]['observation_valid'])

    def test_invalid_anchor_and_nonfinite_parameters_are_rejected(self):
        for raw in [{'noise_zero_range_m':100},{'noise_reference_mae_pct':float('nan')},
                    {'noise_reference_mae_pct':-1},{'noise_reference_range_m':2}]:
            with self.subTest(raw=raw),self.assertRaises(ValueError):self.config(**raw)


if __name__=='__main__':unittest.main(verbosity=2)
