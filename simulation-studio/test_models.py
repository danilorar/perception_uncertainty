import unittest
from settings import DEFAULT, validate
from models import Observation, longitudinal_risk

def vehicle(x=0,y=0,speed=10):
    return {'id':0,'name':'Ego','x':x,'y':y,'z':0,'h':0,'p':0,'r':0,'speed':speed,'vx':speed,'vy':0,'length':4,'width':2,'centerOffsetX':0,'centerOffsetY':0}

class ModelTests(unittest.TestCase):
    def test_truth_gap_matches_analytic_value(self):
        gap,closing,in_path=longitudinal_risk(vehicle(),Observation(1,20,0,0,0,0,4,2))
        self.assertAlmostEqual(gap,16)
        self.assertAlmostEqual(closing,10)
        self.assertTrue(in_path)

    def test_lateral_miss_is_not_in_path(self):
        self.assertFalse(longitudinal_risk(vehicle(),Observation(1,20,4,0,0,0,4,2))[2])

    def test_configuration_rejects_target_changes(self):
        with self.assertRaises(ValueError):validate({'target_speed_kph':15})
        with self.assertRaises(ValueError):validate({'noise_reference_mae_pct':float('nan')})

    def test_only_the_hidrive_controller_remains(self):
        self.assertEqual(validate({})['controller'],'hidrive')
        for controller in ['aeb','aeb_follow','original_trajectory','native_acc','constant_speed']:
            with self.subTest(controller=controller),self.assertRaisesRegex(ValueError,'HiDrive'):
                validate({'controller':controller})

    def test_settings_of_removed_controllers_are_dropped(self):
        config=validate({'sensor_range_m':1,'ttc_threshold_s':2,'observation_period_s':.05,
                         'aeb_trigger_mode':'instant','acc_time_gap_s':1.5})
        self.assertEqual(set(config),set(DEFAULT))

if __name__=='__main__':unittest.main(verbosity=2)
