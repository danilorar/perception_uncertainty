import copy
import unittest
from settings import DEFAULT, validate
from models import Observation, predicted_contact_time, Perception, AEB, advance_ego

def vehicle(x=0,y=0,speed=10):
    return {'id':0,'name':'Ego','x':x,'y':y,'z':0,'h':0,'p':0,'r':0,'speed':speed,'vx':speed,'vy':0,'length':4,'width':2,'centerOffsetX':0,'centerOffsetY':0}

class ModelTests(unittest.TestCase):
    def test_contact_time_matches_analytic_gap(self):
        target=Observation(1,20,0,0,0,0,4,2)
        self.assertAlmostEqual(predicted_contact_time(vehicle(),target,5),1.6)

    def test_lateral_miss_has_no_contact(self):
        target=Observation(1,20,4,0,0,0,4,2)
        self.assertIsNone(predicted_contact_time(vehicle(),target,5))

    def test_crossing_contact(self):
        target=Observation(1,20,-10,0,5,1.5707963267948966,1,1)
        self.assertIsNotNone(predicted_contact_time(vehicle(),target,5))

    def test_stop_distance_without_reversal(self):
        result=advance_ego(vehicle(),-5,3)
        self.assertAlmostEqual(result['x'],10)
        self.assertEqual(result['speed'],0)

    def test_seed_and_truth_immutability(self):
        target=vehicle(20,0,0)|{'id':1,'name':'Target'}
        before=copy.deepcopy(target)
        c=DEFAULT|{'uncertainty':'gaussian_range','seed':123}
        a,b=Perception(c),Perception(c)
        self.assertEqual([a.observe(vehicle(),[target],detections=[target]) for _ in range(10)],[b.observe(vehicle(),[target],detections=[target]) for _ in range(10)])
        self.assertEqual(target,before)

    def test_zero_noise_matches_ideal(self):
        target=vehicle(20,0,0)|{'id':1,'name':'Target'}
        ideal=Perception(DEFAULT).observe(vehicle(),[target],detections=[target])
        zero=Perception(DEFAULT|{'uncertainty':'gaussian_range','range_sigma_m':0}).observe(vehicle(),[target],detections=[target])
        self.assertEqual(ideal,zero)

    def test_native_detection_is_required_in_every_mode(self):
        target=vehicle(200,0,0)|{'id':1,'name':'Target'}
        for mode in ['ideal','recorded_truth','gaussian_range','distance_gaussian_range']:
            p=Perception(DEFAULT|{'uncertainty':mode})
            self.assertEqual(p.observe(vehicle(),[target],detections=[])[0],[])
            with self.assertRaises(TypeError): p.observe(vehicle(),[target])

    def test_measurement_uses_detected_state_not_audit_truth_or_python_gate(self):
        truth=vehicle(20,0,0)|{'id':1,'name':'Target'}
        detected=truth|{'x':200,'vx':4,'vy':2,'width':3}
        obs,rows=Perception(DEFAULT).observe(vehicle(),[truth],detections=[detected])
        self.assertEqual((obs[0].x,obs[0].vx,obs[0].vy,obs[0].width),(200,4,2,3))
        self.assertEqual(rows[0]['true_range_m'],20)
        self.assertEqual(rows[0]['ideal_range_m'],200)

    def test_aeb_delay_and_latching(self):
        ctrl=AEB(DEFAULT|{'controller':'aeb','aeb_trigger_mode':'instant','reaction_delay_s':.2,'ttc_threshold_s':2})
        target=Observation(1,20,0,0,0,0,4,2)
        self.assertEqual(ctrl.command(0,vehicle(),[target])[0],0)
        self.assertEqual(ctrl.command(.1,vehicle(),[])[0],0)
        self.assertEqual(ctrl.command(.2,vehicle(),[])[0],-8)

    def test_configuration_rejects_target_changes(self):
        with self.assertRaises(ValueError):validate({'target_speed_kph':15})
        with self.assertRaises(ValueError):validate({'ego_speed_kph':float('nan')})

if __name__=='__main__':unittest.main(verbosity=2)
