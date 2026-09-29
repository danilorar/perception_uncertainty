"""Source fidelity and control/observation contracts for teacher scenes."""
import math
import unittest
import xml.etree.ElementTree as ET
from models import Perception, AEB
from scenario import Reference
from settings import SCENARIOS, validate


class TeacherDataTests(unittest.TestCase):
    def test_active_selection_contains_only_teacher_cases(self):
        self.assertEqual(set(SCENARIOS), {'gidas_1549178','gidas_1554254','gidas_1554431'})

    def test_reference_preserves_every_source_position_and_heading(self):
        for case in SCENARIOS:
            ref=Reference(case)
            for group in ET.parse(ref.meta['source_file']).findall('.//ManeuverGroup'):
                name=group.find('Actors/EntityRef').get('entityRef')
                for vertex in group.findall('.//Trajectory/Shape/Polyline/Vertex'):
                    state=next(s for s in ref.states(float(vertex.get('time'))) if s['name']==name)
                    point=vertex.find('Position/WorldPosition')
                    for key in ['x','y','z','h']:
                        self.assertEqual(state[key],float(point.get(key)))
            self.assertEqual(ref.duration,9.98)
            self.assertEqual(len(ref.times),999)

    def test_initial_state_is_derived_for_each_scene_and_replay_cannot_drift(self):
        for case in SCENARIOS:
            ref=Reference(case)
            a=next(s for s in ref.states(0) if s['id']==ref.ego_id)
            b=next(s for s in ref.states(.2) if s['id']==ref.ego_id)
            config=validate({'scenario':case})
            self.assertAlmostEqual(config['ego_speed_kph'],math.hypot(b['x']-a['x'],b['y']-a['y'])/.2*3.6)
            self.assertAlmostEqual(config['ego_heading_deg'],math.degrees(a['h']))
            replay=validate({'scenario':case,'ego_speed_kph':5,'ego_heading_deg':40})
            self.assertEqual(replay['ego_speed_kph'],config['ego_speed_kph'])
            experiment=validate({'scenario':case,'controller':'aeb','ego_speed_kph':5,'ego_heading_deg':40})
            self.assertEqual(experiment['ego_speed_kph'],5)
            self.assertEqual(experiment['ego_heading_deg'],40)

    def test_recorded_truth_has_no_added_error_but_requires_detection(self):
        ref=Reference('gidas_1549178')
        ego,target=ref.states(0)
        config=validate({'sensor_range_m':1,'sensor_fov_deg':1})
        obs,rows=Perception(config).observe(ego,[target],detections=[target])
        self.assertEqual(len(obs),1)
        self.assertEqual(rows[0]['observed_range_m'],rows[0]['true_range_m'])
        self.assertEqual(Perception(config).observe(ego,[target],detections=[])[0],[])
        control=AEB(config)
        self.assertEqual(control.command(0,ego,obs)[0],0)
        self.assertIsNone(control.brake_time)


if __name__=='__main__': unittest.main(verbosity=2)
