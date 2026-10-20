"""Source fidelity and initial-state contracts for teacher scenes."""
import math
import unittest
import xml.etree.ElementTree as ET
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
            self.assertEqual(ref.duration,10.01)
            self.assertEqual(len(ref.times),999)

    def test_initial_state_is_derived_for_each_scene_and_cannot_be_overridden(self):
        for case in SCENARIOS:
            ref=Reference(case)
            a=next(s for s in ref.states(0) if s['id']==ref.ego_id)
            b=next(s for s in ref.states(.01) if s['id']==ref.ego_id)
            config=validate({'scenario':case})
            self.assertAlmostEqual(config['ego_speed_kph'],math.hypot(b['x']-a['x'],b['y']-a['y'])/.01*3.6)
            self.assertAlmostEqual(config['ego_heading_deg'],math.degrees(a['h']))
            # The HiDrive controller always starts from the scene's own state.
            override=validate({'scenario':case,'ego_speed_kph':5,'ego_heading_deg':40})
            self.assertEqual(override['ego_speed_kph'],config['ego_speed_kph'])
            self.assertEqual(override['ego_heading_deg'],config['ego_heading_deg'])


if __name__=='__main__': unittest.main(verbosity=2)
