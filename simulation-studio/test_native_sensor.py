"""Exercise the installed esmini DLL, including detection boundaries and units."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

from engine import Engine
from models import Perception
from scenario import Reference, build_scene
from settings import DEFAULT, ROOT


class NativeSensorTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'work')
        self.addCleanup(self.temp.cleanup)
        out=Path(self.temp.name)
        ref=Reference('gidas_1549178')
        self.ego,self.target=copy.deepcopy(ref.states(0))
        for state in [self.ego,self.target]:
            state.update(x=0.,y=0.,z=0.,h=0.,p=0.,r=0.,speed=0.,vx=0.,vy=0.,
                         centerOffsetX=1.,centerOffsetY=0.,centerOffsetZ=.5)
        self.target.update(x=10.,centerOffsetX=2.)
        build_scene(out/'scene.xosc',ref.meta|{'entity_definitions':{}},[self.ego,self.target],10)
        self.engine=Engine()
        self.addCleanup(self.engine.close)
        self.engine.init(out/'scene.xosc',out/'esmini.log')
        self.config=DEFAULT|{'sensor_range_m':10.,'sensor_fov_deg':180.,'uncertainty':'ideal'}
        self.sensor=self.engine.add_object_sensor(self.ego,self.config)

    def sample(self, dt=0.01):
        for state in [self.ego,self.target]: self.engine.report(state)
        self.engine.step(dt)
        return self.engine.detect(self.sensor)

    def test_first_frame_native_gate_uses_reference_point_then_centre_range(self):
        hits=self.sample(0)
        self.assertEqual([s['id'] for s in hits],[self.target['id']])
        obs,rows=Perception(self.config).observe(self.ego,[self.target],detections=hits)
        # Mount x=1, target reference x=10 => gate distance 9; centre x=12 => range 11.
        self.assertEqual(rows[0]['sensor_reference_range_m'],9)
        self.assertEqual(rows[0]['ideal_range_m'],11)
        self.assertEqual(obs[0].x,12)
        self.assertEqual(self.engine.sensors[self.sensor]['fetch_count'],1)

    def test_range_fov_rotation_and_disappearance(self):
        self.assertTrue(self.sample(0))
        self.target['x']=12
        self.assertEqual(self.sample(),[])
        self.target.update(x=1,y=5)  # exactly 90 degrees: native strict FOV boundary
        self.assertEqual(self.sample(),[])
        self.target.update(x=5,y=1)
        self.assertTrue(self.sample())
        self.ego['h']=math.pi
        self.assertEqual(self.sample(),[])
        self.target.update(x=-5,y=0)
        self.assertTrue(self.sample())

    def test_engine_velocity_and_uncertainty_after_detection(self):
        self.target.update(vx=2.,vy=3.,speed=math.sqrt(13))
        hits=self.sample(0)
        self.assertAlmostEqual(hits[0]['vx'],2.)
        self.assertAlmostEqual(hits[0]['vy'],3.)
        c=self.config|{'uncertainty':'gaussian_range','range_bias_m':4.,'range_sigma_m':0.}
        obs,rows=Perception(c).observe(self.ego,[self.target],detections=hits)
        self.assertEqual(rows[0]['observed_range_m'],15)
        self.assertEqual(obs[0].vx,2.)
        self.target['x']=30
        hits=self.sample()
        obs,rows=Perception(c).observe(self.ego,[self.target],detections=hits)
        self.assertEqual(obs,[])
        self.assertIsNone(rows[0]['sampled_range_error_m'])


if __name__=='__main__': unittest.main(verbosity=2)
