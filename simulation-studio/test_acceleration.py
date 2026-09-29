"""Check physical derivatives independently of the production computation."""
import unittest
import numpy as np
from acceleration import estimate_acceleration, TrajectoryAcceleration
from settings import REFERENCES


class AccelerationTests(unittest.TestCase):
    def test_constant_deceleration_in_rotated_world_frame_including_endpoints(self):
        t=np.arange(0,4.01,.01)
        displacement=25*t-1.5*t*t  # v=25-3t, a=-3 m/s².
        direction=np.array([-.6,-.8])
        xy=np.array([123456.,-765432.])+displacement[:,None]*direction
        actual=estimate_acceleration(t,xy)
        np.testing.assert_allclose(actual,-3,atol=1e-6)

    def test_quantized_constant_speed_is_close_to_zero_without_altering_positions(self):
        t=np.arange(0,5,.01)
        xy=np.round(np.column_stack([12.17*t,1.83*t]),2)
        original=xy.copy()
        actual=estimate_acceleration(t,xy)
        self.assertLess(np.max(np.abs(actual)),.3)
        np.testing.assert_array_equal(xy,original)

    def test_stationary_positions_have_zero_acceleration(self):
        t=np.arange(0,1,.01)
        np.testing.assert_array_equal(estimate_acceleration(t,np.tile([30.,15.],(len(t),1))),0)

    def test_same_source_estimate_is_independent_of_output_timestep(self):
        series=TrajectoryAcceleration(REFERENCES/'gidas_1549178/truth.csv',0)
        full={round(float(t),8):float(a) for t,a in zip(series.times,series.values)}
        for dt in [.01,.02,.05]:
            for t in np.arange(0,9.98,dt):
                time=round(float(t),8)
                self.assertAlmostEqual(series.at(time),full[time])
        self.assertFalse(series.metadata['measured'])
        self.assertFalse(series.metadata['controller_input'])
        with self.assertRaises(ValueError):series.at(10)


if __name__=='__main__':unittest.main(verbosity=2)
