"""OU distribution, correlation and per-object streams used for HiDrive range error."""
import math
import statistics
import unittest
from unittest.mock import patch
from models import CorrelatedRangeNoise, range_error_parameters
from settings import validate


class CorrelatedNoiseTests(unittest.TestCase):
    def config(self, **extra):
        return validate({'uncertainty':'distance_ou_range', **extra})

    def test_distance_envelope_and_zero_zone(self):
        config = self.config()
        for distance in [0, 1, 2.99, 3]:
            self.assertEqual(range_error_parameters(config, distance), (0,0,0))
        for distance, mae in [(3.000001,50*.000001/97), (50,50*47/97), (100,50), (200,50*197/97)]:
            mean, sigma, actual_mae = range_error_parameters(config, distance)
            self.assertEqual(mean, 0)
            self.assertAlmostEqual(actual_mae, mae)
            self.assertAlmostEqual(sigma*math.sqrt(2/math.pi), mae)

    def test_stationary_distribution_and_correlation_at_physical_lags(self):
        # Independent realizations, not correlated frames treated as independent.
        for interval in [.01, .1, 1.0]:
            initial, following = [], []
            for seed in range(6000):
                process = CorrelatedRangeNoise(seed, .5)
                initial.append(process.sample(1, 0)[0])
                following.append(process.sample(1, interval)[0])
            with self.subTest(interval=interval):
                self.assertAlmostEqual(statistics.mean(initial), 0, delta=.04)
                self.assertAlmostEqual(statistics.mean(following), 0, delta=.04)
                self.assertAlmostEqual(statistics.pvariance(initial), 1, delta=.06)
                self.assertAlmostEqual(statistics.pvariance(following), 1, delta=.06)
                self.assertAlmostEqual(statistics.mean(map(abs,following)), math.sqrt(2/math.pi), delta=.03)
                self.assertAlmostEqual(statistics.correlation(initial,following), math.exp(-interval/.5), delta=.03)
                rms = math.sqrt(statistics.mean((b-a)**2 for a,b in zip(initial,following)))
                self.assertAlmostEqual(rms, math.sqrt(2*(1-math.exp(-interval/.5))), delta=.04)

    def test_seed_repeatability_and_object_stream_independence(self):
        single, multiple, repeated = (CorrelatedRangeNoise(42, .5) for _ in range(3))
        values = []
        for step in range(30):
            time = step*.01
            if step >= 10:
                multiple.sample(2, time)  # a second object appearing later
            value = single.sample(1, time)[0]
            self.assertEqual(value, multiple.sample(1, time)[0])
            self.assertEqual(value, repeated.sample(1, time)[0])
            values.append(value)
        self.assertGreater(len(set(values)), 1)

    def test_unit_state_continues_through_the_zero_error_zone(self):
        config = self.config()
        process = CorrelatedRangeNoise(1, .5)
        first, second = process.sample(1, 0)[0], process.sample(1, .1)[0]
        self.assertNotEqual(first, second)
        self.assertEqual(range_error_parameters(config, 2)[1]*second, 0)
        self.assertGreater(range_error_parameters(config, 100)[1], 0)

    def test_repeated_time_does_not_draw_again_and_reverse_time_is_rejected(self):
        process = CorrelatedRangeNoise(1,.5)
        value = process.sample(1,2)[0]
        with patch.object(process.streams[1],'gauss',side_effect=AssertionError('Must not draw twice')):
            self.assertEqual(process.sample(1,2),(value,1,0))
        for time in [1,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError): process.sample(1,time)

    def test_retired_models_and_invalid_parameters_are_rejected(self):
        for mode in ['gaussian_range','distance_gaussian_range']:
            with self.assertRaisesRegex(ValueError,'已移除'): validate({'uncertainty':mode})
        for raw in [{'noise_zero_range_m':100},{'noise_reference_mae_pct':float('nan')},
                    {'noise_reference_mae_pct':-1},{'noise_reference_range_m':2},
                    *[{'noise_correlation_time_s':x} for x in [0,-1,True,float('nan'),float('inf'),21]]]:
            with self.subTest(raw=raw),self.assertRaises(ValueError): self.config(**raw)


if __name__ == '__main__': unittest.main(verbosity=2)
