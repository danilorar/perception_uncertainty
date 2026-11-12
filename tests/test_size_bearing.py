"""Distance/azimuth physical-size model: geometry, statistics and actual bridge."""
import copy
import csv
import math
from pathlib import Path
import statistics
import tempfile
from types import SimpleNamespace
import unittest

from pipeline import errors, scenarios
from pipeline.common import ESMINI
from pipeline.config import validate
from pipeline.errors.size_bearing import Error, parameters, sensor_geometry
from pipeline.run import run


def plugin(*, seed=7, label='size_bearing', steps=300, dt=0.01, targets=None, **overrides):
    entry = validate({'seed': seed, 'errors': [{'type': 'size_bearing', **overrides}]})['errors'][0]
    settings = {k: v for k, v in entry.items() if k != 'type'}
    context = SimpleNamespace(seed=seed, label=label, steps=steps, dt_s=dt,
                              targets=targets or [{'id': 1, 'name': 'object_2'}])
    return Error(settings, context)


def target(distance=30.0, bearing_deg=0.0):
    beta = math.radians(bearing_deg)
    truth = dict(x=distance * math.cos(beta), y=distance * math.sin(beta), heading=0.0,
                 length=4.0, width=2.0, height=1.5)
    return dict(name='object_2', id=1, true=truth, true_range=distance, range=distance,
                class_correct=True, seen=True, log={}, **truth)


EGO = dict(x=0.0, y=0.0, heading=0.0)


def corr(a, b):
    ma, mb = statistics.fmean(a), statistics.fmean(b)
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(
        sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))


class SizeBearingTests(unittest.TestCase):
    def test_neutral_preserves_inputs_and_truth_exactly(self):
        model = plugin()
        item = target()
        item['length'] = 5.0  # preserve an earlier plugin's input too
        before = copy.deepcopy(item)
        model.perceive(0, 0.0, EGO, [item])
        self.assertEqual({k: v for k, v in item.items() if k != 'log'},
                         {k: v for k, v in before.items() if k != 'log'})
        self.assertEqual(item['log']['applied'], 1)

    def test_distance_bearing_interaction_has_known_log_ratio(self):
        model = plugin(reference_distance_m=20.0, reference_bearing_deg=30.0,
                       width_mu=[0.0, 0.02, 0.03, 0.05])
        # u=1 and v=1, so log width ratio is 0.02+0.03+0.05=0.1.
        item = target(distance=20.0 * math.expm1(1.0), bearing_deg=30.0)
        model.perceive(0, 0.0, EGO, [item])
        self.assertAlmostEqual(item['width'], 2.0 * math.exp(0.1), places=12)
        self.assertEqual(item['length'], 4.0)
        self.assertEqual(item['range'], item['true_range'])

    def test_mirrored_bearings_have_the_same_surface(self):
        model = plugin(width_mu=[0.0, 0.01, -0.02, 0.03], width_sigma_ref=0.02,
                       width_sigma_log=[0.0, 0.2, 0.3, 0.1])
        self.assertEqual(parameters(model.settings, 50, math.radians(35), 'width'),
                         parameters(model.settings, 50, math.radians(-35), 'width'))

    def test_sensor_mount_and_rotation_define_azimuth_not_target_heading(self):
        model = plugin(sensor_forward_m=2.0, sensor_left_m=1.0, sensor_yaw_deg=10.0)
        ego = dict(x=10.0, y=20.0, heading=math.pi / 2)
        # Sensor at (9,22), pointed at 100 degrees; put target at 130 degrees.
        truth = dict(x=9.0 + 40 * math.cos(math.radians(130)),
                     y=22.0 + 40 * math.sin(math.radians(130)), heading=-1.0)
        distance, bearing = sensor_geometry(model.settings, ego, truth)
        self.assertAlmostEqual(distance, 40.0)
        self.assertAlmostEqual(math.degrees(bearing), 30.0)
        truth['heading'] = 2.0
        self.assertEqual(sensor_geometry(model.settings, ego, truth), (distance, bearing))

    def test_outside_domain_and_wrong_class_pass_through_without_hiding(self):
        model = plugin(width_mu=[0.1, 0, 0, 0])
        for item in (target(200), target(30, 90), target(30, 180), target(0)):
            with self.subTest(item=item):
                model.perceive(0, 0, EGO, [item])
                self.assertEqual(item['width'], 2.0)
                self.assertTrue(item['seen'])
                self.assertEqual(item['log']['skip_reason'], 'outside_domain')
        item = target()
        item['class_correct'] = False
        model.perceive(0, 0, EGO, [item])
        self.assertEqual(item['log']['skip_reason'], 'wrong_class')
        model = plugin(only_correct_class=False, width_mu=[0.1, 0, 0, 0])
        model.perceive(0, 0, EGO, [item])
        self.assertGreater(item['width'], 2.0)

    def test_streams_are_repeatable_and_independent_of_other_targets_and_labels(self):
        one = plugin()
        repeat = plugin()
        extra = plugin(targets=[{'id': 2, 'name': 'other'}, {'id': 1, 'name': 'object_2'}])
        other_label = plugin(label='another')
        self.assertEqual(one.noise, repeat.noise)
        self.assertEqual(one.noise['object_2'], extra.noise['object_2'])
        self.assertNotEqual(one.noise, other_label.noise)

    def test_ou_stationary_variance_time_correlation_and_dimension_correlation(self):
        model = plugin(steps=30000, dt=0.05, correlation_time_s=0.3, length_width_correlation=0.6)
        a, b = zip(*model.noise['object_2'])
        self.assertAlmostEqual(statistics.pvariance(a), 1.0, delta=0.1)
        self.assertAlmostEqual(statistics.fmean(a), 0.0, delta=0.08)
        self.assertAlmostEqual(corr(a[:-1], a[1:]), math.exp(-0.05 / 0.3), delta=0.025)
        self.assertAlmostEqual(corr(a, b), 0.6, delta=0.06)

    def test_sample_log_ratios_match_configured_mean_and_std(self):
        model = plugin(steps=12000, correlation_time_s=0.0,
                       width_mu=[0.02, 0, 0, 0], width_sigma_ref=0.08)
        residuals = []
        for step in range(12000):
            item = target()
            model.perceive(step, step * 0.01, EGO, [item])
            residuals.append(math.log(item['width'] / 2.0))
        self.assertAlmostEqual(statistics.fmean(residuals), 0.02, delta=0.003)
        self.assertAlmostEqual(statistics.pstdev(residuals), 0.08, delta=0.003)

    def test_clipping_is_positive_bounded_and_logged(self):
        model = plugin(length_sigma_ref=0.2, width_sigma_ref=0.2, max_abs_log_error=0.3)
        model.noise['object_2'][0] = (30.0, -30.0)
        item = target()
        model.perceive(0, 0, EGO, [item])
        self.assertAlmostEqual(item['length'], 4 * math.exp(0.3))
        self.assertAlmostEqual(item['width'], 2 * math.exp(-0.3))
        self.assertEqual((item['log']['length_clipped'], item['log']['width_clipped']), (1, 1))

    def test_invalid_surfaces_are_rejected_before_running(self):
        cases = [dict(width_mu=[0, 0]), dict(width_mu=[False, 0, 0, 0]),
                 dict(width_sigma_log=[0, 0, 0, float('nan')]),
                 dict(reference_distance_m=0), dict(max_abs_bearing_deg=90),
                 dict(min_distance_m=100, max_distance_m=50),
                 dict(width_mu=[5, 0, 0, 0]),
                 dict(width_sigma_ref=0.8, width_sigma_log=[0, 1, 0, 0]),
                 dict(parameter_source='')]
        for settings in cases:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                plugin(**settings)

    def test_size_flag_and_log_cross_the_plugin_chain_protocol(self):
        config = validate({'errors': [{'type': 'size_bearing', 'width_mu': [math.log(1.2), 0, 0, 0]}]})
        ego = [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0]
        other = [30.0, 0.0, 0.0, 0.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 2.0, 1.5, 30.0]
        message = b'\t'.join([b'@@PB', b'S', b'0x0.0p+0', b'2', b'object_1',
                               *(v.hex().encode() for v in ego), b'object_2',
                               *(v.hex().encode() for v in other)]) + b'\n'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'applied.tsv'
            chain = errors.Chain(config, scenarios.get('gidas_1549178'), path)
            try:
                reply = chain.answer(message).strip().split(b'\t')
            finally:
                chain.close()
            with path.open(encoding='utf-8') as stream:
                row = next(csv.DictReader(stream, delimiter='\t'))
        self.assertEqual(int(reply[2]), errors.SEEN | errors.SIZE)
        self.assertEqual(float.fromhex(reply[3].decode()), 30.0)
        self.assertAlmostEqual(float.fromhex(reply[-1].decode()), 2.4)
        self.assertAlmostEqual(float(row['size_bearing.estimated_width_m']), 2.4)


@unittest.skipUnless(ESMINI.is_file(), 'esmini is not built')
class SizeBearingEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.folder = Path(cls.temp.name)
        cls.results = {}
        for name, entries in (
                ('baseline', []),
                ('neutral', [{'type': 'size_bearing'}]),
                ('larger', [{'type': 'size_bearing', 'length_mu': [math.log(2.0), 0, 0, 0]}])):
            config = validate({'controller': 'example_aeb', 'seed': 7, 'errors': entries})
            cls.results[name] = run(config, cls.folder / name, plots=False)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def table(self, name, file, delimiter=','):
        with (self.folder / name / file).open(encoding='utf-8') as stream:
            return list(csv.DictReader(stream, delimiter=delimiter))

    def test_neutral_dimensions_preserve_baseline_vehicle_outputs(self):
        self.assertEqual((self.folder / 'baseline/ego.csv').read_bytes(),
                         (self.folder / 'neutral/ego.csv').read_bytes())

    def test_perceived_length_reaches_controller_and_true_dimensions_are_restored(self):
        self.assertLess(self.results['larger']['first_brake_time_s'],
                        self.results['baseline']['first_brake_time_s'])
        baseline = self.table('baseline', 'truth.csv')
        dimensions = {r['object_name']: (r['length_m'], r['width_m']) for r in baseline}
        for row in self.table('larger', 'truth.csv'):
            self.assertEqual((row['length_m'], row['width_m']), dimensions[row['object_name']])
        applied = self.table('larger', 'perception_applied.tsv', '\t')
        self.assertTrue(applied)
        for row in applied:
            self.assertAlmostEqual(float(row['raw_range_m']), float(row['true_range_m']))
            self.assertAlmostEqual(float(row['size_bearing.estimated_length_m']),
                                   2.0 * float(row['size_bearing.true_length_m']))


if __name__ == '__main__':
    unittest.main()
