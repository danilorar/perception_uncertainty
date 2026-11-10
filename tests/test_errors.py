"""Error plugins: the registry, settings, the plugin chain and the frame sequences."""
import csv
import math
from pathlib import Path
import statistics
import tempfile
import unittest

from pipeline import errors, scenarios
from pipeline.config import validate
from pipeline.errors.frames import hold_per_step, iid_frames, markov_frames, markov_parameters, missed_steps, wrong_class_steps
from pipeline.errors.ou import SIGMA_PER_MAE, CorrelatedRangeNoise, unit_samples

TARGET = {'id': 1, 'name': 'object_2'}
DT = 0.01
# ego at the origin at 20 m/s; both boxes 4 m x 2 m
EGO = [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0]
REPLY = ('range', 'x', 'y', 'heading', 'speed', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'length', 'width')


def longest_run(states):
    best = current = 0
    for state in states:
        current = current + 1 if state else 0
        best = max(best, current)
    return best


def config(entries, seed=0):
    return validate({'scenario': 'gidas_1549178', 'errors': entries, 'seed': seed})


def message(step, gap=30.0, speed=5.0):
    """A bridge request as esmini writes it: time, object count, the observer, then every target."""
    target = [gap, 0.0, 0.0, 0.0, speed, speed, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 2.0, 1.5, gap]
    cells = [b'@@PB', b'S', (step * DT).hex().encode(), b'2', b'object_1', *(v.hex().encode() for v in EGO),
             b'object_2', *(v.hex().encode() for v in target)]
    return b'\t'.join(cells) + b'\n'


def reply(line):
    cells = line.rstrip(b'\n').split(b'\t')
    assert cells[:2] == [b'@@PB', b'R'] and len(cells) == 3 + len(REPLY), cells
    return int(cells[2]), dict(zip(REPLY, (float.fromhex(cell.decode()) for cell in cells[3:])))


class ChainRun:
    """A plugin chain fed with made-up esmini requests."""

    def __init__(self, entries, seed=0):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / 'applied.tsv'
        self.chain = errors.Chain(config(entries, seed), scenarios.get('gidas_1549178'), self.path)

    def step(self, step, **target):
        return reply(self.chain.answer(message(step, **target)))

    def log(self):
        self.chain.close()
        with self.path.open(encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream, delimiter='\t'))
        self.folder.cleanup()
        return rows


class ZhuoMaPortTests(unittest.TestCase):
    """Counts from sensitivity_runs.csv of reports/aeb_markov_20261006 (commit 484675b): 200 frames, p=0.1."""

    def test_iid_matches_recorded_frames(self):
        for seed, dropped, longest in ((0, 25, 3), (1, 21, 2)):
            states = iid_frames(0.1, seed, TARGET['id'], 200)
            self.assertEqual((sum(states), longest_run(states)), (dropped, longest))

    def test_markov_matches_recorded_frames(self):
        cases = ((0.2, 0, 33, 11), (0.2, 1, 24, 13), (0.5, 0, 62, 53), (0.5, 1, 6, 6), (1.0, 0, 9, 9), (1.0, 1, 29, 29))
        for mean_s, seed, dropped, longest in cases:
            states = markov_frames(0.1, mean_s, 0.05, seed, TARGET['id'], 200)
            self.assertEqual((sum(states), longest_run(states)), (dropped, longest))

    def test_chain_parameters(self):
        enter, recover = markov_parameters(0.1, 0.5, 0.05)
        self.assertAlmostEqual(recover, 0.1)
        self.assertAlmostEqual(enter, 0.1 * 0.1 / 0.9)
        self.assertEqual(markov_parameters(0, 0.5, 0.05), (0.0, 1.0))
        with self.assertRaises(ValueError):
            markov_parameters(0.1, 0.01, 0.05)

    def test_realized_rate_and_duration_follow_settings(self):
        states = markov_frames(0.1, 0.5, 0.05, 3, TARGET['id'], 400000)
        runs = sum(1 for before, now in zip(states, states[1:]) if now and not before)
        self.assertAlmostEqual(sum(states) / len(states), 0.1, delta=0.01)
        self.assertAlmostEqual(sum(states) * 0.05 / runs, 0.5, delta=0.03)

    def test_frame_state_holds_between_frames(self):
        self.assertEqual(hold_per_step([True, False], 0.05, 0.01, 7), [True] * 5 + [False] * 2)

    def test_identification_stream_is_independent_of_detection(self):
        settings = {'miss_rate': 0.2, 'miss_model': 'markov', 'miss_mean_s': 0.5, 'sensor_period_s': 0.05,
                    'wrong_rate': 0.2, 'wrong_mean_s': 0.5}
        self.assertNotEqual(missed_steps(settings, 0, 1, DT, 1000), wrong_class_steps(settings, 0, 1, DT, 1000))


class OuTests(unittest.TestCase):
    def test_stationary_distribution_and_correlation(self):
        for interval in (0.1, 1.0):
            first, second = [], []
            for seed in range(6000):
                process = CorrelatedRangeNoise(seed, 0.5)
                first.append(process.sample(1, 0))
                second.append(process.sample(1, interval))
            self.assertAlmostEqual(statistics.mean(second), 0, delta=0.04)
            self.assertAlmostEqual(statistics.pvariance(second), 1, delta=0.06)
            self.assertAlmostEqual(statistics.correlation(first, second), math.exp(-interval / 0.5), delta=0.03)

    def test_a_second_object_does_not_change_the_first(self):
        alone, together = CorrelatedRangeNoise(42, 0.5), CorrelatedRangeNoise(42, 0.5)
        for step in range(30):
            time = step * DT
            if step >= 10:
                together.sample(2, time)
            self.assertEqual(alone.sample(1, time), together.sample(1, time))


class SettingsTests(unittest.TestCase):
    def test_registry_finds_every_plugin_and_no_helper(self):
        self.assertEqual(set(errors.all_plugins()), {'detection', 'identification', 'ou', 'size', 'template'})

    def test_defaults_are_filled_in(self):
        entries = config([{'type': 'detection'}, {'type': 'size', 'name': 'big', 'scale': 2}])['errors']
        self.assertEqual(entries[0], {'type': 'detection', **errors.all_plugins()['detection'].SETTINGS})
        self.assertEqual(entries[1], {'type': 'size', 'name': 'big', 'scale': 2.0, 'bias_m': 0.0})
        self.assertIsInstance(entries[1]['scale'], float)

    def test_defaults_are_neutral(self):
        detection, identification = config([{'type': 'detection'}, {'type': 'identification'}])['errors']
        self.assertFalse(any(missed_steps(detection, 0, 1, DT, 1000)))
        self.assertFalse(any(wrong_class_steps(identification, 0, 1, DT, 1000)))

    def test_rejects_bad_entries(self):
        for bad in (None, {}, [{}], [{'type': 'gauss'}], [{'type': 'size', 'nope': 1}], [{'type': 'size', 'scale': 0}],
                    [{'type': 'size', 'scale': True}], [{'type': 'size', 'scale': float('nan')}],
                    [{'type': 'detection', 'miss_model': 'gauss'}], [{'type': 'detection', 'miss_mean_s': 0.01}],
                    [{'type': 'detection', 'sensor_period_s': 0.015}], [{'type': 'detection', 'lost_target': 'hold'}],
                    [{'type': 'detection', 'lost_target_motion': 'frozen'}],
                    [{'type': 'detection', 'lost_target': 'predict', 'lost_target_distance': 'frozen_object'}],
                    [{'type': 'ou', 'zero_range_m': 100}], [{'type': 'ou', 'correlation_time_s': 0}],
                    [{'type': 'size'}, {'type': 'size'}], [{'type': 'size', 'name': 'a.b'}]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                config(bad)

    def test_the_same_type_twice_with_names(self):
        entries = config([{'type': 'template', 'name': 'a', 'sigma_m': 1}, {'type': 'template', 'name': 'b'}])['errors']
        self.assertEqual([entry['name'] for entry in entries], ['a', 'b'])

    def test_old_settings_point_to_errors(self):
        for raw in ({'uncertainty': 'distance_ou_range'}, {'range_factors': {}}, {'noise_reference_mae_pct': 5.0}):
            with self.subTest(raw=raw), self.assertRaisesRegex(ValueError, '"errors"'):
                validate(raw)


class ChainTests(unittest.TestCase):
    def test_neutral_plugin_changes_nothing(self):
        run = ChainRun([{'type': 'size'}])
        flags, seen = run.step(0)
        run.log()
        self.assertEqual(flags, errors.SEEN)
        self.assertEqual((seen['range'], seen['x'], seen['speed'], seen['length']), (30.0, 30.0, 5.0, 4.0))

    def test_ou_adds_the_scaled_sample(self):
        run = ChainRun([{'type': 'ou', 'mae_pct': 5.0}], seed=4)
        z = unit_samples(4, 0.5, TARGET['id'], DT, 20)
        sigma = 100.0 * 5.0 / 100.0 * (30.0 - 3.0) / (100.0 - 3.0) * SIGMA_PER_MAE
        for step in (0, 7, 19):
            self.assertEqual(run.step(step)[1]['range'], 30.0 + sigma * z[step])
        rows = run.log()
        self.assertEqual(float(rows[1]['ou.z']), z[7])
        self.assertEqual(float(rows[1]['raw_range_m']), 30.0 + sigma * z[7])

    def test_identification_replaces_size_on_wrong_frames(self):
        entries = [{'type': 'identification', 'wrong_rate': 0.5, 'wrong_scale': 2.0, 'wrong_mean_s': 0.2},
                   {'type': 'size', 'bias_m': 1.0}]
        run, wrong = ChainRun(entries, seed=2), wrong_class_steps(config(entries, 2)['errors'][0], 2, 1, DT, 300)
        for step in range(300):
            self.assertEqual(run.step(step)[1]['range'], 60.0 if wrong[step] else 31.0)
        rows = run.log()
        self.assertEqual([row['class_correct'] == '0' for row in rows], wrong)
        self.assertTrue(any(wrong) and not all(wrong))

    def test_detection_hides_the_target_on_missed_steps(self):
        entry = {'type': 'detection', 'miss_rate': 0.3, 'miss_mean_s': 0.2}
        run, missed = ChainRun([entry], seed=1), missed_steps(config([entry], 1)['errors'][0], 1, 1, DT, 300)
        flags = [run.step(step)[0] for step in range(300)]
        self.assertEqual([not flag & errors.SEEN for flag in flags], missed)
        self.assertTrue(any(missed))
        self.assertEqual([row['valid'] == '0' for row in run.log()], missed)

    def test_prediction_moves_the_last_range_on(self):
        entry = {'type': 'detection', 'miss_rate': 0.3, 'miss_mean_s': 0.2, 'lost_target': 'predict',
                 'lost_target_motion': 'frozen'}
        run, missed = ChainRun([entry], seed=1), missed_steps(config([entry], 1)['errors'][0], 1, 1, DT, 1000)
        first = missed.index(True, missed.index(False))
        self.assertLess(first, 150)
        for step in range(first):
            run.step(step, gap=30.0 - 0.15 * step)
        # While lost, the true target stops; the controller still gets its old speed and closing.
        flags, seen = run.step(first, gap=29.0, speed=0.0)
        self.assertEqual(flags & (errors.SEEN | errors.MOVING), errors.SEEN | errors.MOVING)
        self.assertEqual(seen['speed'], 5.0)
        self.assertAlmostEqual(seen['range'], 30.0 - 0.15 * (first - 1) - 15.0 * DT, places=9)
        self.assertEqual(run.log()[-1]['predicted'], '1')

    def test_plugins_stack_in_order_with_their_own_streams(self):
        both = ChainRun([{'type': 'template', 'name': 'a', 'sigma_m': 0.5}, {'type': 'template', 'name': 'b', 'sigma_m': 0.5}])
        only = ChainRun([{'type': 'template', 'name': 'a', 'sigma_m': 0.5}])
        for step in range(5):
            both.step(step)
            only.step(step)
        both_rows, only_rows = both.log(), only.log()
        self.assertEqual([row['a.error_m'] for row in both_rows], [row['a.error_m'] for row in only_rows])
        self.assertNotEqual(both_rows[0]['a.error_m'], both_rows[0]['b.error_m'])
        total = 30.0 + float(both_rows[0]['a.error_m']) + float(both_rows[0]['b.error_m'])
        self.assertAlmostEqual(float(both_rows[0]['raw_range_m']), total, places=12)

    def test_a_moved_position_wins_over_range(self):
        class Shift:
            def perceive(self, step, t, ego, targets):
                targets[0]['y'] = 1.0
        run = ChainRun([])
        run.chain.plugins = [('shift', Shift())]
        flags, seen = run.step(0)
        run.log()
        self.assertEqual(flags, errors.SEEN | errors.POSITION)
        self.assertEqual((seen['x'], seen['y']), (30.0, 1.0))


if __name__ == '__main__':
    unittest.main()
