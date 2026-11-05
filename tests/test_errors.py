"""Error models and the noise file."""
import math
from pathlib import Path
import statistics
import tempfile
import unittest

from pipeline import errors
from pipeline.config import validate
from pipeline.errors.factors import DEFAULTS
from pipeline.errors.frames import hold_per_step, iid_frames, markov_frames, markov_parameters, missed_steps, wrong_class_steps
from pipeline.errors.ou import CorrelatedRangeNoise, unit_samples

TARGET = {'id': 1, 'name': 'object_2'}
STEPS = 300


def longest_run(states):
    best = current = 0
    for state in states:
        current = current + 1 if state else 0
        best = max(best, current)
    return best


def factors(values, seed=0):
    return validate({'uncertainty': 'range_factors', 'range_factors': values, 'seed': seed})


def noise_file(config):
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / 'noise.tsv'
        errors.write_noise_file(path, config, 'object_1', [TARGET], Path(folder) / 'applied.tsv', STEPS)
        lines = [line.split('\t') for line in path.read_text(encoding='utf-8').splitlines()]
    return {line[0]: line[1:] for line in lines}


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
            time = step * 0.01
            if step >= 10:
                together.sample(2, time)
            self.assertEqual(alone.sample(1, time), together.sample(1, time))


class ConfigTests(unittest.TestCase):
    def test_defaults_are_neutral(self):
        config = factors({})
        self.assertEqual(config['range_factors'], DEFAULTS)
        self.assertFalse(any(missed_steps(config['range_factors'], 0, 1, 0.01, 1000)))
        self.assertFalse(any(wrong_class_steps(config['range_factors'], 0, 1, 0.01, 1000)))

    def test_rejects_bad_factors(self):
        for bad in (None, {'nope': 1}, {'scale': 0}, {'miss_model': 'gauss'}, {'miss_mean_s': 0.01}, {'sensor_period_s': 0.015},
                    {'scale': True}, {'lost_target': 'hold'}, {'lost_target_motion': 'frozen'},
                    {'lost_target': 'predict', 'lost_target_distance': 'frozen_object'}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                factors(bad)

    def test_factors_only_with_their_model(self):
        for model in ('recorded_truth', 'distance_ou_range'):
            with self.assertRaises(ValueError):
                validate({'uncertainty': model, 'range_factors': {}})

    def test_rejects_bad_ou_settings(self):
        for raw in ({'noise_zero_range_m': 100}, {'noise_reference_mae_pct': float('nan')}, {'noise_reference_mae_pct': -1},
                    {'noise_reference_range_m': 2}, {'noise_correlation_time_s': 0}, {'noise_correlation_time_s': True}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate({'uncertainty': 'distance_ou_range', **raw})


class NoiseFileTests(unittest.TestCase):
    def test_ou_file_holds_the_samples_of_the_stream(self):
        config = validate({'uncertainty': 'distance_ou_range', 'seed': 5})
        keys = noise_file(config)
        self.assertEqual(keys['observer'], ['object_1'])
        self.assertEqual(float(keys['reference_mae_pct'][0]), config['noise_reference_mae_pct'])
        self.assertEqual(keys['z'][0], TARGET['name'])
        self.assertEqual([float(v) for v in keys['z'][1:]], unit_samples(config, TARGET['id'], STEPS))
        self.assertNotIn('missed', keys)

    def test_fixed_errors_without_ou(self):
        keys = noise_file(factors({'scale': 0.9, 'bias_m': 0.5, 'wrong_scale': 1.77, 'wrong_bias_m': -1.45}))
        self.assertEqual(float(keys['reference_mae_pct'][0]), 0.0)
        self.assertEqual((float(keys['scale'][0]), float(keys['bias_m'][0])), (0.9, 0.5))
        self.assertEqual((float(keys['wrong_scale'][0]), float(keys['wrong_bias_m'][0])), (1.77, -1.45))
        self.assertEqual({float(v) for v in keys['z'][1:]}, {0.0})

    def test_flags_cover_every_step(self):
        config = factors({'miss_rate': 0.1, 'miss_mean_s': 0.5, 'wrong_rate': 0.2}, seed=3)
        keys = noise_file(config)
        for key, flags in (('missed', missed_steps), ('wrong', wrong_class_steps)):
            self.assertEqual([v == '1' for v in keys[key][1:]], flags(config['range_factors'], 3, TARGET['id'], 0.01, STEPS))
        self.assertEqual(len(keys['missed']), len(keys['z']))

    def test_lost_target_keys_keep_the_same_misses(self):
        vanish = noise_file(factors({'miss_rate': 0.1}, seed=3))
        self.assertNotIn('predict_when_lost', vanish)
        predict = {'miss_rate': 0.1, 'lost_target': 'predict'}
        frozen = predict | {'lost_target_motion': 'frozen'}
        for values, key in ((predict, 'predict_when_lost'), (frozen, 'freeze_motion_when_lost'),
                            (frozen | {'lost_target_distance': 'frozen_object'}, 'live_observer_when_lost')):
            keys = noise_file(factors(values, seed=3))
            self.assertEqual(keys[key], ['1'])
            self.assertEqual(keys['missed'], vanish['missed'])

    def test_ou_inside_factors_matches_the_ou_model(self):
        inside = noise_file(factors({'ou_mae_pct': 1.4}, seed=3))
        alone = noise_file(validate({'uncertainty': 'distance_ou_range', 'noise_reference_mae_pct': 1.4, 'seed': 3}))
        self.assertEqual(inside['z'], alone['z'])
        self.assertEqual(inside['reference_mae_pct'], alone['reference_mae_pct'])

    def test_identification_stream_is_independent_of_detection(self):
        settings = factors({'miss_rate': 0.2, 'wrong_rate': 0.2})['range_factors']
        self.assertNotEqual(missed_steps(settings, 0, 1, 0.01, 1000), wrong_class_steps(settings, 0, 1, 0.01, 1000))


if __name__ == '__main__':
    unittest.main()
