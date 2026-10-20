"""HiDrive configuration, scene and output conversion checks.

The full run starts the isolated esmini 3.8.1 build when it is installed; the
private controller sources are never opened by these tests.
"""
import csv
import json
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from hidrive_runner import EXECUTABLE, build_hidrive_scene, read_esmini_csv, read_log_values, write_noise_file
from models import CorrelatedRangeNoise
from scenario import Reference
from settings import DEFAULT, validate


class HiDriveTests(unittest.TestCase):
    def test_uses_source_initial_state_and_accepts_range_error(self):
        config = validate(DEFAULT | {'controller': 'hidrive', 'uncertainty': 'recorded_truth'})
        self.assertEqual(config['duration_s'], 10.01)
        self.assertAlmostEqual(config['ego_speed_kph'], Reference(config['scenario']).meta['initial_ego']['ego_speed_kph'])
        self.assertEqual(validate(DEFAULT | {'controller': 'hidrive', 'uncertainty': 'distance_ou_range'})['uncertainty'],
                         'distance_ou_range')

    def test_noise_file_uses_studio_ou_streams(self):
        config = validate(DEFAULT | {'controller': 'hidrive', 'uncertainty': 'distance_ou_range', 'seed': 5})
        targets = [{'id': 1, 'name': 'object_2'}]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'noise.tsv'
            write_noise_file(path, config, 'object_1', targets, Path(folder) / 'applied.tsv', 1.0)
            lines = [line.split(chr(9)) for line in path.read_text(encoding='utf-8').splitlines()]
        keys = {line[0]: line[1:] for line in lines}
        self.assertEqual(keys['observer'], ['object_1'])
        self.assertEqual(float(keys['reference_mae_pct'][0]), config['noise_reference_mae_pct'])
        samples = [float(v) for v in keys['z'][1:]]
        reference = CorrelatedRangeNoise(5, config['noise_correlation_time_s'])
        expected = [reference.sample(1, round(k * config['dt_s'], 8))[0] for k in range(len(samples))]
        self.assertEqual(keys['z'][0], 'object_2')
        self.assertEqual(samples, expected)
        self.assertGreaterEqual(len(samples), round(3.0 / config['dt_s']))

    def test_scene_keeps_source_controller(self):
        meta = Reference('gidas_1549178').meta
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'scene.xosc'
            properties = build_hidrive_scene(path, meta)
            root = ET.parse(path).getroot()
        self.assertEqual(root.find('./RoadNetwork/LogicFile').get('filepath'), meta['road_file'])
        self.assertEqual(properties['mode'], 'additive')
        self.assertIsNotNone(root.find('./Storyboard/Init/Actions/Private[@entityRef="object_1"]//ActivateControllerAction'))

    def test_parses_esmini_csv_and_log_values(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / 'a.csv').write_text(
                'esmini GIT REV: x\nNumber of Vehicles: 1\n'
                'Index [-] , TimeStamp [s] , #1 Entity_Name [-] , #1 Current_Speed [m/s] , #1 lane_id, #1 collision_ids , \n'
                '0, 0.000000, object_1, 13.5, -1, , \n1, 0.010000, object_1, 13.4, -1, 1 , \n', encoding='utf-8')
            (folder / 'a.log').write_text('[0.010] [info] LOG_VALUE:AEB_active:1\n[0.010] [info] other\n'
                                          '[0.020] [info] [ControllerHiDrive.cpp::LogValues::1] LOG_VALUE:Speed:2.5\n',
                                          encoding='utf-8')
            frames = read_esmini_csv(folder / 'a.csv')
            values = read_log_values(folder / 'a.log')
        self.assertEqual(frames[0.01]['object_1']['Current_Speed'], '13.4')
        self.assertEqual(frames[0.01]['object_1']['collision_ids'], '1')
        self.assertEqual(values, {0.01: {'AEB_active': '1'}, 0.02: {'Speed': '2.5'}})

    @unittest.skipUnless(EXECUTABLE.is_file(), 'HiDrive esmini not built')
    def test_full_run_brakes_and_writes_studio_outputs(self):
        from runner import run
        cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'run'
            self.addCleanup(os.chdir, cwd)
            summary = run(DEFAULT | {'controller': 'hidrive', 'uncertainty': 'recorded_truth',
                                     'scenario': 'gidas_1554431', 'export_video': False}, out)
            with (out / 'ego.csv').open(encoding='utf-8') as stream:
                ego = list(csv.DictReader(stream))
            saved = json.loads((out / 'summary.json').read_text(encoding='utf-8'))
            os.chdir(cwd)
        # esmini 3 tags log errors; the only expected one is the teacher road-length note (a source warning).
        self.assertIn(summary['status'], ('completed', 'completed_with_warnings'))
        self.assertTrue(all('does not end at road end' in line for line in saved['esmini_error_lines']),
                        saved['esmini_error_lines'])
        self.assertIsNotNone(saved['first_trigger_time_s'])
        self.assertLess(saved['minimum_acceleration_mps2'], -5)
        self.assertEqual(float(ego[-1]['time_s']), saved['end_time_s'])
        if saved['collision']:
            self.assertEqual(saved['end_reason'], 'collision')

    @unittest.skipUnless(EXECUTABLE.is_file(), 'HiDrive esmini not built')
    def test_zero_range_error_matches_ideal_and_noise_is_logged(self):
        from runner import run
        cwd = os.getcwd()
        base = DEFAULT | {'controller': 'hidrive', 'scenario': 'gidas_1554254', 'export_video': False}
        with tempfile.TemporaryDirectory() as folder:
            self.addCleanup(os.chdir, cwd)
            run(base | {'uncertainty': 'recorded_truth'}, Path(folder) / 'ideal')
            os.chdir(cwd)
            run(base | {'uncertainty': 'distance_ou_range', 'noise_reference_mae_pct': 0.0}, Path(folder) / 'zero')
            os.chdir(cwd)
            ideal, zero = ((Path(folder) / name / 'ego.csv').read_bytes() for name in ['ideal', 'zero'])
            with (Path(folder) / 'zero/observations.csv').open(encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(ideal, zero)
        self.assertTrue(rows and all(r['observation_valid'] == 'True' for r in rows))
        self.assertTrue(all(float(r['sampled_range_error_m']) == 0.0 for r in rows))


if __name__ == '__main__':
    unittest.main()
