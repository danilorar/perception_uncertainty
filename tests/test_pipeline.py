"""Configuration, scenarios, grids and output parsing; full runs when esmini is built.

Controller sources are never opened by these tests.
"""
import csv
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from pipeline import controllers, grid, results, scenarios
from pipeline.common import ESMINI, read_json
from pipeline.config import validate
from pipeline.errors.ou import unit_samples
from pipeline.run import run


def vehicle(x=0.0, y=0.0, speed=10.0):
    return {'x': x, 'y': y, 'h': 0.0, 'speed': speed, 'length': 4.0, 'width': 2.0, 'centerOffsetX': 0.0, 'centerOffsetY': 0.0}


class ConfigTests(unittest.TestCase):
    def test_default_is_hidrive_without_error(self):
        config = validate({})
        self.assertEqual((config['controller'], config['uncertainty']), ('hidrive', 'recorded_truth'))
        self.assertEqual(config['duration_s'], 10.01)

    def test_unknown_values_are_rejected(self):
        for raw in ({'nope': 1}, {'scenario': 'x'}, {'controller': 'x'}, {'uncertainty': 'x'}, {'seed': -1}, {'dt_s': 0.03}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate(raw)

    def test_a_saved_configuration_can_run_again(self):
        config = validate({'uncertainty': 'range_factors', 'range_factors': {'scale': 1.1}})
        self.assertEqual(validate(json.loads(json.dumps(config))), config)

    def test_shipped_configurations_are_valid(self):
        for path in (Path(__file__).parents[1] / 'configs').glob('*.json'):
            with self.subTest(path=path.name):
                validate(read_json(path))


class ControllerTests(unittest.TestCase):
    def test_registry(self):
        found = controllers.all_controllers()
        self.assertEqual(set(found), {'hidrive', 'example_aeb'})
        self.assertEqual(found['hidrive'].esmini_name, 'HiDriveController')
        for controller in found.values():
            for name in controller.sources:  # existence only
                self.assertTrue((controller.folder / name).is_file(), name)


class ScenarioTests(unittest.TestCase):
    def test_objects_start_with_the_ego(self):
        scenario = scenarios.get('gidas_1549178')
        self.assertEqual(scenario.objects[0], {'id': 0, 'name': 'object_1'})
        self.assertEqual(scenario.targets, [{'id': 1, 'name': 'object_2'}])

    def scene(self, controller):
        scenario = scenarios.get('gidas_1549178')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'scene.xosc'
            properties = scenario.write_scene(path, controllers.get(controller))
            return scenario, ET.parse(path).getroot(), properties

    def test_hidrive_keeps_the_properties_of_the_scenario(self):
        scenario, root, properties = self.scene('hidrive')
        self.assertEqual(properties['mode'], 'additive')
        self.assertEqual(root.find('./RoadNetwork/LogicFile').get('filepath'), str(scenario.source.with_suffix('.xodr')))
        self.assertIsNotNone(root.find('./Storyboard/Init/Actions/Private[@entityRef="object_1"]//ActivateControllerAction'))

    def test_another_controller_replaces_name_and_properties(self):
        _, root, properties = self.scene('example_aeb')
        element = root.find('./Entities/ScenarioObject[@name="object_1"]/ObjectController/Controller')
        self.assertEqual(element.get('name'), 'ExampleAEBController')
        self.assertEqual(properties['mode'], 'override')
        self.assertNotIn('timeGap', properties)


class GridTests(unittest.TestCase):
    def test_every_combination_runs(self):
        jobs = grid.expand({'scenarios': ['gidas_1549178'], 'base': {'uncertainty': 'range_factors', 'range_factors.miss_rate': 0.1},
                            'groups': [{'name': 'a', 'vary': {'range_factors.miss_mean_s': [0.2, 0.5], 'noise_zero_range_m': [1.0, 2.0]},
                                        'seeds': {'first': 50, 'count': 2}}]})
        self.assertEqual(len(jobs), 8)
        self.assertEqual({job['seed'] for job in jobs}, {50, 51})
        self.assertEqual(jobs[0]['config']['range_factors']['miss_rate'], 0.1)
        self.assertEqual(jobs[0]['name'], 'gidas_1549178__a__miss_mean_s=0.2__noise_zero_range_m=1.0__seed50')

    def test_short_form_and_default_scenarios(self):
        jobs = grid.expand({'set': {'uncertainty': 'distance_ou_range'}, 'vary': {'noise_reference_mae_pct': [1.0, 2.0]}, 'seeds': 3})
        self.assertEqual(len(jobs), 3 * 2 * 3)

    def test_mistakes_are_rejected(self):
        with self.assertRaises(ValueError):
            grid.expand({'grups': []})
        with self.assertRaises(ValueError):
            grid.expand({'groups': [{'name': 'a', 'set': {'nope': 1}}]})
        with self.assertRaises(ValueError):
            grid.expand({'groups': [{'name': 'a'}, {'name': 'a'}]})

    def test_shipped_grids_are_valid(self):
        for path in (Path(__file__).parents[1] / 'experiments').glob('*.json'):
            with self.subTest(path=path.name):
                self.assertGreater(len(grid.expand(read_json(path))), 0)


class ParsingTests(unittest.TestCase):
    def test_truth_gap(self):
        gap, closing, in_path = results.longitudinal_risk(vehicle(), vehicle(x=20.0, speed=0.0))
        self.assertAlmostEqual(gap, 16)
        self.assertAlmostEqual(closing, 10)
        self.assertTrue(in_path)
        self.assertFalse(results.longitudinal_risk(vehicle(), vehicle(x=20.0, y=4.0))[2])

    def test_esmini_csv_and_log_values(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / 'a.csv').write_text(
                'esmini GIT REV: x\nNumber of Vehicles: 1\n'
                'Index [-] , TimeStamp [s] , #1 Entity_Name [-] , #1 Current_Speed [m/s] , #1 lane_id, #1 collision_ids , \n'
                '0, 0.000000, object_1, 13.5, -1, , \n1, 0.010000, object_1, 13.4, -1, 1 , \n', encoding='utf-8')
            (folder / 'a.log').write_text('[0.010] [info] LOG_VALUE:AEB_active:1\n[0.010] [info] other\n'
                                          '[0.020] [info] [file.cpp::Log::1] LOG_VALUE:Speed:2.5\n', encoding='utf-8')
            frames = results.read_esmini_csv(folder / 'a.csv')
            values = results.read_log_values(folder / 'a.log')
        self.assertEqual(frames[0.01]['object_1']['Current_Speed'], '13.4')
        self.assertEqual(frames[0.01]['object_1']['collision_ids'], '1')
        self.assertEqual(values, {0.01: {'AEB_active': '1'}, 0.02: {'Speed': '2.5'}})


@unittest.skipUnless(ESMINI.is_file(), 'esmini is not built; run: python -m pipeline setup')
class FullRunTests(unittest.TestCase):
    def table(self, path):
        with path.open(encoding='utf-8') as stream:
            return list(csv.DictReader(stream))

    def test_hidrive_brakes_and_writes_outputs(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'run'
            summary = run(validate({'scenario': 'gidas_1554431'}), out, plots=True)
            ego = self.table(out / 'ego.csv')
            self.assertTrue((out / 'speed.png').is_file() and (out / 'range.png').is_file())
        self.assertIn(summary['status'], ('completed', 'completed_with_warnings'))
        self.assertIsNotNone(summary['first_trigger_time_s'])
        self.assertLess(summary['minimum_acceleration_mps2'], -5)
        self.assertEqual(float(ego[-1]['time_s']), summary['end_time_s'])

    def test_zero_error_equals_no_error_and_is_logged(self):
        base = {'scenario': 'gidas_1554254'}
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            run(validate(base), folder / 'none', plots=False)
            run(validate(base | {'uncertainty': 'distance_ou_range', 'noise_reference_mae_pct': 0.0}), folder / 'zero', plots=False)
            self.assertEqual((folder / 'none/ego.csv').read_bytes(), (folder / 'zero/ego.csv').read_bytes())
            rows = self.table(folder / 'zero/observations.csv')
        self.assertTrue(rows and all(row['observation_valid'] == 'True' for row in rows))
        self.assertTrue(all(float(row['sampled_range_error_m']) == 0.0 for row in rows))

    def test_applied_error_uses_the_samples_of_the_noise_file(self):
        config = validate({'scenario': 'gidas_1554254', 'uncertainty': 'distance_ou_range', 'noise_reference_mae_pct': 5.0, 'seed': 1})
        with tempfile.TemporaryDirectory() as folder:
            run(config, Path(folder) / 'run', plots=False)
            rows = self.table(Path(folder) / 'run/observations.csv')
        samples = unit_samples(config, 1, len(rows) + 300)
        self.assertIn(float(rows[10]['range_error_normalized']), samples)

    def test_example_controller_brakes(self):
        with tempfile.TemporaryDirectory() as folder:
            summary = run(validate({'controller': 'example_aeb'}), Path(folder) / 'run', plots=False)
        self.assertIsNotNone(summary['first_brake_time_s'])
        self.assertEqual(summary['minimum_acceleration_mps2'], -8.0)

    def test_error_reaches_the_example_controller(self):
        base = {'controller': 'example_aeb', 'uncertainty': 'range_factors'}
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            near = run(validate(base | {'range_factors': {'scale': 0.5}}), folder / 'near', plots=False)
            exact = run(validate(base | {'range_factors': {}}), folder / 'exact', plots=False)
        self.assertLess(near['first_brake_time_s'], exact['first_brake_time_s'])


if __name__ == '__main__':
    unittest.main()
