"""Verify comparison behavior using rendered axes and independent input data."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import plots
from plot_axes import AXES_VERSION
from settings import write_json


class SharedPlotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.rendered = {}
        self.notes = {}
        self.drawn = []
        # Retain the real Matplotlib axes and layout without PNG compression.
        def save(fig, path, **kwargs):
            fig.canvas.draw()
            if kwargs['format'] == 'png':
                name = path.name.split('.')[0]
                self.rendered[path.parent.name, name] = [
                    {'x': list(ax.get_xlim()), 'y': list(ax.get_ylim()),
                     'xticks': list(ax.get_xticks()), 'yticks': list(ax.get_yticks()),
                     'rectangle': list(ax.get_position().bounds)} for ax in fig.axes]
                self.drawn.append((path.parent.name, name))
                self.notes[path.parent.name, name] = [text.get_text() for text in fig.texts]
            path.write_bytes(b'plot')
        self.mock = patch('matplotlib.figure.Figure.savefig', save)
        self.mock.start()
        self.addCleanup(self.mock.stop)

    def run_data(self, name, *, scenario='one', peak=40, end=9.98,
                 status='completed', controller='aeb', missing=False):
        out = self.root / name
        out.mkdir()
        write_json(out / 'config.json', {'scenario': scenario, 'controller': controller,
                                        'duration_s': end})
        write_json(out / 'summary.json', {'status': status})
        datasets = {
            'ego.csv': [dict(time_s=t, speed_kph=s, ideal_ego_speed_kph=50,
                             command_accel_mps2=a, trajectory_accel_mps2=-2)
                        for t, s, a in [(0, 50, 0), (end, 0, -8)]],
            'observations.csv': [
                dict(time_s=0, object_name='car', true_range_m=25,
                     observed_range_m='' if missing else peak,
                     estimated_range_m='' if missing else 30),
                dict(time_s=end-.03, object_name='car', true_range_m=3,
                     observed_range_m='nan' if missing else 2,
                     estimated_range_m='' if missing else 3)]}
        for name, rows in datasets.items():
            with (out / name).open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        return out

    def test_new_extremes_update_old_figures_and_preserve_simulation(self):
        first = self.run_data('first')
        protected = {p.name: p.read_bytes() for p in first.iterdir()}
        plots.make_plots(first)
        initial_limit = self.rendered['first', 'range'][0]['y'][1]
        other = self.run_data('other-scenario', scenario='two', peak=2000)
        plots.make_plots(other)
        different = self.rendered['other-scenario', 'range']
        second = self.run_data('second', peak=53.42, end=15)
        plots.make_plots(second)
        for name in ('speed', 'range'):
            self.assertEqual(self.rendered['first', name], self.rendered['second', name])
        actual = self.rendered['second', 'range']
        self.assertGreater(actual[0]['y'][1], initial_limit)
        self.assertGreaterEqual(actual[0]['y'][1], 53.42)
        self.assertGreaterEqual(actual[0]['x'][1], 15)
        self.assertLessEqual(actual[1]['y'][0], -1)
        self.assertGreaterEqual(actual[1]['y'][1], 53.42-25)
        self.assertGreater(actual[1]['y'][0], -actual[1]['y'][1])
        self.assertEqual(self.rendered['other-scenario', 'range'], different)
        self.assertLess(actual[0]['y'][1], different[0]['y'][1])
        for name, before in protected.items():
            self.assertEqual((first / name).read_bytes(), before)
        self.assertTrue((first / f'postprocessing-backups/before-shared-axes-v{AXES_VERSION}/range.png').exists())
        before = json.loads((first / 'plot_axes.json').read_text())
        plots.make_plots(first)
        self.assertEqual(before, json.loads((first / 'plot_axes.json').read_text()))

    def test_incomplete_runs_do_not_pollute_scales_and_missing_values_work(self):
        self.run_data('partial', peak=10000, status='running')
        self.run_data('failed', peak=10000, status='failed')
        out = self.run_data('valid', missing=True)
        plots.make_plots(out)
        actual = self.rendered['valid', 'range']
        self.assertGreaterEqual(actual[0]['y'][1], 25)
        self.assertLess(actual[0]['y'][1], 100)
        self.assertGreater(actual[1]['y'][1], actual[1]['y'][0])
        self.assertEqual(set(key[0] for key in self.rendered), {'valid'})

    def test_speed_only_keeps_range_and_groups_speed_and_acceleration_separately(self):
        out = self.run_data('aeb')
        replay = self.run_data('replay', controller='original_trajectory')
        plots.make_plots(out)
        range_before = json.loads((out / 'plot_axes.json').read_text())['range']
        range_groups = json.loads((self.root / 'plot-axes.json').read_text())['scenarios']['one']['y_groups']['range']
        self.drawn.clear()
        plots.make_plots(out, speed_only=True)
        self.assertEqual(self.drawn, [('aeb', 'speed')])
        self.assertEqual(json.loads((out / 'plot_axes.json').read_text())['range'], range_before)
        self.assertEqual(self.rendered['aeb', 'speed'][0], self.rendered['replay', 'speed'][0])
        self.assertNotEqual(self.rendered['aeb', 'speed'][1]['y'], self.rendered['replay', 'speed'][1]['y'])
        self.assertEqual(json.loads((self.root / 'plot-axes.json').read_text())['scenarios']['one']['y_groups']['range'], range_groups)

    def test_run_still_exporting_video_gets_new_scales(self):
        pending = self.run_data('exporting-video', status='running')
        plots.make_plots(pending)
        later = self.run_data('later', peak=53.42)
        plots.make_plots(later)
        self.assertEqual(self.rendered['exporting-video', 'range'], self.rendered['later', 'range'])
        self.assertGreaterEqual(self.rendered['exporting-video', 'range'][0]['y'][1], 53.42)

    def test_removed_peak_shrinks_remaining_shared_axes(self):
        first = self.run_data('first', peak=53.42)
        second = self.run_data('second', peak=48)
        extreme = self.run_data('old-extreme', peak=75)
        plots.make_plots(first)
        self.assertGreaterEqual(self.rendered['first', 'range'][0]['y'][1], 75)
        # Move a result out of the active collection, like archiving a run.
        archive = self.root / 'archive'
        archive.mkdir()
        extreme.rename(archive / extreme.name)
        plots.make_plots(first)
        self.assertEqual(self.rendered['first', 'range'], self.rendered['second', 'range'])
        self.assertEqual(self.rendered['first', 'range'][0]['y'], [0., 60.])
        registry = json.loads((self.root / 'plot-axes.json').read_text())
        self.assertAlmostEqual(registry['scenarios']['one']['range'][1], 53.42)

    def test_one_record_can_have_independent_range_axes_without_affecting_peers(self):
        first = self.run_data('first', peak=53.42)
        second = self.run_data('second', peak=48)
        extreme = self.run_data('extreme', peak=141, end=15)
        plots.make_plots(first)
        before = (first / 'plot_axes.json').read_bytes()
        protected = {p: p.read_bytes() for out in (first, second, extreme)
                     for p in out.iterdir() if p.suffix == '.csv' or p.name in ('config.json', 'summary.json')}
        write_json(extreme / 'plot_axes_policy.json', {'independent_y': ['range']})
        # Explicit exceptions still override the automatic magnitude groups.
        plots.make_plots(extreme)
        self.assertEqual((first / f'postprocessing-backups/before-shared-axes-v{AXES_VERSION}/plot_axes.json').read_bytes(), before)
        regular = self.rendered['first', 'range']
        independent = self.rendered['extreme', 'range']
        self.assertEqual(regular, self.rendered['second', 'range'])
        self.assertEqual(regular[0]['y'], [0., 60.])
        self.assertGreaterEqual(independent[0]['y'][1], 141)
        self.assertGreaterEqual(independent[1]['y'][1], 116)
        for shared, separate in zip(regular, independent):
            self.assertEqual(shared['x'], separate['x'])
            self.assertEqual(shared['xticks'], separate['xticks'])
            self.assertEqual(shared['rectangle'], separate['rectangle'])
        self.assertEqual(self.rendered['first', 'speed'], self.rendered['extreme', 'speed'])
        for path, data in protected.items():
            self.assertEqual(path.read_bytes(), data)
        # Rendering an ordinary run must continue respecting the exception.
        plots.make_plots(first)
        self.assertEqual(self.rendered['first', 'range'], regular)
        self.assertEqual(self.rendered['extreme', 'range'], independent)
        registry = json.loads((self.root / 'plot-axes.json').read_text())
        self.assertAlmostEqual(registry['scenarios']['one']['range'][1], 53.42)
        manifest = json.loads((extreme / 'plot_axes.json').read_text())
        self.assertEqual(manifest['range']['y_scope'], 'run')
        self.assertEqual(manifest['speed']['y_scope'], 'scenario')
        # Removing the policy restores automatic grouping, without clipping peaks.
        (extreme / 'plot_axes_policy.json').unlink()
        plots.make_plots(first)
        self.assertEqual(self.rendered['first', 'range'], regular)
        self.assertGreaterEqual(self.rendered['extreme', 'range'][0]['y'][1], 141)
        self.assertEqual(json.loads((extreme / 'plot_axes.json').read_text())['range']['y_scope'], 'group')

    def test_independent_runs_do_not_expand_each_other_and_speed_only_keeps_range(self):
        first = self.run_data('first', peak=141)
        second = self.run_data('second', peak=800)
        for out in (first, second):
            write_json(out / 'plot_axes_policy.json', {'independent_y': ['range']})
        plots.make_plots(first)
        self.assertLess(self.rendered['first', 'range'][0]['y'][1],
                        self.rendered['second', 'range'][0]['y'][1])
        before = (first / 'range.png').read_bytes()
        spec = json.loads((first / 'plot_axes.json').read_text())['range']
        plots.make_plots(first, speed_only=True)
        self.assertEqual((first / 'range.png').read_bytes(), before)
        self.assertEqual(json.loads((first / 'plot_axes.json').read_text())['range'], spec)

    def test_old_cache_cannot_expand_actual_bounds_and_errors_fit_data(self):
        first = self.run_data('first', peak=53.42)
        second = self.run_data('second', peak=5.33)
        write_json(self.root / 'plot-axes.json', {'version': 1, 'scenarios': {
            'one': {'range': [0., 141.], 'range_error': [-100., 100.]}}})
        plots.make_plots(first)
        actual = self.rendered['first', 'range']
        self.assertEqual(actual[0]['y'], [0., 60.])
        self.assertEqual(actual[1]['y'], [-30., 40.])
        self.assertEqual(actual, self.rendered['second', 'range'])

    def test_large_runs_share_a_separate_group_without_flattening_small_runs(self):
        first = self.run_data('first', peak=53.42)
        self.run_data('second', peak=48)
        plots.make_plots(first)
        ordinary = self.rendered['first', 'range']
        high = self.run_data('high', peak=141, end=15)
        self.run_data('also-high', peak=155, end=15)
        plots.make_plots(high)
        for name in ('first', 'second'):
            axes = self.rendered[name, 'range']
            self.assertEqual([a['y'] for a in axes], [a['y'] for a in ordinary])
            self.assertEqual([a['yticks'] for a in axes], [a['yticks'] for a in ordinary])
        large = self.rendered['high', 'range']
        self.assertEqual(large, self.rendered['also-high', 'range'])
        self.assertGreaterEqual(large[0]['y'][1], 155)
        self.assertGreaterEqual(large[1]['y'][1], 130)
        for small_axis, large_axis in zip(self.rendered['first', 'range'], large):
            for key in ('x', 'xticks', 'rectangle'):
                self.assertEqual(small_axis[key], large_axis[key])
        self.assertEqual(self.rendered['first', 'speed'], self.rendered['high', 'speed'])
        self.assertIn('range: group 1/2 (2 runs)', self.notes['first', 'range'][0])
        self.assertIn('range: group 2/2 (2 runs)', self.notes['high', 'range'][0])

    def test_range_and_error_group_independently_and_keep_zero_and_negative_peaks(self):
        first = self.run_data('first')
        for name, errors in [('first', (0, 0)), ('small', (-10, 10)),
                             ('similar', (-12, 12)), ('negative-spike', (-450, 10))]:
            out = first if name == 'first' else self.run_data(name)
            with (out / 'observations.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=['time_s', 'object_name', 'true_range_m',
                                                            'observed_range_m', 'estimated_range_m'])
                writer.writeheader()
                writer.writerows(dict(time_s=t, object_name='car', true_range_m=500,
                                      observed_range_m=500+err, estimated_range_m=500)
                                 for t, err in enumerate(errors))
        plots.make_plots(first)
        axes = {name: self.rendered[name, 'range']
                for name in ('first', 'small', 'similar', 'negative-spike')}
        self.assertTrue(all(item[0] == axes['first'][0] for item in axes.values()))
        self.assertEqual(axes['first'][1]['y'], [-1, 1])
        self.assertEqual(axes['small'][1], axes['similar'][1])
        self.assertGreater(axes['small'][1]['y'][0], -30)
        self.assertLessEqual(axes['negative-spike'][1]['y'][0], -450)
        self.assertGreaterEqual(axes['negative-spike'][1]['y'][1], 10)

    def test_intermediate_runs_cannot_bridge_small_and_extreme_scales(self):
        runs = [self.run_data(f'run-{index}', peak=peak)
                for index, peak in enumerate((30, 53.42, 85, 140, 230, 380))]
        plots.make_plots(runs[-1])
        self.assertLessEqual(self.rendered['run-0', 'range'][0]['y'][1], 80)
        self.assertGreaterEqual(self.rendered['run-5', 'range'][0]['y'][1], 380)
        before = {out.name: (out / 'plot_axes.json').read_bytes() for out in runs}
        plots.make_plots(runs[0])
        self.assertEqual(before, {out.name: (out / 'plot_axes.json').read_bytes() for out in runs})

    def test_group_membership_recomputed_after_peak_is_corrected(self):
        first = self.run_data('first', peak=53.42)
        corrected = self.run_data('corrected', peak=141)
        plots.make_plots(first)
        self.assertNotEqual(self.rendered['first', 'range'], self.rendered['corrected', 'range'])
        data = (corrected / 'observations.csv').read_text()
        (corrected / 'observations.csv').write_text(data.replace(',141,', ',48,'))
        plots.make_plots(first)
        self.assertEqual(self.rendered['first', 'range'], self.rendered['corrected', 'range'])
        self.assertEqual(self.rendered['first', 'range'][0]['y'], [0, 60])
        self.assertEqual(json.loads((first / 'plot_axes.json').read_text())['range']['y_scope'], 'scenario')


if __name__ == '__main__':
    unittest.main(verbosity=2)
