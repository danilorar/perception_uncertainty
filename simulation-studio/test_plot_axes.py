"""Verify comparison behavior using rendered axes and independent input data."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import plots
from settings import write_json


class SharedPlotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.rendered = {}
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
        second = self.run_data('second', peak=141, end=15)
        plots.make_plots(second)
        for name in ('speed', 'range'):
            self.assertEqual(self.rendered['first', name], self.rendered['second', name])
        actual = self.rendered['second', 'range']
        self.assertGreater(actual[0]['y'][1], initial_limit)
        self.assertGreaterEqual(actual[0]['y'][1], 141)
        self.assertGreaterEqual(actual[0]['x'][1], 15)
        self.assertLessEqual(actual[1]['y'][0], -1)
        self.assertGreaterEqual(actual[1]['y'][1], 141-25)
        self.assertEqual(actual[1]['y'][0], -actual[1]['y'][1])
        self.assertEqual(self.rendered['other-scenario', 'range'], different)
        self.assertLess(actual[0]['y'][1], different[0]['y'][1])
        for name, before in protected.items():
            self.assertEqual((first / name).read_bytes(), before)
        self.assertTrue((first / 'postprocessing-backups/before-shared-axes-v1/range.png').exists())
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

    def test_speed_only_does_not_rewrite_range_and_replay_shares_scales(self):
        out = self.run_data('aeb')
        replay = self.run_data('replay', controller='original_trajectory')
        plots.make_plots(out)
        range_before = json.loads((out / 'plot_axes.json').read_text())['range']
        self.drawn.clear()
        plots.make_plots(out, speed_only=True)
        self.assertEqual(self.drawn, [('aeb', 'speed')])
        self.assertEqual(json.loads((out / 'plot_axes.json').read_text())['range'], range_before)
        self.assertEqual(self.rendered['aeb', 'speed'], self.rendered['replay', 'speed'])

    def test_run_still_exporting_video_gets_new_scales(self):
        pending = self.run_data('exporting-video', status='running')
        plots.make_plots(pending)
        later = self.run_data('later', peak=300)
        plots.make_plots(later)
        self.assertEqual(self.rendered['exporting-video', 'range'], self.rendered['later', 'range'])
        self.assertGreaterEqual(self.rendered['exporting-video', 'range'][0]['y'][1], 300)


if __name__ == '__main__':
    unittest.main(verbosity=2)
