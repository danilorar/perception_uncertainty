"""History deletion must preserve other runs, reject active runs, and refit plots."""
import csv
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import app
import plots
from settings import write_json


class HistoryDeletionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.runs = self.root / 'runs'
        self.runs.mkdir()
        for name, value in [('ROOT', self.root), ('RUNS', self.runs), ('ACTIVE', {})]:
            mock = patch.object(app, name, value)
            mock.start()
            self.addCleanup(mock.stop)

    def record(self, name, *, status='completed', scenario='one', peak=40):
        out = self.runs / name
        out.mkdir()
        config = {'scenario': scenario, 'controller': 'aeb', 'duration_s': 10}
        write_json(out / 'config.json', config)
        write_json(out / 'summary.json', {'run_id': name, 'status': status, 'config': config})
        datasets = {
            'ego.csv': [dict(time_s=t, speed_kph=s, command_accel_mps2=a)
                        for t, s, a in [(0, 50, 0), (10, 0, -8)]],
            'observations.csv': [dict(time_s=0, object_name='car', true_range_m=25,
                                      observed_range_m=peak, estimated_range_m=30)]}
        for filename, rows in datasets.items():
            with (out / filename).open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        (out / 'nested').mkdir()
        (out / 'nested/raw.bin').write_bytes(b'original raw data')
        return out

    def test_batch_deletes_only_selected_and_preserves_backup(self):
        first = self.record('first', scenario='first')
        second = self.record('second', status='failed', scenario='second')
        keep = self.record('keep', scenario='keep')
        original = {str(p.relative_to(first)): p.read_bytes() for p in first.rglob('*') if p.is_file()}
        result = app.delete_runs(['first', 'second', 'first'])
        self.assertEqual(result, {'deleted': ['first', 'second'], 'warnings': []})
        self.assertFalse(first.exists())
        self.assertFalse(second.exists())
        self.assertTrue(keep.exists())
        backup = next((self.root / 'work/deleted-runs').glob('*/first'))
        self.assertEqual(original, {str(p.relative_to(backup)): p.read_bytes()
                                    for p in backup.rglob('*') if p.is_file()})

    def test_invalid_batches_leave_every_record_in_place(self):
        keep = self.record('keep')
        for ids in [None, 'keep', [], [None], ['keep'] * 101,
                    ['keep', '../outside'], ['keep', 'missing'], ['keep', str(keep)]]:
            with self.subTest(ids=ids):
                with self.assertRaises(ValueError):
                    app.delete_runs(ids)
                self.assertTrue(keep.exists())
        self.assertFalse((self.root / 'work/deleted-runs').exists())

    def test_running_or_not_yet_monitored_workers_block_entire_batch(self):
        keep = self.record('keep')
        for status in ['running', 'starting', 'completed']:
            with self.subTest(status=status):
                running = self.record(status, status=status)
                if status == 'completed':
                    app.ACTIVE[status] = object()
                with self.assertRaisesRegex(ValueError, '正在运行'):
                    app.delete_runs(['keep', status])
                self.assertTrue(keep.exists())
                self.assertTrue(running.exists())

    def test_move_failure_restores_already_moved_records(self):
        first = self.record('first')
        second = self.record('second')
        rename = Path.rename
        def failing_rename(source, destination):
            if source == second:
                raise PermissionError('file in use')
            return rename(source, destination)
        with patch.object(Path, 'rename', failing_rename):
            with self.assertRaises(PermissionError):
                app.delete_runs(['first', 'second'])
        self.assertTrue((first / 'nested/raw.bin').is_file())
        self.assertTrue((second / 'nested/raw.bin').is_file())

    def test_redirected_directory_is_rejected(self):
        outside = self.root / 'outside'
        outside.mkdir()
        keep = self.record('keep')
        alias = self.runs / 'alias'
        resolve = Path.resolve
        # This also covers Windows junctions without requiring symlink privileges.
        def resolve_alias(path, *args, **kwargs):
            return outside if path == alias else resolve(path, *args, **kwargs)
        with patch.object(Path, 'resolve', resolve_alias):
            with self.assertRaisesRegex(ValueError, '无效运行目录'):
                app.delete_runs(['keep', 'alias'])
        self.assertTrue(keep.exists())
        self.assertTrue(outside.exists())

    def test_removed_peak_refits_remaining_axes_and_removes_last_cache(self):
        first = self.record('first', peak=53.42)
        second = self.record('second', peak=48)
        extreme = self.record('extreme', peak=75)
        other = self.record('other', scenario='two', peak=900)
        # Keep actual plotting/scale calculation, only avoid PNG compression.
        def save(fig, path, **kwargs):
            path.write_bytes(b'plot')
        with patch('matplotlib.figure.Figure.savefig', save):
            plots.make_plots(first)
            plots.make_plots(other)
            other_before = (other / 'plot_axes.json').read_bytes()
            before = json.loads((first / 'plot_axes.json').read_text())
            self.assertGreaterEqual(before['range']['y'][0]['limits'][1], 75)
            result = app.delete_runs(['extreme'])
            self.assertEqual(result['warnings'], [])
            axes = json.loads((first / 'plot_axes.json').read_text())
            self.assertEqual(axes['range']['y'][0]['limits'], [0, 60])
            self.assertEqual(axes, json.loads((second / 'plot_axes.json').read_text()))
            self.assertEqual((other / 'plot_axes.json').read_bytes(), other_before)
            self.assertFalse(extreme.exists())
            self.assertEqual(app.delete_runs(['first', 'second'])['warnings'], [])
            registry = json.loads((self.runs / 'plot-axes.json').read_text())
            self.assertNotIn('one', registry['scenarios'])
            self.assertIn('two', registry['scenarios'])

    def test_plot_failure_reports_committed_deletion(self):
        self.record('keep')
        removed = self.record('removed')
        with patch('plots.make_plots', side_effect=OSError('cannot write plot')):
            result = app.delete_runs(['removed'])
        self.assertEqual(result['deleted'], ['removed'])
        self.assertEqual(len(result['warnings']), 1)
        self.assertFalse(removed.exists())

    def test_http_authentication_and_deleted_record_access(self):
        self.record('first')
        server = app.ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        url = f'http://127.0.0.1:{server.server_port}'
        body = json.dumps({'run_ids': ['first']}).encode()
        for headers in [{}, {'X-Studio-Token': app.TOKEN, 'Origin': 'https://outside.test'}]:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(url + '/api/runs/delete', body, headers), timeout=5)
            self.assertEqual(error.exception.code, 403)
        with urlopen(Request(url + '/api/runs/delete', body,
                             {'X-Studio-Token': app.TOKEN}), timeout=5) as response:
            self.assertEqual(json.load(response)['deleted'], ['first'])
        with urlopen(url + '/api/runs', timeout=5) as response:
            self.assertEqual(json.load(response), [])
        for path in ['/api/run/first', '/runs/first/summary.json']:
            with self.assertRaises(HTTPError) as error:
                urlopen(url + path, timeout=5)
            self.assertEqual(error.exception.code, 404)


if __name__ == '__main__':
    unittest.main(verbosity=2)
