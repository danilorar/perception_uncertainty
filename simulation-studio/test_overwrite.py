"""Overwrite is explicit, keeps the record identity, and retains original data."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import app
from settings import write_json


class OverwriteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.runs = self.root / 'runs'
        self.runs.mkdir()
        self.config = {'scenario': 'one', 'seed': 3, 'export_video': False}
        self.process = Mock()
        self.process.wait.return_value = 0
        self.spawned = []
        def spawn(command, **kwargs):
            self.spawned.append((command, kwargs))
            self.addCleanup(kwargs['stdout'].close)
            return self.process
        for target, value in [('ROOT', self.root), ('RUNS', self.runs), ('ACTIVE', {}), ('RUN_JOBS', {})]:
            self.enterContext(patch.object(app, target, value))
        self.validate = self.enterContext(patch.object(app, 'validate', side_effect=lambda raw: dict(raw)))
        self.spawn = self.enterContext(patch.object(app.subprocess, 'Popen', side_effect=spawn))
        self.thread = self.enterContext(patch.object(app.threading, 'Thread'))
        self.refresh = self.enterContext(patch.object(app, 'refresh_plot_scenarios'))

    def record(self, ident='original', status='completed', scenario='one'):
        out = self.runs / ident
        out.mkdir()
        config = {**self.config, 'scenario': scenario, 'seed': 1}
        write_json(out / 'config.json', config)
        write_json(out / 'summary.json', {'run_id': ident, 'status': status, 'config': config})
        write_json(out / 'plot_axes_policy.json', {'independent_y': ['range']})
        (out / 'video.mp4').write_bytes(b'old video must not appear as a new video')
        (out / 'nested').mkdir()
        (out / 'nested/raw.csv').write_text('original data', encoding='utf-8')
        return out

    def finish(self, ident, status='completed', code=0):
        out = self.runs / ident
        out.mkdir(exist_ok=True)
        write_json(out / 'summary.json', {'run_id': ident, 'status': status, 'config': self.config})
        self.process.wait.return_value = code
        app.monitor(ident, self.process, self.spawned[-1][1]['stdout'])

    def test_new_run_keeps_selected_history_and_does_not_inherit_its_policy(self):
        old = self.record()
        before = (old / 'summary.json').read_bytes()
        result = app.start_run(self.config)
        self.assertNotEqual(result['run_id'], old.name)
        self.assertFalse(result['overwritten'])
        self.assertEqual((old / 'summary.json').read_bytes(), before)
        self.assertNotIn('--plot-axes-policy', self.spawned[0][0])
        pending = app.run_summary(result['run_id'], details=True)
        self.assertEqual(pending['status'], 'starting')
        self.assertEqual(pending['config'], self.config)
        self.assertEqual(pending['files'], [])
        self.finish(result['run_id'])
        self.assertEqual(len(list(self.runs.glob('*/summary.json'))), 2)

    def test_overwrite_keeps_id_backs_up_every_file_and_forwards_plot_exception(self):
        old = self.record()
        original = {str(p.relative_to(old)): p.read_bytes() for p in old.rglob('*') if p.is_file()}
        result = app.start_run(self.config, old.name)
        self.assertEqual(result['run_id'], old.name)
        self.assertTrue(result['overwritten'])
        backup = Path(app.RUN_JOBS[old.name]['backup'])
        self.assertEqual({str(p.relative_to(backup)): p.read_bytes() for p in backup.rglob('*') if p.is_file()}, original)
        command = self.spawned[0][0]
        self.assertEqual(command[command.index('--output') + 1], str(old))
        self.assertEqual(command[command.index('--plot-axes-policy') + 1], str(backup / 'plot_axes_policy.json'))
        self.assertEqual(app.run_summary(old.name, details=True)['files'], [])
        self.finish(old.name)
        self.assertFalse((old / 'video.mp4').exists())
        self.assertTrue((backup / 'video.mp4').is_file())
        self.assertEqual(len(list(self.runs.glob('*/summary.json'))), 1)
        self.assertEqual(json.loads((old / 'replacement.json').read_text())['backup'], str(backup))
        self.assertEqual(app.ACTIVE, {})
        self.assertEqual(app.RUN_JOBS, {})

    def test_invalid_settings_or_missing_record_never_archive_any_data(self):
        old = self.record()
        with patch.object(app, 'validate', side_effect=ValueError('invalid config')):
            with self.assertRaises(ValueError):
                app.start_run({}, old.name)
        for ident in ['missing', '../outside', str(old)]:
            with self.subTest(ident=ident), self.assertRaises(ValueError):
                app.start_run(self.config, ident)
        self.assertTrue((old / 'video.mp4').is_file())
        self.spawn.assert_not_called()
        self.assertFalse((self.root / 'work/replaced-runs').exists())

    def test_active_run_and_running_record_are_rejected(self):
        old = self.record(status='running')
        with self.assertRaises(app.RunConflict):
            app.start_run(self.config, old.name)
        app.ACTIVE['other'] = self.process
        for replace in [None, old.name]:
            with self.assertRaises(app.RunConflict):
                app.start_run(self.config, replace)
        self.spawn.assert_not_called()
        self.assertTrue(old.exists())

    def test_launch_failure_restores_original_record(self):
        old = self.record()
        original = (old / 'summary.json').read_bytes()
        self.spawn.side_effect = OSError('cannot launch')
        with self.assertRaises(OSError):
            app.start_run(self.config, old.name)
        self.assertEqual((old / 'summary.json').read_bytes(), original)
        self.assertTrue((old / 'video.mp4').exists())
        self.assertEqual(app.ACTIVE, {})

    def test_worker_failure_keeps_backup_and_reports_failure(self):
        old = self.record()
        app.start_run(self.config, old.name)
        backup = Path(app.RUN_JOBS[old.name]['backup'])
        self.process.wait.return_value = 1
        # A process that fails even before creating its output gets diagnostics.
        app.monitor(old.name, self.process, self.spawned[-1][1]['stdout'])
        summary = app.run_summary(old.name)
        self.assertEqual(summary['status'], 'failed')
        self.assertIn(str(backup), summary['error'])
        self.assertTrue((backup / 'nested/raw.csv').is_file())
        self.assertTrue((old / 'worker.log').exists())
        self.assertEqual(app.ACTIVE, {})

    def test_overwriting_with_a_different_scenario_refits_both_groups(self):
        old = self.record(scenario='other')
        app.start_run(self.config, old.name)
        self.finish(old.name)
        self.refresh.assert_called_once_with({'one', 'other'})

    def test_cancelled_overwrite_retains_backup_and_refits_peers(self):
        old = self.record()
        app.start_run(self.config, old.name)
        backup = Path(app.RUN_JOBS[old.name]['backup'])
        self.finish(old.name, status='cancelled', code=1)
        self.assertEqual(app.run_summary(old.name)['status'], 'cancelled')
        self.assertTrue(backup.is_dir())
        self.refresh.assert_called_once_with({'one'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
