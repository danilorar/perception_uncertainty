"""Source navigation resolves current code and launches only approved targets."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import source_links


class SourceLinksTests(unittest.TestCase):
    def test_every_flow_target_resolves_to_its_current_excerpt(self):
        links = source_links.flow_source_links()
        self.assertEqual(len(links), 12)
        self.assertEqual(len(links['inject']), 2)
        self.assertEqual(len(links['output']), 2)
        self.assertEqual({entry['id'] for group in links.values() for entry in group},
                         set(source_links.TARGETS))
        for entries in links.values():
            for entry in entries:
                with self.subTest(source_id=entry['id']):
                    path = Path(entry['path'])
                    self.assertTrue(path.is_relative_to(source_links.ROOT))
                    lines = path.read_text(encoding='utf-8-sig').splitlines()
                    target = source_links.TARGETS[entry['id']]
                    if target.anchor:
                        self.assertIn(target.anchor, lines[entry['line'] - 1])
                    else:
                        self.assertIn(f'def {target.symbol.split(".")[-1]}(', lines[entry['line'] - 1])

    def test_unknown_ids_never_launch_a_process(self):
        with patch.object(source_links.subprocess, 'Popen') as launch:
            for source_id in (None, [], {}, '', '../runner.py', 'runner.py', '--help'):
                with self.subTest(source_id=source_id), self.assertRaises(ValueError):
                    source_links.open_source(source_id)
            launch.assert_not_called()

    def test_unavailable_source_does_not_break_other_flow_links(self):
        resolve = source_links._resolve_source
        def temporarily_invalid(source_id):
            if source_id == 'inject.apply':
                raise RuntimeError('Source is being edited')
            return resolve(source_id)
        with patch.object(source_links, '_resolve_source', side_effect=temporarily_invalid):
            links = source_links.flow_source_links()
        self.assertEqual(len(links), 12)
        self.assertEqual([entry['id'] for entry in links['inject']], ['inject.begin'])
        self.assertEqual(len(links['output']), 2)

    def test_position_tracks_edits_and_falls_back_to_surrounding_symbol(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'runner.py'
            path.write_text('def run():\n    c = validate(config)\n', encoding='utf-8')
            with patch.object(source_links, 'ROOT', root):
                self.assertEqual(source_links._resolve_source('config')['line'], 2)
                path.write_text('\n\n\ndef run():\n    c = validate(config)\n', encoding='utf-8')
                self.assertEqual(source_links._resolve_source('config')['line'], 5)
                path.write_text('\n\ndef run():\n    changed_call()\n', encoding='utf-8')
                self.assertEqual(source_links._resolve_source('config')['line'], 3)

    def test_missing_source_or_symbol_reports_error(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(source_links, 'ROOT', Path(directory)):
            with self.assertRaisesRegex(RuntimeError, '源码文件'):
                source_links._resolve_source('config')
            (Path(directory) / 'runner.py').write_text('def other():\n    pass\n', encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, '无法定位源码'):
                source_links._resolve_source('config')

    def test_registry_path_cannot_escape_project(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            root = parent / 'project'
            root.mkdir()
            (parent / 'outside.py').write_text('def run():\n    pass\n', encoding='utf-8')
            with patch.object(source_links, 'ROOT', root), patch.dict(source_links.TARGETS, {
                'bad': source_links.SourceTarget('../outside.py', 'run'),
            }), self.assertRaisesRegex(RuntimeError, '项目目录'):
                source_links.open_source('bad')

    def test_launch_passes_spaced_paths_as_arguments_without_a_console(self):
        with tempfile.TemporaryDirectory(prefix='source links ') as directory:
            root = Path(directory)
            (root / 'runner.py').write_text('def run():\n    c = validate(config)\n', encoding='utf-8')
            editor = root / 'Microsoft VS Code/Code.exe'
            process = Mock()
            process.wait.side_effect = subprocess.TimeoutExpired('Code.exe', .75)
            with patch.object(source_links, 'ROOT', root), \
                    patch.object(source_links, '_find_vscode', return_value=editor), \
                    patch.object(source_links.subprocess, 'Popen', return_value=process) as launch, \
                    patch.dict(source_links.os.environ, {'ELECTRON_RUN_AS_NODE': '1'}):
                entry = source_links.open_source('config')
            args, kwargs = launch.call_args
            self.assertEqual(args[0], [str(editor), '--new-window', str(root.resolve()),
                                     '--goto', f'{root.resolve() / "runner.py"}:2:1'])
            self.assertFalse(kwargs['shell'])
            self.assertNotIn('ELECTRON_RUN_AS_NODE', kwargs['env'])
            self.assertEqual(kwargs['creationflags'],
                             subprocess.CREATE_NO_WINDOW if source_links.os.name == 'nt' else 0)
            self.assertEqual(entry['line'], 2)
            self.assertTrue((root / 'work/source-links/vscode-launch.log').is_file())

    def test_windows_path_shim_resolves_to_gui_executable(self):
        with tempfile.TemporaryDirectory() as directory:
            install = Path(directory) / 'Microsoft VS Code'
            (install / 'bin').mkdir(parents=True)
            (install / 'Code.exe').touch()
            with patch.object(source_links.shutil, 'which', side_effect=[str(install / 'bin/code.cmd'), None]):
                self.assertEqual(source_links._find_vscode(), (install / 'Code.exe').resolve())

    def test_missing_editor_and_launch_errors_are_reported(self):
        with patch.object(source_links.shutil, 'which', return_value=None), \
                patch.object(Path, 'is_file', return_value=False), \
                self.assertRaisesRegex(RuntimeError, '未找到 Visual Studio Code'):
            source_links._find_vscode()
        with tempfile.TemporaryDirectory() as directory, patch.object(source_links, 'ROOT', Path(directory)):
            (Path(directory) / 'runner.py').write_text('def run():\n    pass\n', encoding='utf-8')
            with patch.object(source_links, '_find_vscode', return_value=Path(directory) / 'Code.exe'):
                with patch.object(source_links.subprocess, 'Popen', side_effect=OSError('access denied')), \
                        self.assertRaisesRegex(RuntimeError, '无法启动 VS Code'):
                    source_links.open_source('config')
                with patch.object(source_links.subprocess, 'Popen') as launch:
                    launch.return_value.wait.return_value = 3
                    with self.assertRaisesRegex(RuntimeError, '退出码 3'):
                        source_links.open_source('config')


if __name__ == '__main__':
    unittest.main(verbosity=2)
