"""Desktop launches must not reuse a sandbox-owned service or stop active runs."""
import io
import json
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

import app


def response(data):
    return io.BytesIO(json.dumps(data).encode())


class DesktopServiceTests(unittest.TestCase):
    def health(self, **extra):
        return {'app':'tme180-simulation-studio','root':str(app.ROOT),
                'process_user':'desktop_user',**extra}

    @patch('app.process_user',return_value='desktop_user')
    def test_same_user_reuses_without_stopping_service(self, user):
        with patch('app.urlopen') as request:
            self.assertTrue(app.reuse_service('http://127.0.0.1:8976',self.health()))
            request.assert_not_called()

    @patch('app.process_user',return_value='desktop_user')
    def test_other_account_is_retired_before_desktop_launch(self, user):
        with patch('app.urlopen',side_effect=[response({'token':'test-token'}),
                response({'ok':True}),URLError('closed')]) as request:
            self.assertFalse(app.reuse_service('http://127.0.0.1:8976',
                                              self.health(process_user='sandbox_user')))
            shutdown=request.call_args_list[1].args[0]
            self.assertEqual(shutdown.get_method(),'POST')
            self.assertEqual(shutdown.full_url,'http://127.0.0.1:8976/api/shutdown')
            self.assertEqual(shutdown.get_header('X-studio-token'),'test-token')

    @patch('app.process_user',return_value='desktop_user')
    def test_active_service_refusal_blocks_takeover(self, user):
        busy=HTTPError('http://127.0.0.1:8976/api/shutdown',400,'active',{},None)
        with patch('app.urlopen',side_effect=[response({'token':'test-token'}),busy]) as request:
            with self.assertRaisesRegex(RuntimeError,'实验正在运行'):
                app.reuse_service('http://127.0.0.1:8976',self.health(process_user='sandbox_user'))
            self.assertEqual(request.call_count,2)

    def test_other_workspace_is_not_stopped(self):
        with patch('app.urlopen') as request:
            self.assertFalse(app.reuse_service('http://127.0.0.1:8976',self.health(root='unrelated')))
            request.assert_not_called()

    def test_invalid_run_cannot_launch_explorer(self):
        with patch('app.os.startfile') as start:
            with self.assertRaises(ValueError): app.open_results_folder('../outside')
            start.assert_not_called()


if __name__=='__main__': unittest.main(verbosity=2)
