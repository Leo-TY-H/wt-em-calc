"""A changed equation requests a restart before any stale job is queued."""
import json
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import em_server


class EquationRestartTest(unittest.TestCase):
    def test_changed_equations_return_retryable_response_and_stop_server(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),em_server.Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        try:
            with patch.object(em_server,'fingerprint',return_value='changed'):
                request=Request(f'http://127.0.0.1:{server.server_port}/api/jobs',
                                data=b'{}',headers={'Content-Type':'application/json'})
                with self.assertRaises(HTTPError) as caught:
                    urlopen(request,timeout=5)
                self.assertEqual(caught.exception.code,503)
                self.assertEqual(json.load(caught.exception)['code'],'server_restarting')
            thread.join(timeout=5)
            self.assertFalse(thread.is_alive())
            self.assertTrue(em_server.RESTART_REQUESTED.is_set())
        finally:
            server.server_close()
            em_server.RESTART_REQUESTED.clear()


if __name__=='__main__':unittest.main()
