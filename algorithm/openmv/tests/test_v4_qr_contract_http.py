"""Check the repository's QR demonstration endpoint over loopback HTTP."""
import http.client
import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'sim2real' / 'labs'))
import test_v4_qr_lab as lab


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if not lab.handle_post(self):
            self.send_error(404)

    def respond_json(self, status, data):
        payload = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *args):
        pass


class QRContractHttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = HTTPServer(('127.0.0.1', 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.thread.join()
        cls.server.server_close()

    def post(self, scenario):
        connection = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        try:
            connection.request('POST', '/api/test/v4_qr_lab', json.dumps({'scenario': scenario}),
                               {'Content-Type':'application/json'})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def test_documented_outcomes(self):
        for name in lab.SCENARIOS:
            with self.subTest(name=name):
                status, result = self.post(name)
                self.assertEqual(status, 200)
                expected = ('DONE' if name in ('success','duplicate_seq','qr_ack_lost') else
                            'CANCELLED' if name == 'cancel' else 'FAULT')
                self.assertEqual(result['state'], expected)
                self.assertEqual(result['accepted_qr'], int(expected == 'DONE'))
        self.assertEqual(self.post('unknown')[0], 400)
