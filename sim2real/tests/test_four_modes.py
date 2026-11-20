"""Offline HTTP acceptance checks (ROS2/STM32 not required)."""
from __future__ import annotations
import base64
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_mode_gateway import mode_get, mode_post


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=APP_DIR, **kwargs)

    def respond_json(self, code, obj):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not mode_get(self):
            self.respond_json(404, {'error': 'No route'})

    def do_POST(self):
        if not mode_post(self):
            self.respond_json(404, {'error': 'No route'})

    def log_message(self, *args):
        pass


APP_DIR = ''


class FourModeHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global APP_DIR
        cls.temp = tempfile.TemporaryDirectory()
        APP_DIR = cls.temp.name
        (Path(APP_DIR)/'index.html').write_text('<h1>four mode test</h1>', encoding='utf-8')
        (Path(APP_DIR)/'keyboard.html').write_text('<h1>keyboard feedback test</h1>', encoding='utf-8')
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=3)
        cls.temp.cleanup()

    def get(self, path):
        with urlopen(self.url+path, timeout=2) as res:
            data=res.read()
            return json.loads(data) if res.headers.get_content_type() == 'application/json' else data.decode('utf-8')

    def post(self, path, obj):
        request = Request(self.url+path, method='POST', data=json.dumps(obj).encode('utf-8'),
                          headers={'Content-Type':'application/json'})
        with urlopen(request, timeout=2) as res:
            return json.load(res)

    def test_four_pages_share_frontend(self):
        for mode in ('sim','test','real'):
            with self.subTest(mode=mode):
                self.assertIn('four mode test', self.get('/'+mode))
        self.assertIn('keyboard feedback test', self.get('/keyboard'))

    def test_test_and_real_data_never_mix(self):
        start_real=self.get('/api/real/telemetry')['seq']
        start_test=self.get('/api/test/telemetry')['seq']
        row={'x':1.25,'y':0.25,'yaw':0.3,'vx':0.1,'vy':0,'wz':0.05,
             'source':'mock_example','source_type':'REAL'}
        output=self.post('/api/test/telemetry',row)
        self.assertEqual(output['source_type'],'TEST')
        result=self.get('/api/test/telemetry?after='+str(start_test))
        self.assertEqual(result['samples'][-1]['source_type'],'TEST')
        self.assertEqual(result['samples'][-1]['x'],1.25)
        self.assertEqual(self.get('/api/real/telemetry')['seq'],start_real)
        self.assertFalse(self.get('/api/real/telemetry')['online'])

    def test_ros_state_and_grid_isolation(self):
        self.post('/api/test/ros/state', {'paths':{'demo':[]}, 'topics':{},'display_frame':'map'})
        self.assertTrue(self.get('/api/test/ros/state')['online'])
        self.assertEqual(self.get('/api/real/ros/state')['seq'],0)
        grid={'name':'map','width':2,'height':2,'resolution':.05,'origin_x':0,'origin_y':0,
              'origin_yaw':0,'data':base64.b64encode(bytes([0,100,255,0])).decode('ascii')}
        self.post('/api/test/ros/grid',grid)
        self.assertEqual(self.get('/api/test/ros/grid/map')['grid']['width'],2)
        self.assertEqual(self.get('/api/real/ros/grid/map')['seq'],0)

    def test_invalid_telemetry_and_bad_crc_like_json_rejected(self):
        with self.assertRaises(HTTPError) as failure:
            self.post('/api/test/telemetry',{'x':'NaN','y':0,'yaw':0,'vx':0,'vy':0,'wz':0})
        self.assertEqual(failure.exception.code,400)
        with self.assertRaises(HTTPError) as failure:
            self.post('/api/test/ros/grid',{'name':'map','width':2,'height':2,'resolution':1,
                        'data':'not base64'})
        self.assertEqual(failure.exception.code,400)

    def test_keyboard_remote_command_disabled(self):
        with self.assertRaises(HTTPError) as failure:
            self.post('/api/keyboard/command',{'vx':1})
        self.assertEqual(failure.exception.code,403)
        self.assertIs(self.get('/api/mode/status')['keyboard_control'],False)

    def test_cross_origin_upload_rejected(self):
        req=Request(self.url+'/api/test/telemetry',method='POST',data=b'{}',
                    headers={'Content-Type':'application/json','Origin':'https://evil.invalid'})
        with self.assertRaises(HTTPError) as failure:
            urlopen(req,timeout=2)
        self.assertEqual(failure.exception.code,403)


if __name__ == '__main__':
    unittest.main()
