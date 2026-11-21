"""TEST suite offline acceptance; no ROS2/MCU dependency."""
from __future__ import annotations

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
from types import SimpleNamespace
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
import test_inspection
from test_telemetry_bridge import MonitorState, decode, wheel_snapshot, finite
from web_mode_gateway import mode_get, mode_post

APP_DIR = None


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=APP_DIR, **kwargs)

    def respond_json(self, status, obj):
        data=json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type','application/json')
        self.send_header('Content-Length',str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not (test_inspection.handle_get(self) or mode_get(self)):
            self.respond_json(404,{'error':'Not found'})

    def do_POST(self):
        if not (test_inspection.handle_post(self) or mode_post(self)):
            self.respond_json(404,{'error':'Not found'})

    def log_message(self, *args):
        pass


def example():
    return {'schema':'car_test_inspection_v1','source_type':'MOCK',
            'topics':{'link':{'count':80,'age_s':0.1,'hz':2},
                      'wheels':{'count':200,'age_s':0.04,'hz':20}},
            'samples':{'link':{'connected':True,'heartbeat_fresh':True,'received_frames':123,'crc_errors':0},
                       'wheels':{'target_pulse_per_s':[1,2,3,4],'actual_pulse_per_s':[2,3,4,5],
                                 'per_wheel_pid_output_valid':True}},
            'events':[{'id':1,'kind':'VISION','text':'SEQ received'}]}


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global APP_DIR
        cls.temp=tempfile.TemporaryDirectory();APP_DIR=cls.temp.name
        (Path(APP_DIR)/'index.html').write_text('Car TEST',encoding='utf-8')
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        cls.worker=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.worker.start()
        cls.url='http://127.0.0.1:'+str(cls.server.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.worker.join(timeout=2);cls.temp.cleanup()

    def get(self,route):
        with urlopen(self.url+route,timeout=3) as r:return json.load(r)

    def post(self,route,obj,headers=None):
        req=Request(self.url+route,data=json.dumps(obj).encode('utf-8'),method='POST',
                    headers={'Content-Type':'application/json',**(headers or {})})
        with urlopen(req,timeout=3) as r:return json.load(r)

    def test_post_and_read_live_mock(self):
        before=self.get('/api/test/inspection')['seq']
        got=self.post('/api/test/inspection',example())
        self.assertEqual(got['seq'],before+1)
        item=self.get('/api/test/inspection')
        self.assertTrue(item['online'])
        self.assertEqual(item['source_type'],'MOCK')
        self.assertTrue(item['snapshot']['topics']['wheels']['fresh'])
        self.assertEqual(item['snapshot']['samples']['wheels']['actual_pulse_per_s'],[2,3,4,5])
        self.assertFalse(item['snapshot']['control_enabled'])

    def test_real_api_isolation(self):
        real=self.get('/api/real/telemetry')
        self.post('/api/test/inspection',example())
        self.assertEqual(self.get('/api/real/telemetry')['seq'],real['seq'])
        with self.assertRaises(HTTPError) as err:
            self.post('/api/real/inspection',example())
        self.assertEqual(err.exception.code,404)

    def test_reject_wrong_source_and_schema(self):
        for key,value in [('source_type','REAL'),('schema','unknown')]:
            with self.subTest(field=key):
                bad=example();bad[key]=value
                with self.assertRaises(HTTPError) as err:self.post('/api/test/inspection',bad)
                self.assertEqual(err.exception.code,400)

    def test_reject_nonfinite_and_unknown_ros_field(self):
        for invalid in (float('nan'),float('inf')):
            bad=example();bad['samples']['wheels']['actual_pulse_per_s'][1]=invalid
            with self.assertRaises(HTTPError) as err:self.post('/api/test/inspection',bad)
            self.assertEqual(err.exception.code,400)
        bad=example();bad['samples']['link']['motor_enable']=True
        with self.assertRaises(HTTPError) as err:self.post('/api/test/inspection',bad)
        self.assertEqual(err.exception.code,400)

    def test_origin_rejected_and_keyboard_still_disabled(self):
        with self.assertRaises(HTTPError) as err:
            self.post('/api/test/inspection',example(),{'Origin':'https://attacker.invalid'})
        self.assertEqual(err.exception.code,403)
        with self.assertRaises(HTTPError) as err:self.post('/api/keyboard/command',{'vx':1})
        self.assertEqual(err.exception.code,403)

    def test_stale_becomes_offline_without_fake_motion(self):
        self.post('/api/test/inspection',example())
        original=test_inspection.STORE['last_mono']
        with patch.object(test_inspection.time,'monotonic',return_value=original+2.5):
            status=self.get('/api/test/inspection')
        self.assertFalse(status['online'])
        self.assertEqual(status['snapshot']['source_type'],'MOCK')


class DecoderTests(unittest.TestCase):
    def test_missing_wheel_pid_is_null_not_zero(self):
        msg=SimpleNamespace(target_pulse_per_s=[1.0,2.,3.,4.],actual_pulse_per_s=[2.,3.,4.,5.],
            target_mps=[0.,0.,0.,0.],actual_mps=[0.,0.,0.,0.],
            max_abs_pid_output=float('nan'),per_wheel_pid_output_valid=False)
        got=wheel_snapshot(msg)
        self.assertIsNone(got['max_abs_pid_output'])
        self.assertFalse(got['per_wheel_pid_output_valid'])
        self.assertIsNone(finite(float('nan')))

    def test_wrong_wheel_count_rejected(self):
        msg=SimpleNamespace(target_pulse_per_s=[1],actual_pulse_per_s=[1],
            target_mps=[1],actual_mps=[1],max_abs_pid_output=0,per_wheel_pid_output_valid=True)
        with self.assertRaises(ValueError):decode('wheels',msg)

    def test_topic_counts_and_events_are_true_received(self):
        st=MonitorState();base=800.0
        msg=SimpleNamespace(event_type=0x90,cmd_id=21,original_command_type=0x83,
            status_code=0,progress_percent=0,phase_code=0,error_code=0,control_owner=1)
        st.consume('feedback',msg,base);st.consume('feedback',msg,base+.1)
        snap=st.snapshot(base+.12)
        self.assertEqual(snap['topics']['feedback']['count'],2)
        self.assertEqual(snap['topics']['feedback']['hz'],10)
        self.assertIn('ACK',snap['events'][0]['text'])
        self.assertNotIn('COMPLETED',snap['events'][0]['text'])
        self.assertEqual(snap['topics']['wheels']['count'],0)
        self.assertFalse(snap['topics']['wheels']['fresh'])


if __name__=='__main__':unittest.main()
