"""Offline acceptance for command integration and read-only keyboard feedback API."""
from __future__ import annotations

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from keyboard_feedback_gateway import keyboard_get, keyboard_post
from ros2_keyboard_feedback_bridge import MotionObserver, advance_pose


class FakeClock:
    def __init__(self):
        self.now = 100.0
    def __call__(self):
        return self.now
    def step(self, dt):
        self.now += dt


class MotionTests(unittest.TestCase):
    def test_unbounded_body_frame_motion(self):
        self.assertAlmostEqual(advance_pose((50,-20,0), (2,0,0), 100)[0], 250)
        p = advance_pose((0,0,math.pi/2), (1,0,0), 2)
        self.assertAlmostEqual(p[0],0,places=7)
        self.assertAlmostEqual(p[1],2,places=7)
        self.assertAlmostEqual(advance_pose((0,0,0),(0,1,0),3)[1],3)

    def test_translation_and_rotation_together(self):
        p = advance_pose((0,0,0),(1,0,math.pi/2),1)
        self.assertAlmostEqual(p[0],2/math.pi,places=7)
        self.assertAlmostEqual(p[1],2/math.pi,places=7)
        self.assertAlmostEqual(p[2],math.pi/2)

    def test_timeout_does_not_publish_motor_commands(self):
        clock=FakeClock()
        observer=MotionObserver(.5,clock)
        self.assertEqual(observer.snapshot()[1],(0,0,0))
        observer.observe(-2,1,-1)
        clock.step(.2)
        self.assertEqual(observer.snapshot()[1],(-2,1,-1))
        clock.step(1)
        pose,cmd,recent=observer.snapshot()
        self.assertFalse(recent)
        self.assertEqual(cmd,(0,0,0))
        self.assertAlmostEqual(pose[2],-.5)
        clock.step(50)
        self.assertEqual(observer.snapshot()[0],pose)


APP_DIR=''
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,directory=APP_DIR,**kwargs)
    def respond_json(self,status,obj):
        body=json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type','application/json')
        self.send_header('Content-Length',str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        if not keyboard_get(self): self.respond_json(404,{'error':'missing'})
    def do_POST(self):
        if not keyboard_post(self): self.respond_json(404,{'error':'missing'})
    def log_message(self,*args):
        pass


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global APP_DIR
        cls.tmp=tempfile.TemporaryDirectory()
        APP_DIR=cls.tmp.name
        (Path(APP_DIR)/'keyboard.html').write_text('<h1>keyboard viewer</h1>',encoding='utf-8')
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()
        cls.base='http://127.0.0.1:'+str(cls.server.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)
        cls.tmp.cleanup()

    def get(self,path):
        with urlopen(self.base+path,timeout=2) as res:
            s=res.read().decode('utf-8')
            return json.loads(s) if res.headers.get_content_type()=='application/json' else s

    def post(self,obj,headers=None):
        request=Request(self.base+'/api/keyboard/feedback',
                        data=json.dumps(obj).encode('utf-8'),method='POST',
                        headers={'Content-Type':'application/json',**(headers or {})})
        return urlopen(request,timeout=2)

    def test_page_and_feedback_isolation(self):
        self.assertIn('keyboard viewer',self.get('/keyboard'))
        self.assertFalse(self.get('/api/keyboard/feedback')['online'])
        payload=dict(x=100000,y=-1234,yaw=25,vx=-1,vy=.5,wz=3.25,
                     command_recent=True,topic='/my_existing_keyboard_twist')
        with self.post(payload) as response:
            self.assertEqual(json.load(response)['source_type'],'KEYBOARD')
        feedback=self.get('/api/keyboard/feedback')
        self.assertTrue(feedback['online'])
        self.assertEqual(feedback['sample']['x'],100000)
        self.assertEqual(feedback['sample']['topic'],'/my_existing_keyboard_twist')

    def test_reject_nan_and_cross_origin(self):
        valid=dict(x=0,y=0,yaw=0,vx=0,vy=0,wz=0,
                   command_recent=False,topic='/cmd_vel')
        for bad in ({**valid,'vy':float('nan')},{**valid,'x':True},{**valid,'command_recent':1}):
            with self.assertRaises(HTTPError) as e: self.post(bad)
            self.assertEqual(e.exception.code,400)
        with self.assertRaises(HTTPError) as e:
            self.post(valid,{'Origin':'https://evil.invalid'})
        self.assertEqual(e.exception.code,403)


if __name__=='__main__':
    unittest.main()
