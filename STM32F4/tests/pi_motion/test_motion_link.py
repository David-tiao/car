from pathlib import Path
import sys
TEST_DIR = Path(__file__).resolve().parent
STM = TEST_DIR.parents[1]
REPO = STM.parent
FUSION = REPO / "algorithm/car_ws/maps/competition_v1/fusion_stage"
sys.path.insert(0, str(FUSION))
import ctypes as C
import pathlib, subprocess, tempfile, unittest
from motion_link import *
ROOT=pathlib.Path(__file__).resolve().parent
class CF(C.Structure):
    _fields_=[('kind',C.c_uint8),('session',C.c_uint32),('seq',C.c_uint32),('a',C.c_int32*6)]
class FB(C.Structure):
    _fields_=[('x',C.c_int32),('y',C.c_int32),('yaw',C.c_int32),('sample_ms',C.c_uint32),('valid',C.c_uint8),('stationary',C.c_uint8)]
class Core(C.Structure):
    _fields_=[(n,C.c_uint32) for n in ('session','last_session','last_seq','action_seq','last_rx','started','still_since','last_sample')]+[(n,C.c_uint8) for n in ('state','reason','count','have_sample','last_reason')]+[(n,C.c_int32) for n in ('gx','gy','gyaw')]+[('last_command',CF)]
class Stream(C.Structure):
    _fields_=[('bytes',C.c_uint8*38),('n',C.c_uint8)]
class Q(C.Structure):
    _fields_=[('session',C.c_uint32),('started',C.c_uint32),('state',C.c_uint8)]
class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();out=pathlib.Path(cls.temp.name)/'review.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-shared','-fPIC','-I'+str(STM/'App'),str(STM/'App/motion_link.c'),str(ROOT/'qr_gate.c'),'-o',str(out)],check=True)
        cls.lib=C.CDLL(str(out)); p=C.POINTER
        for name,args,result in [
            ('ml_encode',[p(CF),p(C.c_uint8)],C.c_int),('ml_decode',[p(C.c_uint8),p(CF)],C.c_int),
            ('ml_init',[p(Core)],None),('ml_grant',[p(Core),C.c_uint32,C.c_uint8,C.c_uint32],C.c_int),
            ('ml_tick',[p(Core),p(FB),C.c_uint8,C.c_uint32],None),
            ('ml_receive',[p(Core),p(CF),p(FB),C.c_uint8,C.c_uint32,p(CF)],C.c_int),
            ('ml_intent',[p(Core),p(C.c_int32)],C.c_int),('ml_stream_init',[p(Stream)],None),
            ('ml_stream_push',[p(Stream),C.c_uint8,p(CF)],C.c_int),
            ('q_begin',[p(Q),C.c_uint32,C.c_uint32],None),('q_event',[p(Q),C.c_uint8,C.c_uint32,C.c_uint8,C.c_uint32],None)]:
            f=getattr(cls.lib,name);f.argtypes=args;f.restype=result
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def setUp(self):self.c=Core();self.lib.ml_init(C.byref(self.c));self.fb=FB(0,0,0,0,1,1);self.h=HostReview(11)
    def grant(self):self.assertEqual(self.lib.ml_grant(C.byref(self.c),11,1,0),1)
    def packet(self,frame,now,enable=1,update=True):
        if update:self.fb.sample_ms=now
        cf=CF();reply=CF();wire=frame if isinstance(frame,bytes) else frame.encode()
        self.assertTrue(self.lib.ml_decode((C.c_uint8*38).from_buffer_copy(wire),C.byref(cf)))
        ok=self.lib.ml_receive(C.byref(self.c),C.byref(cf),C.byref(self.fb),enable,now,C.byref(reply))
        if not ok:return None
        out=(C.c_uint8*38)();self.assertTrue(self.lib.ml_encode(C.byref(reply),out));return bytes(out)
    def command(self,kind,now,*args):
        w=self.h.request(kind,*args);r=self.packet(w,now);self.assertIsNotNone(r);self.assertTrue(self.h.accept(r));return Frame.decode(r)
    def arm(self):self.grant();self.command(ARM,0)
    def tick(self,t,update=True,enable=1):
        if update:self.fb.sample_ms=t
        self.lib.ml_tick(C.byref(self.c),C.byref(self.fb),enable,t)
    def intent(self):
        a=(C.c_int32*3)();return tuple(a) if self.lib.ml_intent(C.byref(self.c),a) else None
    def test_codec(self):
        for args in [(0,)*6,(-2147483648,2147483647,-300,90000,1,-1)]:
            f=Frame(TRANSLATE,0xffffffff,0x7fffffff,args);wire=f.encode();cf=CF();buf=(C.c_uint8*38).from_buffer_copy(wire);self.assertTrue(self.lib.ml_decode(buf,C.byref(cf)));out=(C.c_uint8*38)();self.assertTrue(self.lib.ml_encode(C.byref(cf),out));self.assertEqual(bytes(out),wire)
            buf[15]^=1;self.assertFalse(self.lib.ml_decode(buf,C.byref(cf)))
    def test_stream(self):
        a=Frame(ARM,11,1).encode();b=Frame(HEARTBEAT,11,2).encode();bad=bytearray(a);bad[19]^=3;data=b'junkM'+bad+a+b
        s=Stream();self.lib.ml_stream_init(C.byref(s));out=[];cf=CF();p=StreamDecoder();py=[]
        for byte in data:
            if self.lib.ml_stream_push(C.byref(s),byte,C.byref(cf)):out.append(cf.seq)
            py.extend(p.feed(bytes([byte])))
        self.assertEqual(out,[1,2]);self.assertEqual([x.seq for x in py],out)
    def test_no_remote_arm(self):self.assertIsNone(self.packet(Frame(ARM,11,1),0));self.assertIsNone(self.intent())
    def test_grant_requires_local(self):self.assertFalse(self.lib.ml_grant(C.byref(self.c),11,0,0));self.assertEqual(self.c.session,0)
    def test_move_and_stop_confirmation(self):
        self.arm();r=self.command(TRANSLATE,10,100,0);self.assertEqual(r.args[:2],(MOVING,2));self.assertEqual(self.intent(),(100,0,0));self.fb.x=100
        for t in (20,70,120):self.tick(t)
        self.assertEqual(self.c.state,MOVING);self.tick(170);self.assertEqual(self.c.state,DONE);self.assertIsNone(self.intent());r=self.command(HEARTBEAT,180);self.assertEqual(r.args[:2],(DONE,2));self.assertIsNone(self.h.action)
    def test_body_axis_and_turn(self):
        self.fb.yaw=90000;self.arm();self.command(TRANSLATE,1,0,200);self.assertEqual(self.intent(),(-200,0,90000))
        self.fb.x=-200
        for t in (10,60,110,160):self.tick(t)
        self.command(HEARTBEAT,170);self.command(TURN,180,90000);self.assertEqual(self.intent(),(-200,0,-180000))
    def test_replay_no_reexecution(self):
        self.arm();w=self.h.request(TRANSLATE,100,0);r=self.packet(w,10);self.assertEqual(self.h.retry(),w);self.assertTrue(self.h.accept(r));self.assertEqual(self.intent(),(100,0,0));r=Frame.decode(self.packet(w,100));self.assertEqual(r.args[5],OK);self.assertEqual(self.c.last_rx,10);self.assertEqual(self.intent(),(100,0,0));self.tick(510);self.assertEqual(self.c.reason,TIMEOUT);self.assertIsNone(self.intent())
    def test_rejected_replay(self):
        self.arm();f=Frame(TRANSLATE,11,2,(1,1,0,0,0,0));self.assertEqual(Frame.decode(self.packet(f,1)).args[5],BADARG);self.assertEqual(Frame.decode(self.packet(f,2)).args[5],BADARG);self.assertIsNone(self.intent())
    def test_bad_sequence(self):
        self.arm();self.assertEqual(Frame.decode(self.packet(Frame(TRANSLATE,11,3,(10,0,0,0,0,0)),1)).args[5],BADSEQ);self.assertIsNone(self.intent())
    def test_invalid_move(self):
        self.arm()
        for seq,kind,args in [(2,TRANSLATE,(301,0,0,0,0,0)),(3,TURN,(180000,0,0,0,0,0)),(4,TRANSLATE,(0,0,0,0,0,0))]:self.assertEqual(Frame.decode(self.packet(Frame(kind,11,seq,args),seq)).args[5],BADARG)
        self.assertIsNone(self.intent())
    def test_second_move_busy(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.assertEqual(Frame.decode(self.packet(Frame(TRANSLATE,11,3,(10,0,0,0,0,0)),2)).args[5],BUSY);self.assertEqual(self.intent(),(100,0,0))
        with self.assertRaises(RuntimeError):self.h.request(TRANSLATE,10,0)
    def test_stop_preempts(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.packet(Frame(STOP,11,1),2);self.assertEqual(self.c.reason,STOPPED);self.assertIsNone(self.intent());self.assertFalse(self.lib.ml_grant(C.byref(self.c),11,1,3));self.assertTrue(self.lib.ml_grant(C.byref(self.c),12,1,3))
    def test_interlock(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.tick(2,enable=0);self.assertEqual(self.c.reason,INTERLOCK);self.assertIsNone(self.intent())
    def test_feedback_stale(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.tick(102,update=False);self.assertEqual(self.c.reason,FEEDBACK);self.assertIsNone(self.intent())
    def test_repeated_sample_no_done(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.fb.x=100;self.tick(10)
        for t in (20,30,40,50,60):self.tick(t,update=False)
        self.assertEqual(self.c.count,1);self.assertEqual(self.c.state,MOVING)
    def test_not_stationary(self):
        self.arm();self.command(TRANSLATE,1,100,0);self.fb.x=100;self.fb.stationary=0
        for t in (10,60,110,160,210):self.tick(t)
        self.assertEqual(self.c.state,MOVING)
    def test_move_timeout_with_heartbeat(self):
        self.arm();self.command(TRANSLATE,1,100,0)
        for t in range(200,8001,200):self.command(HEARTBEAT,t)
        self.tick(8001);self.assertEqual(self.c.reason,MOVE_TIMEOUT);self.assertIsNone(self.intent())
    def test_wrong_session(self):
        self.arm();self.assertIsNone(self.packet(Frame(STOP,99,2),1));self.assertEqual(self.c.state,IDLE)
    def test_host_correlation(self):
        self.grant();w=self.h.request(ARM);self.assertFalse(self.h.accept(Frame(STATUS,99,1,(IDLE,0,0,0,0,OK)).encode()));self.assertEqual(self.h.retry(),w);self.assertTrue(self.h.accept(self.packet(w,0)))
    def test_heading_gate(self):
        self.grant();self.fb.yaw=45000;r=Frame.decode(self.packet(Frame(ARM,11,1),0));self.assertEqual(r.args[5],REJECT);self.assertIsNone(self.intent())
    def test_qr_sequence(self):
        q=Q();self.lib.q_begin(C.byref(q),23,0)
        def ev(e,s=23,t=1,still=1):self.lib.q_event(C.byref(q),e,s,still,t)
        ev(3);self.assertEqual(q.state,0);ev(1);self.assertEqual(q.state,1);ev(2,99);self.assertEqual(q.state,1);ev(3);self.assertEqual(q.state,1);ev(2);self.assertEqual(q.state,2);ev(3);self.assertEqual(q.state,3)
    def test_qr_timeout_motion_cancel(self):
        for event,t,still in [(4,5000,1),(1,1,0),(5,1,1)]:
            q=Q();self.lib.q_begin(C.byref(q),23,0);self.lib.q_event(C.byref(q),event,23,still,t);self.assertEqual(q.state,4)
    def test_split_orthogonal_action(self):
        self.assertEqual(split_orthogonal_action((0,0,90000),(850,0,90000),'TRANSLATE'),[(TRANSLATE,(0,-300)),(TRANSLATE,(0,-300)),(TRANSLATE,(0,-250))])
        self.assertEqual(split_orthogonal_action((0,0,90000),(0,0,-180000),'TURN_90'),[(TURN,(90000,))])
        self.assertEqual(split_orthogonal_action((0,0,0),(0,-20,0),'MICRO_TRANSLATE'),[(TRANSLATE,(0,-20))])
        for g,k in [((1,1,0),'TRANSLATE'),((0,0,-180000),'TURN_90'),((21,0,0),'MICRO_TRANSLATE')]:
            with self.assertRaises(ValueError):split_orthogonal_action((0,0,0),g,k)
    def test_malformed_host_status(self):
        self.grant();w=self.h.request(ARM);self.assertFalse(self.h.accept(Frame(STATUS,11,1,(IDLE,0,-2**31,-2**31,-2**31,OK)).encode()));self.assertTrue(self.h.failed)
    def test_clock_wrap(self):
        self.assertTrue(self.lib.ml_grant(C.byref(self.c),11,1,0xffffff00));self.fb.sample_ms=0xffffff00
        self.assertTrue(self.h.accept(self.packet(self.h.request(ARM),0xffffff00)))
        self.tick(0x20);self.assertEqual(self.c.state,IDLE)
        self.tick(0x100);self.assertEqual(self.c.reason,TIMEOUT)
if __name__=='__main__':unittest.main()
