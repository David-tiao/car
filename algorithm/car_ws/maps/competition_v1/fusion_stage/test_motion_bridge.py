import json
import os
import pty
import unittest
from motion_bridge_core import MotionBridge
from motion_serial import PosixSerial
from motion_link import Frame, ARM, HEARTBEAT, TRANSLATE, TURN, STOP, STATUS, IDLE, MOVING, DONE, OK

BASE=1_800_000_000_000_000_000

def wall(t):return BASE+round(t*1e9)

class Wire:
    def __init__(self):self.sent=[];self.inbox=b'';self.closed=False;self.partial=False;self.broken=False
    def read(self,n):
        if self.broken:raise OSError('disconnect')
        data,self.inbox=self.inbox[:n],self.inbox[n:];return data
    def write(self,data):
        self.sent.append(Frame.decode(data));return len(data)-1 if self.partial else len(data)
    def close(self):self.closed=True
    def reply(self,state,action,pose=(0,0,0),reason=OK,session=23):
        req=self.sent[-1];self.inbox+=Frame(STATUS,session,req.seq,(state,action,*pose,reason)).encode()

class BridgeTests(unittest.TestCase):
    def setUp(self):self.w=Wire();self.b=MotionBridge(23,self.w,operator_enabled=True);self.seq=0
    def permit(self,t,gid=0,kind='IDLE',start=None,goal=None,**kw):
        self.seq+=1
        p=dict(schema='car_motion_safety_v1',session=23,seq=self.seq,stamp_ns=wall(t),frame_id='odom',allow=True,
               local_grant_confirmed=True,tf_verified=True,goal_id=gid,kind=kind,start=start,goal=goal,
               center_bounds_mm=[-10,-10,1000,1000],turn_clear=True);p.update(kw)
        return self.b.safety(json.dumps(p),t,wall(t))
    def goal(self,t,gid=1,start=(0,0,0),goal=(850,0,0),kind='TRANSLATE',**kw):
        g=dict(schema='car_motion_goal_v1',session=23,goal_id=gid,stamp_ns=wall(t),frame_id='odom',kind=kind,start=start,goal=goal);g.update(kw)
        return self.b.goal(json.dumps(g),t,wall(t))
    def arm(self):
        self.assertTrue(self.permit(0));self.b.tick(0);self.assertEqual(self.w.sent[-1].kind,ARM)
        self.w.reply(IDLE,0);self.b.tick(.01);self.assertEqual(self.b.pi.phase,'IDLE')
    def move(self):
        self.arm();self.assertTrue(self.permit(.02,1,'TRANSLATE',[0,0,0],[850,0,0]));self.assertTrue(self.goal(.02));self.b.tick(.03)
        self.assertEqual(self.w.sent[-1].args[:2],(300,0));self.w.reply(MOVING,2);self.b.tick(.04)
    def test_disabled_never_sends(self):
        self.b=MotionBridge(23,self.w);self.permit(0);self.b.tick(0);self.assertFalse(self.w.sent)
    def test_no_permit_never_sends(self):self.b.tick(0);self.assertFalse(self.w.sent)
    def test_wrong_session_never_arms(self):self.assertFalse(self.permit(0,session=99));self.b.tick(0);self.assertFalse(self.w.sent)
    def test_no_grant_never_arms(self):self.assertFalse(self.permit(0,local_grant_confirmed=False));self.assertTrue(self.w.closed);self.assertFalse(self.w.sent)
    def test_stale_safety_never_arms(self):self.assertFalse(self.permit(.3,stamp_ns=wall(0)));self.assertFalse(self.w.sent)
    def test_future_safety_never_arms(self):self.assertFalse(self.permit(0,stamp_ns=wall(.1)));self.assertFalse(self.w.sent)
    def test_idle_heartbeat_and_pose(self):
        self.arm();self.b.tick(.11);self.assertEqual(self.w.sent[-1].kind,HEARTBEAT)
        self.w.reply(IDLE,0,(1,2,0));self.b.tick(.12);s=self.b.status(.12)
        self.assertEqual(s['pose_mm_mdeg'],[1,2,0]);self.assertFalse(s['velocity_available']);self.assertEqual(s['feedback_seq'],2)
    def test_full_motion_actual_residual(self):
        self.move();self.b.tick(.14);self.w.reply(DONE,2,(296,0,0));self.b.tick(.15)
        self.assertEqual(self.w.sent[-1].args[:2],(300,0));self.w.reply(MOVING,4,(296,0,0));self.b.tick(.16)
        self.permit(.20,1,'TRANSLATE',[0,0,0],[850,0,0]);self.b.tick(.26);self.w.reply(DONE,4,(598,0,0));self.b.tick(.27)
        self.assertEqual(self.w.sent[-1].args[:2],(252,0));self.w.reply(MOVING,6,(598,0,0));self.b.tick(.28)
        self.b.tick(.38);self.w.reply(DONE,6,(850,0,0));self.b.tick(.39)
        self.assertEqual(self.b.status(.39)['completed_goal_id'],1);self.assertEqual(self.b.pi.completed,1)
        self.assertFalse(self.goal(.40));self.assertFalse(self.b.closed)
    def test_no_goal_without_scope(self):self.arm();self.assertFalse(self.goal(.02));self.assertEqual(self.w.sent[-1].kind,STOP)
    def test_stale_goal_ignored(self):self.arm();self.assertFalse(self.goal(.3,stamp_ns=wall(0)));self.assertIsNone(self.b.active_goal)
    def test_invalid_diagonal(self):
        self.arm();self.assertFalse(self.goal(.02,goal=(100,100,0)));self.assertTrue(self.b.closed)
    def test_wrong_frame(self):self.arm();self.assertFalse(self.goal(.02,frame_id='map'));self.assertTrue(self.b.closed)
    def test_revoke_in_motion(self):
        self.move();self.assertTrue(self.permit(.05,allow=False,seq=1));self.assertEqual(self.w.sent[-1].kind,STOP);self.assertTrue(self.b.closed)
        self.assertFalse(self.b.status(.05)['physical_stop_confirmed']);n=len(self.w.sent);self.b.tick(.1);self.assertEqual(n,len(self.w.sent))
    def test_expired_permit_stops(self):self.move();self.b.tick(.28);self.assertEqual(self.w.sent[-1].kind,STOP);self.assertTrue(self.b.closed)
    def test_replay_cannot_refresh(self):
        self.move();self.assertFalse(self.permit(.2,1,'TRANSLATE',[0,0,0],[850,0,0],seq=2));self.b.tick(.28);self.assertTrue(self.b.closed)
    def test_scope_change_stops(self):self.move();self.assertFalse(self.permit(.05));self.assertEqual(self.w.sent[-1].kind,STOP)
    def test_corridor_shrink_stops(self):
        self.move();self.assertFalse(self.permit(.05,1,'TRANSLATE',[0,0,0],[850,0,0],center_bounds_mm=[-10,-10,200,20]));self.assertTrue(self.b.closed)
    def test_out_of_bounds_feedback_stops(self):
        self.move();self.b.tick(.14);self.w.reply(MOVING,2,(1,-20,0));self.b.tick(.15);self.assertEqual(self.w.sent[-1].kind,STOP)
    def test_wrong_done_stops(self):
        self.move();self.b.tick(.14);self.w.reply(DONE,2,(200,0,0));self.b.tick(.15);self.assertTrue(self.b.closed);self.assertEqual(self.w.sent[-1].kind,STOP)
    def test_turn_without_clearance(self):
        self.arm();self.permit(.02,1,'TURN_90',[0,0,0],[0,0,90000],turn_clear=False)
        self.assertFalse(self.goal(.02,goal=(0,0,90000),kind='TURN_90'));self.assertTrue(self.b.closed)
    def test_turn_with_clearance(self):
        self.arm();self.permit(.02,1,'TURN_90',[0,0,0],[0,0,90000]);self.assertTrue(self.goal(.02,goal=(0,0,90000),kind='TURN_90'));self.b.tick(.03);self.assertEqual(self.w.sent[-1].kind,TURN)
    def test_partial_write(self):self.w.partial=True;self.permit(0);self.b.tick(0);self.assertTrue(self.w.closed);self.assertFalse(self.b.stop_written)
    def test_disconnect(self):self.move();self.w.broken=True;self.b.tick(.05);self.assertTrue(self.w.closed);self.assertFalse(self.b.status(.05)['pose_valid'])
    def test_duplicate_json_rejected(self):self.assertFalse(self.b.safety('{"session":23,"session":23}',0,wall(0)));self.assertTrue(self.b.closed)
    def test_shutdown_stop(self):self.move();self.b.shutdown();self.assertEqual(self.w.sent[-1].kind,STOP);self.assertTrue(self.b.closed)
    def test_heartbeat_retry_timeout(self):
        self.permit(0);self.b.tick(0);first=self.w.sent[-1];self.b.tick(.13);self.assertEqual(first,self.w.sent[-1]);self.permit(.2);self.b.tick(.26);self.assertEqual(first,self.w.sent[-1]);self.b.tick(.39);self.assertEqual(self.w.sent[-1].kind,STOP);self.assertTrue(self.b.closed)
    def test_wrong_session_reply_ignored(self):
        self.permit(0);self.b.tick(0);self.w.reply(IDLE,0,session=99);self.b.tick(.01);self.assertEqual(self.b.pi.phase,'ARMING');self.assertFalse(self.b.status(.01)['pose_valid'])
    def test_fragmented_reply(self):
        self.permit(0);self.b.tick(0);self.w.reply(IDLE,0);wire=self.w.inbox;self.w.inbox=wire[:17];self.b.tick(.01);self.assertFalse(self.b.status(.01)['pose_valid']);self.w.inbox=wire[17:];self.b.tick(.02);self.assertTrue(self.b.status(.02)['pose_valid'])
    def test_clock_regression(self):self.move();self.b.tick(.02);self.assertEqual(self.w.sent[-1].kind,STOP);self.assertTrue(self.b.closed)

class PtyTests(unittest.TestCase):
    def test_real_posix_pty_fragmentation_and_shutdown(self):
        master,slave=pty.openpty();port=os.ttyname(slave);os.set_blocking(master,False)
        transport=PosixSerial(port);b=MotionBridge(23,transport,operator_enabled=True)
        try:
            p=dict(schema='car_motion_safety_v1',session=23,seq=1,stamp_ns=wall(0),frame_id='odom',allow=True,local_grant_confirmed=True,tf_verified=True,goal_id=0,kind='IDLE',start=None,goal=None,center_bounds_mm=[-10,-10,100,100],turn_clear=False)
            self.assertTrue(b.safety(json.dumps(p),0,wall(0)));b.tick(0)
            req=Frame.decode(os.read(master,38));self.assertEqual(req.kind,ARM)
            reply=Frame(STATUS,23,req.seq,(IDLE,0,0,0,0,OK)).encode()
            os.write(master,reply[:10]);b.tick(.01);self.assertFalse(b.status(.01)['pose_valid'])
            os.write(master,reply[10:]);b.tick(.02);self.assertTrue(b.status(.02)['pose_valid'])
            b.shutdown();self.assertEqual(Frame.decode(os.read(master,38)).kind,STOP)
        finally:b.shutdown();os.close(slave);os.close(master)
    def test_exclusive_open(self):
        master,slave=pty.openpty();port=os.ttyname(slave);a=PosixSerial(port)
        try:
            with self.assertRaises(OSError):PosixSerial(port)
        finally:a.close();os.close(slave);os.close(master)

if __name__=='__main__':unittest.main()
