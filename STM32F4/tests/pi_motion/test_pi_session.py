from pathlib import Path
import sys
TEST_DIR = Path(__file__).resolve().parent
STM = TEST_DIR.parents[1]
REPO = STM.parent
FUSION = REPO / "algorithm/car_ws/maps/competition_v1/fusion_stage"
sys.path.insert(0, str(FUSION))
import unittest
from motion_link import *
from pi_session import PiSession

class SessionTests(unittest.TestCase):
    def setUp(self):
        self.checked=[];self.p=PiSession(23,self.clearance)
    def clearance(self,a,b,k):self.checked.append((a,b,k));return True
    def reply(self,wire,t,state,action,pose=(0,0,0),reason=OK,session=23):
        f=Frame.decode(wire);return self.p.receive(Frame(STATUS,session,f.seq,(state,action,*pose,reason)).encode(),t)
    def arm(self,pose=(0,0,0)):
        self.p.start(True,True);w=self.p.tick(0,True);self.assertEqual(Frame.decode(w).kind,ARM)
        self.assertTrue(self.reply(w,.01,IDLE,0,pose));self.assertEqual(self.p.phase,'IDLE')
    def test_no_start_without_grant(self):
        self.assertIsNone(self.p.tick(0,True));self.assertRaises(ValueError,self.p.start,False,True)
    def test_chunks_from_feedback(self):
        self.arm();self.assertTrue(self.p.queue_action('TRANSLATE',(0,0,0),(850,0,0)))
        w=self.p.tick(.02,True);self.assertEqual(Frame.decode(w).args[:2],(300,0));self.reply(w,.03,MOVING,2)
        self.assertIsNone(self.p.tick(.04,True));h=self.p.tick(.13,True);self.assertEqual(Frame.decode(h).kind,HEARTBEAT)
        self.reply(h,.14,DONE,2,(296,0,0));w=self.p.tick(.15,True);self.assertEqual(Frame.decode(w).args[:2],(300,0));self.reply(w,.16,MOVING,4,(296,0,0))
        h=self.p.tick(.26,True);self.reply(h,.27,DONE,4,(598,0,0));w=self.p.tick(.28,True);self.assertEqual(Frame.decode(w).args[:2],(252,0));self.reply(w,.29,MOVING,6,(598,0,0))
        h=self.p.tick(.39,True);self.reply(h,.40,DONE,6,(850,0,0));self.assertEqual(self.p.phase,'IDLE');self.assertEqual(self.p.completed,1);self.assertEqual(len(self.checked),3)
    def test_turn(self):
        self.arm();self.p.queue_action('TURN_90',(0,0,0),(0,0,90000));w=self.p.tick(.02,True)
        self.assertEqual(Frame.decode(w).args[0],90000);self.assertTrue(self.reply(w,.03,DONE,2,(0,0,90000)));self.assertEqual(self.p.completed,1)
    def test_reply_timeout(self):
        self.p.start(True,True);w=self.p.tick(0,True);self.assertEqual(self.p.tick(.13,True),w);self.assertEqual(self.p.tick(.26,True),w)
        self.assertEqual(Frame.decode(self.p.tick(.39,True)).kind,STOP);self.assertEqual(self.p.reason,'REPLY_TIMEOUT');self.assertIsNone(self.p.tick(.40,True))
    def test_wrong_status(self):
        self.arm();self.p.queue_action('TRANSLATE',(0,0,0),(100,0,0));w=self.p.tick(.02,True)
        self.assertFalse(self.reply(w,.03,MOVING,2,session=99));self.assertIsNotNone(self.p.host.pending)
        self.assertFalse(self.reply(w,.04,DONE,2,(0,0,0)));self.assertEqual(self.p.reason,'DONE_POSE_MISMATCH');self.assertEqual(Frame.decode(self.p.tick(.05,True)).kind,STOP)
    def test_clearance_revoked(self):
        self.arm();self.p.clearance=lambda *_:False;self.p.queue_action('TRANSLATE',(0,0,0),(100,0,0))
        self.assertEqual(Frame.decode(self.p.tick(.02,True)).kind,STOP);self.assertEqual(self.p.reason,'CLEARANCE_REJECTED')
    def test_supervisor_revoked_during_pending(self):
        self.arm();self.p.queue_action('TRANSLATE',(0,0,0),(100,0,0));self.p.tick(.02,True)
        self.assertEqual(Frame.decode(self.p.tick(.03,False)).kind,STOP);self.assertEqual(self.p.reason,'SUPERVISOR_REVOKED')
    def test_odom_mismatch(self):
        self.arm((20,0,0));self.assertFalse(self.p.queue_action('TRANSLATE',(0,0,0),(100,0,0)));self.assertEqual(self.p.reason,'START_POSE_MISMATCH')
    def test_micro_residual(self):
        self.arm((-5,0,0));self.p.queue_action('MICRO_TRANSLATE',(0,0,0),(20,0,0));self.assertEqual(Frame.decode(self.p.tick(.02,True)).kind,STOP);self.assertEqual(self.p.reason,'MICRO_RESIDUAL_LIMIT')
    def test_clock(self):
        self.arm();self.assertEqual(Frame.decode(self.p.tick(.005,True)).kind,STOP);self.assertEqual(self.p.reason,'INVALID_CLOCK')
    def test_late_reply(self):
        self.p.start(True,True);w=self.p.tick(0,True);self.assertFalse(self.reply(w,.4,IDLE,0));self.assertEqual(self.p.reason,'LATE_REPLY')
    def test_unexpected_moving_status(self):
        self.arm();h=self.p.tick(.11,True);self.assertEqual(Frame.decode(h).kind,HEARTBEAT)
        self.assertFalse(self.reply(h,.12,MOVING,99));self.assertEqual(self.p.reason,'UNEXPECTED_IDLE_STATUS')
    def test_small_micro_not_claimed(self):
        self.arm();self.assertRaises(ValueError,self.p.queue_action,'MICRO_TRANSLATE',(0,0,0),(5,0,0));self.assertEqual(self.p.completed,0)
    def test_fragmented_status(self):
        self.p.start(True,True);w=self.p.tick(0,True);r=Frame(STATUS,23,Frame.decode(w).seq,(IDLE,0,0,0,0,OK)).encode()
        self.assertFalse(self.p.receive(r[:17],.01));self.assertTrue(self.p.receive(r[17:],.02));self.assertEqual(self.p.phase,'IDLE')
if __name__=='__main__':unittest.main()
