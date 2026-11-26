import unittest
import math
from segment_executor import Executor, Pose, Limits, Sample

L=Limits(.02,.04,.01,.02,.12,100,30,80,50,80,3)
class Test(unittest.TestCase):
    def setUp(self):
        self.e=Executor('demo',Pose(1,1,0),L,0)
        self.e.link(0,7,True,True)
        self.sample(0,1)
        self.start=Pose(1,1,0); self.goal=Pose(1.5,1,0)
    def sample(self,t,source,x=0,y=0,v=0,imu=True,reached=True):
        return self.e.telemetry(t,Sample(source,Pose(x,y,0),v,0,0,reached,imu))
    def load(self,route=None,check=lambda a,b:True):
        return self.e.load_route(1,'demo','QR',self.goal,route or [self.start,self.goal],check)
    def issue(self):
        self.load(); return self.e.issue(2)
    def ack(self):
        self.e.feedback(3,1,0x90,2,0,0x83)
    def complete(self):
        self.ack(); self.e.feedback(4,1,0x94,3)
    def settle(self,x=.5):
        for i in range(3): self.sample(5+i,4+i,x)
    def test_tf(self):
        p=Pose(.4,-.3,math.pi/2); x=Pose(.2,.5,-.3)
        actual=p.inverse().compose(p.compose(x))
        for a,b in zip((actual.x,actual.y,actual.yaw),(x.x,x.y,x.yaw)): self.assertAlmostEqual(a,b)
    def test_target(self):
        d=self.issue(); self.assertEqual(d['target_odom'],Pose(.5,0,0)); self.assertFalse(d['hardware_authorized'])
    def test_no_double_issue(self):
        self.issue(); self.assertIsNone(self.e.issue(3))
    def test_complete_before_ack(self):
        self.issue(); self.assertFalse(self.e.feedback(3,1,0x94,2)); self.assertEqual(self.e.state,'WAIT_ACK')
    def test_wrong_command(self):
        self.issue(); self.assertFalse(self.e.feedback(3,2,0x90,2,0,0x83))
    def test_wrong_type(self):
        self.issue(); self.assertFalse(self.e.feedback(3,1,0x90,2,0,0x82))
    def test_wrong_status(self):
        self.issue(); self.e.feedback(3,1,0x90,2,1,0x83); self.assertEqual(self.e.state,'FAULT')
    def test_ack_timeout(self):
        self.issue(); self.assertEqual(self.e.tick(32),'FAULT')
    def test_result_timeout(self):
        self.issue(); self.ack(); self.assertEqual(self.e.tick(83),'FAULT')
    def test_stale_feedback(self):
        self.issue(); self.assertFalse(self.e.feedback(3,1,0x90,0,0,0x83))
    def test_no_advance_on_ack(self):
        self.issue(); self.ack(); self.assertEqual(self.e.state,'WAIT_RESULT')
    def test_no_advance_on_completion(self):
        self.issue(); self.complete(); self.assertEqual(self.e.state,'WAIT_SETTLE')
    def test_settled(self):
        self.issue(); self.complete(); self.settle(); self.assertEqual(self.e.state,'WAIT_VISION')
    def test_moving_resets_count(self):
        self.issue(); self.complete(); self.sample(5,4,.5); self.sample(6,5,.5,v=.1); self.sample(7,6,.5); self.sample(8,7,.5); self.assertEqual(self.e.state,'WAIT_SETTLE')
    def test_duplicate_sample(self):
        self.issue(); self.complete(); self.sample(5,4,.5); self.sample(6,4,.5); self.sample(7,4,.5); self.assertEqual(self.e.stable,1)
    def test_deviation(self):
        self.issue(); self.complete(); self.sample(5,4,.3); self.assertEqual(self.e.reason,'EXCESS_POSITION_DEVIATION')
    def test_link_loss(self):
        self.issue(); self.e.link(3,7,False,False); self.assertEqual(self.e.state,'FAULT')
    def test_session_change(self):
        self.issue(); self.e.link(3,8,True,True); self.assertEqual(self.e.reason,'PEER_SESSION_CHANGED')
    def test_diagonal(self):
        with self.assertRaises(ValueError): self.load([self.start,Pose(1.2,1.2,0),self.goal])
    def test_corridor_rejected(self):
        self.assertFalse(self.load(check=lambda a,b:False)); self.assertEqual(self.e.state,'FAULT')
    def test_map_changed(self):
        self.assertFalse(self.e.load_route(1,'other','QR',self.goal,[self.start,self.goal],lambda a,b:True))
    def test_vision_token(self):
        self.issue(); self.complete(); self.settle(); self.assertFalse(self.e.vision_done(8,2,'QR',True)); self.assertEqual(self.e.state,'WAIT_VISION')
    def test_two_stages(self):
        self.issue(); self.complete(); self.settle(.49); self.assertTrue(self.e.vision_done(8,1,'QR',True,'156+123+516+231',True))
        actual=self.e.stop_pose; goal=Pose(actual.x,1.4,0)
        self.assertTrue(self.e.load_route(9,'demo','RAW',goal,[actual,goal],lambda a,b:True)); self.assertEqual(self.e.issue(10)['cmd_id'],2)
        self.e.feedback(11,2,0x90,7,0,0x83); self.e.feedback(12,2,0x94,8)
        for i in range(3): self.sample(13+i,9+i,.49,.4)
        self.assertTrue(self.e.vision_done(16,2,'RAW',True)); self.assertEqual(self.e.state,'DONE')
    def test_replan_after_axis(self):
        self.goal=Pose(1.5,1.5,0); self.load([self.start,Pose(1.5,1,0),self.goal]); self.e.issue(2); self.complete(); self.settle(.49)
        self.assertEqual(self.e.state,'NEED_REPLAN'); self.assertIsNone(self.e.issue(8))
        actual=self.e.stop_pose
        self.assertTrue(self.e.load_route(9,'demo','QR',self.goal,[actual,Pose(1.5,1,0),self.goal],lambda a,b:True))
    def test_pitch_not_ready(self):
        self.issue(); self.complete(); self.settle(); self.assertFalse(self.e.vision_done(8,1,'QR',True,'156+123+516+231',False)); self.assertEqual(self.e.state,'WAIT_VISION')
    def test_invalid_qr(self):
        self.issue(); self.complete(); self.settle(); self.e.vision_done(8,1,'QR',True,'111+123+111+123',True); self.assertEqual(self.e.reason,'INVALID_QR')
    def test_invalid_imu(self):
        self.issue(); self.sample(3,2,imu=False); self.assertEqual(self.e.tick(4),'FAULT')
    def test_vision_timeout(self):
        self.issue(); self.complete(); self.settle(); self.assertEqual(self.e.tick(87),'FAULT')
    def test_vision_failure(self):
        self.issue(); self.complete(); self.settle(); self.e.vision_done(8,1,'QR',False); self.assertEqual(self.e.state,'FAULT')
    def test_actual_origin_required(self):
        with self.assertRaises(ValueError): self.load([Pose(1.01,1,0),self.goal])
    def test_unexpected_motion(self):
        self.issue(); self.complete(); self.settle(); self.sample(8,7,.5,v=.1); self.assertEqual(self.e.state,'FAULT')

if __name__=='__main__': unittest.main()
