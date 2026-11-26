import copy
import math
import unittest
from fusion_core import Planner, SimHandoff, SimMission, orthogonal_actions, visual_step, task_code_valid, project_pixel_to_plane

# Synthetic input using exact graph/zone structure inspected at ea5a90a;
# no fixture values imply measured positions, dimensions or calibration.
NODES={c+str(r):(x,y) for c,x in zip('ABC',(.35,1.20,2.05)) for r,y in enumerate((.35,1.20,2.05),1)}
EDGES={f'{a}-{b}':(a,b) for a in NODES for b in NODES if a<b and math.isclose(math.dist(NODES[a],NODES[b]),.85)}
ZONES={'field_size':[2.4,2.4],'yellow':[[.55,.55,1,1],[1.4,.55,1.85,1],[.55,1.4,1,1.85],[1.4,1.4,1.85,1.85]],'work':[[0,.91,.15,1.49],[.91,0,1.49,.15]],'raw_disk':{'center':[1.2,2.4],'radius':.15}}
SCENE={'schema':'car_startup_scene_v1','status':'PREVIEW_READY_UNVERIFIED_TARGETS','ready':True,'scan_stamp_ns':123456789,'identity_verified':False,'navigation_authorized':False,'blocked_roads':['A1-A2','A1-B1'],'obstacles':[{'x':.35,'y':.75},{'x':.75,'y':.35}], 'qr_board':{'state':'GEOMETRY_CANDIDATE','identity_verified':False,'position_map_m':[2.4,1.2]},'raw_disk':{'state':'GEOMETRY_CANDIDATE','identity_verified':False,'position_map_m':[1.2,2.47]}}
START=(2.05,2.05,0.)

def planner(radius=.16): return Planner(NODES,EDGES,ZONES,radius,(.34,.65),(.35,.60),.03,.03)

class Tests(unittest.TestCase):
    def setUp(self): self.p=planner();self.q=self.p.plan(SCENE,START,'qr_board');self.d=self.p.plan(SCENE,START,'raw_disk')
    def test_plans_and_goal_yaw(self):
        for target,plan in [('qr_board',self.q),('raw_disk',self.d)]:
            self.assertEqual(plan['status'],'OBSERVATION_POSE_PREVIEW');self.assertFalse(plan['navigation_authorized']);self.assertFalse(plan['identity_verified'])
            self.assertEqual(plan['path'][0],START[:2]);self.assertEqual(tuple(plan['goal_pose'][:2]),plan['path'][-1])
            p=plan['goal_pose'];t=SCENE[target]['position_map_m']
            self.assertIn(round(p[2], 6), {round(v, 6) for v in (0., math.pi/2, math.pi, -math.pi/2)})
            self.assertTrue(abs(t[0]-p[0]) <= self.p.axis_tol or abs(t[1]-p[1]) <= self.p.axis_tol)
            if abs(t[0]-p[0]) <= self.p.axis_tol:
                self.assertIn(round(p[2], 6), {round(math.pi/2,6), round(-math.pi/2,6)})
            if abs(t[1]-p[1]) <= self.p.axis_tol:
                self.assertIn(round(p[2], 6), {round(0.,6), round(math.pi,6)})
            for a,b in zip(plan['path'],plan['path'][1:]):self.assertTrue(self.p.segment_clear(a,b,self.p.circles(SCENE)))
    def test_blocked_start(self):
        s=copy.deepcopy(SCENE);s['blocked_roads']=list(EDGES);s['obstacles'].append({'x':2.05,'y':2.05});self.assertEqual(self.p.plan(s,START,'qr_board')['status'],'UNSAFE_START')
    def test_no_path(self):
        s=copy.deepcopy(SCENE);s['blocked_roads']=list(EDGES);self.assertEqual(self.p.plan(s,START,'qr_board')['path'],[])
    def test_invalid_inputs(self):
        for k,v in [('ready',False),('status','CONFLICT_STOP'),('navigation_authorized',True),('scan_stamp_ns',0),('blocked_roads',['bogus']),('obstacles',[{'x':float('nan'),'y':1}])]:
            with self.subTest(k=k):
                s=copy.deepcopy(SCENE);s[k]=v
                with self.assertRaises(ValueError):self.p.plan(s,START,'qr_board')
    def test_keepout_and_body(self):
        self.assertFalse(self.p.segment_clear((.35,.35),(1.2,1.2),[]));self.assertFalse(self.p.segment_clear((.1,.35),(.1,1.2),[]));self.assertNotEqual(planner(.30).plan(SCENE,START,'qr_board')['status'],'OBSERVATION_POSE_PREVIEW')

    def test_no_slanted_goal_pose(self):
        skew=copy.deepcopy(SCENE)
        skew['qr_board']['position_map_m']=[2.32,1.28]
        out=self.p.plan(skew,START,'qr_board')
        self.assertEqual(out['status'],'NO_ORTHOGONAL_OBSERVATION_POSE')
        self.assertIsNone(out['goal_pose'])
    def test_dynamic_circle(self): self.assertFalse(self.p.segment_clear((2.05,1.0),(2.05,1.2),[((2.05,1.1),.05,'obstacle')]))
    def test_bad_calibration(self):
        with self.assertRaises(ValueError):visual_step((12,4),(0,0,0,0))
        with self.assertRaises(ValueError):visual_step((12,float('nan')),(200,0,0,200))
        self.assertLessEqual(math.hypot(*visual_step((100,100),(200,0,0,200))),.02000001)
        self.assertEqual(visual_step((4,0),(200,0,0,200)),(-.01,-0.))
    def start(self,p):
        f=SimHandoff(p,7,(200,0,0,200));f.event({'type':'ARRIVAL','session':7,'pose':p['goal_pose'],'stationary':True},0.);self.assertEqual(f.state,'CONFIRM');return f
    def vision(self,p,n,now,**kw):
        return dict(type='VISION',session=7,frame=n,stamp=now,target=p['target'],target_map_m=p['target_map_m'],stationary=True,pitch_stable=True,code='156+123+516+231',disk_geometry_and_colors=True,error_px=[0,0],**kw)
    def test_qr_ack_pitch(self):
        f=self.start(self.q)
        for i in range(1,4):f.event(self.vision(self.q,i,i*.1),i*.1)
        self.assertEqual(f.state,'WAIT_QR_ACK');f.event({'type':'QR_ACK','session':8,'code':f.code},.4);self.assertEqual(f.state,'WAIT_QR_ACK')
        f.event({'type':'QR_ACK','session':7,'code':f.code},.5);self.assertEqual(f.state,'WAIT_PITCH');f.event({'type':'PITCH_READY','session':7,'view':'RAW','stationary':True},.6);self.assertEqual(f.state,'DONE');self.assertFalse(f.output()['hardware_authorized'])
    def test_disk_step(self):
        f=self.start(self.d)
        for i in range(1,4):f.event(self.vision(self.d,i,i*.1),i*.1)
        self.assertEqual(f.state,'ALIGN');e=self.vision(self.d,4,.4);e['error_px']=[20,0];f.event(e,.4);self.assertEqual(f.state,'WAIT_STEP');self.assertIsNotNone(f.output()['sim_delta_body_m'])
        f.event({'type':'STEP_DONE','session':7,'step':0,'stationary':True},.5);self.assertEqual(f.state,'WAIT_STEP')
        f.event({'type':'STEP_DONE','session':7,'step':1,'stationary':True},.6);self.assertEqual(f.state,'ALIGN')
        for i in range(5,8):f.event(self.vision(self.d,i,i*.1+.2),i*.1+.2)
        self.assertEqual(f.state,'DONE')
    def test_replay_stale_wrong_target(self):
        f=self.start(self.q);e=self.vision(self.q,1,.1);f.event(e,.1);f.event(e,.2);self.assertEqual(f.hits,0)
        e=self.vision(self.q,2,0);f.event(e,1);self.assertEqual(f.hits,0)
        e=self.vision(self.q,3,1.1);e['target_map_m']=[0,0];f.event(e,1.1);self.assertEqual(f.hits,0)
    def test_timeout_conflict(self):
        f=self.start(self.q);f.event({'type':'TICK','session':7},16);self.assertEqual(f.state,'ABORT')
        f=self.start(self.q);f.event({'type':'TICK','session':7,'conflict':True},.1);self.assertEqual(f.state,'ABORT')
    def test_calibrated_projection(self):
        # Downward optical axis: synthetic camera at z=.5, observed plane z=.1.
        r=(1,0,0,0,-1,0,0,0,-1)
        p=project_pixel_to_plane((180,120),(200,200,160,120),(1.2,2.,.5),r,(0,0,.1),(0,0,1))
        self.assertAlmostEqual(p[0],1.24);self.assertAlmostEqual(p[1],2.);self.assertAlmostEqual(p[2],.1)
        with self.assertRaises(ValueError):project_pixel_to_plane((160,120),(200,200,160,120),(1.2,2.,.5),r,(0,0,.1),(1,0,0))
        with self.assertRaises(ValueError):project_pixel_to_plane((160,120),(200,200,160,120),(1.2,2.,.5),r,(0,0,.9),(0,0,1))
        with self.assertRaises(ValueError):project_pixel_to_plane((160,120),(200,200,160,120),(1.2,2.,.5),(1,0,0,0,1,0,0,0,-1),(0,0,.1),(0,0,1))
    def test_task_code(self):
        self.assertTrue(task_code_valid('156+123+516+231'));self.assertFalse(task_code_valid('111+123+111+321'));self.assertFalse(task_code_valid('156+123+123+321'))

class MissionTests(unittest.TestCase):
    def setUp(self):
        self.p=planner();self.m=SimMission(self.p,SCENE,START,20,(200.,0.,0.,200.))
        self.t=0.;self.seq=0;self.session=self.m.session

    def send(self,typ,**kw):
        if self.session!=self.m.session:self.session=self.m.session;self.seq=0
        self.seq+=1;self.t=round(self.t+.1,4)
        e=dict(source='SIM_TEST_ONLY',type=typ,session=self.session,seq=self.seq,stamp=self.t)
        e.update(kw)
        return self.m.event(e,self.t)

    def arrive(self):
        for _ in range(100):
            out=self.m.output();intent=out['sim_intent']
            self.assertEqual(out['phase'],'NAV')
            if intent['kind']=='VERIFY_ARRIVAL':
                return self.send('ARRIVAL',pose=intent['pose'],stationary=True)
            self.send('ACTION_DONE',action=intent['action'],pose=intent['pose'],stationary=True)
        self.fail('navigation action loop')

    def vision(self,n,**kw):
        e=dict(frame=n,pose=self.m.pose,stationary=True,pitch_stable=True,
               view='QR' if self.m.target=='qr_board' else 'RAW',target=self.m.target,
               target_map_m=self.m.plan['target_map_m'],code='156+123+516+231',
               disk_geometry_and_colors=True,error_px=[0,0]);e.update(kw)
        return self.send('VISION',**e)

    def qr_confirm(self):
        self.arrive();self.assertEqual(self.m.phase,'QR_CONFIRM')
        for n in range(1,4):self.vision(n)
        self.assertEqual(self.m.phase,'WAIT_QR_ACK')

    def qr_ack(self):
        self.qr_confirm();self.send('QR_ACK',code=self.m.code,pose=self.m.pose,stationary=True)
        self.assertEqual(self.m.phase,'WAIT_QR_DONE')

    def raw_confirm(self):
        self.qr_ack();self.send('QR_DONE',pose=self.m.pose,stationary=True)
        self.send('CAMERA_IDLE',view='RAW',pitch_stable=True,pose=self.m.pose,stationary=True)
        self.assertEqual(self.m.session,21);self.arrive();self.assertEqual(self.m.phase,'RAW_CONFIRM')
        for n in range(1,4):self.vision(n)
        self.assertEqual(self.m.phase,'RAW_ALIGN')

    def test_full_continuous_mission(self):
        self.raw_confirm();self.vision(4,error_px=[14,-6]);self.assertEqual(self.m.phase,'WAIT_STEP')
        intent=self.m.output()['sim_intent'];self.assertTrue(0. in intent['delta_body_m'])
        self.assertAlmostEqual(intent['pose'][2],math.pi/2)
        self.assertTrue(self.p.axis_segment(self.m.pose,intent['pose']))
        self.send('STEP_DONE',step=intent['step'],pose=intent['pose'],stationary=True)
        for n in range(5,8):self.vision(n,error_px=[2,-1])
        out=self.m.output();self.assertEqual(out['phase'],'READY_PICK');self.assertTrue(out['qr_handoff_complete'])
        self.assertTrue(out['sim_ready_pick']);self.assertEqual(len(self.m.plans),2)
        self.assertFalse(out['hardware_authorized']);self.assertFalse(out['navigation_authorized']);self.assertFalse(out['identity_verified'])
        self.assertIsNone(out['sim_intent'])

    def test_orthogonal_route_and_heading(self):
        plan=self.p.plan(SCENE,(2.08,2.08,.02),'qr_board')
        self.assertEqual(plan['status'],'OBSERVATION_POSE_PREVIEW')
        for a,b in zip(plan['path'],plan['path'][1:]):self.assertTrue(self.p.axis_segment(a,b))
        prev=[*plan['path'][0],plan['start_heading_rad']]
        for act in plan['actions']:
            pose=act['pose']
            if act['kind']=='TRANSLATE':
                self.assertTrue(self.p.axis_segment(prev,pose));self.assertAlmostEqual(prev[2],pose[2])
            else:
                self.assertEqual(prev[:2],pose[:2]);self.assertAlmostEqual(abs(Planner.wrap(pose[2]-prev[2])),math.pi/2)
            prev=pose
        self.assertEqual(prev,plan['goal_pose'])
        acts=orthogonal_actions([(1.2,1.2)],0.,math.pi)
        self.assertEqual(len(acts),2);self.assertTrue(all(x['kind']=='TURN_90' for x in acts))
        with self.assertRaises(ValueError):self.p.plan(SCENE,(2.05,2.05,.4),'qr_board')

    def test_no_arrival_before_actions(self):
        self.send('ARRIVAL',pose=self.m.plan['goal_pose'],stationary=True)
        self.assertEqual(self.m.phase,'NAV');self.assertEqual(self.m.action,0)
        self.send('ACTION_DONE',action=1,pose=self.m.plan['actions'][0]['pose'],stationary=True)
        self.assertEqual(self.m.action,0)

    def test_bad_navigation_feedback_aborts(self):
        self.send('ACTION_DONE',action=0,pose=[0.,0.,0.],stationary=True)
        self.assertEqual(self.m.phase,'ABORT');self.assertIsNone(self.m.output()['sim_intent'])
        self.send('STOP');self.assertEqual(self.m.phase,'ABORT')

    def test_ack_done_idle_sequence(self):
        self.qr_confirm();self.send('QR_ACK',code='156+123+516+231',session=99,pose=self.m.pose,stationary=True)
        self.assertEqual(self.m.phase,'WAIT_QR_ACK')
        self.send('QR_ACK',code='wrong',pose=self.m.pose,stationary=True);self.assertEqual(self.m.phase,'WAIT_QR_ACK')
        self.send('QR_ACK',code=self.m.code,pose=self.m.pose,stationary=True)
        self.send('CAMERA_IDLE',view='RAW',pitch_stable=True,pose=self.m.pose,stationary=True)
        self.assertEqual(self.m.phase,'WAIT_QR_DONE')
        self.send('QR_DONE',pose=self.m.pose,stationary=True);self.assertEqual(self.m.phase,'WAIT_QR_IDLE')
        self.send('CAMERA_IDLE',view='RAW',pitch_stable=False,pose=self.m.pose,stationary=True)
        self.assertEqual(self.m.phase,'ABORT');self.assertFalse(self.m.qr_done)

    def test_qr_idle_timeout(self):
        self.qr_ack();self.send('QR_DONE',pose=self.m.pose,stationary=True)
        self.t+=16;self.send('TICK');self.assertEqual(self.m.reason,'TIMEOUT')
        self.assertEqual(self.m.phase,'ABORT')

    def test_replay_and_stale_action(self):
        intent=self.m.output()['sim_intent'];self.send('ACTION_DONE',action=0,pose=intent['pose'],stationary=True)
        self.assertEqual(self.m.action,1)
        intent=self.m.output()['sim_intent'];self.send('ACTION_DONE',action=1,seq=1,pose=intent['pose'],stationary=True)
        self.assertEqual(self.m.action,1)
        self.send('ACTION_DONE',action=1,stamp=0.,pose=intent['pose'],stationary=True)
        self.assertEqual(self.m.action,1)

    def test_stale_session_after_handoff(self):
        self.raw_confirm();self.send('VISION',session=20,frame=100,pose=self.m.pose,stationary=True,
                                    view='QR',pitch_stable=True,error_px=[100,100])
        self.assertEqual(self.m.phase,'RAW_ALIGN');self.assertEqual(self.m.child.steps,0)

    def test_unsafe_micro_excursion(self):
        self.m=SimMission(self.p,SCENE,START,20,(200.,0.,0.,200.),max_excursion_m=.005)
        self.raw_confirm();self.vision(4,error_px=[30,0]);self.assertEqual(self.m.reason,'UNSAFE_VISUAL_STEP')
        self.assertIsNone(self.m.output()['sim_intent'])

    def test_wrong_micro_feedback(self):
        self.raw_confirm();self.vision(4,error_px=[14,-6]);intent=self.m.output()['sim_intent']
        self.send('STEP_DONE',step=0,pose=intent['pose'],stationary=True);self.assertEqual(self.m.phase,'WAIT_STEP')
        self.send('STEP_DONE',step=intent['step'],pose=[0.,0.,0.],stationary=True)
        self.assertEqual(self.m.reason,'BAD_MICRO_FEEDBACK');self.assertIsNone(self.m.output()['sim_intent'])

    def test_visual_pose_drift(self):
        self.arrive();self.vision(1,pose=[self.m.pose[0]+.03,self.m.pose[1],self.m.pose[2]])
        self.assertEqual(self.m.reason,'NOT_STATIONARY_OR_POSE_DRIFT')

    def test_conflict_latches_abort(self):
        self.send('TICK',conflict=True);self.assertEqual(self.m.phase,'ABORT')
        self.send('ARRIVAL',pose=self.m.plan['goal_pose'],stationary=True);self.assertEqual(self.m.phase,'ABORT')
        self.assertFalse(self.m.output()['sim_ready_pick'])

    def test_replan_after_actual_qr_pose(self):
        self.qr_ack();self.send('QR_DONE',pose=self.m.pose,stationary=True)
        measured=[self.m.pose[0]+.004,self.m.pose[1]-.004,self.m.pose[2]]
        self.send('CAMERA_IDLE',view='RAW',pitch_stable=True,pose=measured,stationary=True)
        self.assertEqual(self.m.phase,'NAV');self.assertEqual(tuple(measured[:2]),self.m.plans[-1]['path'][0])
        for a,b in zip(self.m.plan['path'],self.m.plan['path'][1:]):self.assertTrue(self.p.axis_segment(a,b))

    def test_invalid_source_and_clock(self):
        self.send('TICK',source='REAL');self.assertEqual(self.m.reason,'INVALID_EVENT_SOURCE')
        self.m=SimMission(self.p,SCENE,START,20,(200.,0.,0.,200.))
        self.send('TICK');self.m.event({'source':'SIM_TEST_ONLY','type':'TICK'},0.)
        self.assertEqual(self.m.reason,'INVALID_CLOCK')

if __name__=='__main__':unittest.main()
