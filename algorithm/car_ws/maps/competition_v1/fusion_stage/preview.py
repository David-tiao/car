#!/usr/bin/env python3
"""Standalone read-only preview for existing competition_v1 scene messages.
No commands, UART, Nav2 actions, or hardware activation. ROS imports only in ros mode.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import time
import sys
from fusion_core import Planner, SimHandoff, SimMission


def emit(obj):
    print(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False))


def load_graph(repo):
    root=Path(repo).expanduser().resolve()/ 'algorithm/car_ws/maps/competition_v1'
    spec=importlib.util.spec_from_file_location('review_road_topology',root/'topology_stage6/road_topology.py')
    if spec is None or spec.loader is None: raise ValueError('road topology not found')
    module=importlib.util.module_from_spec(spec)
    old=sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode=True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode=old
    zones=json.loads((root/'zones.json').read_text(encoding='utf-8'))
    return module.NODES,module.EDGES,zones


def demo():
    from test_fusion import planner, SCENE, START
    p=planner();qr=p.plan(SCENE,START,'qr_board');disk=p.plan(SCENE,qr['goal_pose'],'raw_disk')
    result={'source':'SYNTHETIC_DEMO_NOT_HARDWARE','plans':[qr,disk],'events':[]}
    for session,plan in enumerate((qr,disk),1):
        if plan['status']!='OBSERVATION_POSE_PREVIEW':continue
        f=SimHandoff(plan,session,(200.,0.,0.,200.))
        def add(t,**event):
            event['session']=session
            out=f.event(event,t)
            result['events'].append({'time_s':t,'input':event,'output':out})
        add(0.,type='ARRIVAL',pose=plan['goal_pose'],stationary=True)
        def frame(t,n,err):
            add(t,type='VISION',frame=n,stamp=t,target=plan['target'],target_map_m=plan['target_map_m'],stationary=True,pitch_stable=True,code='156+123+516+231',disk_geometry_and_colors=True,error_px=err)
        for n in range(1,4):frame(n*.1,n,[0,0])
        if session==1:
            add(.4,type='QR_ACK',code='156+123+516+231');add(.5,type='PITCH_READY',view='RAW',stationary=True)
        else:
            frame(.4,4,[14,-6]);add(.6,type='STEP_DONE',step=1,stationary=True)
            for n in range(5,8):frame(.7+(n-5)*.1,n,[2,-1])
    emit(result)
    return 0 if all(p['status']=='OBSERVATION_POSE_PREVIEW' for p in result['plans']) and result['events'][-1]['output']['state']=='DONE' else 1


def mission_demo():
    """Drive one continuous simulated mission using explicit synthetic feedback."""
    from test_fusion import planner, SCENE, START
    m=SimMission(planner(),SCENE,START,1,(200.,0.,0.,200.))
    events=[]; t=0.; seq=0; session=m.session
    def add(typ,**fields):
        nonlocal t,seq,session
        if session!=m.session:session=m.session;seq=0
        seq+=1;t=round(t+.1,4)
        e=dict(source='SIM_TEST_ONLY',type=typ,session=session,seq=seq,stamp=t,**fields)
        out=m.event(e,t);events.append({'time_s':t,'input':e,'output':out})
        return out
    def arrival():
        for _ in range(100):
            out=m.output();intent=out['sim_intent']
            if out['phase']!='NAV':return
            if intent['kind']=='VERIFY_ARRIVAL':
                add('ARRIVAL',pose=intent['pose'],stationary=True);return
            add('ACTION_DONE',action=intent['action'],pose=intent['pose'],stationary=True)
        raise RuntimeError('too many synthetic navigation actions')
    def frame(n,err):
        add('VISION',frame=n,pose=m.pose,stationary=True,pitch_stable=True,
            view='QR' if m.target=='qr_board' else 'RAW',target=m.target,
            target_map_m=m.plan['target_map_m'],code='156+123+516+231',
            disk_geometry_and_colors=True,error_px=err)
    arrival()
    if m.phase=='QR_CONFIRM':
        for n in range(1,4):frame(n,[0,0])
        add('QR_ACK',code='156+123+516+231',pose=m.pose,stationary=True)
        add('QR_DONE',pose=m.pose,stationary=True)
        # Synthetic delay for servo travel; not a measured SG90 settling time.
        t=round(t+1.2,4)
        add('CAMERA_IDLE',view='RAW',pitch_stable=True,pose=m.pose,stationary=True)
    arrival()
    if m.phase=='RAW_CONFIRM':
        for n in range(1,4):frame(n,[0,0])
        frame(4,[14,-6])
        if m.phase=='WAIT_STEP':
            intent=m.output()['sim_intent']
            add('STEP_DONE',step=intent['step'],pose=intent['pose'],stationary=True)
        for n in range(5,8):frame(n,[2,-1])
    result=dict(source='SYNTHETIC_DEMO_NOT_HARDWARE',plans=m.plans,events=events,final=m.output())
    emit(result)
    return 0 if m.phase=='READY_PICK' else 1


def ros(args,planner):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import String

    class Preview(Node):
        def __init__(self):
            super().__init__('fusion_approach_review_preview')
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path=self.create_publisher(RosPath,'/competition/fusion_review/path',qos)
            self.markers=self.create_publisher(MarkerArray,'/competition/fusion_review/markers',qos)
            self.status=self.create_publisher(String,'/competition/fusion_review/status',qos)
            self.sub=self.create_subscription(String,'/competition/lidar/scene_status',self.on_scene,qos)
            self.scene=None;self.received=None
            self.timer=self.create_timer(.5,self.tick)
            self.get_logger().warning('Read-only stationary review. No navigation authorization. Never use this preview while moving.')

        def on_scene(self,msg):
            self.received=time.monotonic()
            try:
                if len(msg.data)>65536: raise ValueError('oversized scene')
                self.scene=json.loads(msg.data)
            except (ValueError,TypeError):self.scene=None

        def tick(self):
            stamp=self.get_clock().now()
            out={'schema':'car_fusion_approach_v1','status':'WAIT_FRESH_SCENE','preview_only':True,'identity_verified':False,'navigation_authorized':False,'path':[],'goal_pose':None,'target':args.target}
            if isinstance(self.scene,dict) and self.received is not None and time.monotonic()-self.received<=2.5:
                scan=self.scene.get('scan_stamp_ns')
                if type(scan) is not int or not 0<=stamp.nanoseconds-scan<=args.max_scan_age*1e9:
                    out['status']='STALE_SCAN_EPOCH'
                else:
                    try:out=planner.plan(self.scene,args.start_pose,args.target)
                    except (ValueError,TypeError,KeyError,OverflowError) as exc:
                        out['status']='REJECTED_SCENE';out['reason']=str(exc)
            path=RosPath();path.header.frame_id='map';path.header.stamp=stamp.to_msg()
            arr=MarkerArray();delete=Marker();delete.header=path.header;delete.action=Marker.DELETEALL;arr.markers.append(delete)
            goal=out['goal_pose']
            if goal:
                # Show the actual planned heading, not the tangent of a lateral move.
                # Coincident poses represent stationary quarter turns at the endpoint.
                poses=[[ *out['path'][0], out['start_heading_rad']]] + [a['pose'] for a in out['actions']]
                for x,y,yaw in poses:
                    pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y
                    pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
                arrow=Marker();arrow.header=path.header;arrow.ns='review_goal';arrow.id=0;arrow.type=Marker.ARROW;arrow.action=Marker.ADD
                arrow.pose.position.x,arrow.pose.position.y=goal[:2];arrow.pose.position.z=.10;arrow.pose.orientation.z=math.sin(goal[2]/2);arrow.pose.orientation.w=math.cos(goal[2]/2)
                arrow.scale.x=.20;arrow.scale.y=.04;arrow.scale.z=.04;arrow.color.r=.3;arrow.color.g=.95;arrow.color.b=.6;arrow.color.a=1.;arr.markers.append(arrow)
                label=Marker();label.header=path.header;label.ns='review_goal';label.id=1;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
                label.pose.position.x,label.pose.position.y=goal[:2];label.pose.position.z=.20;label.scale.z=.075;label.color.r=label.color.g=label.color.b=label.color.a=1.
                label.text='PREVIEW '+args.target;arr.markers.append(label)
            self.path.publish(path);self.markers.publish(arr);self.status.publish(String(data=json.dumps(out,allow_nan=False)))

    rclpy.init();node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


def main():
    ap=argparse.ArgumentParser(description=__doc__);sp=ap.add_subparsers(dest='mode',required=True)
    sp.add_parser('demo',help='synthetic planner and event replay, no ROS')
    sp.add_parser('mission-demo',help='continuous synthetic mission and guarded feedback, no ROS')
    for mode in ('plan','ros'):
        sub=sp.add_parser(mode)
        sub.add_argument('--repo',required=True)
        sub.add_argument('--start-pose',nargs=3,type=float,required=True,metavar=('X','Y','YAW_RAD'),help='measured stationary base centre and yaw in map')
        sub.add_argument('--envelope-m',type=float,required=True,help='measured circumscribed radius plus margin')
        sub.add_argument('--qr-range-m',nargs=2,type=float,required=True,metavar=('MIN','MAX'))
        sub.add_argument('--disk-range-m',nargs=2,type=float,required=True,metavar=('MIN','MAX'))
        sub.add_argument('--obstacle-uncertainty-m',type=float,required=True)
        sub.add_argument('--target-uncertainty-m',type=float,required=True)
        sub.add_argument('--target',choices=('qr_board','raw_disk'),required=True)
        if mode=='plan':sub.add_argument('--scene',required=True,help='saved scene JSON, historical preview only')
        else:sub.add_argument('--max-scan-age',type=float,default=20.,help='snapshot age seconds, static test only')
    args=ap.parse_args()
    if args.mode=='demo':return demo()
    if args.mode=='mission-demo':return mission_demo()
    if not all(math.isfinite(v) for v in args.start_pose):ap.error('nonfinite start pose')
    if args.mode=='ros' and (not math.isfinite(args.max_scan_age) or not 0<args.max_scan_age<=120):ap.error('invalid max scan age')
    try:
        p=Planner(*load_graph(args.repo),args.envelope_m,args.qr_range_m,args.disk_range_m,args.obstacle_uncertainty_m,args.target_uncertainty_m)
        if args.mode=='ros':return ros(args,p)
        path=Path(args.scene)
        if path.stat().st_size>65536:raise ValueError('oversized scene')
        out=p.plan(json.loads(path.read_text(encoding='utf-8')),args.start_pose,args.target)
        out['source']='SAVED_SNAPSHOT_PREVIEW_ONLY';emit(out)
        return 0 if out['goal_pose'] else 2
    except (ValueError,TypeError,KeyError,OSError,ImportError,OverflowError) as exc:ap.exit(2,'ERROR: '+str(exc)+'\n')

if __name__=='__main__':raise SystemExit(main())
