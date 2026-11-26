#!/usr/bin/env python3
"""Isolated ROS2 adapter. Only /car/test/segment/* topics. No hardware IO.
Feed test telemetry, link, feedback, plan and aggregate vision events externally.
"""
import argparse
from dataclasses import asdict
import json
import os
import time
from segment_executor import Executor, Limits, Pose, Sample

PREFIX='/car/test/segment/'

def make_node(config):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from car_interfaces.msg import ChassisTelemetry, CommandFeedback, LinkStatus
    class SegmentTest(Node):
        def __init__(self):
            super().__init__('car_segment_review_test')
            self.e=Executor(config['map_id'],Pose(**config['map_to_odom']),Limits(**config['limits']),config['success_status'])
            self.intent=self.create_publisher(String,PREFIX+'intent',10)
            self.state=self.create_publisher(String,PREFIX+'state',10)
            self.create_subscription(ChassisTelemetry,PREFIX+'telemetry',self.telemetry,10)
            self.create_subscription(CommandFeedback,PREFIX+'feedback',self.feedback,10)
            self.create_subscription(LinkStatus,PREFIX+'link',self.link,10)
            self.create_subscription(String,PREFIX+'plan',self.plan,10)
            self.create_subscription(String,PREFIX+'vision',self.vision,10)
            self.create_timer(.05,self.advance)
        def now(self): return time.monotonic_ns()//1_000_000
        def guard(self, callback):
            try: callback()
            except (ValueError,TypeError,KeyError,OverflowError) as exc:
                self.e._fault('INVALID_TEST_INPUT: '+str(exc)); self.get_logger().error(str(exc))
        def telemetry(self,m):
            self.guard(lambda: self.e.telemetry(self.now(),Sample(m.source_time_ms,Pose(m.x_m,m.y_m,m.yaw_rad),m.vx_mps,m.vy_mps,m.wz_radps,m.target_reached,m.imu_valid)))
        def feedback(self,m):
            self.guard(lambda: self.e.feedback(self.now(),m.cmd_id,m.event_type,m.source_time_ms,m.status_code,m.original_command_type))
        def link(self,m):
            self.guard(lambda: self.e.link(self.now(),m.peer_session_id,m.connected,m.heartbeat_fresh))
        def plan(self,m):
            def apply():
                d=json.loads(m.data)
                if d['mode']!='SIM_ONLY' or d['fixture_corridors_valid'] is not True: raise ValueError('simulation fixture only')
                self.e.load_route(self.now(),d['map_id'],d['stage'],Pose(**d['goal']),[Pose(**p) for p in d['route']],lambda a,b:True)
            self.guard(apply)
        def vision(self,m):
            def apply():
                d=json.loads(m.data)
                if d['mode']!='SIM_ONLY': raise ValueError('simulation fixture only')
                self.e.vision_done(self.now(),d['token'],d['stage'],d['success'],d.get('task_code'),d.get('pitch_ready',False))
            self.guard(apply)
        def advance(self):
            def apply():
                d=self.e.issue(self.now())
                if d:
                    d['target_map']=asdict(d['target_map']); d['target_odom']=asdict(d['target_odom'])
                    self.intent.publish(String(data=json.dumps(d)))
                self.state.publish(String(data=json.dumps(self.e.snapshot())))
            self.guard(apply)
    return SegmentTest()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',required=True); a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73': p.error('requires isolated ROS_DOMAIN_ID=73')
    with open(a.config,encoding='utf-8') as f: config=json.load(f)
    if config.get('mode')!='SIM_ONLY': p.error('SIM_ONLY config required')
    import rclpy
    rclpy.init(); node=None
    try:
        node=make_node(config); rclpy.spin(node)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
