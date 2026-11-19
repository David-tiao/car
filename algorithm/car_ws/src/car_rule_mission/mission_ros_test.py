#!/usr/bin/env python3
"""ROS2 TEST mission replay. Never publishes /cmd_vel, /odom or real CAR-LINK commands.

Launch only inside isolated ROS_DOMAIN_ID=73; topics are scoped to /car/test/mission.
"""
import argparse
import json
from pathlib import Path

from mission_execution import MissionError
from mission_test_session import PtyMissionSession
from rule_mission_core import PlanError, build_mission


def make_node(start_zone='upper_right', task_code='156+123+516+231',
              blocked=(), fail_at=None, drop_ack_at=None, hz=2.0):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath

    class TestMission(Node):
        def __init__(self):
            super().__init__('car_test_mission_replay')
            plan = build_mission(start_zone, blocked, provenance='DEMO_ONLY')
            if plan['status'] == 'NO_ROUTE':
                raise MissionError('NO_ROUTE: TEST mission refuses to begin')
            self.session = PtyMissionSession(plan, task_code, fail_at, drop_ack_at)
            self.state_pub = self.create_publisher(String, '/car/test/mission/state', 10)
            self.feedback_pub = self.create_publisher(String, '/car/test/mission/feedback', 10)
            self.pose_pub = self.create_publisher(PoseStamped, '/car/test/mission/road_node_pose', 10)
            self.path_pub = self.create_publisher(RosPath, '/car/test/mission/road_node_trace', 10)
            self.last_event_count = 0
            self.timer = self.create_timer(1.0 / hz, self.advance)
            self.get_logger().warning(
                'TEST MOCK_ONLY, topology anchor pose (not odom), no UART or real control')

        def advance(self):
            try:
                emitted = self.session.advance() if self.session.state == 'READY' else []
                for event in emitted:
                    self.feedback_pub.publish(String(data=json.dumps(
                        {'source_type': 'MOCK', 'hardware_authorized': False, **event})))
                snapshot = self.session.snapshot()
                self.state_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = snapshot['x_m']
                pose.pose.position.y = snapshot['y_m']
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)
                trace = RosPath()
                trace.header = pose.header
                for x, y in self.session.trace:
                    item = PoseStamped()
                    item.header = trace.header
                    item.pose.position.x = x
                    item.pose.position.y = y
                    item.pose.orientation.w = 1.0
                    trace.poses.append(item)
                self.path_pub.publish(trace)
                if snapshot['state'] in ('DONE', 'FAULT') and self.last_event_count != -1:
                    self.get_logger().info('TEST final: %s', snapshot['state'])
                    self.last_event_count = -1
            except (OSError, MissionError, ValueError) as exc:
                self.get_logger().error('TEST loopback error: %s', exc)
                self.timer.cancel()
                self.session.close()
                raise

        def destroy_node(self):
            self.session.close()
            return super().destroy_node()

    return TestMission()


def main():
    p = argparse.ArgumentParser(description='ROS2 TEST-only PTY mission replay')
    p.add_argument('--start-zone', choices=('upper_right', 'lower_right'), default='upper_right')
    p.add_argument('--task-code', default='156+123+516+231')
    p.add_argument('--block', action='append', default=[])
    p.add_argument('--fail-at', type=int)
    p.add_argument('--drop-ack-at', type=int)
    p.add_argument('--hz', type=float, default=2.0)
    args = p.parse_args()
    if not (0.2 <= args.hz <= 10.0):
        p.error('hz must be in 0.2..10')
    import rclpy
    rclpy.init()
    node = None
    try:
        node = make_node(args.start_zone, args.task_code, args.block,
                         args.fail_at, args.drop_ack_at, args.hz)
        rclpy.spin(node)
    except (MissionError, PlanError) as exc:
        p.exit(2, 'TEST error: {}\n'.format(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
