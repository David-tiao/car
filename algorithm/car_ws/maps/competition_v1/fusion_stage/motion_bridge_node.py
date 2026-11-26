#!/usr/bin/env python3
"""Standalone ROS2 motion bridge candidate. Default standby: no tty and no TX.
Run only as a single-threaded executor. No publishers on /odom or /tf.
"""
import json
import math
import time
from motion_bridge_core import MotionBridge
from motion_serial import PosixSerial

PREFIX = '/competition/motion/'


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped

    class MotionNode(Node):
        def __init__(self):
            super().__init__('car_pi_motion_bridge')
            ro = ParameterDescriptor(read_only=True)
            enabled = self.declare_parameter('enabled', False, ro).value
            port = self.declare_parameter('port', '', ro).value
            session = self.declare_parameter('session', 0, ro).value
            # Live transport requires explicit local process configuration AND a
            # separate MCU grant. The safety topic alone cannot grant MCU motion.
            operator = self.declare_parameter('operator_enabled', False, ro).value
            self.bridge = None
            self.start_error = None
            self.last_pose_seq = 0
            self.last_publish = -1.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.feedback = self.create_publisher(String, PREFIX+'feedback', qos)
            self.status_pub = self.create_publisher(String, PREFIX+'status', qos)
            self.pose_pub = self.create_publisher(PoseStamped, PREFIX+'pose', qos)
            self.goal_sub = self.create_subscription(String, PREFIX+'goal', self.goal, qos)
            self.safety_sub = self.create_subscription(String, PREFIX+'safety', self.safety, qos)
            if enabled:
                try:
                    if self.get_parameter('use_sim_time').value or not operator:
                        raise ValueError('real-time clock and explicit operator configuration required')
                    if type(session) is not int or not 1 <= session <= 0xffffffff:
                        raise ValueError('explicit independent session required')
                    transport = PosixSerial(port)
                    try: self.bridge = MotionBridge(session, transport, operator_enabled=operator)
                    except Exception:
                        transport.close(); raise
                except (OSError, ValueError, RuntimeError) as exc:
                    self.start_error = str(exc)
                    self.get_logger().error('Serial startup rejected: '+str(exc))
            else:
                self.get_logger().info('Standby: no serial port opened, no commands sent')
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.01, self.tick, clock=self.steady)

        def goal(self, msg):
            if self.bridge: self.bridge.goal(msg.data, time.monotonic(), time.time_ns())

        def safety(self, msg):
            if self.bridge: self.bridge.safety(msg.data, time.monotonic(), time.time_ns())

        def tick(self):
            now = time.monotonic()
            if self.bridge:
                if self.get_parameter('use_sim_time').value:
                    self.bridge.shutdown()
                self.bridge.tick(now)
                status = self.bridge.status(now)
            else:
                status = dict(schema='car_motion_feedback_v1', phase='FAULT' if self.start_error else 'STANDBY',
                              reason=self.start_error, pose_valid=False, pose_mm_mdeg=None,
                              velocity_available=False, navigation_authorized=False,
                              physical_stop_confirmed=False)
            # Fresh accepted STATUS only: do not republish an old pose as new.
            if status.get('pose_valid') and status['feedback_seq'] != self.last_pose_seq:
                self.last_pose_seq = status['feedback_seq']
                msg = PoseStamped(); msg.header.frame_id = 'odom'
                msg.header.stamp = self.get_clock().now().to_msg()  # receive time, NOT MCU sample time
                x,y,yaw = status['pose_mm_mdeg']
                msg.pose.position.x=x/1000.;msg.pose.position.y=y/1000.
                half=math.radians(yaw/1000.)/2
                msg.pose.orientation.z=math.sin(half);msg.pose.orientation.w=math.cos(half)
                self.pose_pub.publish(msg)
                self.feedback.publish(String(data=json.dumps(status, allow_nan=False)))
            if now-self.last_publish>=.1:
                self.last_publish=now
                self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

        def close(self):
            if self.bridge: self.bridge.shutdown()

    rclpy.init()
    node = None; executor = SingleThreadedExecutor()
    try:
        node = MotionNode();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close();executor.remove_node(node);node.destroy_node()
        executor.shutdown()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__': main()
