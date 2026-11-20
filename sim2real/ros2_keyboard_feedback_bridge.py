#!/usr/bin/env python3
"""Read existing ROS2 velocity commands for browser motion estimation; never publish control."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def advance_pose(pose, cmd, seconds):
    """Integrate holonomic body-frame velocity with no movement or map bounds."""
    x, y, yaw = pose
    vx, vy, wz = cmd
    if seconds <= 0:
        return pose
    next_yaw = yaw + wz * seconds
    if abs(wz) < 1e-10:
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * seconds
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * seconds
    else:
        sine_delta = math.sin(next_yaw) - math.sin(yaw)
        cosine_delta = math.cos(next_yaw) - math.cos(yaw)
        x += (vx * sine_delta + vy * cosine_delta) / wz
        y += (-vx * cosine_delta + vy * sine_delta) / wz
    return x, y, next_yaw


class MotionObserver:
    """Integrate published Twist values. Not an FSM; estimated pose is not odometry."""
    def __init__(self, timeout, clock=time.monotonic):
        self.timeout = timeout
        self.clock = clock
        self.pose = (0.0, 0.0, 0.0)
        self.last_tick = clock()
        self.last_command_at = None
        self.command = (0.0, 0.0, 0.0)

    def _advance_to(self, now):
        if self.last_command_at is not None:
            valid_until = min(now, self.last_command_at + self.timeout)
            seconds = max(0.0, valid_until - self.last_tick)
            self.pose = advance_pose(self.pose, self.command, seconds)
        self.last_tick = now

    def observe(self, vx, vy, wz):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return False
        now = self.clock()
        self._advance_to(now)
        self.command = (float(vx), float(vy), float(wz))
        self.last_command_at = now
        return True

    def snapshot(self):
        now = self.clock()
        self._advance_to(now)
        recent = self.last_command_at is not None and now - self.last_command_at < self.timeout
        return self.pose, self.command if recent else (0.0, 0.0, 0.0), recent


def main():
    parser = argparse.ArgumentParser(description='ROS2 keyboard velocity -> read-only website feedback')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--stamped', action='store_true', help='Use geometry_msgs/TwistStamped instead of Twist')
    parser.add_argument('--server', default='http://127.0.0.1:8080')
    parser.add_argument('--hz', type=float, default=20.0)
    parser.add_argument('--timeout', type=float, default=0.5, help='Stop estimating after this much command silence')
    args = parser.parse_args()
    if not args.server.startswith(('http://127.0.0.1:', 'http://localhost:', 'http://[::1]:')):
        parser.error('--server must point to localhost')
    if not (1 <= args.hz <= 60) or not math.isfinite(args.hz):
        parser.error('--hz must be in [1, 60]')
    if not (0.05 <= args.timeout <= 10) or not math.isfinite(args.timeout):
        parser.error('--timeout must be in [0.05, 10]')
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist, TwistStamped
    except ImportError as exc:
        sys.exit('ROS2 Python libraries missing. Source ROS2 Humble setup.bash first: ' + str(exc))

    observer = MotionObserver(args.timeout)
    rclpy.init()

    class Bridge(Node):
        def __init__(self):
            super().__init__('car_keyboard_feedback_observer')
            self.warning_at = 0.0
            message_type = TwistStamped if args.stamped else Twist
            self.subscription = self.create_subscription(message_type, args.topic, self.on_command, 10)
            self.timer = self.create_timer(1 / args.hz, self.send_feedback)
            self.get_logger().info(f'Observing {args.topic} ({message_type.__name__}); feedback ONLY, no motor output')

        def on_command(self, message):
            cmd = message.twist if args.stamped else message
            if not observer.observe(cmd.linear.x, cmd.linear.y, cmd.angular.z):
                self.get_logger().warning('Rejected non-finite ROS2 Twist')

        def send_feedback(self):
            pose, velocity, recent = observer.snapshot()
            payload = dict(zip(('x', 'y', 'yaw', 'vx', 'vy', 'wz'), (*pose, *velocity)))
            payload['command_recent'] = bool(recent)
            payload['topic'] = args.topic
            req = Request(args.server.rstrip('/') + '/api/keyboard/feedback',
                          data=json.dumps(payload, allow_nan=False).encode('utf-8'),
                          headers={'Content-Type': 'application/json'}, method='POST')
            try:
                with urlopen(req, timeout=.25) as response:
                    if response.status != 200:
                        raise RuntimeError('HTTP ' + str(response.status))
            except (HTTPError, URLError, OSError, RuntimeError, ValueError, OverflowError) as exc:
                now = time.monotonic()
                if now - self.warning_at > 5:
                    self.warning_at = now
                    self.get_logger().warning('Website feedback failed: ' + str(exc))

    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
