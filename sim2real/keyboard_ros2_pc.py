#!/usr/bin/env python3
"""Computer-side ROS2 mapping: local HTTP keyboard intent -> ROS2 TwistStamped.

No /cmd_vel, real STM32 serial or hardware authority. Use ROS2 Humble after
sourcing /opt/ros/humble/setup.bash. Pi node must independently gate messages.
"""
import argparse
import json
import math
import time
from urllib.error import URLError
from urllib.request import urlopen


def is_local_url(url):
    from urllib.parse import urlsplit
    p = urlsplit(url)
    return p.scheme == "http" and p.hostname in ("localhost", "127.0.0.1", "::1")


def parse_snapshot(data):
    if not isinstance(data, dict) or data.get("hardware_authorized") is not False:
        raise ValueError("Expected unprivileged keyboard snapshot")
    if data.get("frame_id") != "base_link" or data.get("transport") != "PC_ROS2_CANDIDATE_ONLY":
        raise ValueError("Wrong keyboard frame/source")
    values = (data.get("vx"), data.get("vy"), data.get("wz"))
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        raise ValueError("Nonfinite command")
    if (max(abs(values[0]), abs(values[1])) > 0.25 or abs(values[2]) > 0.5):
        raise ValueError("Unsafe keyboard target magnitude")
    # A stale browser or lost HTTP session produces only zeros.
    if not data.get("active") or not data.get("deadman") or (
            data.get("age_ms") is None or data["age_ms"] > 300):
        return (0.0, 0.0, 0.0)
    return tuple(float(v) for v in values)


def main():
    parser = argparse.ArgumentParser(description="PC keyboard -> ROS2 candidate only (NO motors)")
    parser.add_argument("--server", default="http://127.0.0.1:8090")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()
    if not is_local_url(args.server):
        parser.error("Keyboard HTTP endpoint must be hosted on THIS PC's localhost")
    if not 5 <= args.hz <= 30:
        parser.error("hz must be 5..30")
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
    except ImportError as exc:
        parser.error("ROS2 Humble Python environment required: " + str(exc))

    rclpy.init()
    class KeyboardPc(Node):
        def __init__(self):
            super().__init__("car_keyboard_pc_source")
            self.pub = self.create_publisher(
                TwistStamped, "/car/teleop/desired_twist", 10)
            self.health = self.create_publisher(
                String, "/car/teleop/pc_status", 10)
            self.failures = 0
            self.timer = self.create_timer(1.0/args.hz, self.tick)
            self.get_logger().warning(
                "OFFLINE INTERFACE ONLY: publishes /car/teleop/desired_twist, NOT /cmd_vel")
        def tick(self):
            active, target, reason = False, (0.0, 0.0, 0.0), "HTTP_LOST"
            try:
                with urlopen(args.server.rstrip("/") +
                             "/api/keyboard/intent", timeout=0.15) as resp:
                    data = json.loads(resp.read(4096))
                target = parse_snapshot(data)
                active = any(abs(v) > 1e-8 for v in target)
                reason = "CANDIDATE_ONLY" if active else "ZERO_OR_DEADMAN"
                self.failures = 0
            except (URLError, OSError, ValueError, TypeError, KeyError) as exc:
                self.failures += 1
                if self.failures % 40 == 1:
                    self.get_logger().warning("Keyboard endpoint lost; forcing zero: " + str(exc))
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x, msg.twist.linear.y = target[0], target[1]
            msg.twist.angular.z = target[2]
            self.pub.publish(msg)
            status = String()
            status.data = json.dumps({
                "source": "PC_KEYBOARD_LOCAL", "state": reason,
                "motor_authorized": False,
                "topic": "/car/teleop/desired_twist"}, ensure_ascii=False)
            self.health.publish(status)
    node = KeyboardPc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
