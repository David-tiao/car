#!/usr/bin/env python3
"""Computer-side ROS2 mapping: local HTTP keyboard intent -> ROS2 TwistStamped.

No /cmd_vel, real STM32 serial or hardware authority. Use ROS2 Humble after
sourcing /opt/ros/humble/setup.bash. Pi node must independently gate messages.
"""
import argparse
import json
import math
import time
import uuid
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


TOPICS = {
    "local_ros2": "/car/teleop/local_test_twist",
    "pi_stm32": "/car/teleop/desired_twist",
}


def select_topic(data):
    mode = data.get("route_mode")
    if mode not in TOPICS:
        raise ValueError("Invalid keyboard route mode")
    return TOPICS[mode]


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
            self.publishers = {
                mode: self.create_publisher(TwistStamped, topic, 10)
                for mode, topic in TOPICS.items()
            }
            self.previous_mode = None
            # Numbered SOFTWARE diagnostics travel on a separate topic.
            # The diagnostic heartbeat measures its own DDS delivery;
            # it is not an ACK for the TwistStamped or a motor response.
            self.diag_pub = self.create_publisher(
                String, "/car/teleop/local_test_diag", 10)
            self.diag_session = uuid.uuid4().hex
            self.diag_seq = 0
            self.health = self.create_publisher(
                String, "/car/teleop/pc_status", 10)
            self.failures = 0
            self.timer = self.create_timer(1.0/args.hz, self.tick)
            self.get_logger().warning(
                "DRY RUN only: local ROS2 loopback or Pi candidate; NO /cmd_vel / motor TX")
        def tick(self):
            active, target, reason = False, (0.0, 0.0, 0.0), "HTTP_LOST"
            selected = self.previous_mode or "local_ros2"
            try:
                with urlopen(args.server.rstrip("/") +
                             "/api/keyboard/intent", timeout=0.15) as resp:
                    data = json.loads(resp.read(4096))
                target = parse_snapshot(data)
                select_topic(data)  # validate before changing the selected publisher
                selected = data["route_mode"]
                active = any(abs(v) > 1e-8 for v in target)
                reason = "CANDIDATE_ONLY" if active else "ZERO_OR_DEADMAN"
                self.failures = 0
            except (URLError, OSError, ValueError, TypeError, KeyError) as exc:
                # Fail closed if the website vanishes or sends an invalid
                # route after a valid nonzero velocity field was parsed.
                target = (0.0, 0.0, 0.0)
                active = False
                selected = self.previous_mode or "local_ros2"
                self.failures += 1
                if self.failures % 40 == 1:
                    self.get_logger().warning("Keyboard endpoint lost; forcing zero: " + str(exc))
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x, msg.twist.linear.y = target[0], target[1]
            msg.twist.angular.z = target[2]
            if self.previous_mode != selected and self.previous_mode in self.publishers:
                stop = TwistStamped()
                stop.header.stamp = msg.header.stamp
                stop.header.frame_id = "base_link"
                self.publishers[self.previous_mode].publish(stop)
            self.publishers[selected].publish(msg)
            if selected == "local_ros2":
                diagnostic = String()
                diagnostic.data = json.dumps({
                    "kind": "packet", "channel": "pc_local_ros2",
                    "session": self.diag_session,
                    "seq": self.diag_seq,
                    "sent_s": time.time()
                }, separators=(",", ":"))
                self.diag_pub.publish(diagnostic)
                self.diag_seq += 1
            self.previous_mode = selected
            status = String()
            status.data = json.dumps({
                "source": "PC_KEYBOARD_LOCAL", "state": reason,
                "motor_authorized": False,
                "topic": TOPICS[selected], "route_mode": selected,
                "keyboard_to_real_vehicle": False}, ensure_ascii=False)
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
