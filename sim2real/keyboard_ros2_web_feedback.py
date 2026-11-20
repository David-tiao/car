#!/usr/bin/env python3
"""ROS2 topic -> localhost keyboard mapping status, WITHOUT a Raspberry Pi.

Subscribes the local PC topic and (when available) Pi candidate topic.
These are ROS2 delivery acknowledgements only, NEVER motor measurements.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

TOPICS = {
    "local_ros2": "/car/teleop/local_test_twist",
    "pi_stm32": "/car/teleop/stm32_candidate",
}


def validate_message(source, frame_id, stamp_s, vx, vy, wz):
    if source not in TOPICS or frame_id != "base_link":
        raise ValueError("Wrong source or frame for keyboard ROS2 bridge")
    vals = (stamp_s, vx, vy, wz)
    if not all(type(v) in (float, int) and math.isfinite(v) for v in vals):
        raise ValueError("Nonfinite ROS2 sample")
    if math.hypot(vx, vy) > 0.25001 or abs(wz) > 0.50001:
        raise ValueError("Unbounded ROS2 candidate")
    return {
        "source": source, "topic": TOPICS[source],
        "frame_id": frame_id, "stamp_s": float(stamp_s),
        "vx": float(vx), "vy": float(vy), "wz": float(wz)
    }


def main():
    parser = argparse.ArgumentParser(description="ROS2 local/Pi candidate -> /keyboard webpage; no hardware")
    parser.add_argument("--server", default="http://127.0.0.1:8090")
    args = parser.parse_args()
    from keyboard_ros2_pc import is_local_url
    if not is_local_url(args.server):
        parser.error("ROS2 feedback must POST to localhost only")
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
    except ImportError as exc:
        parser.error("ROS2 Humble not installed/sourced: " + str(exc))

    rclpy.init()
    class Feedback(Node):
        def __init__(self):
            super().__init__("car_keyboard_ros2_web_feedback")
            self._keyboard_subscriptions = []
            self.last_notice = 0.0
            for key, topic in TOPICS.items():
                self._keyboard_subscriptions.append(self.create_subscription(
                    TwistStamped, topic,
                    lambda msg, source=key: self.forward(source, msg), 10))
            self.get_logger().info(
                "Subscribing local loopback and future Pi candidate; only HTTP status, no motors")
        def forward(self, source, msg):
            h = msg.header
            try:
                packet = validate_message(
                    source, h.frame_id,
                    h.stamp.sec + h.stamp.nanosec / 1e9,
                    msg.twist.linear.x, msg.twist.linear.y,
                    msg.twist.angular.z)
                body = json.dumps(packet, separators=(",", ":")).encode("utf-8")
                request = Request(
                    args.server.rstrip("/") + "/api/keyboard/ros2_feedback",
                    data=body, method="POST",
                    headers={"Content-Type": "application/json"})
                with urlopen(request, timeout=0.20) as response:
                    if response.status != 200:
                        raise ValueError("HTTP " + str(response.status))
            except (URLError, OSError, ValueError, TypeError) as exc:
                if time.monotonic() - self.last_notice > 5.0:
                    self.last_notice = time.monotonic()
                    self.get_logger().warning("Local ROS2 web feedback unavailable: " + str(exc))

    node = Feedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
