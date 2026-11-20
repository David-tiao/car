#!/usr/bin/env python3
"""Raspberry Pi 5 ROS2: guarded PC keyboard subscriber -> CANDIDATE only.

NO STM32 transmitter is linked. No /cmd_vel publisher. An eventual hardware
adapter needs a separately reviewed CAR-LINK command contract and arming.
"""
import json
import math
import time

MAX_LINE = 0.25
MAX_YAW = 0.5
WATCHDOG_SEC = 0.30


def validate_candidate(vx, vy, wz, frame):
    if frame != "base_link" or not all(
            type(v) in (int, float) and math.isfinite(v) for v in (vx, vy, wz)):
        return None
    if math.hypot(vx, vy) > MAX_LINE + 1e-6 or abs(wz) > MAX_YAW + 1e-6:
        return None
    return (float(vx), float(vy), float(wz))


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit("Install/source ROS2 Humble first: " + str(exc))

    rclpy.init()
    class PiGate(Node):
        def __init__(self):
            super().__init__("car_keyboard_pi_guard")
            self.last_message = 0.0
            self.target = (0.0, 0.0, 0.0)
            self.rejected = 0
            self.sub = self.create_subscription(
                TwistStamped, "/car/teleop/desired_twist", self.rx, 10)
            self.out = self.create_publisher(
                TwistStamped, "/car/teleop/stm32_candidate", 10)
            self.health = self.create_publisher(
                String, "/car/teleop/pi_guard_status", 10)
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().warning(
                "SIM/DRY-RUN ONLY: ROS2 candidate -> STM32 TX NOT CONNECTED")
        def rx(self, msg):
            candidate = validate_candidate(
                msg.twist.linear.x, msg.twist.linear.y,
                msg.twist.angular.z, msg.header.frame_id)
            # ROS stamp freshness is checked against this Pi's ROS clock,
            # not the PC wall clock; a time-sync mismatch fails closed.
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if candidate is None or not -0.05 <= age <= WATCHDOG_SEC:
                self.target = (0.0, 0.0, 0.0)
                self.last_message = 0.0
                self.rejected += 1
                return
            self.target = candidate
            self.last_message = time.monotonic()
        def tick(self):
            safe = self.last_message > 0 and (
                time.monotonic() - self.last_message <= WATCHDOG_SEC)
            desired = self.target if safe else (0.0, 0.0, 0.0)
            state = "ROS2_CANDIDATE_ONLY" if safe else "WATCHDOG_STOP"
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = desired[0]
            message.twist.linear.y = desired[1]
            message.twist.angular.z = desired[2]
            self.out.publish(message)
            health = String()
            health.data = json.dumps({
                "state": state, "motor_authorized": False,
                "uart_tx_enabled": False,
                "last_rx_age_ms": round(
                    (time.monotonic()-self.last_message)*1000)
                    if self.last_message else None,
                "rejected": self.rejected,
                "candidate_topic": "/car/teleop/stm32_candidate"})
            self.health.publish(health)
    node = PiGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
