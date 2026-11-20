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



class CandidateGuard:
    """Pure-Python fail-closed Pi-side ROS2 candidate gate.

    Separate from rclpy to run exact watchdog/clock-skew/replay tests on a PC.
    It never creates or opens a physical serial transmitter.
    """

    def __init__(self):
        self.target = (0.0, 0.0, 0.0)
        self.last_message = None
        self.rejected = 0
        self.accepted = 0
        self.last_stamp_s = None
        self.reason = "NO_MESSAGE"

    def receive(self, vx, vy, wz, frame, stamp_s, clock_s, monotonic_s):
        values = (stamp_s, clock_s, monotonic_s)
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in values):
            self._reject("INVALID_TIME")
            return False
        candidate = validate_candidate(vx, vy, wz, frame)
        if candidate is None:
            self._reject("INVALID_VELOCITY_OR_FRAME")
            return False
        age = clock_s - stamp_s
        if not -0.05 <= age <= WATCHDOG_SEC:
            self._reject("STALE_OR_FUTURE_STAMP")
            return False
        if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s:
            self._reject("DUPLICATE_OR_REORDERED_STAMP")
            return False
        self.target = candidate
        self.last_message = monotonic_s
        self.last_stamp_s = stamp_s
        self.accepted += 1
        self.reason = "CANDIDATE_ONLY"
        return True

    def _reject(self, reason):
        self.rejected += 1
        self.target = (0.0, 0.0, 0.0)
        self.last_message = None
        self.reason = reason

    def tick(self, monotonic_s):
        if type(monotonic_s) not in (int, float) or not math.isfinite(monotonic_s):
            raise ValueError("Finite monotonic clock required")
        age = None if self.last_message is None else monotonic_s - self.last_message
        live = age is not None and 0 <= age <= WATCHDOG_SEC
        return {
            "state": "ROS2_CANDIDATE_ONLY" if live else "WATCHDOG_STOP",
            "candidate": self.target if live else (0.0, 0.0, 0.0),
            "uart_tx_enabled": False,
            "motor_authorized": False,
            "reason": self.reason if live else (
                self.reason if self.last_message is None else "WATCHDOG_EXPIRED"),
            "last_rx_age_ms": round(age * 1000) if age is not None else None,
            "rejected": self.rejected,
            "accepted": self.accepted,
        }


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        from rclpy.time import Time
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit("Install/source ROS2 Humble first: " + str(exc))

    rclpy.init()
    class PiGate(Node):
        def __init__(self):
            super().__init__("car_keyboard_pi_guard")
            self.guard = CandidateGuard()
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
            stamp = Time.from_msg(msg.header.stamp)
            now_ros = self.get_clock().now()
            # Check both ROS timestamps and host watchdog. NTP/chrony drift
            # will refuse commands rather than silently enabling movement.
            stamp_s = stamp.nanoseconds / 1e9
            now_ros_s = now_ros.nanoseconds / 1e9
            self.guard.receive(
                msg.twist.linear.x, msg.twist.linear.y,
                msg.twist.angular.z, msg.header.frame_id,
                stamp_s, now_ros_s, time.monotonic())

        def tick(self):
            report = self.guard.tick(time.monotonic())
            desired = report["candidate"]
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = desired[0]
            message.twist.linear.y = desired[1]
            message.twist.angular.z = desired[2]
            self.out.publish(message)
            health = String()
            health.data = json.dumps({
                **{k: v for k, v in report.items() if k != "candidate"},
                "candidate_topic": "/car/teleop/stm32_candidate"
            }, ensure_ascii=False)
            self.health.publish(health)
    node = PiGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble can signal Ctrl+C by already shutting down the ROS context.
        pass
    finally:
        node.destroy_node()
        # Never call shutdown() twice: SIGINT may have shut the context down.
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
