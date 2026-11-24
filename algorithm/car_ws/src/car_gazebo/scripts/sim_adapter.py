#!/usr/bin/env python3
"""Dedicated Gazebo SIM data adapter. Never opens physical UART."""
import math
import os
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from generate_scene import (SIM_DOMAIN, SIM_PREFIX, SIM_MODEL, WHEELS,
                            guarded_twist, wheel_rates)


class SimulationAdapter(Node):
    def __init__(self):
        super().__init__("car_gazebo_sim_adapter")
        if (os.environ.get("ROS_DOMAIN_ID") != SIM_DOMAIN or
                os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
            raise RuntimeError("SIM requires ROS_DOMAIN_ID=84 and ROS_LOCALHOST_ONLY=1")
        if not self.get_parameter("use_sim_time").value:
            raise RuntimeError("SIM requires use_sim_time:=true")
        self.declare_parameter("command_source", "manual")
        source = self.get_parameter("command_source").value
        if source not in ("manual", "nav2"):
            raise RuntimeError("command_source must be manual or nav2")
        input_topic = SIM_PREFIX + ("/cmd_vel" if source == "manual" else "/nav_checked_cmd")
        self.command = (0., 0., 0.)
        self.command_wall = self.odom_wall = self.scan_wall = self.clock_wall = -math.inf
        self.clock_stamp = self.last_odom_stamp = self.last_scan_stamp = 0
        self.wheel_angles = [0.] * 4
        self.last_warning = -math.inf
        self.drive = self.create_publisher(Twist, SIM_PREFIX + "/drive_cmd", 1)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        # OdometryPublisher uses worldPose; map/world/odom share origin in SIM.
        identity = TransformStamped()
        identity.header.frame_id = "map"
        identity.child_frame_id = "odom"
        identity.transform.rotation.w = 1.
        self.static_tf.sendTransform(identity)
        self.create_subscription(ClockMsg, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, SIM_PREFIX + "/odom_raw", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SIM_PREFIX + "/scan_raw", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, input_topic, self.on_command, 1)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady)
        self.get_logger().info("SIM only input: " + input_topic + "; no STM32 or REAL writes")

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 2.:
            self.get_logger().warning(message)
            self.last_warning = now

    def on_clock(self, msg):
        stamp = Time.from_msg(msg.clock).nanoseconds
        if stamp < self.clock_stamp:
            self.command = (0., 0., 0.)
            self.command_wall = self.odom_wall = self.scan_wall = -math.inf
            self.last_odom_stamp = self.last_scan_stamp = 0
            self.wheel_angles = [0.] * 4
        if stamp != self.clock_stamp:
            self.clock_wall = time.monotonic()
            self.clock_stamp = stamp

    def fresh_stamp(self, stamp):
        # SIM clock and delivery are asynchronous; allow 0.1s forward skew.
        return stamp > 0 and -100_000_000 <= self.clock_stamp - stamp <= 500_000_000

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.command_wall = time.monotonic()

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if (msg.header.frame_id != "odom" or msg.child_frame_id != "base_link" or
                not self.fresh_stamp(stamp) or stamp <= self.last_odom_stamp or
                not all(math.isfinite(a) for a in (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                                    v.x, v.y, w.z)) or
                abs(q.x) > .02 or abs(q.y) > .02 or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .03):
            self.warn("Rejected SIM odom: invalid frame, stamp or planar pose")
            return
        rates = wheel_rates(v.x, v.y, w.z)
        dt = (stamp - self.last_odom_stamp) / 1e9 if self.last_odom_stamp else 0.
        if 0 < dt <= .5:
            self.wheel_angles = [math.remainder(a + r*dt, 2*math.pi)
                                 for a, r in zip(self.wheel_angles, rates)]
        self.last_odom_stamp = stamp
        self.odom_wall = time.monotonic()
        self.odom.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = p.x
        tf.transform.translation.y = p.y
        tf.transform.translation.z = 0.
        tf.transform.rotation = q
        self.tf.sendTransform(tf)
        j = JointState()
        j.header = msg.header
        j.name = ["wheel_" + s + "_joint" for s, _, _ in WHEELS]
        j.position = self.wheel_angles
        j.velocity = list(rates)
        self.joints.publish(j)

    def on_scan(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        frame = msg.header.frame_id.replace("::", "/").strip("/")
        allowed = {"laser_frame", SIM_MODEL + "/base_link/gpu_lidar",
                   "car_sim/" + SIM_MODEL + "/base_link/gpu_lidar",
                   "world/car_sim/" + SIM_MODEL + "/base_link/gpu_lidar"}
        if (frame not in allowed or not self.fresh_stamp(stamp) or
                stamp <= self.last_scan_stamp or not msg.ranges or
                not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment,
                                                    msg.range_min, msg.range_max)) or
                msg.angle_increment <= 0 or not 0 <= msg.range_min < msg.range_max):
            self.warn("Rejected SIM scan frame/stamp/geometry: " + msg.header.frame_id)
            return
        self.last_scan_stamp = stamp
        self.scan_wall = time.monotonic()
        msg.header.frame_id = "laser_frame"
        self.scan.publish(msg)

    def tick(self):
        now = time.monotonic()
        vx, vy, wz = guarded_twist(self.command, now-self.command_wall,
                                   now-self.odom_wall, now-self.scan_wall, now-self.clock_wall)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.drive.publish(cmd)


def main():
    rclpy.init()
    node = None
    try:
        node = SimulationAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.drive.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
