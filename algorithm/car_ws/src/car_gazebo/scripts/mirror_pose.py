#!/usr/bin/env python3
"""Read a live map->base_link TF and update only the dedicated Gazebo mirror."""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class Mirror(Node):
    def __init__(self):
        super().__init__("car_gazebo_real_mirror")
        if os.environ.get("ROS_DOMAIN_ID") == "84":
            raise RuntimeError("REAL mirror is forbidden in dedicated Gazebo SIM domain 84")
        self.declare_parameter("enable_real_mirror", False)
        if not self.get_parameter("enable_real_mirror").value:
            raise RuntimeError("Explicit enable_real_mirror:=true is required")
        if self.get_parameter("use_sim_time").value:
            raise RuntimeError("REAL mirror requires wall time, not simulated time")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(SetEntityPose, "/world/car_mirror/set_pose")
        self.pending = None
        self.deadline = 0.0
        self.last_warning = 0.0
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info("Read-only REAL TF mirror; no hardware commands")

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 2.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def tick(self):
        if self.pending is not None:
            if self.pending.done():
                try:
                    reply = self.pending.result()
                    if reply is None or not reply.success:
                        self.warn("Gazebo rejected pose; display not confirmed")
                except Exception as exc:
                    self.warn("Gazebo service failed: " + str(exc))
                self.pending = None
            elif time.monotonic() >= self.deadline:
                # Never accumulate requests or retry an unresolved command.
                self.client.remove_pending_request(self.pending)
                self.pending.cancel()
                self.pending = None
                self.warn("Gazebo pose request timed out")
                return
            else:
                return
        if not self.client.service_is_ready():
            self.warn("Mirror set_pose service unavailable")
            return
        try:
            t = self.buffer.lookup_transform("map", "base_link", Time())
        except TransformException as exc:
            self.warn("No real map->base_link TF: " + str(exc))
            return
        stamp = Time.from_msg(t.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if stamp.nanoseconds == 0 or not -0.1 <= age <= 0.5:
            self.warn("Stale/future TF; retaining last displayed pose")
            return
        p, q = t.transform.translation, t.transform.rotation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.warn("Nonfinite pose; no update")
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0.99 <= norm <= 1.01:
            self.warn("Invalid quaternion; no update")
            return
        x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        request = SetEntityPose.Request()
        request.entity.name = "car_mirror_model"
        request.entity.type = Entity.MODEL
        request.pose.position.x = p.x
        request.pose.position.y = p.y
        request.pose.position.z = 0.001
        request.pose.orientation.z = math.sin(yaw/2)
        request.pose.orientation.w = math.cos(yaw/2)
        self.pending = self.client.call_async(request)
        self.deadline = time.monotonic() + 0.5


def main():
    rclpy.init()
    node = None
    try:
        node = Mirror()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
