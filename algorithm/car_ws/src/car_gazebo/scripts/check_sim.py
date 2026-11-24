#!/usr/bin/env python3
"""Live SIM acceptance. Read-only by default, explicit test may trigger SIM route."""
import argparse
import json
import math
import os
from pathlib import Path
import time
from follow_locked_route import load_topology, route_contract, matching_lock, fresh_lock_scan


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--topology", type=Path, required=True)
    p.add_argument("--start", default="C3")
    p.add_argument("--goal", default="A1")
    p.add_argument("--ready-timeout", type=float, default=60.)
    p.add_argument("--execute-route", action="store_true")
    p.add_argument("--route-timeout", type=float, default=110.)
    args = p.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "84" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        p.error("SIM requires dedicated local ROS domain 84")
    if not all(math.isfinite(v) and 0 < v <= 120 for v in (args.ready_timeout, args.route_timeout)):
        p.error("timeouts must be within 0..120 seconds")
    nodes, edges = load_topology(args.topology)
    if args.start not in nodes or args.goal not in nodes or args.start == args.goal:
        p.error("distinct existing topology nodes required")

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, JointState
    from rosgraph_msgs.msg import Clock
    from nav2_msgs.action import FollowPath
    from lifecycle_msgs.srv import GetState
    from lifecycle_msgs.msg import State
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class Check(Node):
        def __init__(self):
            super().__init__("car_gazebo_acceptance")
            self.data, self.wall, self.progress, self.last_stamp = {}, {}, {}, {}
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            for topic, typ, name, qos in (
                    ("/clock", Clock, "clock", qos_profile_sensor_data),
                    ("/odom", Odometry, "odom", qos_profile_sensor_data),
                    ("/scan", LaserScan, "scan", qos_profile_sensor_data),
                    ("/joint_states", JointState, "joints", qos_profile_sensor_data),
                    ("/car_gz/drive_cmd", Twist, "drive", qos_profile_sensor_data),
                    ("/map", OccupancyGrid, "map", latched),
                    ("/keepout_filter_mask", OccupancyGrid, "mask", latched),
                    ("/competition/road_status", String, "road", latched),
                    ("/competition/obstacle_lock_status", String, "lock", latched),
                    ("/competition/locked_road_path", PathMsg, "path", latched),
                    ("/car_gz/route_status", String, "execution", latched)):
                self.create_subscription(typ, topic, lambda m, n=name: self.receive(n, m), qos)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.action = ActionClient(self, FollowPath, "/follow_path")
            self.lifecycle = self.create_client(GetState, "/controller_server/get_state")
            self.lifecycle_future = None
            self.controller_active = False
            self.lifecycle_wall = -math.inf
            self.create_timer(.5, self.poll_lifecycle)
            self.start_client = self.create_client(Trigger, "/car_gz/follow_locked_route")
            self.cancel_client = self.create_client(Trigger, "/car_gz/cancel_locked_route")
            self.moved = False
            self.initial = nodes[args.start]
            self.goal = nodes[args.goal]

        def poll_lifecycle(self):
            if self.lifecycle_future is not None:
                if not self.lifecycle_future.done():
                    return
                try:
                    reply = self.lifecycle_future.result()
                    self.controller_active = (reply is not None and
                        reply.current_state.id == State.PRIMARY_STATE_ACTIVE)
                except Exception:
                    self.controller_active = False
                self.lifecycle_wall = time.monotonic()
                self.lifecycle_future = None
            if self.lifecycle.service_is_ready():
                self.lifecycle_future = self.lifecycle.call_async(GetState.Request())

        def receive(self, name, msg):
            if name in ("clock", "odom", "scan", "joints"):
                stamp = Time.from_msg(msg.clock if name == "clock" else msg.header.stamp).nanoseconds
                if stamp <= self.last_stamp.get(name, -1):
                    return
                self.last_stamp[name] = stamp
                self.progress[name] = self.progress.get(name, 0)+1
            self.data[name] = msg
            self.wall[name] = time.monotonic()
            if name == "odom":
                xy = self.xy()
                self.moved |= all(math.isfinite(v) for v in xy) and math.dist(xy, self.initial) > .08

        def xy(self):
            pt = self.data["odom"].pose.pose.position
            return pt.x, pt.y

        def recent(self, name, age=1.):
            return time.monotonic() - self.wall.get(name, -math.inf) <= age

        def state(self, name):
            try:
                return json.loads(self.data[name].data)
            except (KeyError, ValueError, TypeError):
                return {}

        def stopped(self):
            if not self.recent("drive") or not self.recent("odom"):
                return False
            cmd = self.data["drive"]
            twist = self.data["odom"].twist.twist
            return (max(abs(cmd.linear.x), abs(cmd.linear.y), abs(cmd.angular.z)) <= 1e-6
                    and math.hypot(twist.linear.x, twist.linear.y) <= .025
                    and abs(twist.angular.z) <= .05)

        def problems(self):
            missing = ["advancing " + n for n in ("clock", "odom", "scan", "joints")
                       if not self.recent(n) or self.progress.get(n, 0) < 2]
            if missing:
                return missing
            odom, scan, joints = (self.data[k] for k in ("odom", "scan", "joints"))
            if (odom.header.frame_id != "odom" or odom.child_frame_id != "base_link" or
                    not all(math.isfinite(v) for v in self.xy()) or
                    math.dist(self.xy(), self.initial) > .06):
                missing.append("initial odom pose/frame")
            if scan.header.frame_id != "laser_frame" or sum(
                    math.isfinite(x) and scan.range_min <= x <= scan.range_max for x in scan.ranges) < 4:
                missing.append("finite simulated lidar returns")
            if set(joints.name) != {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"}:
                missing.append("wheel joints")
            if not self.buffer.can_transform("map", "laser_frame", Time()):
                missing.append("map -> laser_frame TF")
            for name in ("map", "mask"):
                grid = self.data.get(name)
                if (grid is None or grid.header.frame_id != "map" or
                        abs(grid.info.resolution-.01) > 1e-6 or
                        grid.info.width != 280 or grid.info.height != 280 or
                        len(grid.data) != 280*280):
                    missing.append(name + " geometry")
            try:
                msg = self.data["path"]
                if msg.header.frame_id != "map":
                    raise ValueError("path frame")
                points = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
                key = route_contract(self.state("road"), points, nodes, edges)
                if (key[0][0] != args.start or key[0][-1] != args.goal or
                        not matching_lock(self.state("lock"), key) or
                        not fresh_lock_scan(self.state("lock"), self.last_stamp.get("clock", 0)) or
                        any(not self.recent(n, 2.5) for n in ("road", "lock", "path"))):
                    raise ValueError("lock or route mismatch")
            except (KeyError, ValueError, TypeError):
                missing.append("confirmed live locked path")
            if not (self.controller_active and time.monotonic()-self.lifecycle_wall < 2. and
                    self.action.server_is_ready() and self.start_client.service_is_ready()
                    and self.cancel_client.service_is_ready()):
                missing.append("Nav2 action and SIM trigger")
            if not self.stopped():
                missing.append("stationary, zero drive")
            return missing

        def wait_until(self, predicate, seconds):
            end = time.monotonic() + seconds
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.10)
                if predicate():
                    return True
            return False

        def request(self, client):
            if not client.service_is_ready():
                raise RuntimeError("SIM service unavailable")
            future = client.call_async(Trigger.Request())
            if not self.wait_until(future.done, 5.):
                raise RuntimeError("SIM service timeout")
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError("SIM service refused: " + (response.message if response else "no response"))
            return response.message

    rclpy.init(args=[])
    node = None
    requested = False
    try:
        node = Check()
        if not node.wait_until(lambda: not node.problems(), args.ready_timeout):
            print("LOCK DIAGNOSTIC:", node.state("lock"), flush=True)
            print("ROAD DIAGNOSTIC:", node.state("road"), flush=True)
            raise RuntimeError("READINESS failed: " + ", ".join(node.problems()))
        print("PASS READY: clock/scan/odom/joints, TF, map/mask, live lock, route, zero drive", flush=True)
        print("LOCKED ROUTE:", node.state("road").get("route"), flush=True)
        if not args.execute_route:
            return 0
        requested = True
        print(node.request(node.start_client), flush=True)
        def terminal():
            return node.recent("execution", 1.) and node.state("execution").get("state") in (
                "SUCCEEDED", "FAILED", "STOPPED", "REJECTED")
        if not node.wait_until(terminal, args.route_timeout):
            raise RuntimeError("route action timeout")
        result = node.state("execution")
        if result.get("state") != "SUCCEEDED" or not node.moved:
            raise RuntimeError("route did not execute: " + str(result))
        if not node.wait_until(lambda: node.stopped() and math.dist(node.xy(), node.goal) <= .06, 5.):
            raise RuntimeError("final pose or zero-drive check failed")
        print("PASS ROUTE: goal error %.4f m, zero drive, stationary odom" %
              math.dist(node.xy(), node.goal), flush=True)
        requested = False
        return 0
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print("FAIL SIM ACCEPTANCE:", str(exc), flush=True)
        return 1
    finally:
        if node is not None:
            if requested and rclpy.ok():
                try:
                    node.request(node.cancel_client)
                except Exception as exc:
                    print("cancel not acknowledged:", str(exc), flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
