#!/usr/bin/env python3
"""SIM-only action gate: existing locked Dijkstra path -> Nav2 FollowPath."""
import importlib.util
import json
import math
import os
from pathlib import Path
import time


def load_topology(path):
    spec = importlib.util.spec_from_file_location("car_existing_topology", path)
    if spec is None or spec.loader is None:
        raise ValueError("invalid topology file")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.NODES, mod.EDGES


def route_contract(status, points, nodes, edges):
    """Validate the same road IDs and coordinates on both ROS messages."""
    if not isinstance(status, dict) or (
            status.get("status") != "LOCKED_READY" or status.get("ready") is not True or
            status.get("frozen") is not True or status.get("conflict") is not False or
            status.get("source") != "LIDAR_CONFIRMED"):
        raise ValueError("route not confirmed by lidar")
    names = status.get("route")
    blocked = status.get("blocked")
    if (not isinstance(names, list) or not 2 <= len(names) <= len(nodes) or
            not all(isinstance(n, str) and n in nodes for n in names) or
            len(set(names)) != len(names) or not isinstance(blocked, list) or
            not blocked or not all(isinstance(e, str) and e in edges for e in blocked)):
        raise ValueError("invalid node/road set")
    road_by_nodes = {frozenset(v): k for k, v in edges.items()}
    for a, b in zip(names, names[1:]):
        key = road_by_nodes.get(frozenset((a, b)))
        if key is None or key in blocked:
            raise ValueError("route crosses blocked or nonexistent road")
    if len(points) != len(names):
        raise ValueError("path/status length mismatch")
    for name, point in zip(names, points):
        if not all(math.isfinite(v) for v in point) or math.dist(nodes[name], point) > .001:
            raise ValueError("path/status coordinates mismatch")
    return tuple(names), tuple(sorted(set(blocked)))


def matching_lock(lock, route_key):
    """Require the original lock heartbeat as well as the derived route."""
    if (not isinstance(lock, dict) or lock.get("state") != "LOCKED" or
            lock.get("locked") is not True or lock.get("map_received") is not True or
            lock.get("expected_obstacles") != 2):
        return False
    obstacles, roads = lock.get("obstacles"), lock.get("blocked_roads")
    return (isinstance(obstacles, list) and len(obstacles) == 2 and
            isinstance(roads, list) and all(isinstance(r, str) for r in roads) and
            tuple(sorted(set(roads))) == route_key[1])


def fresh_lock_scan(lock, clock_stamp):
    """A live lock heartbeat must contain recent exact-time transformed scan evidence."""
    stamp = lock.get("scan_tf_stamp_ns") if isinstance(lock, dict) else None
    return (isinstance(lock, dict) and lock.get("scan_tf_ready") is True
            and type(stamp) is int and stamp > 0
            and type(clock_stamp) is int and clock_stamp > 0
            and -100_000_000 <= clock_stamp - stamp <= 1_500_000_000)


def dense_path(points, spacing=.025):
    if len(points) < 2 or not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("invalid interpolation")
    output = [points[0]]
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if not math.isfinite(length) or length <= 0:
            raise ValueError("invalid segment")
        count = math.ceil(length / spacing)
        output.extend((a[0] + (b[0]-a[0])*i/count,
                       a[1] + (b[1]-a[1])*i/count) for i in range(1, count+1))
    return output


def segment_paths(names, nodes):
    """One immutable path per adjacent edge of the already validated locked route."""
    if len(names) < 2:
        raise ValueError("route requires at least one edge")
    return tuple(tuple(dense_path([nodes[a], nodes[b]])) for a, b in zip(names, names[1:]))


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "84" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Route execution requires isolated local SIM domain 84")
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as PathMsg
    from nav2_msgs.action import FollowPath
    from rosgraph_msgs.msg import Clock as ClockMsg
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class RouteGate(Node):
        def __init__(self):
            super().__init__("car_sim_route_gate")
            if not self.get_parameter("use_sim_time").value:
                raise RuntimeError("SIM route gate requires simulation clock")
            self.declare_parameter("topology_file", "")
            path = Path(self.get_parameter("topology_file").value)
            if not path.is_absolute() or not path.is_file():
                raise RuntimeError("explicit existing topology file required")
            self.nodes, self.edges = load_topology(path)
            self.status = self.lock_status = None
            self.lock_wall = -math.inf
            self.clock_reset = False
            self.points = []
            self.xy = None
            self.odom_speed = None
            self.completed_segments = 0
            self.total_segments = 0
            self.status_wall = self.path_wall = self.odom_wall = self.scan_wall = self.clock_wall = -math.inf
            self.clock_stamp = 0
            self.run = None
            self.state = "WAIT_EXPLICIT_START"
            self.last_reason = ""
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, "/competition/road_status", self.on_status, latched)
            self.create_subscription(String, "/competition/obstacle_lock_status", self.on_lock, latched)
            self.create_subscription(PathMsg, "/competition/locked_road_path", self.on_path, latched)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(ClockMsg, "/clock", self.on_clock, qos_profile_sensor_data)
            self.create_subscription(Twist, "/car_gz/nav_cmd_vel", self.on_velocity, 1)
            self.out = self.create_publisher(Twist, "/car_gz/nav_checked_cmd", 1)
            self.report = self.create_publisher(String, "/car_gz/route_status", latched)
            self.client = ActionClient(self, FollowPath, "/follow_path")
            self.create_service(Trigger, "/car_gz/follow_locked_route", self.start)
            self.create_service(Trigger, "/car_gz/cancel_locked_route", self.cancel)
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.10, self.tick, clock=self.steady)
            self.get_logger().info("SIM only; explicit service trigger required for each route")

        def on_status(self, msg):
            try:
                self.status = json.loads(msg.data)
            except (TypeError, ValueError):
                self.status = None
            self.status_wall = time.monotonic()

        def on_lock(self, msg):
            try:
                self.lock_status = json.loads(msg.data)
            except (TypeError, ValueError):
                self.lock_status = None
            self.lock_wall = time.monotonic()

        def on_path(self, msg):
            self.points = []
            if msg.header.frame_id == "map" and all(
                    p.header.frame_id == "map" for p in msg.poses):
                self.points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.path_wall = time.monotonic()

        def on_odom(self, msg):
            p = msg.pose.pose.position
            v, w = msg.twist.twist.linear, msg.twist.twist.angular
            if (msg.header.frame_id == "odom" and msg.child_frame_id == "base_link" and
                    all(math.isfinite(a) for a in (p.x, p.y, v.x, v.y, w.z)) and
                    self.valid_stamp(msg.header.stamp)):
                self.xy = (p.x, p.y)
                self.odom_speed = (math.hypot(v.x, v.y), abs(w.z))
                self.odom_wall = time.monotonic()

        def on_scan(self, msg):
            if msg.header.frame_id == "laser_frame" and msg.ranges and self.valid_stamp(msg.header.stamp):
                self.scan_wall = time.monotonic()

        def on_clock(self, msg):
            stamp = Time.from_msg(msg.clock).nanoseconds
            if stamp < self.clock_stamp:
                self.clock_reset = True
                self.stop("SIM_CLOCK_RESET_RESTART_NAV")
                self.odom_wall = self.scan_wall = -math.inf
                self.status_wall = self.path_wall = self.lock_wall = -math.inf
            if stamp != self.clock_stamp:
                self.clock_stamp = stamp
                self.clock_wall = time.monotonic()

        def valid_stamp(self, stamp):
            value = Time.from_msg(stamp).nanoseconds
            return value > 0 and -100_000_000 <= self.clock_stamp - value <= 500_000_000

        def live_contract(self):
            now = time.monotonic()
            if self.clock_reset:
                raise ValueError("SIM clock reset: restart navigation to discard old lock")
            if (any(now-v > 2.5 for v in (self.status_wall, self.path_wall, self.lock_wall)) or
                    any(now-v > .5 for v in (self.odom_wall, self.scan_wall, self.clock_wall))):
                raise ValueError("stale lock, route or simulation data")
            key = route_contract(self.status, self.points, self.nodes, self.edges)
            if not matching_lock(self.lock_status, key):
                raise ValueError("original lidar lock disagrees with route")
            if not fresh_lock_scan(self.lock_status, self.clock_stamp):
                raise ValueError("no recent exact-time TF scan evidence")
            return key

        def start(self, request, response):
            del request
            try:
                if self.run is not None:
                    raise ValueError("previous action has not terminated")
                key = self.live_contract()
                if self.xy is None or math.dist(self.xy, self.points[0]) > .06:
                    raise ValueError("robot must be at route start within 0.06 m")
                if not self.client.server_is_ready():
                    raise ValueError("Nav2 FollowPath action not ready")
                legs = segment_paths(key[0], self.nodes)
                self.completed_segments, self.total_segments = 0, len(legs)
                self.run = {"key": key, "legs": legs, "index": 0,
                            "started": time.monotonic(), "phase": "REQUESTED",
                            "handle": None, "stopping": False}
                self.send_segment()
                response.success = True
                response.message = "SIM route requested; each locked edge is executed and verified in order"
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def send_segment(self):
            run = self.run
            leg = run["legs"][run["index"]]
            if (self.xy is None or math.dist(self.xy, leg[0]) > .06
                    or self.live_contract() != run["key"]
                    or not self.client.server_is_ready()):
                self.finish("FAILED", "segment start or live contract invalid")
                raise ValueError(self.last_reason)
            path = PathMsg()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            for x, y in leg:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.w = 1.
                path.poses.append(p)
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = "FollowPath"
            goal.goal_checker_id = "goal_checker"
            run["phase"], run["request_wall"] = "REQUESTED", time.monotonic()
            run["handle"] = None
            self.state, self.last_reason = "REQUESTED", ""
            a, b = run["key"][0][run["index"]:run["index"]+2]
            self.get_logger().info(f"SIM SEGMENT REQUEST: {a}->{b} ({run['index']+1}/{len(run['legs'])})")
            try:
                self.client.send_goal_async(goal).add_done_callback(self.accepted)
            except Exception as exc:
                self.finish("FAILED", str(exc))
                raise

        def accepted(self, future):
            if self.run is None:
                return
            try:
                handle = future.result()
            except Exception as exc:
                self.finish("FAILED", str(exc))
                return
            if handle is None or not handle.accepted:
                self.finish("STOPPED" if self.run["stopping"] else "REJECTED",
                            self.last_reason if self.run["stopping"] else "Nav2 rejected goal")
                return
            self.run["handle"] = handle
            if self.run["stopping"]:
                handle.cancel_goal_async()
            else:
                self.run["phase"] = self.state = "FOLLOWING"
            handle.get_result_async().add_done_callback(self.finished)

        def stop(self, reason):
            self.out.publish(Twist())
            if self.run is not None and not self.run["stopping"]:
                if self.run["phase"] == "SETTLING":
                    self.finish("STOPPED", reason)
                    return
                self.run["stopping"] = True
                self.state, self.last_reason = "STOPPING", reason
                if self.run["handle"] is not None:
                    self.run["handle"].cancel_goal_async()

        def cancel(self, request, response):
            del request
            self.stop("OPERATOR_CANCEL")
            response.success = True
            response.message = "SIM velocity blocked; action cancellation requested"
            return response

        def finish(self, state, reason):
            self.out.publish(Twist())
            self.run = None
            self.state, self.last_reason = state, reason

        def finished(self, future):
            if self.run is None:
                return
            try:
                status = future.result().status
                if self.run["stopping"]:
                    self.finish("STOPPED", self.last_reason)
                    return
                leg = self.run["legs"][self.run["index"]]
                if (status != GoalStatus.STATUS_SUCCEEDED or self.xy is None or
                        self.live_contract() != self.run["key"] or
                        time.monotonic()-self.odom_wall > .5 or math.dist(self.xy, leg[-1]) > .06):
                    self.finish("FAILED", f"Nav2 segment {self.run['index']+1} status={status}, pose={self.xy}, target={leg[-1]}")
                    return
                self.out.publish(Twist())
                self.run["handle"] = None
                self.run["phase"] = self.state = "SETTLING"
                self.run["settle_wall"] = time.monotonic()
            except Exception as exc:
                self.finish("FAILED", str(exc))

        def settle(self):
            run = self.run
            now = time.monotonic()
            self.out.publish(Twist())
            if now-run["settle_wall"] > 3.:
                self.finish("FAILED", "segment stationary confirmation timeout")
                return
            target = run["legs"][run["index"]][-1]
            # Fresh post-result odometry and a zero-command interval at each junction.
            if (now-run["settle_wall"] < .15 or self.odom_wall < run["settle_wall"]
                    or self.odom_speed is None or self.odom_speed[0] > .025
                    or self.odom_speed[1] > .05 or math.dist(self.xy, target) > .06):
                return
            self.completed_segments += 1
            a, b = run["key"][0][run["index"]:run["index"]+2]
            self.get_logger().info(f"SIM SEGMENT PASS: {a}->{b}, error={math.dist(self.xy, target):.4f} m, stationary")
            if self.completed_segments == self.total_segments:
                self.finish("SUCCEEDED", "all locked edges completed with stationary odometry")
                return
            run["index"] += 1
            try:
                self.send_segment()
            except Exception as exc:
                self.finish("FAILED", str(exc))

        def valid_run(self):
            if self.run is None or self.run["stopping"]:
                return False
            try:
                if self.live_contract() != self.run["key"]:
                    raise ValueError("locked route changed")
                if time.monotonic()-self.run["started"] > 120.:
                    raise ValueError("route wall-time limit")
                if (self.run["phase"] == "REQUESTED" and
                        time.monotonic()-self.run["request_wall"] > 3.):
                    raise ValueError("action acceptance timeout")
            except ValueError as exc:
                self.stop(str(exc))
                return False
            return True

        def on_velocity(self, msg):
            if self.valid_run() and self.run["phase"] == "FOLLOWING" and self.run["handle"] is not None:
                if all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
                    self.out.publish(msg)
                else:
                    self.stop("NONFINITE_NAV_COMMAND")

        def tick(self):
            if not self.valid_run():
                self.out.publish(Twist())
            elif self.run["phase"] == "SETTLING":
                self.settle()
            elif self.run["phase"] != "FOLLOWING" or self.run["handle"] is None:
                self.out.publish(Twist())
            self.report.publish(String(data=json.dumps({"source_type": "GAZEBO_SIM",
                "state": self.state, "reason": self.last_reason, "active": self.run is not None,
                "completed_segments": self.completed_segments, "total_segments": self.total_segments})))

    rclpy.init()
    node = None
    try:
        node = RouteGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop("SHUTDOWN")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
