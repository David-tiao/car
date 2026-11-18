#!/usr/bin/env python3
"""Read-only ROS2 readiness check. Does not open a device or command a robot."""
import argparse
import json
import math
import os
import sys
import time


def run(mode, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    class Inspector(Node):
        def __init__(self):
            super().__init__('competition_lock_preflight_%d' % os.getpid())
            self.grid = None
            self.scan = None
            self.lock_status = None
            self.route_status = None
            self.buffer = Buffer()
            self.tf_listener = TransformListener(self.buffer, self)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            if mode == 'modules':
                self.create_subscription(String, '/competition/obstacle_lock_status',
                                         self.on_lock, qos)
                self.create_subscription(String, '/competition/road_status',
                                         self.on_route, qos)

        def on_map(self, msg):
            self.grid = msg

        def on_scan(self, msg):
            self.scan = msg

        def on_lock(self, msg):
            try:
                self.lock_status = json.loads(msg.data)
            except ValueError:
                self.lock_status = None

        def on_route(self, msg):
            try:
                self.route_status = json.loads(msg.data)
            except ValueError:
                self.route_status = None

    rclpy.init()
    node = Inspector()
    try:
        names = ['%s%s' % (ns.rstrip('/'), '/' + name) for name, ns
                 in node.get_node_names_and_namespaces()]
        if mode == 'inputs' and any(x in names for x in
                                    ('/competition_obstacle_lock', '/competition_stage6_fixed_route',
                                     '/competition_stage6_road_topology')):
            raise RuntimeError('已有固定路线节点运行，拒绝重复启动；先检查 operate_lock.sh status')
        expires = time.monotonic() + seconds
        fail_reason = '超时等待 /scan、/map 或有效 map->laser TF'
        while time.monotonic() < expires:
            rclpy.spin_once(node, timeout_sec=.15)
            if mode == 'modules':
                if (isinstance(node.lock_status, dict) and
                        isinstance(node.route_status, dict) and
                        node.route_status.get('status') in
                        ('WAIT_FOR_LOCK', 'LOCKED_READY', 'NO_ROUTE', 'CONFLICT_STOP') and
                        node.lock_status.get('state') in ('SCANNING', 'LOCKED', 'CONFLICT')):
                    print('[READY] 感知与规划状态话题均收到有效 JSON')
                    print('[STATE] obstacle=%s road=%s' %
                          (node.lock_status['state'], node.route_status['status']))
                    return 0
                fail_reason = '等待 /competition/obstacle_lock_status 与 /competition/road_status'
                continue
            if node.grid is None or node.scan is None:
                continue
            g, s = node.grid, node.scan
            if g.header.frame_id != 'map' or g.info.resolution <= 0 or not g.data:
                fail_reason = '/map 无效：需要 map frame 与有效 OccupancyGrid'
                continue
            if not s.header.frame_id or not math.isfinite(s.angle_increment) or not s.ranges:
                fail_reason = '/scan 无效：缺少 frame_id 或有效量测'
                continue
            if not any(math.isfinite(x) and s.range_min <= x <= s.range_max for x in s.ranges):
                fail_reason = '/scan 无有效量测'
                continue
            try:
                node.buffer.lookup_transform('map', s.header.frame_id,
                                             Time.from_msg(s.header.stamp),
                                             timeout=Duration(seconds=.05))
            except Exception as exc:
                fail_reason = 'map 到 %s 的 TF 不可用：%s' % (s.header.frame_id, exc)
                continue
            print('[READY] /scan、/map、map->%s TF 已收到' % s.header.frame_id)
            print('[WARN] TF 能查询成功不代表真实小车位姿已标定；禁止在假 TF 下移动')
            return 0
        raise RuntimeError(fail_reason)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['inputs', 'modules'], default='inputs')
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout 必须为正数')
    try:
        return run(args.mode, args.timeout)
    except (ImportError, RuntimeError) as exc:
        print('[ERROR] %s' % exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
