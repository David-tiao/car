#!/usr/bin/env python3
"""ROS 2 Humble: static lidar scan -> map-frame clusters -> irreversible road lock.

Do NOT use the old stage5 TEST TF for a moving robot. No motor commands.
"""
import argparse
import json
import math
import time
from pathlib import Path

from lock_core import ObstacleLock, laser_clusters


def static_map_free(grid, x, y):
    """No invention of obstacles at known static occupied or unknown cells."""
    if grid is None:
        return False
    info = grid.info
    if (grid.header.frame_id != 'map' or info.resolution <= 0 or
            not math.isfinite(info.resolution) or info.width <= 0 or info.height <= 0 or
            len(grid.data) != info.width * info.height):
        return False
    # Current project's maps have zero yaw; reject other map geometry explicitly.
    q = info.origin.orientation
    if abs(q.x) > 1e-4 or abs(q.y) > 1e-4 or abs(q.z) > 1e-4 or abs(q.w - 1.) > 1e-3:
        return False
    i = math.floor((x - info.origin.position.x) / info.resolution)
    j = math.floor((y - info.origin.position.y) / info.resolution)
    if not 0 <= i < info.width or not 0 <= j < info.height:
        return False
    return grid.data[j * info.width + i] == 0


def main():
    parser = argparse.ArgumentParser(description='静止起点的雷达障碍物地图位置与道路锁定')
    parser.add_argument('--expected', type=int, default=2, help='本次测试确认的障碍物数量')
    parser.add_argument('--hits', type=int, default=5)
    parser.add_argument('--settle-frames', type=int, default=3)
    parser.add_argument('--corridor-radius', type=float, default=.23)
    parser.add_argument('--snapshot', type=Path, default=None,
                        help='可选：将本次锁定状态另存为 JSON；不自动加载旧比赛状态')
    args = parser.parse_args()
    if (args.expected < 1 or args.hits < 1 or args.settle_frames < 1 or
            not math.isfinite(args.corridor_radius) or args.corridor_radius <= 0):
        parser.error('expected、hits、settle-frames、corridor-radius 必须为正数')

    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

    class LockNode(Node):
        def __init__(self):
            super().__init__('competition_obstacle_lock')
            self.grid = None
            self.engine = ObstacleLock(expected=args.expected, required_hits=args.hits,
                                       settle_frames=args.settle_frames,
                                       corridor_radius=args.corridor_radius)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.pub_status = self.create_publisher(String, '/competition/obstacle_lock_status', latched)
            self.pub_markers = self.create_publisher(MarkerArray, '/competition/locked_obstacle_markers', latched)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan,
                                                     qos_profile_sensor_data)
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.timer = self.create_timer(1.0, self.publish)
            self.last_warning = 0.
            self.saved = False
            self.get_logger().warning('仅限静止已知位姿的识别测试！移动底盘必须接入真实 TF。')

        def on_map(self, msg):
            self.grid = msg

        def on_scan(self, msg):
            if self.grid is None or self.engine.conflict:
                # A conflict freezes nothing new; keep publishing a diagnostic.
                # If the third target disappears while still scanning, restart the
                # test process before attempting a new lock.
                return
            try:
                trans = self.buffer.lookup_transform('map', msg.header.frame_id,
                    Time.from_msg(msg.header.stamp), timeout=Duration(seconds=.1))
            except Exception as exc:
                now = time.monotonic()
                if now - self.last_warning > 4:
                    self.get_logger().warning(f'map -> {msg.header.frame_id} TF 无效: {exc}')
                    self.last_warning = now
                return
            tr = trans.transform.translation
            q = trans.transform.rotation
            # Only a horizontal laser and a planar map transform are supported.
            # A valid TF alone cannot prove the real mounting pose is calibrated.
            if (not all(math.isfinite(v) for v in (tr.x, tr.y, tr.z,
                                                   q.x, q.y, q.z, q.w)) or
                    abs(q.x) > .02 or abs(q.y) > .02 or
                    abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .03):
                now = time.monotonic()
                if now - self.last_warning > 4:
                    self.get_logger().error('无法使用倾斜、非单位或非有限的 map->laser_frame TF')
                    self.last_warning = now
                return
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            groups = laser_clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                                    min_range=max(msg.range_min, .10),
                                    max_range=min(msg.range_max, 1.80))
            candidates = []
            for lx, ly, count in groups:
                x = tr.x + c * lx - s * ly
                y = tr.y + s * lx + c * ly
                # Known field; do not accept readings in exterior / fixtures.
                if not 0 <= x <= 2.4 or not 0 <= y <= 2.4:
                    continue
                if not static_map_free(self.grid, x, y):
                    continue
                candidates.append((x, y))
            previous = self.engine.state
            self.engine.observe(candidates)
            if self.engine.state != previous:
                self.get_logger().warning('障碍物状态: %s, %s' %
                                          (self.engine.state, self.engine.status()))
            if self.engine.locked_obstacles is not None and args.snapshot and not self.saved:
                try:
                    # Exclusive create: never overwrite a previous competition record.
                    with args.snapshot.open('x', encoding='utf-8') as snapshot:
                        json.dump(self.engine.status(), snapshot, ensure_ascii=False, indent=2)
                    self.saved = True
                    self.get_logger().info(f'已保存锁定快照: {args.snapshot}')
                except OSError as exc:
                    self.get_logger().error(f'无法保存快照 {args.snapshot}: {exc}')
                    # Scan-side publication still reports LOCKED; file persistence
                    # was not requested as a precondition for planning.
                    self.saved = True

        def publish(self):
            data = self.engine.status()
            data['map_received'] = self.grid is not None
            self.pub_status.publish(String(data=json.dumps(data, ensure_ascii=False)))
            if not data['locked']:
                return
            arr = MarkerArray()
            for i, obj in enumerate(data['obstacles']):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = 'locked_obstacles', i
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose.position.x, marker.pose.position.y = obj['x'], obj['y']
                marker.pose.position.z = .05
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = .06
                marker.scale.z = .10
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .1, .1, 1.
                arr.markers.append(marker)
            self.pub_markers.publish(arr)

    rclpy.init()
    node = LockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
