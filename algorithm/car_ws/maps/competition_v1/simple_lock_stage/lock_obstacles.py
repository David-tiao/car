#!/usr/bin/env python3
"""ROS 2 Humble: static lidar scan -> map-frame clusters -> irreversible road lock.

Do NOT use the old stage5 TEST TF for a moving robot. No motor commands.
"""
import argparse
from collections import deque
import json
import math
import time
from pathlib import Path

from lock_core import ObstacleLock, laser_clusters


class PendingScans:
    """Bounded FIFO of distinct timestamped scans; never block the TF executor."""
    def __init__(self, max_pending=16, max_wait=.5):
        self.pending = deque()
        self.max_pending, self.max_wait = max_pending, max_wait
        self.last_stamp = 0
        self.expired = self.overflow = self.rejected = 0

    def push(self, stamp, msg, now):
        if stamp <= self.last_stamp:
            self.rejected += 1
            return
        self.last_stamp = stamp
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.overflow += 1
        self.pending.append((stamp, msg, now))

    def drain(self, now, lookup, consume, limit=4):
        for _ in range(limit):
            if not self.pending:
                break
            stamp, msg, arrived = self.pending[0]
            if not 0 <= now - arrived <= self.max_wait:
                self.pending.popleft()
                self.expired += 1
                continue
            transform = lookup(msg)
            if transform is None:
                break  # yield to executor; TF subscriptions must be able to run
            self.pending.popleft()
            consume(stamp, msg, transform)

    def counters(self):
        return {"pending": len(self.pending), "expired": self.expired,
                "overflow": self.overflow, "rejected": self.rejected}


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
    from rclpy.clock import Clock, ClockType
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
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.pending = PendingScans()
            self.last_processed_wall = -math.inf
            self.last_processed_stamp = 0
            self.processed_count = 0
            self.timer = self.create_timer(1.0, self.publish, clock=self.steady)
            self.scan_timer = self.create_timer(.02, self.process_pending, clock=self.steady)
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
            stamp = Time.from_msg(msg.header.stamp).nanoseconds
            if (stamp <= 0 or not msg.header.frame_id or not msg.ranges or
                    not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment,
                                                       msg.range_min, msg.range_max)) or
                    msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max):
                return
            self.pending.push(stamp, msg, time.monotonic())

        def lookup_scan(self, msg):
            try:
                # No timeout: this callback and the TransformListener share an executor.
                # Exact nonzero scan stamp, never latest-time or fabricated TF.
                return self.buffer.lookup_transform('map', msg.header.frame_id,
                    Time.from_msg(msg.header.stamp))
            except Exception as exc:
                now = time.monotonic()
                if now - self.last_warning > 4:
                    self.get_logger().warning(f'等待同时间戳 TF: {exc}; queue={self.pending.counters()}')
                    self.last_warning = now
                return None

        def process_pending(self):
            if self.grid is None or self.engine.conflict:
                self.pending.pending.clear()
                return
            self.pending.drain(time.monotonic(), self.lookup_scan, self.process_scan)

        def process_scan(self, stamp, msg, trans):
            if self.engine.conflict:
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
            self.last_processed_wall = time.monotonic()
            self.last_processed_stamp = stamp
            self.processed_count += 1
            previous = self.engine.state
            self.engine.observe(candidates)
            if self.engine.state != previous:
                self.get_logger().warning('障碍物状态: %s, %s' %
                                          (self.engine.state, self.engine.status()))
                if self.engine.conflict:
                    confirmed = [(round(t.x, 4), round(t.y, 4), t.hits)
                                 for t in self.engine.tracks
                                 if t.hits >= self.engine.required_hits and t.missed == 0]
                    self.get_logger().warning(
                        'CONFLICT evidence: scan_points=%d angle_min=%.6f increment=%.6f '
                        'clusters=%d map_candidates=%s confirmed_tracks=%s' %
                        (len(msg.ranges), msg.angle_min, msg.angle_increment,
                         len(groups), [(round(x, 4), round(y, 4)) for x, y in candidates],
                         confirmed))
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
            data['scan_tf_ready'] = (not self.engine.conflict and
                                     time.monotonic() - self.last_processed_wall <= .5)
            data['scan_tf_stamp_ns'] = self.last_processed_stamp
            data['scan_tf_processed'] = self.processed_count
            data['scan_queue'] = self.pending.counters()
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
