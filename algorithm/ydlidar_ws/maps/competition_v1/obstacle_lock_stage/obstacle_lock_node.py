#!/usr/bin/env python3
"""ROS 2 Humble: 开赛双静态障碍物 -> 逻辑道路格 -> 锁定 -> 规划地图。
只做感知/规划预处理，不发布 /cmd_vel，不控制 STM32。
"""
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import MarkerArray, Marker

from obstacle_lock_core import (ObstacleLocker, NODES, EDGES, _distance_to_edge,
                                split_scan_into_objects)


def map_idx(msg, x, y):
    info = msg.info
    if info.resolution <= 0:
        return None
    col = math.floor((x - info.origin.position.x)/info.resolution)
    row = math.floor((y - info.origin.position.y)/info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return int(row*info.width + col)
    return None


def compatible_maps(a, b):
    aa, bb = a.info, b.info
    return (aa.width == bb.width and aa.height == bb.height
            and abs(aa.resolution-bb.resolution) < 1e-7
            and abs(aa.origin.position.x-bb.origin.position.x) < 1e-5
            and abs(aa.origin.position.y-bb.origin.position.y) < 1e-5
            and abs(aa.origin.orientation.z) < 1e-6
            and abs(bb.origin.orientation.z) < 1e-6)


def static_blocked_edges(mask, halfwidth=0.17):
    """保留原本的禁行图约束，未知区域视为不允许。"""
    result = set()
    for edge, (a, b) in EDGES.items():
        (ax,ay), (bx,by) = NODES[a], NODES[b]
        dx,dy = bx-ax, by-ay
        length = math.hypot(dx,dy)
        nx,ny = -dy/length, dx/length
        steps = max(1, math.ceil(length/.02))
        for j in range(steps+1):
            t=j/steps
            x,y = ax+t*dx,ay+t*dy
            if any((lambda idx: idx is None or mask.data[idx] != 0)(
                   map_idx(mask,x+u*nx,y+u*ny)) for u in (-halfwidth,0,halfwidth)):
                result.add(edge)
                break
    return result


def make_planning_map(base, keepout, locked, corridor_halfwidth,
                      band_halfthickness, ready):
    """生成独立规划图；未就绪时整张图占用，避免误当作已探测完毕。"""
    out = OccupancyGrid()
    out.header.frame_id = 'map'
    out.info = copy.deepcopy(base.info)
    if not ready:
        out.data = [100] * (base.info.width*base.info.height)
        return out

    data = list(base.data)
    for i in range(len(data)):
        if keepout.data[i] != 0 or data[i] < 0:
            data[i] = 100
    origin = base.info.origin.position
    res = base.info.resolution
    width = int(base.info.width)
    height = int(base.info.height)
    for obs in locked:
        _, _, (cx,cy) = _distance_to_edge(obs.x,obs.y,obs.cell)
        a,b = EDGES[obs.cell]
        (ax,ay),(bx,by) = NODES[a],NODES[b]
        dx,dy = bx-ax,by-ay
        norm = math.hypot(dx,dy)
        ux,uy = dx/norm,dy/norm
        nx,ny = -uy,ux
        radius = corridor_halfwidth + band_halfthickness
        c0 = max(0,int(math.floor((cx-radius-origin.x)/res)))
        c1 = min(width-1,int(math.ceil((cx+radius-origin.x)/res)))
        r0 = max(0,int(math.floor((cy-radius-origin.y)/res)))
        r1 = min(height-1,int(math.ceil((cy+radius-origin.y)/res)))
        for row in range(r0,r1+1):
            py = origin.y+(row+.5)*res
            for col in range(c0,c1+1):
                px = origin.x+(col+.5)*res
                lon = abs((px-cx)*ux+(py-cy)*uy)
                lat = abs((px-cx)*nx+(py-cy)*ny)
                if lon <= band_halfthickness and lat <= corridor_halfwidth:
                    data[row*width+col] = 100
    out.data = data
    return out


class CompetitionObstacleLock(Node):
    def __init__(self):
        super().__init__('competition_obstacle_lock')
        self.declare_parameter('start', 'C3')
        self.declare_parameter('goal', 'A3')
        self.declare_parameter('min_frames', 20)
        self.declare_parameter('min_hits', 4)
        self.declare_parameter('max_laser_range', 2.2)
        self.declare_parameter('corridor_halfwidth', 0.29)
        self.declare_parameter('barrier_halfthickness', 0.065)
        self.declare_parameter('stale_timeout', 3.0)
        self.start = self.get_parameter('start').value
        self.goal = self.get_parameter('goal').value
        if self.start not in NODES or self.goal not in NODES:
            raise ValueError('start/goal 必须为已有道路节点，如 C3、A3、B2')
        self.locker = ObstacleLocker(
            expected=2,
            min_frames=int(self.get_parameter('min_frames').value),
            min_hits=int(self.get_parameter('min_hits').value))
        self.base = None
        self.mask = None
        self.last_valid_scan = 0.0
        self.tf_error = ''
        self.static_blocked = set()
        self.mask_problem = ''
        self.tf_buf = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buf, self)
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos_latched)
        self.create_subscription(OccupancyGrid, '/keepout_filter_mask', self.on_mask, qos_latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, '/competition/obstacle_lock_status', qos_latched)
        self.pub_map = self.create_publisher(OccupancyGrid, '/competition/planning_map', qos_latched)
        self.pub_path = self.create_publisher(RosPath, '/competition/locked_road_path', qos_latched)
        self.pub_marker = self.create_publisher(MarkerArray, '/competition/locked_obstacle_markers', qos_latched)
        self.create_timer(1.0, self.publish)
        self.saved = False
        self.get_logger().warning('仅检测、锁定和生成路线；不会发送电机指令。')

    def on_map(self, msg):
        self.base = msg
        self.verify_maps()

    def on_mask(self, msg):
        self.mask = msg
        self.verify_maps()

    def verify_maps(self):
        if self.base is None or self.mask is None:
            return
        if not compatible_maps(self.base, self.mask):
            self.mask_problem = 'MAP_GEOMETRY_MISMATCH'
            return
        if self.base.header.frame_id != 'map' or self.mask.header.frame_id != 'map':
            self.mask_problem = 'MAP_FRAME_MISMATCH'
            return
        self.mask_problem = ''
        self.static_blocked = static_blocked_edges(self.mask)

    def is_scan_allowed(self, x, y):
        i = map_idx(self.base,x,y)
        j = map_idx(self.mask,x,y)
        return (i is not None and j is not None
                and self.base.data[i] == 0 and self.mask.data[j] == 0)

    def on_scan(self, msg):
        if self.base is None or self.mask is None or self.mask_problem:
            return
        try:
            tf = self.tf_buf.lookup_transform(
                'map', msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.tf_error = str(e)[:200]
            return
        self.tf_error = ''
        self.last_valid_scan = time.monotonic()
        tr = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)
        points = []
        max_range = float(self.get_parameter('max_laser_range').value)
        for i, dist in enumerate(msg.ranges):
            if not math.isfinite(dist) or not msg.range_min <= dist <= min(msg.range_max,max_range):
                points.append(None)
                continue
            a = msg.angle_min + i*msg.angle_increment
            xx,yy = dist*math.cos(a),dist*math.sin(a)
            x,y = tr.x+cy*xx-sy*yy, tr.y+sy*xx+cy*yy
            points.append((x,y) if self.is_scan_allowed(x,y) else None)
        candidates = split_scan_into_objects(points)
        self.locker.observe(candidates)

    def publish(self):
        stale_sec = float(self.get_parameter('stale_timeout').value)
        scan_fresh = (self.last_valid_scan > 0 and time.monotonic()-self.last_valid_scan < stale_sec)
        ok_maps = self.base is not None and self.mask is not None and not self.mask_problem
        state = ('WAIT_MAP' if not ok_maps else
                 'WAIT_TF_OR_SCAN' if not scan_fresh else self.locker.state)
        locked_and_fresh = state == 'LOCKED' and ok_maps and scan_fresh
        blocked = sorted(self.static_blocked | self.locker.blocked_cells)
        route = self.locker.route(self.start,self.goal,self.static_blocked) if locked_and_fresh else []
        ready = locked_and_fresh and bool(route)
        if self.mask_problem:
            reason = self.mask_problem
        elif not scan_fresh:
            reason = self.tf_error or 'SCAN_MISSING_OR_STALE'
        elif locked_and_fresh and not route:
            reason = 'NO_ROUTE'
        else:
            reason = ''
        status = dict(
            state=state, ready=ready, scans=self.locker.frame_no,
            expected_objects=2,
            locked_objects=[dict(x=round(o.x,3),y=round(o.y,3),road_cell=o.cell)
                            for o in self.locker.locked_objects],
            blocked_road_cells=blocked,
            route=route,
            reason=reason,
            ambiguous=self.locker.last_rejects[:5],
        )
        self.pub_status.publish(String(data=json.dumps(status,ensure_ascii=False)))
        if self.base is None or self.mask is None or self.mask_problem:
            return
        out = make_planning_map(
            self.base,self.mask,self.locker.locked_objects,
            float(self.get_parameter('corridor_halfwidth').value),
            float(self.get_parameter('barrier_halfthickness').value),
            ready=ready)
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(out)
        p = RosPath()
        p.header = out.header
        for n in route:
            pose = PoseStamped()
            pose.header = out.header
            pose.pose.position.x,pose.pose.position.y = NODES[n]
            pose.pose.orientation.w = 1.0
            p.poses.append(pose)
        self.pub_path.publish(p)
        marks=MarkerArray()
        for i,o in enumerate(self.locker.locked_objects):
            m=Marker()
            m.header=out.header
            m.ns='two_static_obstacles'
            m.id=i
            m.type=Marker.SPHERE
            m.action=Marker.ADD
            m.pose.position.x=o.x
            m.pose.position.y=o.y
            m.pose.position.z=.04
            m.pose.orientation.w=1.0
            m.scale.x=m.scale.y=.10
            m.scale.z=.08
            m.color.r=1.0
            m.color.g=.2
            m.color.b=.1
            m.color.a=1.0
            marks.markers.append(m)
        self.pub_marker.publish(marks)
        if self.locker.locked_objects and not self.saved:
            pth = Path(__file__).resolve().parent / 'last_lock_report.json'
            pth.write_text(json.dumps(status,ensure_ascii=False,indent=2))
            self.saved=True
            self.get_logger().warning('检测结果已锁定，报告写入: '+str(pth))


def main():
    rclpy.init()
    node=CompetitionObstacleLock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
