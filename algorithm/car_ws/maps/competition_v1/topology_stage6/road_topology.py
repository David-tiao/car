#!/usr/bin/env python3
"""
工创比赛第六阶段：道路级路线和可视化。

保留原有道路拓扑、Dijkstra 和标记生成方法；使用确认后的雷达
障碍物道路锁定结果，仅在第一次收到有效锁定时计算任务路线。
/competition/locked_road_path 仅供 RViz 预览，不直接控制底盘。
"""

import argparse
import heapq
import json
import math
import time

COL = {'A': 0.35, 'B': 1.20, 'C': 2.05}
ROW = {'1': 0.35, '2': 1.20, '3': 2.05}

NODES = {
    c+r: (x, y)
    for c, x in COL.items()
    for r, y in ROW.items()
}

EDGES = {}

for r in ROW:
    for a, b in [('A', 'B'), ('B', 'C')]:
        EDGES[f'{a}{r}-{b}{r}'] = (
            f'{a}{r}', f'{b}{r}'
        )

for c in COL:
    for a, b in [('1', '2'), ('2', '3')]:
        EDGES[f'{c}{a}-{c}{b}'] = (
            f'{c}{a}', f'{c}{b}'
        )

# 半宽150mm + 暂定20mm裕量
# 实车仍需校核车体尺寸、转弯和定位误差
CLEARANCE = 0.17


def canonical(raw):
    raw = raw.upper().strip()

    if raw in EDGES:
        return raw

    a, b = raw.split('-') if '-' in raw else ('', '')
    reverse = f'{b}-{a}'

    if reverse in EDGES:
        return reverse

    raise ValueError('无效路段：' + raw)


def shortest(start, goal, blocked):
    queue = [(0.0, (start,))]
    best = {start: 0.0}

    while queue:
        cost, path = heapq.heappop(queue)
        here = path[-1]

        if cost > best.get(here, math.inf) + 1e-7:
            continue

        if here == goal:
            return list(path)

        for key, (a, b) in EDGES.items():
            if key in blocked or here not in (a, b):
                continue

            nxt = b if here == a else a
            nd = cost + math.dist(
                NODES[here], NODES[nxt]
            )

            if nd < best.get(nxt, math.inf) - 1e-7:
                best[nxt] = nd
                heapq.heappush(
                    queue, (nd, path + (nxt,))
                )

    return []


def plan_mission(waypoints, blocked):
    """Calculate each requested road-level leg once; None means unreachable."""
    if len(waypoints) < 2:
        raise ValueError('mission needs at least two nodes')
    routes = []
    for a, b in zip(waypoints, waypoints[1:]):
        leg = shortest(a, b, blocked)
        if not leg:
            return None
        routes.append(leg)
    return routes


def validate_lock(data, expected_count):
    """Accept only fully confirmed lock data consistent with road coordinates."""
    if (not isinstance(data, dict) or data.get('state') != 'LOCKED' or
            data.get('locked') is not True or data.get('map_received') is not True or
            data.get('expected_obstacles') != expected_count):
        return None
    obstacles = data.get('obstacles')
    roads = data.get('blocked_roads')
    if not isinstance(obstacles, list) or len(obstacles) != expected_count:
        return None
    if not isinstance(roads, list) or not roads:
        return None
    try:
        # Imported only at call time: lock_core imports this file's EDGES/NODES.
        from pathlib import Path
        import sys
        lock_dir = str(Path(__file__).resolve().parent.parent / 'simple_lock_stage')
        if lock_dir not in sys.path:
            sys.path.insert(0, lock_dir)
        from lock_core import roads_at
        radius = float(data.get('corridor_radius', 0.23))
        if not math.isfinite(radius) or radius <= 0:
            return None
        normalized = {canonical(v) for v in roads}
        centers = [(float(v['x']), float(v['y'])) for v in obstacles]
        if not all(math.isfinite(x) and math.isfinite(y) for x, y in centers):
            return None
        if any(math.dist(centers[i], centers[j]) < 0.07
               for i in range(len(centers)) for j in range(i + 1, len(centers))):
            return None
        declared = set()
        for obj in obstacles:
            if not isinstance(obj.get('roads'), list) or not obj['roads']:
                return None
            expected = set(roads_at(float(obj['x']), float(obj['y']), radius))
            if not expected or {canonical(v) for v in obj['roads']} != expected:
                return None
            declared.update(expected)
        if declared != normalized:
            return None
    except (KeyError, ValueError, TypeError, AttributeError):
        return None
    return frozenset(normalized)


def index_at(msg, x, y):
    info = msg.info

    col = math.floor(
        (x - info.origin.position.x) / info.resolution
    )

    row = math.floor(
        (y - info.origin.position.y) / info.resolution
    )

    if not (
        0 <= row < info.height
        and 0 <= col < info.width
    ):
        return None

    return row * info.width + col


def scan_edges(cost, mask):
    """
    在每条道路中心线及左右安全带内取样。

    static:
        与禁行区域冲突或存在未知地图单元的路段。

    dynamic:
        Costmap 中存在实体障碍物的路段。
    """

    static = set()
    dynamic = set()

    for key, (a, b) in EDGES.items():

        (x0, y0), (x1, y1) = (
            NODES[a], NODES[b]
        )

        dx = x1 - x0
        dy = y1 - y0

        length = math.hypot(dx, dy)

        nx = -dy / length
        ny = dx / length

        fatal = set()
        unsafe = False

        n = math.ceil(length / 0.02)

        for i in range(n + 1):

            f = i / n
            x = x0 + f * dx
            y = y0 + f * dy

            for off in (
                -0.17, -0.13, -0.09,
                -0.05, 0.0,
                0.05, 0.09, 0.13, 0.17
            ):

                px = x + off * nx
                py = y + off * ny

                mi = index_at(mask, px, py)

                if mi is None or mask.data[mi] != 0:
                    unsafe = True
                    continue

                ci = index_at(cost, px, py)

                if ci is None or cost.data[ci] < 0:
                    unsafe = True

                elif cost.data[ci] >= 99:
                    fatal.add(ci)

        if unsafe:
            static.add(key)

        if len(fatal) >= 2:
            dynamic.add(key)

    return static, dynamic


def run():

    parser = argparse.ArgumentParser(
        description="田字形道路拓扑规划测试"
    )

    parser.add_argument(
        '--start',
        choices=sorted(NODES),
        default='C1'
    )

    parser.add_argument(
        '--goal',
        choices=sorted(NODES),
        default='A3'
    )

    parser.add_argument(
        '--block',
        action='append',
        default=[]
    )

    parser.add_argument(
        '--demo',
        action='store_true'
    )

    parser.add_argument('--mission', nargs='+', choices=sorted(NODES), default=None,
                        help='依次访问的图节点，暂非比赛实际工位坐标')
    parser.add_argument('--expected-obstacles', type=int, default=2)
    parser.add_argument('--manual-test', action='store_true',
                        help='仅用于离线/ROS 话题演示，人工道路直接冻结')

    args = parser.parse_args()
    if args.expected_obstacles < 1:
        parser.error('--expected-obstacles 必须为正整数')
    waypoints = args.mission or [args.start, args.goal]

    manual = {
        canonical(v)
        for v in args.block
    }

    if args.demo:
        print("道路节点：", sorted(NODES))
        print("路段总数：", len(EDGES))
        print("人工封路：", sorted(manual))

        path = shortest(
            args.start, args.goal, manual
        )

        print(
            "规划结果：",
            " -> ".join(path)
            if path else "无可通行路线"
        )
        print("逐段路线：", plan_mission(waypoints, manual))
        return

    import rclpy

    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        DurabilityPolicy,
        ReliabilityPolicy
    )

    from nav_msgs.msg import OccupancyGrid, Path
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import String

    class Topology(Node):

        def __init__(self):
            super().__init__(
                'competition_stage6_road_topology'
            )

            self.mask = None
            self.cost = None
            self.last_cost = 0.0

            self.manual = manual
            self.frozen = False
            self.blocked = frozenset()
            self.routes = None
            self.source = 'WAIT_FOR_OBSTACLE_LOCK'
            self.conflict = False
            self.obstacles = []

            self.still_static = set()
            self.still_dynamic = set()

            self.found = {
                k: 0 for k in EDGES
            }

            self.cleared = {
                k: 0 for k in EDGES
            }

            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE
            )

            self.sm = self.create_subscription(
                OccupancyGrid,
                '/keepout_filter_mask',
                self.got_mask,
                qos
            )

            self.sc = self.create_subscription(
                OccupancyGrid,
                '/global_costmap/costmap',
                self.got_cost,
                qos
            )

            self.pub_graph = self.create_publisher(
                MarkerArray,
                '/competition/road_graph',
                qos
            )

            self.pub_path = self.create_publisher(
                Path,
                '/competition/road_path',
                qos
            )

            self.pub_status = self.create_publisher(
                String,
                '/competition/road_status',
                qos
            )
            self.pub_locked_path = self.create_publisher(
                Path, '/competition/locked_road_path', qos
            )
            self.sub_lock = self.create_subscription(
                String, '/competition/obstacle_lock_status', self.on_lock, qos
            )

            self.timer = self.create_timer(
                1.0,
                self.publish
            )

            self.get_logger().warning(
                "只发布固定道路路线预览，不发送电机指令，也不触发 Nav2 FollowPath！"
            )
            if args.manual_test:
                self.freeze(manual, [], 'MANUAL_TEST')

        def got_mask(self, msg):
            self.mask = msg

        def got_cost(self, msg):
            # Keep the original Costmap subscriber for compatibility, but never
            # update blocked roads from it during this fixed-route stage.
            self.cost = msg
            self.last_cost = time.monotonic()

        def freeze(self, roads, obstacles, source):
            if self.frozen:
                return
            self.blocked = frozenset(roads) | frozenset(self.manual)
            self.obstacles = list(obstacles)
            self.routes = plan_mission(waypoints, self.blocked)
            self.source = source
            self.frozen = True
            self.get_logger().warning(
                '锁定封路=%s，固定任务路线=%s' % (sorted(self.blocked), self.routes)
            )

        def on_lock(self, msg):
            try:
                data = json.loads(msg.data)
            except (TypeError, ValueError):
                self.get_logger().error('无法解析 obstacle_lock_status JSON')
                return
            if not isinstance(data, dict):
                return
            if self.frozen:
                if data.get('state') == 'CONFLICT' and not self.conflict:
                    self.conflict = True
                    self.get_logger().error('新增目标冲突，冻结道路不变；不能继续视为可安全执行')
                return
            roads = validate_lock(data, args.expected_obstacles)
            if roads is not None:
                self.freeze(roads, data['obstacles'], 'LIDAR_CONFIRMED')

        def marker(
            self, ident, ns, typ, color,
            points=(), pos=None, label='', size=.04
        ):

            m = Marker()

            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = ns
            m.id = ident
            m.type = typ
            m.action = Marker.ADD

            m.pose.orientation.w = 1.0

            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 1.0

            m.scale.x = size

            if typ == Marker.LINE_STRIP:

                m.points = [
                    Point(
                        x=float(x),
                        y=float(y),
                        z=0.055
                    )
                    for x, y in points
                ]

            elif pos is not None:

                m.pose.position.x = pos[0]
                m.pose.position.y = pos[1]

                if typ == Marker.TEXT_VIEW_FACING:
                    m.pose.position.z = 0.12
                    m.text = label
                    m.scale.z = 0.095

                else:
                    m.pose.position.z = 0.06
                    m.scale.y = size
                    m.scale.z = size

            return m

        def publish(self):
            # The timer republishes an already frozen result for RViz only.
            # plan_mission() is called exclusively in freeze().
            ready = self.frozen and self.routes is not None and not self.conflict
            blocked = set(self.blocked)
            path = self.routes[0] if self.routes else []

            pm = Path()
            pm.header.frame_id = 'map'
            pm.header.stamp = self.get_clock().now().to_msg()
            for name in path:
                pose = PoseStamped()
                pose.header = pm.header
                pose.pose.position.x = NODES[name][0]
                pose.pose.position.y = NODES[name][1]
                pose.pose.orientation.w = 1.0
                pm.poses.append(pose)
            self.pub_path.publish(pm)
            self.pub_locked_path.publish(pm)

            markers = MarkerArray()

            for i, (key, (a, b)) in enumerate(
                EDGES.items()
            ):

                if key in blocked:
                    color = (1.0, 0.15, 0.15)

                elif key in self.still_static:
                    color = (1.0, 0.55, 0.0)

                else:
                    color = (0.12, 0.78, 0.2)

                markers.markers.append(
                    self.marker(
                        i,
                        'roads',
                        Marker.LINE_STRIP,
                        color,
                        [NODES[a], NODES[b]],
                        size=0.03
                    )
                )

            for i, key in enumerate(sorted(NODES)):

                markers.markers.append(
                    self.marker(
                        i, 'dots',
                        Marker.SPHERE,
                        (0.1, 0.55, 1.0),
                        pos=NODES[key],
                        size=0.055
                    )
                )

                markers.markers.append(
                    self.marker(
                        i, 'text',
                        Marker.TEXT_VIEW_FACING,
                        (1.0, 1.0, 1.0),
                        pos=NODES[key],
                        label=key
                    )
                )

            markers.markers.append(
                self.marker(
                    0,
                    'chosen',
                    Marker.LINE_STRIP,
                    (0.0, 0.9, 1.0),
                    [NODES[x] for x in path],
                    size=0.055
                )
            )

            self.pub_graph.publish(markers)

            result = {
                'ready': ready,
                'frozen': self.frozen,
                'start': args.start,
                'goal': args.goal,
                'blocked': sorted(blocked),
                'blocked_by_lidar_costmap': [],  # deprecated; no dynamic Costmap lock
                'route': path,
                'source': self.source,
                'conflict': self.conflict,
                'waypoints': waypoints,
                'mission_routes': self.routes,
                'status': ('WAIT_FOR_LOCK' if not self.frozen else
                           'CONFLICT_STOP' if self.conflict else
                           'NO_ROUTE' if self.routes is None else 'LOCKED_READY'),
                'note': 'Path 仅供 RViz 预览，尚未与 Nav2 Controller 联调'
            }
            self.pub_status.publish(String(data=json.dumps(result, ensure_ascii=False)))

    rclpy.init()
    node = Topology()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run()
