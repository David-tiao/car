#!/usr/bin/env python3
"""比赛双静态障碍物的逻辑分区、确认和锁定核心（不依赖 ROS）。

道路分区直接复用 topology_stage6/road_topology.py 的 9 节点/12 路段图。
一个障碍物=一个空间聚类；一条被封闭的道路=一个逻辑大格子。
注意两者数量不应强行画等号：2 个障碍物可能落在同一路段。
"""
from dataclasses import dataclass
import math
import sys
from pathlib import Path

# 与既有第六阶段道路拓扑保持唯一事实来源
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'topology_stage6'))
from road_topology import NODES, EDGES, shortest  # noqa: E402


def _distance_to_edge(x, y, edge):
    a, b = EDGES[edge]
    ax, ay = NODES[a]
    bx, by = NODES[b]
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((x-ax)*dx + (y-ay)*dy)/(dx*dx + dy*dy)))
    cx, cy = ax + t*dx, ay + t*dy
    return math.hypot(x-cx, y-cy), t, (cx, cy)


def classify_road_cell(x, y, max_dist=0.29, ambiguity_gap=0.055,
                       node_buffer=0.12):
    """道路段归属；交叉口/大格边界不强行猜测，返回 (None, reason)。"""
    ranked = sorted(((_distance_to_edge(x, y, edge)[0], edge) for edge in EDGES))
    best_dist, cell = ranked[0]
    if best_dist > max_dist:
        return None, 'OUTSIDE_ROAD_CELLS'
    second_dist = ranked[1][0]
    # 不确定的路口：观测偏差不足以唯一确定是哪条大格
    if second_dist <= max_dist and second_dist - best_dist < ambiguity_gap:
        return None, 'AMBIGUOUS_CELL_BORDER'
    _, fraction, _ = _distance_to_edge(x, y, cell)
    a, b = EDGES[cell]
    segment_length = math.dist(NODES[a], NODES[b])
    if fraction * segment_length < node_buffer or (1-fraction)*segment_length < node_buffer:
        return None, 'NEAR_INTERSECTION'
    return cell, 'OK'


@dataclass
class Track:
    x: float
    y: float
    cell: str
    hits: int
    last_frame: int


@dataclass(frozen=True)
class LockedObject:
    x: float
    y: float
    cell: str


class ObstacleLocker:
    """一次比赛启动时重置；只确认两个对象；锁定后不自动解锁。"""
    def __init__(self, expected=2, min_hits=4, min_frames=20,
                 match_radius=0.17, max_missing=3):
        self.expected = expected
        self.min_hits = min_hits
        self.min_frames = min_frames
        self.match_radius = match_radius
        self.max_missing = max_missing
        self.frame_no = 0
        self.tracks = []
        self.locked_objects = ()
        self.state = 'COLLECTING'
        self.uncertain_count = 0
        self.last_rejects = []

    def observe(self, xy_candidates):
        """每帧输入已过滤的物体中心坐标；同帧多个点必须先聚类。"""
        self.frame_no += 1
        candidates = []
        rejects = []
        for x, y in xy_candidates:
            cell, reason = classify_road_cell(x, y)
            if cell is None:
                rejects.append({'x': round(x, 3), 'y': round(y, 3), 'reason': reason})
            else:
                # 同一帧反复上报同一物体，不能增加置信度
                if not any(math.hypot(x-u, y-v) < self.match_radius
                           for u, v, _ in candidates):
                    candidates.append((x, y, cell))
        self.last_rejects = rejects
        self.uncertain_count = self.uncertain_count + 1 if rejects else 0
        used = set()
        for x, y, cell in candidates:
            nearby = [
                (math.hypot(x-t.x, y-t.y), i)
                for i, t in enumerate(self.tracks)
                if i not in used and t.cell == cell
                and self.frame_no - t.last_frame <= self.max_missing + 1
            ]
            if nearby:
                d, i = min(nearby)
                if d <= self.match_radius:
                    t = self.tracks[i]
                    # 缓慢修正近似坐标，防止测量抖动
                    t.x = .75*t.x + .25*x
                    t.y = .75*t.y + .25*y
                    t.hits += 1
                    t.last_frame = self.frame_no
                    used.add(i)
                    continue
            self.tracks.append(Track(x, y, cell, 1, self.frame_no))
            used.add(len(self.tracks)-1)
        # 删除长时间未确认的噪声点；已锁对象另行保存，永不删除
        self.tracks = [t for t in self.tracks
                       if t.hits >= self.min_hits
                       or self.frame_no-t.last_frame <= self.max_missing]
        # 锁定后继续观察用于识别明显冲突，但绝不解除旧锁
        # 两个静态障碍物可先后被看到；已可靠确认的轨迹不会因暂时遮挡被抹掉。
        confirmed = [t for t in self.tracks if t.hits >= self.min_hits]
        # 若已锁定两物体，后来出现第三个稳定且位置不同的物体，立即进入冲突。
        extra_after_lock = bool(self.locked_objects) and any(
            all(math.hypot(t.x-o.x, t.y-o.y) > self.match_radius
                for o in self.locked_objects)
            for t in confirmed
        )
        if len(confirmed) > self.expected or extra_after_lock or self.uncertain_count >= self.min_hits:
            self.state = 'CONFLICT'
            return self.state
        if self.state == 'CONFLICT':
            return self.state  # 冲突也锁定，需人工重启/检查，不自动放行
        if self.locked_objects:
            self.state = 'LOCKED'
            return self.state
        if self.frame_no < self.min_frames:
            self.state = 'COLLECTING'
            return self.state
        # 两个独立物体，且没有待定的可疑第三个物体，才能宣布锁定
        pending = any(t.hits >= 2 and t not in confirmed
                      and self.frame_no-t.last_frame <= self.max_missing
                      for t in self.tracks)
        if len(confirmed) == self.expected and not rejects and not pending:
            self.locked_objects = tuple(LockedObject(t.x, t.y, t.cell)
                                        for t in confirmed)
            self.state = 'LOCKED'
        else:
            self.state = 'WAIT_TWO_OBSTACLES'
        return self.state

    @property
    def blocked_cells(self):
        return set(o.cell for o in self.locked_objects)

    def route(self, start, goal, static_blocked=()):
        if self.state != 'LOCKED':
            return []
        return shortest(start, goal, self.blocked_cells | set(static_blocked))


def split_scan_into_objects(points, gap=0.11, min_points=3, max_spread=0.32):
    """输入按激光束角度排序的地图坐标点或 None；输出候选物体中心。

    这是保守几何聚类，不是黑色识别；仅通过静态地图/规则掩膜筛选背景。
    """
    groups, group = [], []
    for p in list(points) + [None]:
        if p is not None and (not group or math.dist(p, group[-1]) <= gap):
            group.append(p)
        else:
            if group:
                groups.append(group)
            group = [p] if p is not None else []
    centers = []
    for group in groups:
        if len(group) < min_points:
            continue
        # 两端近似宽度，过滤较大静态结构；阈值需要实物标定
        span = max(math.dist(group[i], group[j])
                   for i, j in ((0, len(group)-1), (0, len(group)//2)))
        if span > max_spread:
            continue
        x = sum(p[0] for p in group)/len(group)
        y = sum(p[1] for p in group)/len(group)
        centers.append((x, y))
    return centers
