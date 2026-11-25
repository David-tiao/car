#!/usr/bin/env python3
"""Pure-Python lidar obstacle confirmation for the competition road graph.

The position is the laser-return cluster centroid, NOT a calibrated object center.
No clearance / side-passing computation: any matched road is blocked in full.
"""
import math
from dataclasses import dataclass
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'topology_stage6'))
from road_topology import EDGES, NODES  # noqa: E402


def point_segment_distance(x, y, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = min(1., max(0., ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - ax - t * dx, y - ay - t * dy)


def roads_at(x, y, corridor_radius=.23):
    """Lock ALL roads meeting a cluster in a junction; never attempt to pass alongside."""
    return sorted(key for key, (a, b) in EDGES.items()
                  if point_segment_distance(x, y, NODES[a], NODES[b]) <= corridor_radius)


def laser_clusters(ranges, angle_min, angle_increment, min_range=.10,
                   max_range=1.80, max_neighbor_gap=.075,
                   max_cluster_span=.14, min_cluster_points=2):
    """Group adjacent laser returns into compact candidates in the sensor frame.

    NOTE: an unrecognized fixed fixture can also form a compact cluster.
    The caller MUST filter known mapped fixtures and verify on the real field.
    """
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment, min_range,
                                          max_range, max_neighbor_gap, max_cluster_span))
            or angle_increment == 0 or min_range <= 0 or max_range < min_range
            or max_neighbor_gap <= 0 or max_cluster_span <= 0 or min_cluster_points < 2):
        return []
    runs = []
    current = []

    def close():
        if current:
            runs.append(current)

    for i, r in enumerate(ranges):
        if not math.isfinite(r) or not min_range <= r <= max_range:
            close()
            current = []
            continue
        angle = angle_min + i * angle_increment
        p = (r * math.cos(angle), r * math.sin(angle))
        if current and math.dist(current[-1], p) > max_neighbor_gap:
            close()
            current = []
        current.append(p)
    close()

    # A complete circular scan may split one object at its angular seam.
    # Merge only adjacent valid endpoints of a one-revolution scan, BEFORE
    # point-count and span filtering. A partial field of view is never joined.
    step = abs(angle_increment)
    sweep = max(0, len(ranges) - 1) * step
    circular = 0 < step <= .1 and abs(sweep - math.tau) <= 1.01 * step
    if (circular and len(runs) > 1
            and min_range <= ranges[0] <= max_range
            and min_range <= ranges[-1] <= max_range
            and math.dist(runs[-1][-1], runs[0][0]) <= max_neighbor_gap):
        # Inclusive -pi..+pi scans contain one repeated angular sample.
        # Do not count that duplicated endpoint as independent evidence.
        duplicate_angle = abs(sweep - math.tau) <= .01 * step
        runs[0] = runs[-1] + runs[0][int(duplicate_angle):]
        runs.pop()

    groups = []
    for run in runs:
        if len(run) < min_cluster_points:
            continue
        xs = [p[0] for p in run]
        ys = [p[1] for p in run]
        # Reject walls and large fixtures after seam joining, not before.
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if span <= max_cluster_span:
            groups.append((sum(xs) / len(xs), sum(ys) / len(ys), len(run)))
    return groups


@dataclass
class Track:
    x: float
    y: float
    hits: int = 1
    missed: int = 0


class ObstacleLock:
    """Confirm expected distinct objects across frames, then hold the lock forever."""

    def __init__(self, expected=2, required_hits=5, settle_frames=3,
                 match_radius=.10, corridor_radius=.23):
        if (expected < 1 or required_hits < 1 or settle_frames < 1 or
                not math.isfinite(match_radius) or not math.isfinite(corridor_radius) or
                match_radius <= 0 or corridor_radius <= 0):
            raise ValueError('confirmation parameters must be positive and finite')
        self.expected = expected
        self.required_hits = required_hits
        self.settle_frames = settle_frames
        self.match_radius = match_radius
        self.corridor_radius = corridor_radius
        self.tracks = []
        self.settled = 0
        self.locked_obstacles = None
        self.conflict = False

    def observe(self, candidates):
        """candidates: [(map_x, map_y), ...], one tuple per compact cluster."""
        used = set()
        for t in self.tracks:
            t.missed += 1
        for x, y in candidates:
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            # Map matching already restricts candidates to the competition roads.
            if not roads_at(x, y, self.corridor_radius):
                continue
            near = [(math.dist((x, y), (t.x, t.y)), i)
                    for i, t in enumerate(self.tracks) if i not in used]
            if near:
                dist, i = min(near)
            else:
                dist, i = math.inf, None
            if dist <= self.match_radius:
                t = self.tracks[i]
                # Exponential smoothing reduces random scan noise.
                t.x = .7 * t.x + .3 * x
                t.y = .7 * t.y + .3 * y
                t.hits += 1
                t.missed = 0
                used.add(i)
            else:
                self.tracks.append(Track(x, y))
                used.add(len(self.tracks) - 1)
        self.tracks = [t for t in self.tracks if t.missed < 3]
        confirmed = [t for t in self.tracks if t.hits >= self.required_hits and t.missed == 0]

        if self.locked_obstacles is not None:
            # Freeze positions and roads. A third stable target raises a conflict flag,
            # NEVER an automatic route change.
            if any(all(math.dist((t.x, t.y), (o['x'], o['y'])) > self.match_radius
                       for o in self.locked_obstacles) for t in confirmed):
                self.conflict = True
            return self.state

        if len(confirmed) > self.expected:
            self.conflict = True
            self.settled = 0
        elif len(confirmed) == self.expected:
            self.conflict = False
            self.settled += 1
            if self.settled >= self.settle_frames:
                self.locked_obstacles = [
                    {'x': round(t.x, 4), 'y': round(t.y, 4),
                     'roads': roads_at(t.x, t.y, self.corridor_radius)}
                    for t in sorted(confirmed, key=lambda t: (t.x, t.y))
                ]
        else:
            self.conflict = False
            self.settled = 0
        return self.state

    @property
    def state(self):
        if self.conflict:
            return 'CONFLICT'
        if self.locked_obstacles is not None:
            return 'LOCKED'
        return 'SCANNING'

    def status(self):
        obstacles = self.locked_obstacles or []
        return {
            'state': self.state,
            'locked': self.locked_obstacles is not None,
            'obstacles': obstacles,
            'blocked_roads': sorted({r for o in obstacles for r in o['roads']}),
            'expected_obstacles': self.expected,
            'corridor_radius': self.corridor_radius,
            'note': '障碍物坐标为激光反射点簇质心，尚未实测标定精度'
        }
