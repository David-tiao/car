"""Map the existing 5×5 field's blocked cells to 3×3 race road edges.

The browser labels 0..24 from bottom upwards in serpentine order. A red
480mm cell blocks any road *centerline* with a nonzero segment inside it.
This is a topology-only test convention, NOT a real footprint collision test.
"""
from __future__ import annotations

from road_topology import EDGES, NODES, canonical

CELL_SIZE_M = 2.4 / 5.0


def cell_rc(number):
    if type(number) is not int or number not in range(25):
        raise ValueError("blocked cell must be integer 0..24")
    row, offset = divmod(number, 5)
    col = offset if row % 2 == 0 else 4 - offset
    return col, row


def _interval_overlap_segment(a, b, left, right):
    if abs(a - b) < 1e-12:
        return (0.0, 1.0) if left < a < right else None
    lower = min((left - a) / (b - a), (right - a) / (b - a))
    upper = max((left - a) / (b - a), (right - a) / (b - a))
    return max(0.0, lower), min(1.0, upper)


def _crosses_cell(a, b, col, row):
    margin = 1e-8  # avoid counting a line merely tangent to a cell boundary
    x0, y0 = a
    x1, y1 = b
    x_range = _interval_overlap_segment(
        x0, x1, col * CELL_SIZE_M + margin, (col + 1) * CELL_SIZE_M - margin)
    y_range = _interval_overlap_segment(
        y0, y1, row * CELL_SIZE_M + margin, (row + 1) * CELL_SIZE_M - margin)
    if x_range is None or y_range is None:
        return False
    lo = max(x_range[0], y_range[0])
    hi = min(x_range[1], y_range[1])
    return hi - lo > 1e-9


def blocked_roads_from_cells(cell_ids):
    if not isinstance(cell_ids, list) or len(cell_ids) > 2:
        raise ValueError("blocked_cells must be 0–2 cell IDs")
    if len(set(cell_ids)) != len(cell_ids):
        raise ValueError("duplicate blocked cell")
    blocked = set()
    mapping = []
    for n in cell_ids:
        col, row = cell_rc(n)
        hits = sorted(
            road for road, (a, b) in EDGES.items()
            if _crosses_cell(NODES[a], NODES[b], col, row))
        blocked.update(hits)
        mapping.append({"cell": n, "col": col, "row": row, "roads": hits})
    return sorted(blocked), mapping


def merge_blocked_roads(cell_ids, manual):
    if not isinstance(manual, list) or len(manual) > 8:
        raise ValueError("manual road list too long")
    manual_set = set()
    for item in manual:
        if not isinstance(item, str) or len(item) > 8:
            raise ValueError("invalid road")
        try:
            manual_set.add(canonical(item))
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid road: " + str(item)) from exc
    derived, mapping = blocked_roads_from_cells(cell_ids)
    return sorted(set(derived) | manual_set), mapping
