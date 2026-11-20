"""Deterministic, physics-light keyboard vehicle digital twin. NO hardware.

Input is *received local ROS2* TwistStamped, not a fake REAL /odom channel.
Two-dimensional body-frame mecanum kinematics, acceleration limits, conservative
footprint collisions and optional faults. It does NOT model wheel slip or PID.
"""
from __future__ import annotations

from collections import deque
import math

FIELD_M = 2.4
GRID_M = 0.48
FOOTPRINT_RADIUS_M = 0.185   # conservative circle for ~260x290 mm body
ACCEL_MPS2 = 0.5
YAW_ACCEL_RPS2 = 1.2
MAX_DT_S = 0.3
SUBSTEP_S = 0.02
KINEMATIC_FAULTS = frozenset(("none", "motor_stall", "slow_response", "link_loss"))
FORBIDDEN = (
    (0.55, 0.55, 1.0, 1.0),
    (1.4, 0.55, 1.85, 1.0),
    (0.55, 1.4, 1.0, 1.85),
    (1.4, 1.4, 1.85, 1.85),
)
STOP = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def cell_box(cell_id):
    if type(cell_id) is not int or not 0 <= cell_id < 25:
        raise ValueError("Bad field cell: expect integer 0..24")
    row, index = divmod(cell_id, 5)
    col = index if row % 2 == 0 else 4-index
    return (col * GRID_M, row * GRID_M,
            (col + 1) * GRID_M, (row + 1) * GRID_M)


def intersects_disc_rect(x, y, radius, bounds):
    left, bottom, right, top = bounds
    dx = max(left-x, 0.0, x-right)
    dy = max(bottom-y, 0.0, y-top)
    return dx*dx + dy*dy <= radius*radius


def valid_blocked_cells(cells):
    if not isinstance(cells, list) or len(cells) > 2 or any(
            type(x) is not int or not 0 <= x < 25 for x in cells) or (
            len(set(cells)) != len(cells)):
        raise ValueError("0–2 unique blocked cell IDs are required")
    return sorted(cells)


def clamp_delta(value, target, maximum):
    return value + max(-maximum, min(maximum, target-value))


def wrap_angle(radians):
    return math.atan2(math.sin(radians), math.cos(radians))


class KeyboardChassisTwin:
    def __init__(self):
        self.reset("upper_right", [])

    def reset(self, zone, blocked_cells):
        if zone not in ("upper_right", "lower_right"):
            raise ValueError("Start zone must be upper_right or lower_right")
        cells = valid_blocked_cells(blocked_cells)
        self.zone = zone
        self.blocked_cells = cells
        self.x, self.y = 2.16, 2.16 if zone == "upper_right" else 0.24
        self.yaw = math.pi  # visualization only: start from east, facing west
        self.twist = dict(STOP)
        self.target = dict(STOP)
        self.fault_mode = "none"
        self.state = "READY"
        self.collision_reason = ""
        self.last_time = None
        self.samples = 0
        self.trail = deque(maxlen=240)
        self.trail.append([round(self.x, 4), round(self.y, 4)])
        self.run_seconds = 0.0
        self.generation = getattr(self, "generation", 0) + 1
        return self.snapshot()

    def set_fault(self, mode):
        if mode not in KINEMATIC_FAULTS:
            raise ValueError("Unknown mock chassis fault")
        self.fault_mode = mode
        # The user is changing the fault model; don't replay old velocity.
        self.target = dict(STOP)
        self.twist = dict(STOP)
        self.last_time = None
        return self.snapshot()

    def collision(self, x, y):
        r = FOOTPRINT_RADIUS_M
        if x-r < 0 or y-r < 0 or x+r > FIELD_M or y+r > FIELD_M:
            return "BOUNDARY"
        if any(intersects_disc_rect(x, y, r, rect) for rect in FORBIDDEN):
            return "FIXED_STATION_KEEP_OUT"
        if any(intersects_disc_rect(x, y, r, cell_box(i))
               for i in self.blocked_cells):
            return "BLOCKED_CELL"
        # Raw turntable centre: a geometry-only keepout; no physical lidar.
        if math.hypot(x-1.2, y-2.4) <= r+0.15:
            return "RAW_TURNTABLE"
        return ""

    def advance(self, now, input_twist=None, accepted_ros2=False):
        if not isinstance(now, (int, float)) or not math.isfinite(now):
            raise ValueError("Finite monotonic now required")
        if input_twist is None:
            input_twist = STOP
        values = [input_twist.get(k) for k in ("vx", "vy", "wz")]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError("Finite body-frame velocity required")
        if math.hypot(values[0], values[1]) > .25001 or abs(values[2]) > .50001:
            raise ValueError("Chassis mock speed limits exceeded")
        if self.last_time is None:
            self.last_time = now
            return self.snapshot()
        dt = max(0.0, min(MAX_DT_S, now - self.last_time))
        self.last_time = now
        if self.state == "OBSTACLE_STOP":
            return self.snapshot()
        if self.fault_mode == "link_loss" or not accepted_ros2:
            desired = dict(STOP)
        elif self.fault_mode == "motor_stall":
            desired = dict(STOP)
        else:
            desired = {k: float(input_twist[k]) for k in STOP}
        self.target = dict(desired)
        if dt > 0:
            n = max(1, math.ceil(dt / SUBSTEP_S))
            h = dt / n
            acceleration = ACCEL_MPS2 * (.28 if self.fault_mode == "slow_response" else 1.0)
            yaw_acceleration = YAW_ACCEL_RPS2 * (.28 if self.fault_mode == "slow_response" else 1.0)
            for _ in range(n):
                for axis in ("vx", "vy"):
                    self.twist[axis] = clamp_delta(
                        self.twist[axis], desired[axis], acceleration*h)
                self.twist["wz"] = clamp_delta(
                    self.twist["wz"], desired["wz"], yaw_acceleration*h)
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                nx = self.x + (c*self.twist["vx"]-s*self.twist["vy"])*h
                ny = self.y + (s*self.twist["vx"]+c*self.twist["vy"])*h
                reason = self.collision(nx, ny)
                if reason:
                    self.state = "OBSTACLE_STOP"
                    self.collision_reason = reason
                    self.target = dict(STOP)
                    self.twist = dict(STOP)
                    break
                self.x, self.y = nx, ny
                self.yaw = wrap_angle(self.yaw + self.twist["wz"]*h)
                self.run_seconds += h
            self.samples += 1
            point = [round(self.x, 4), round(self.y, 4)]
            if self.trail[-1] != point:
                self.trail.append(point)
            if self.state != "OBSTACLE_STOP":
                self.state = "RUNNING" if any(abs(v) > 0.001
                                              for v in self.twist.values()) else "STOPPED"
        return self.snapshot()

    def snapshot(self):
        error = {k: round(self.target[k]-self.twist[k], 4) for k in STOP}
        error["linear_norm_mps"] = round(math.hypot(error["vx"], error["vy"]), 4)
        return {
            "schema": "car_keyboard_mock_chassis_v1",
            "source_type": "MOCK_KINEMATIC_ONLY",
            "hardware_authorized": False,
            "real_odom": False, "ros2_transport_required": True,
            "frame": "field_preview_m_not_odom",
            "zone": self.zone, "state": self.state,
            "fault_mode": self.fault_mode,
            "collision_reason": self.collision_reason,
            "blocked_cells": list(self.blocked_cells),
            "generation": self.generation,
            "x_m": round(self.x, 4), "y_m": round(self.y, 4),
            "yaw_rad": round(self.yaw, 5),
            "velocity": {k: round(v, 4) for k, v in self.twist.items()},
            "input_twist": {k: round(v, 4) for k, v in self.target.items()},
            "model_error": error,
            "samples": self.samples,
            "sim_time_s": round(self.run_seconds, 3),
            "trace": list(self.trail),
            "limitations": "Acceleration-capped 2D mock, fixed KEEP_OUT/cells only; not PID, wheel encoder, lidar, SLAM or REAL odom",
        }
