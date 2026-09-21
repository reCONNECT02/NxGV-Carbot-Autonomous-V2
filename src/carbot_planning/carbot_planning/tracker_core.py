"""BLOCK 13 core (no ROS): turn the path into a request (V4 vehicle.js Controller).

update(): pure pursuit on the path point `lookahead` ahead (9.5 cm road, 6.5 cm
parking), steering limited by the wheelbase / minimum turning radius, speed
v = min(vmax, 0.07 + 0.07 / (1 + |curvature|)), slowed near the end
(remain * 1.8, at least 1.8 cm/s) and before a gear change (2.5 cm/s within 14
points). At a gear change: brake to < 0.6 cm/s, hold 0.35 s, then switch.
Arrival: within 2 cm (road) / 6 mm (parking) of the end, or level with it and
within 2.5 cm / 6 cm sideways; `arrived` once the car has stopped.

road_steer(): V4 app.js step() - in ROAD the steering aims at the
camera-conditioned local target (block 10's selected lateral line) instead of
the raw route, with the same pure-pursuit law.
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from .corridor_core import closest


@dataclass
class TrackerCfg:
    lookahead_road: float = 0.095
    lookahead_parking: float = 0.065
    speed_base: float = 0.07
    speed_curve: float = 0.07
    end_gain: float = 1.8
    end_min_speed: float = 0.018
    end_index_points: int = 8
    gear_window_points: int = 14
    gear_window_speed: float = 0.025
    gear_lookahead_points: int = 2
    gear_stop_speed: float = 0.006
    gear_hold_s: float = 0.35
    arrive_road: float = 0.020
    arrive_parking: float = 0.006
    arrive_lateral_road: float = 0.025
    arrive_lateral_parking: float = 0.060
    arrive_behind: float = 0.004
    arrived_speed: float = 0.002
    max_speed: float = 0.18
    parking_speed: float = 0.06

    @classmethod
    def from_params(cls, p) -> 'TrackerCfg':
        return cls(lookahead_road=float(p('lookahead_road_m')), lookahead_parking=float(p('lookahead_parking_m')),
                   speed_base=float(p('speed_base_mps')), speed_curve=float(p('speed_curve_mps')),
                   end_gain=float(p('end_slowdown_gain')), end_min_speed=float(p('end_min_speed_mps')),
                   end_index_points=int(p('end_index_points')),
                   gear_window_points=int(p('gear_window_points')), gear_window_speed=float(p('gear_window_speed_mps')),
                   gear_lookahead_points=int(p('gear_lookahead_points')),
                   gear_stop_speed=float(p('gear_stop_speed_mps')), gear_hold_s=float(p('gear_hold_s')),
                   arrive_road=float(p('arrive_road_m')), arrive_parking=float(p('arrive_parking_m')),
                   arrive_lateral_road=float(p('arrive_lateral_road_m')),
                   arrive_lateral_parking=float(p('arrive_lateral_parking_m')),
                   arrive_behind=float(p('arrive_behind_m')), arrived_speed=float(p('arrived_speed_mps')),
                   max_speed=float(p('limits.max_speed_mps')), parking_speed=float(p('limits.parking_speed_mps')))


def pursuit_steer(pose, tx: float, ty: float, g) -> float:
    """atan(wb * 2 y / (x^2 + y^2)) towards a target, clamped to the steering limit."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x, y = (tx - pose[0]) * c + (ty - pose[1]) * s, -(tx - pose[0]) * s + (ty - pose[1]) * c
    return max(-g.max_steer, min(g.max_steer, math.atan(g.wb * 2 * y / max(0.002, x * x + y * y))))


class Tracker:

    def __init__(self, cfg: TrackerCfg, g):
        self.cfg = cfg
        self.g = g
        self.reset()

    def reset(self) -> None:
        self.index = 0
        self.gear = 1
        self.gear_hold = 0.0
        self.error = 0.0
        self.target = None

    def update(self, path: np.ndarray, pose, speed: float, t: float, parking: bool = False) -> Dict:
        """path N x 4 (x, y, a, dir). -> {speed, steer, arrived, reason}."""
        c, g = self.cfg, self.g
        self.index, self.error = closest(path, pose[0], pose[1], self.index)
        end = path[-1]
        remain = math.hypot(pose[0] - end[0], pose[1] - end[1])
        end_index = self.index > len(path) - c.end_index_points
        ce, se = math.cos(end[2]), math.sin(end[2])
        ex = (pose[0] - end[0]) * ce + (pose[1] - end[1]) * se
        ey = -(pose[0] - end[0]) * se + (pose[1] - end[1]) * ce
        end_dir = end[3] or 1
        if end_index and (remain < (c.arrive_parking if parking else c.arrive_road) or
                          (ex * end_dir > -c.arrive_behind and
                           abs(ey) < (c.arrive_lateral_parking if parking else c.arrive_lateral_road))):
            return {'speed': 0.0, 'steer': 0.0, 'arrived': abs(speed) < c.arrived_speed,
                    'reason': 'ARRIVED' if abs(speed) < c.arrived_speed else 'STOPPING AT END'}
        direction = int(path[min(self.index + c.gear_lookahead_points, len(path) - 1)][3] or 1)
        if direction != self.gear:
            if abs(speed) > c.gear_stop_speed:
                return {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'GEAR STOP'}
            if not self.gear_hold:
                self.gear_hold = t + c.gear_hold_s
            if t < self.gear_hold:
                return {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'GEAR STOP'}
            self.gear = direction
            self.gear_hold = 0.0
        look = c.lookahead_parking if parking else c.lookahead_road
        k = self.index
        while k < len(path) - 1 and math.hypot(path[k, 0] - pose[0], path[k, 1] - pose[1]) < look and \
                int(path[k + 1, 3] or 1) == direction:
            k += 1
        tx, ty = float(path[k, 0]), float(path[k, 1])
        if k == len(path) - 1 and remain < look:
            d = look - remain
            tx += direction * d * math.cos(end[2])
            ty += direction * d * math.sin(end[2])
        self.target = (tx, ty)
        cp, sp = math.cos(pose[2]), math.sin(pose[2])
        qx, qy = (tx - pose[0]) * cp + (ty - pose[1]) * sp, -(tx - pose[0]) * sp + (ty - pose[1]) * cp
        curv = 2 * qy / max(0.0004, qx * qx + qy * qy)
        delta = max(-g.max_steer, min(g.max_steer, math.atan(g.wb * curv)))
        v = c.parking_speed if parking else c.max_speed
        v = min(v, c.speed_base + c.speed_curve / (1 + abs(curv)))
        if end_index:
            v = min(v, max(c.end_min_speed, remain * c.end_gain))
        dirs = path[self.index + 1:, 3]
        change = np.flatnonzero((dirs != 0) & (dirs != direction))
        if len(change) and change[0] + 1 < c.gear_window_points:
            v = min(v, c.gear_window_speed)
        return {'speed': v * direction, 'steer': delta, 'arrived': False, 'reason': 'TRACKING'}

    def road_steer(self, local_path: np.ndarray, pose) -> Optional[float]:
        """V4 app.js: steer towards the selected local line (block 10 LOCAL_PATH)."""
        if local_path is None or not len(local_path):
            return None
        k, _ = closest(local_path, pose[0], pose[1], 0)
        while k < len(local_path) - 1 and \
                math.hypot(local_path[k, 0] - pose[0], local_path[k, 1] - pose[1]) < self.cfg.lookahead_road:
            k += 1
        self.target = (float(local_path[k, 0]), float(local_path[k, 1]))
        return pursuit_steer(pose, self.target[0], self.target[1], self.g)


def split_gears(path: np.ndarray):
    """V4 vehicle.js splitGears: one list per gear; the cusp pose starts the next list."""
    P = np.asarray(path, float)
    if not len(P):
        return []
    out, cur = [], [P[0]]
    for p in P[1:]:
        if cur and cur[-1][3] != p[3]:
            out.append(np.asarray(cur))
            last = cur[-1].copy()
            last[3] = p[3]
            cur = [last]
        cur.append(p)
    out.append(np.asarray(cur))
    return out


class GearSequencer:
    """V4 app.js transition() for parking: track one gear section at a time,
    stop and hold `gear_hold_s` between sections, report `arrived` only at the end
    of the last one. (Cusp and terminal-heading replans belong to block 11.)"""

    def __init__(self, tracker: Tracker, hold_s: float):
        self.tracker = tracker
        self.hold_s = hold_s
        self.parts = []
        self.k = 0
        self.hold_until = 0.0
        self.key = None

    def set_path(self, path: np.ndarray, key) -> None:
        if key == self.key:
            return
        self.key = key
        self.parts = split_gears(path)
        self.k = 0
        self.hold_until = 0.0
        self.tracker.reset()

    def active(self) -> Optional[np.ndarray]:
        return self.parts[self.k] if self.k < len(self.parts) else None

    def update(self, pose, speed: float, t: float) -> Dict:
        part = self.active()
        if part is None:
            return {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'NO PARKING PATH'}
        if t < self.hold_until:
            return {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'GEAR CHANGE HOLD'}
        r = self.tracker.update(part, pose, speed, t, parking=True)
        if r['arrived']:
            if self.k + 1 < len(self.parts):
                self.k += 1
                self.tracker.reset()
                self.tracker.gear = int(self.parts[self.k][0][3] or 1)
                self.hold_until = t + self.hold_s
                return {'speed': 0.0, 'steer': 0.0, 'arrived': False,
                        'reason': f'GEAR CHANGE {self.k}/{len(self.parts) - 1}'}
            return dict(r, reason='PARKING PATH COMPLETE')
        return dict(r, reason=f'{r["reason"]} (section {self.k + 1}/{len(self.parts)})')
