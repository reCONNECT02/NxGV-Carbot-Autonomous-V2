"""Pure-Python port of the V4 core.js geometry helpers (no ROS imports).

Units: metres, radians. Pose = dict/obj with x, y, a (heading).
Later phases build the planners on these; keep them bit-for-bit faithful
to core.js so simulator results can be compared.
"""
import math
from dataclasses import dataclass
from typing import List, Tuple


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Pose:
    x: float
    y: float
    a: float


@dataclass
class Geometry:
    l: float        # noqa: E741  car length
    w: float        # car width
    wb: float       # wheelbase
    r: float        # min rear-axle turning radius
    rear: float     # rear axle -> rear bumper
    front: float    # rear axle -> front bumper
    track: float
    max_steer: float


def geometry(v: dict) -> Geometry:
    """v = vehicle.* parameters (common.yaml)."""
    wb = v['wheelbase_m']
    r = v['min_turning_radius_m']
    return Geometry(l=v['car_length_m'], w=v['car_width_m'], wb=wb, r=r,
                    rear=v['rear_overhang_m'], front=v['car_length_m'] - v['rear_overhang_m'],
                    track=v['car_width_m'] - v['wheel_width_m'], max_steer=math.atan(wb / r))


def bicycle(p: Pose, d: float, k: float) -> Pose:
    """Advance pose p by arc length d at curvature k (core.js bicycle)."""
    if abs(k) < 1e-9:
        return Pose(p.x + d * math.cos(p.a), p.y + d * math.sin(p.a), p.a)
    a = p.a + d * k
    return Pose(p.x + (math.sin(a) - math.sin(p.a)) / k,
                p.y + (-math.cos(a) + math.cos(p.a)) / k, wrap(a))


def to_world(p: Pose, qx: float, qy: float) -> Tuple[float, float]:
    c, s = math.cos(p.a), math.sin(p.a)
    return p.x + qx * c - qy * s, p.y + qx * s + qy * c


def to_local(p: Pose, qx: float, qy: float) -> Tuple[float, float]:
    x, y = qx - p.x, qy - p.y
    c, s = math.cos(p.a), math.sin(p.a)
    return x * c + y * s, -x * s + y * c


def footprint(p: Pose, g: Geometry, pad: float = 0.0) -> List[Tuple[float, float]]:
    corners = [(-g.rear - pad, -g.w / 2 - pad), (g.front + pad, -g.w / 2 - pad),
               (g.front + pad, g.w / 2 + pad), (-g.rear - pad, g.w / 2 + pad)]
    return [to_world(p, x, y) for x, y in corners]
