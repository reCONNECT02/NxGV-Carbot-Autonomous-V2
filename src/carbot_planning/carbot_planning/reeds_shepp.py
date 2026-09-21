"""Reeds-Shepp connections (no ROS): faithful port of V4 reeds-shepp.js.

V4: "Reeds-Shepp analytic families adapted from PythonRobotics, MIT. Atsushi
Sakai and Videh Patel. Twelve base families x time reversal x reflection.
Every accepted endpoint is independently checked by bicycle integration
before collision validation."

Used by block 11 (parking) and block 12 (recovery). Verified against the JS in
Node (test_reeds_shepp.py, tools/v4_harness/rs_v4.js).

Path points: N x 5 array [x, y, yaw, dir, k] (dir +1 forward / -1 reverse,
k = curvature of the segment the point belongs to). Every segment starts with
a copy of the previous segment's last pose carrying the new dir / k, exactly
as V4 pushes it, so a cusp shows up as two identical poses with different dir.
"""
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

PI = math.pi
H = PI / 2


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def _acos(x: float) -> float:
    return math.acos(_clamp(x, -1.0, 1.0))


def _asin(x: float) -> float:
    return math.asin(_clamp(x, -1.0, 1.0))


def _polar(x: float, y: float) -> Tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


# --------------------------------------------------------------------------- the 12 families (V4 order)
def _f1(x, y, p):
    u, t = _polar(x - math.sin(p), y - 1 + math.cos(p))
    v = wrap(p - t)
    if 0 <= t <= PI and 0 <= v <= PI:
        return [t, u, v], 'LSL'


def _f2(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r * r >= 4:
        u = math.sqrt(r * r - 4)
        t = wrap(b + math.atan2(2, u))
        v = wrap(t - p)
        if t >= 0 and v >= 0:
            return [t, u, v], 'LSR'


def _f3(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r <= 4:
        a = _acos(r / 4)
        t = wrap(a + b + H)
        u = wrap(PI - 2 * a)
        v = wrap(p - t - u)
        return [t, -u, v], 'LRL'


def _f4(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r <= 4:
        a = _acos(r / 4)
        t = wrap(a + b + H)
        u = wrap(PI - 2 * a)
        v = wrap(-p + t + u)
        return [t, -u, -v], 'LRL'


def _f5(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if 1e-8 < r <= 4:
        u = _acos(1 - r * r / 8)
        a = _asin(2 * math.sin(u) / r)
        t = wrap(-a + b + H)
        v = wrap(t - u - p)
        return [t, u, -v], 'LRL'


def _f6(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r <= 2:
        a = _acos((r + 2) / 4)
        t = wrap(b + a + H)
        u = wrap(a)
        v = wrap(p - t + 2 * u)
        if t >= 0 and u >= 0 and v >= 0:
            return [t, u, -u, -v], 'LRLR'


def _f7(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    u2 = (20 - r * r) / 16
    if 0 <= u2 <= 1 and r > 1e-8:
        u = _acos(u2)
        a = _asin(2 * math.sin(u) / r)
        t = wrap(b + a + H)
        v = wrap(t - p)
        if t >= 0 and v >= 0:
            return [t, -u, -u, v], 'LRLR'


def _f8(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r >= 2:
        q = math.sqrt(r * r - 4)
        u = q - 2
        a = math.atan2(2, q)
        t = wrap(b + a + H)
        v = wrap(t - p + H)
        if t >= 0 and v >= 0:
            return [t, -H, -u, -v], 'LRSL'


def _f9(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 2:
        t = wrap(b + H)
        u = r - 2
        v = wrap(p - t - H)
        if t >= 0 and v >= 0:
            return [t, -H, -u, -v], 'LRSR'


def _f10(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r >= 2:
        q = math.sqrt(r * r - 4)
        u = q - 2
        a = math.atan2(q, 2)
        t = wrap(b - a + H)
        v = wrap(t - p - H)
        if t >= 0 and v >= 0:
            return [t, u, H, -v], 'LSRL'


def _f11(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 2:
        t = wrap(b)
        u = r - 2
        v = wrap(p - t - H)
        if t >= 0 and v >= 0:
            return [t, u, H, -v], 'LSLR'


def _f12(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 4:
        q = math.sqrt(r * r - 4)
        u = q - 4
        a = math.atan2(2, q)
        t = wrap(b + a + H)
        v = wrap(t - p)
        if t >= 0 and v >= 0:
            return [t, -H, -u, -H, v], 'LRSLR'


FAMILIES: List[Callable] = [_f1, _f2, _f3, _f4, _f5, _f6, _f7, _f8, _f9, _f10, _f11, _f12]


# --------------------------------------------------------------------------- sampling
def _segment(x: float, y: float, a: float, d: float, k: float, n: int) -> np.ndarray:
    """core.js bicycle(origin, d*i/n, k) for i = 1..n (vectorised, same float ops)."""
    s = d * np.arange(1, n + 1, dtype=float) / n
    if abs(k) < 1e-9:
        return np.column_stack([x + s * math.cos(a), y + s * math.sin(a), np.full(n, a)])
    a2 = a + s * k
    return np.column_stack([x + (np.sin(a2) - math.sin(a)) / k, y + (-np.cos(a2) + math.cos(a)) / k,
                            np.arctan2(np.sin(a2), np.cos(a2))])


@dataclass
class RSPath:
    points: np.ndarray          # N x 5 [x, y, a, dir, k]
    cost: float                 # total |length|
    types: str                  # e.g. 'LSR'
    lengths: List[float]        # signed segment lengths (m)
    variant: int
    valid: bool = False
    reject: str = ''
    extra: dict = field(default_factory=dict)

    @property
    def reverse_length(self) -> float:
        return float(sum(-d for d in self.lengths if d < 0))


def candidates(start, goal, r: float, step: float = 0.006) -> List[RSPath]:
    """V4 CarbotRS.candidates: every family x {plain, flip, reflect, both}, sampled at
    `step`, endpoint re-checked (1e-5 m / rad), sorted by length (stable, as V8)."""
    sx, sy, sa = start
    gx, gy, ga = goal
    c, s = math.cos(sa), math.sin(sa)
    lx, ly = (gx - sx) * c + (gy - sy) * s, -(gx - sx) * s + (gy - sy) * c
    x, y, phi = lx / r, ly / r, wrap(ga - sa)
    out: List[RSPath] = []
    for f in FAMILIES:
        for variant in range(4):
            flip = variant in (1, 3)
            reflect = variant >= 2
            q = f(-x if flip else x, -y if reflect else y, -phi if flip != reflect else phi)
            if not q:
                continue
            lengths = [v * r * (-1 if flip else 1) for v in q[0]]
            types = [('R' if t == 'L' else 'L' if t == 'R' else 'S') if reflect else t for t in q[1]]
            px, py, pa = sx, sy, sa
            parts = []
            cost = 0.0
            for d, t in zip(lengths, types):
                if abs(d) < 1e-8:
                    continue
                dr = 1.0 if d > 0 else -1.0
                k = 1 / r if t == 'L' else -1 / r if t == 'R' else 0.0
                n = int(math.ceil(abs(d) / step))
                seg = _segment(px, py, pa, d, k, n)
                parts.append(np.array([[px, py, pa, dr, k]]))
                parts.append(np.column_stack([seg, np.full(n, dr), np.full(n, k)]))
                px, py, pa = seg[-1]
                cost += abs(d)
            if not parts or math.hypot(px - gx, py - gy) > 1e-5 or abs(wrap(pa - ga)) > 1e-5:
                continue
            out.append(RSPath(np.vstack(parts), cost, ''.join(types), lengths, variant))
    out.sort(key=lambda q: q.cost)
    return out


def body_valid(course, g, pts: np.ndarray, pad: float) -> bool:
    """course.bodyClear(p, c, pad) for every pose (V4 check, vectorised)."""
    if not len(pts):
        return False
    return bool(course.body_clear_many(pts[:, 0], pts[:, 1], pts[:, 2], g, pad).all())


def plan(start, goal, course, g, pad: float = 0.003, step: float = 0.006) -> Tuple[Optional[RSPath], List[RSPath]]:
    """V4 CarbotRS.plan: every candidate checked (for the GUI audit), the shortest
    collision-free one wins. -> (winner or None, evaluated)."""
    allc = candidates(start, goal, g.r, step)
    if allc:
        P = np.vstack([q.points for q in allc])
        ok = course.body_clear_many(P[:, 0], P[:, 1], P[:, 2], g, pad)
        i = 0
        for q in allc:
            n = len(q.points)
            q.valid = bool(ok[i:i + n].all())
            q.reject = '' if q.valid else 'Footprint leaves drivable area'
            i += n
    winner = next((q for q in allc if q.valid), None)
    return winner, allc


def to_path4(points: np.ndarray) -> np.ndarray:
    """N x 5 -> N x 4 [x, y, a, dir] (the planning Path convention)."""
    return np.asarray(points, float)[:, :4].copy()
