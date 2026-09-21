"""Road evidence for blocks 09 / 10 (V4 guidance.js evidence, perception.js support).

V4 asks "what do we know about the road at world point w?":
  1. the live camera road grid, if it is younger than live_max_age (0.30 s):
     grown road -> ROAD, paint -> PAINT (perception.support: +1 / -1)
  2. else local memory, if the cell is younger than max_age and its
     uncertainty base + per_m * travel_since_m is below the limit
  3. else nothing.

Frames on the real car (V4 has one simulated frame per estimate):
  * the road grid (block 03) is in base_link at the camera stamp -> world
    points are brought into it with the TRACK pose at that stamp (V4 uses
    cameraOdom for the same reason);
  * memory (block 04) lives in the ODOM frame of /odom -> world (track)
    points are mapped with T_track_odom = local_pose (+) odom^-1 at the same
    stamp (block 05 stamps its pose with the /odom stamp).

Everything is vectorised: pass N x 2 arrays of track-frame points.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

ROAD, PAINT = 1, 2


@dataclass
class Grid:
    rows: int
    cols: int
    res: float
    x0: float
    y0: float
    kind: np.ndarray                    # rows x cols uint8
    grown: np.ndarray                   # rows x cols uint8
    age: Optional[np.ndarray] = None    # memory: seconds since seen (-1 empty)
    travel: Optional[np.ndarray] = None  # memory: metres driven since seen (-1 empty)
    stamp: float = 0.0

    @classmethod
    def from_msg(cls, g, stamp: float) -> 'Grid':
        r, c = int(g.rows), int(g.cols)
        kind = np.frombuffer(bytes(g.kind), np.uint8).reshape(r, c)
        grown = np.frombuffer(bytes(g.grown), np.uint8).reshape(r, c) if len(g.grown) == r * c \
            else np.zeros((r, c), np.uint8)
        age = np.asarray(g.age_s, np.float32).reshape(r, c) if len(g.age_s) == r * c else None
        trav = np.asarray(g.travel_since_m, np.float32).reshape(r, c) if len(g.travel_since_m) == r * c else None
        return cls(r, c, float(g.resolution_m), float(g.origin_x_m), float(g.origin_y_m),
                   kind, grown, age, trav, stamp)

    def cells(self, lx: np.ndarray, ly: np.ndarray):
        """(row, col, inside) of grid-frame points (row along +x, col along +y)."""
        r = np.floor((lx - self.x0) / self.res).astype(np.int64)
        c = np.floor((ly - self.y0) / self.res).astype(np.int64)
        ok = (r >= 0) & (c >= 0) & (r < self.rows) & (c < self.cols)
        return np.where(ok, r, 0), np.where(ok, c, 0), ok


@dataclass
class EvidenceCfg:
    live_max_age_s: float = 0.30
    unc_base_m: float = 0.004
    unc_per_m: float = 0.006
    unc_limit_m: float = 0.02


def to_local(pose, X, Y):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    dx, dy = X - x, Y - y
    return dx * c + dy * s, -dx * s + dy * c


def to_world(pose, lx, ly):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return x + lx * c - ly * s, y + lx * s + ly * c


class Evidence:

    def __init__(self, cfg: EvidenceCfg):
        self.cfg = cfg
        self.live: Optional[Grid] = None
        self.live_pose = None               # track pose at the live grid stamp
        self.mem: Optional[Grid] = None
        self.t_track_odom = None            # (x, y, a): odom point -> track point
        self.now = 0.0

    def set_live(self, grid: Grid, pose_track) -> None:
        self.live, self.live_pose = grid, pose_track

    def set_memory(self, grid: Grid, t_track_odom) -> None:
        self.mem, self.t_track_odom = grid, t_track_odom

    def live_age(self) -> float:
        return float('inf') if self.live is None else self.now - self.live.stamp

    def live_fresh(self) -> bool:
        return self.live is not None and self.live_pose is not None and \
            self.live_age() < self.cfg.live_max_age_s

    # ------------------------------------------------------------------ V4 support()
    def support(self, X, Y) -> np.ndarray:
        """perception.support on the live grid: +1 grown road, -1 paint, 0 else
        (regardless of age; callers check live_fresh())."""
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        out = np.zeros(X.shape, np.int8)
        if self.live is None or self.live_pose is None:
            return out
        lx, ly = to_local(self.live_pose, X, Y)
        r, c, ok = self.live.cells(lx, ly)
        g = self.live.grown[r, c] > 0
        p = self.live.kind[r, c] == PAINT
        out[ok & g] = 1
        out[ok & ~g & p] = -1
        return out

    # ------------------------------------------------------------------ memory
    def memory(self, X, Y, max_age: float, check_uncertainty: bool = True) -> np.ndarray:
        """Memory kind (0 / ROAD / PAINT) at track points, V4 LocalMemory.query +
        guidance.evidence age and uncertainty limits."""
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        out = np.zeros(X.shape, np.uint8)
        if self.mem is None or self.t_track_odom is None:
            return out
        ox, oy = to_local(self.t_track_odom, X, Y)          # track -> odom
        r, c, ok = self.mem.cells(ox, oy)
        k = self.mem.kind[r, c]
        good = ok & ((k == ROAD) | (k == PAINT))
        if self.mem.age is not None:
            # memory age at publication + time since the memory grid was published
            age = self.mem.age[r, c] + max(0.0, self.now - self.mem.stamp)
            good &= (self.mem.age[r, c] >= 0) & (age < max_age)
        if check_uncertainty and self.mem.travel is not None:
            unc = self.cfg.unc_base_m + self.cfg.unc_per_m * np.abs(self.mem.travel[r, c])
            good &= unc < self.cfg.unc_limit_m
        out[good] = k[good]
        return out

    # ------------------------------------------------------------------ V4 evidence()
    def query(self, X, Y, max_age: float = 3.0) -> Tuple[np.ndarray, np.ndarray]:
        """-> (kind 0/ROAD/PAINT, live mask) at track points."""
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        kind = np.zeros(X.shape, np.uint8)
        live = np.zeros(X.shape, bool)
        if self.live_fresh():
            s = self.support(X, Y)
            live = s != 0
            kind[s == 1] = ROAD
            kind[s == -1] = PAINT
        rest = ~live
        if rest.any():
            kind[rest] = self.memory(X[rest], Y[rest], max_age)
        return kind, live


def se2_compose(a, b):
    """a (+) b for (x, y, yaw) tuples."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + b[0] * c - b[1] * s, a[1] + b[0] * s + b[1] * c, a[2] + b[2]


def se2_inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -(a[0] * c + a[1] * s), -(-a[0] * s + a[1] * c), -a[2]


def track_from_odom(local_pose, odom_pose):
    """T such that track_point = T (+) odom_point, from the two poses of the car
    at the same instant."""
    return se2_compose(local_pose, se2_inverse(odom_pose))
