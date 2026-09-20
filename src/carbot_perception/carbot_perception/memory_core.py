"""BLOCK 04 memory, pure numpy port of V4 vehicle.js LocalMemory.

V4:  cells keyed by round(x/res), round(y/res) in the ODOM frame; each keeps
     the exact odom point, kind (1 road / 2 paint), stamp and the travelled
     distance at observation. expire() drops cells older than 25 s once there
     are more than 26000. Consumers (guidance.evidence, local planner, parking)
     choose their own max age; driving also rejects cells whose motion-based
     uncertainty 0.004 + 0.006 * |distance_now - distance_cell| exceeds 0.02 m.

Here the hash map is a toroidal array of size x size slots (default 256 x 256
= 6.4 m at 2.5 cm), so a whole 10 000-cell grid is stored with a few numpy
operations. Each slot remembers its full key; two keys only collide when they
are a whole ring apart (6.4 m), by which time the older one has expired.
"""
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class OdomPose:
    x: float
    y: float
    a: float
    distance: float     # cumulative path length (m)


class LocalMemory:

    def __init__(self, res: float = 0.025, ring: int = 256, max_cells: int = 26000,
                 expire_age: float = 25.0):
        self.res = float(res)
        self.ring = int(ring)
        self.max_cells = int(max_cells)
        self.expire_age = float(expire_age)
        n = self.ring * self.ring
        self.key_x = np.full(n, np.iinfo(np.int32).min, np.int32)
        self.key_y = np.full(n, np.iinfo(np.int32).min, np.int32)
        self.px = np.zeros(n, np.float32)
        self.py = np.zeros(n, np.float32)
        self.kind = np.zeros(n, np.uint8)
        self.stamp = np.full(n, -np.inf)
        self.dist = np.zeros(n)
        self.last = 0.0
        self.fresh = 0

    def _slot(self, kx: np.ndarray, ky: np.ndarray) -> np.ndarray:
        return (np.mod(kx, self.ring) * self.ring + np.mod(ky, self.ring)).astype(np.int64)

    def size(self, t: float) -> int:
        return int((self.stamp > t - self.expire_age).sum())

    def integrate(self, kind: np.ndarray, local_x: np.ndarray, local_y: np.ndarray,
                  pose: OdomPose, t: float) -> int:
        """kind/local_x/local_y: flat arrays for the cells of one base_link grid."""
        k = np.asarray(kind).ravel()
        sel = (k == 1) | (k == 2)
        lx, ly = np.asarray(local_x).ravel()[sel], np.asarray(local_y).ravel()[sel]
        c, s = np.cos(pose.a), np.sin(pose.a)
        wx = pose.x + lx * c - ly * s
        wy = pose.y + lx * s + ly * c
        kx = np.round(wx / self.res).astype(np.int32)
        ky = np.round(wy / self.res).astype(np.int32)
        slot = self._slot(kx, ky)
        self.key_x[slot] = kx
        self.key_y[slot] = ky
        self.px[slot] = wx
        self.py[slot] = wy
        self.kind[slot] = k[sel]
        self.stamp[slot] = t
        self.dist[slot] = pose.distance
        self.last = t
        self.fresh = int(sel.sum())
        if self.size(t) > self.max_cells:
            self.expire(t)
        return self.fresh

    def expire(self, t: float) -> None:
        old = self.stamp < t - self.expire_age
        self.stamp[old] = -np.inf
        self.kind[old] = 0

    def query(self, x: np.ndarray, y: np.ndarray, t: float, max_age: float
              ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """-> (found, kind, age, distance_at_observation) for odom points."""
        kx = np.round(np.asarray(x) / self.res).astype(np.int32)
        ky = np.round(np.asarray(y) / self.res).astype(np.int32)
        slot = self._slot(kx, ky)
        age = t - self.stamp[slot]
        found = (self.key_x[slot] == kx) & (self.key_y[slot] == ky) & (age < max_age)
        return found, np.where(found, self.kind[slot], 0), np.where(found, age, np.inf), self.dist[slot]

    def window(self, cx: float, cy: float, half: float, t: float, max_age: float):
        """Dense axis-aligned odom window centred on (cx, cy).
        -> (origin_x, origin_y, n, kind, age, dist) with row along +x, col along +y;
        origin = lower edge of cell (0, 0)."""
        n = int(round(2 * half / self.res))
        ix0 = int(np.floor(cx / self.res)) - n // 2
        iy0 = int(np.floor(cy / self.res)) - n // 2
        live = self.stamp > t - max_age
        kx, ky = self.key_x[live], self.key_y[live]
        r, c = kx - ix0, ky - iy0
        ok = (r >= 0) & (r < n) & (c >= 0) & (c < n)
        kind = np.zeros((n, n), np.uint8)
        age = np.full((n, n), -1.0, np.float32)
        dist = np.zeros((n, n), np.float64)
        kind[r[ok], c[ok]] = self.kind[live][ok]
        age[r[ok], c[ok]] = (t - self.stamp[live][ok]).astype(np.float32)
        dist[r[ok], c[ok]] = self.dist[live][ok]
        return (ix0 - 0.5) * self.res, (iy0 - 0.5) * self.res, n, kind, age, dist


def uncertainty(distance_now: float, distance_cell, base: float, per_m: float):
    """V4 guidance.evidence: 0.004 + 0.006 * |distance_now - distance_cell|."""
    return base + per_m * np.abs(distance_now - np.asarray(distance_cell))


class OdomHistory:
    """Short odom history for looking up the pose at a camera timestamp."""

    def __init__(self, keep_s: float = 2.0):
        self.keep = float(keep_s)
        self.t = []
        self.p = []          # (x, y, a, distance)
        self.distance = 0.0

    def add(self, t: float, x: float, y: float, a: float) -> None:
        if self.p:
            px, py = self.p[-1][0], self.p[-1][1]
            if t < self.t[-1]:            # clock jumped back (bag replay restart)
                self.t.clear()
                self.p.clear()
            else:
                self.distance += float(np.hypot(x - px, y - py))
                if t == self.t[-1]:
                    self.t.pop()
                    self.p.pop()
        self.t.append(t)
        self.p.append((x, y, a, self.distance))
        while self.t and self.t[0] < t - self.keep:
            self.t.pop(0)
            self.p.pop(0)

    def latest(self):
        return (self.t[-1], OdomPose(*self.p[-1])) if self.t else (None, None)

    def at(self, t: float, max_extrapolate: float = 0.1):
        """Interpolated pose at time t, or None if t is outside the history."""
        if not self.t or t < self.t[0] - max_extrapolate or t > self.t[-1] + max_extrapolate:
            return None
        if t <= self.t[0]:
            return OdomPose(*self.p[0])
        if t >= self.t[-1]:
            return OdomPose(*self.p[-1])
        i = int(np.searchsorted(self.t, t))
        t0, t1 = self.t[i - 1], self.t[i]
        f = (t - t0) / max(t1 - t0, 1e-9)
        a0, a1 = self.p[i - 1], self.p[i]
        da = np.arctan2(np.sin(a1[2] - a0[2]), np.cos(a1[2] - a0[2]))
        return OdomPose(a0[0] + f * (a1[0] - a0[0]), a0[1] + f * (a1[1] - a0[1]),
                        float(a0[2] + f * da), a0[3] + f * (a1[3] - a0[3]))
