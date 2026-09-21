"""BLOCK 01 prior map geometry: faithful numpy port of V4 core.js `Course`.

Loads config/data/track_map.yaml (every primitive written out as data) and
rebuilds the V4 drivable field exactly:

  field[j, i] at (x, y) = (i * res, j * res), initialised to -10, then the max of
    * lane/2 - distance to every centreline sample within +-search cells
      (centrelines sampled every 0.025 m in DRAWING units, then scaled)
    * signed distance inside each drivable rect (negative outside)
    * distance to the edge inside each flare polygon
  clearance(x, y) = bilinear interpolation of the field, -10 outside the raster.

clearance > 0 means drivable, and its value is the distance to the nearest
road edge (0.15 m on a 30 cm lane centreline). Used by block 05 (camera edge
to map registration), and later by blocks 07-12 and the GUI map tab.

Pure numpy, no ROS. Units: metres, radians, track frame.
"""
import hashlib
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

SAMPLE_STEP = 0.025          # V4 sampleLine / arc default step


def sample_line(x0, y0, x1, y1, step=SAMPLE_STEP) -> np.ndarray:
    n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
    t = np.arange(n + 1) / n
    return np.stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t], axis=1)


def sample_arc(cx, cy, r, a0, a1, step=SAMPLE_STEP) -> np.ndarray:
    n = max(1, int(math.ceil(abs(a1 - a0) * r / step)))
    a = a0 + (a1 - a0) * np.arange(n + 1) / n
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _in_poly(x: np.ndarray, y: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd rule, same test as core.js inPoly (vectorised)."""
    inside = np.zeros(x.shape, bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[j]
        cross = (ay > y) != (by > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = (bx - ax) * (y - ay) / (by - ay) + ax
        inside ^= cross & (x < xi)
        j = i
    return inside


def _seg_dist(x, y, a, b) -> np.ndarray:
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = np.clip(((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
    return np.hypot(x - a[0] - t * dx, y - a[1] - t * dy)


def js_round(v: float) -> int:
    """JavaScript Math.round (half up); Python round() is half-to-even."""
    return int(math.floor(v + 0.5))


class Course:
    """V4 Course rebuilt from track_map.yaml."""

    def __init__(self, tm: Dict):
        self.tm = tm
        self.sx = float(tm['course_length_m']) / 7.00
        self.sy = float(tm['course_width_m']) / 5.00
        ext = tm['extent_m']
        self.w = float(ext[0]) * self.sx
        self.h = float(ext[1]) * self.sy
        self.lane = float(tm['lane_width_m'])
        self.res = float(tm['field_resolution_m'])
        self.search = int(tm['field_search_cells'])
        self.nx = int(math.ceil(self.w / self.res)) + 1
        self.ny = int(math.ceil(self.h / self.res)) + 1

        # centrelines: sampled in drawing units, then scaled (V4 order)
        self.paths: List[np.ndarray] = []
        for c in tm['centrelines']:
            if 'line' in c:
                p = sample_line(*c['line'])
            else:
                p = sample_arc(*c['arc'])
            self.paths.append(p * np.array([self.sx, self.sy]))
        self.areas = {k: self._rect(v) for k, v in (tm.get('areas') or {}).items()}
        self.rects = [self.areas[k] for k in tm.get('drivable_areas', [])]
        east = np.array(tm['flares']['east'], float)
        flares = [east]
        for d in tm['flares'].get('derived', []):
            if d == 'mirror_x_4p8':
                flares.append(np.stack([4.8 - east[:, 0], east[:, 1]], 1))
            elif d == 'rotate_about_roundabout':
                flares.append(np.stack([2.4 - (east[:, 1] - 0.8), 0.8 + (east[:, 0] - 2.4)], 1))
            else:
                raise ValueError(f'track_map.yaml: unknown flare derivation {d}')
        self.flares = [f * np.array([self.sx, self.sy]) for f in flares]
        self.field = self._build_field()

    # ------------------------------------------------------------------ build
    def _rect(self, r) -> Tuple[float, float, float, float]:
        x0, x1, y0, y1 = (float(v) for v in r)
        return (x0 * self.sx, x1 * self.sx, y0 * self.sy, y1 * self.sy)

    def _build_field(self) -> np.ndarray:
        res, n = self.res, self.search
        field = np.full((self.ny, self.nx), -10.0, np.float32)
        off = np.arange(-n, n + 1)
        for path in self.paths:
            for qx, qy in path:
                ix, iy = js_round(qx / res), js_round(qy / res)
                xs, ys = ix + off, iy + off
                okx = (xs >= 0) & (xs < self.nx)
                oky = (ys >= 0) & (ys < self.ny)
                if not okx.any() or not oky.any():
                    continue
                xs, ys = xs[okx], ys[oky]
                d = self.lane / 2 - np.hypot(xs[None, :] * res - qx, ys[:, None] * res - qy)
                blk = field[ys[0]:ys[-1] + 1, xs[0]:xs[-1] + 1]
                np.maximum(blk, d.astype(np.float32), out=blk)
        gx = np.arange(self.nx) * res
        gy = np.arange(self.ny) * res
        X, Y = np.meshgrid(gx, gy)          # [j, i]
        for (x0, x1, y0, y1) in self.rects:
            inside = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
            din = np.minimum.reduce([X - x0, x1 - X, Y - y0, y1 - Y])
            dout = -np.hypot(np.maximum.reduce([x0 - X, np.zeros_like(X), X - x1]),
                             np.maximum.reduce([y0 - Y, np.zeros_like(Y), Y - y1]))
            np.maximum(field, np.where(inside, din, dout).astype(np.float32), out=field)
        for poly in self.flares:
            inside = _in_poly(X, Y, poly)
            if not inside.any():
                continue
            d = np.min([_seg_dist(X[inside], Y[inside], poly[k], poly[(k + 1) % len(poly)])
                        for k in range(len(poly))], axis=0)
            sub = field[inside]
            field[inside] = np.maximum(sub, d.astype(np.float32))
        return field

    # ------------------------------------------------------------------ query
    def clearance(self, x, y):
        """Signed distance to the road edge (>0 drivable). Scalars or arrays."""
        scalar = np.isscalar(x) and np.isscalar(y)
        xx = np.asarray(x, float) / self.res
        yy = np.asarray(y, float) / self.res
        i = np.floor(xx).astype(np.int64)
        j = np.floor(yy).astype(np.int64)
        ok = (i >= 0) & (j >= 0) & (i + 1 < self.nx) & (j + 1 < self.ny)
        ic, jc = np.where(ok, i, 0), np.where(ok, j, 0)
        u, v = xx - i, yy - j
        f = self.field
        val = (1 - v) * ((1 - u) * f[jc, ic] + u * f[jc, ic + 1]) + \
            v * ((1 - u) * f[jc + 1, ic] + u * f[jc + 1, ic + 1])
        val = np.where(ok, val, -10.0)
        return float(val) if scalar else val

    def gradient(self, x, y, h: float = 0.012):
        """Central-difference gradient of clearance (V4 visualUpdate, h = 0.012)."""
        gx = (self.clearance(np.asarray(x) + h, y) - self.clearance(np.asarray(x) - h, y)) / (2 * h)
        gy = (self.clearance(x, np.asarray(y) + h) - self.clearance(x, np.asarray(y) - h)) / (2 * h)
        return gx, gy

    # ------------------------------------------------------------------ features
    def pose(self, key: str) -> Tuple[float, float, float]:
        p = self.tm[key]
        return float(p['x']) * self.sx, float(p['y']) * self.sy, float(p.get('a', 0.0))

    def start_pose(self) -> Tuple[float, float, float]:
        return self.pose('start_pose')

    def surface(self, x: float, y: float) -> float:
        """Floor height (hill + bump), V4 Course.surface."""
        e, b = self.tm['elevation'], self.tm['speed_bump']
        x, y = x / self.sx, y / self.sy
        z = 0.0
        if e['y0'] < y < e['y1'] and e['x0'] < x < e['x1']:
            d, hgt, ramp, flat = x - e['x0'], e['height_m'], e['ramp_m'], e['flat_end_m']
            if d < ramp:
                z = hgt * (1 - math.cos(math.pi * d / ramp)) / 2
            elif d < flat:
                z = hgt
            else:
                z = hgt * (1 + math.cos(math.pi * (d - flat) / ramp)) / 2
        hw = b['half_width_m']
        if b['y0'] < y < b['y1'] and abs(x - b['x']) < hw:
            z = max(z, b['height_m'] * math.sqrt(max(0.0, 1 - ((x - b['x']) / hw) ** 2)))
        return z

    def in_tunnel(self, x: float, y: float) -> bool:
        t = self.tm['tunnel']
        xx, yy = x / self.sx - t['cx'], y / self.sy - t['cy']
        r = math.hypot(xx, yy)
        return xx <= t['x_max_rel'] and yy <= t['y_max_rel'] and t['r_in'] < r < t['r_out']

    def in_area(self, name: str, x: float, y: float, pad: float = 0.0) -> bool:
        x0, x1, y0, y1 = self.areas[name]
        return x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad


_CACHE: Dict[str, Course] = {}


def load_course(path: str, tm: Optional[Dict] = None) -> Course:
    """Course for a track_map.yaml path (built once per file content)."""
    import yaml
    with open(path, 'rb') as f:
        raw = f.read()
    key = hashlib.sha1(raw).hexdigest()
    if key not in _CACHE:
        _CACHE[key] = Course(tm if tm is not None else yaml.safe_load(raw))
    return _CACHE[key]


def map_sha1(path: str) -> str:
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()
