"""BLOCK 01 prior map geometry: V4 core.js `Course`, for track_map.yaml v1 AND v2.

version 1 (phase 1 layout, V4 reference: config/data/v4_reference/track_map.yaml)
  Every V4 primitive written out as data. Rebuilds the V4 drivable field
  exactly (verified against core.js in Node, phase 3):
    field[j, i] at (x, y) = (i * res, j * res), initialised to -10, then the max of
      * lane/2 - distance to every centreline sample within +-search cells
        (centrelines sampled every 0.025 m in DRAWING units, then scaled)
      * signed distance inside each drivable rect (negative outside)
      * distance to the edge inside each flare polygon
  Features (start pose, light, gates, bump, hill, tunnel) live in the same file.

version 2 (the team's tools/map/map_builder.py output, the stack default)
  ROAD GEOMETRY ONLY: control points -> centrelines / areas through
  carbot_common.map_geometry (a verbatim port of map_builder.py). The field is
  built the V4 way from those centrelines and polygons, on a raster that
  starts at (x0, y0) instead of 0 (a fitted map may extend anywhere).
  Features come from data/track_features.yaml (hand-edited, "measure on site"),
  whose poses may point at mission.yaml poses ({mission_pose: P0}).

clearance(x, y) > 0 means drivable, and its value is the distance to the nearest
road edge (0.15 m on a 30 cm lane centreline). Used by blocks 05, 07-13 and the
GUI map tab.

Pure numpy, no ROS. Units: metres, radians, track frame.
"""
import hashlib
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import map_geometry as mg

SAMPLE_STEP = 0.025          # V4 sampleLine / arc default step
PAINT_WIDTH_M = 0.03         # V4 PHOTO_DIMENSIONS.parkingBorderCm / divider width


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
    inside = np.zeros(np.shape(x), bool)
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


def rect_poly(x0, x1, y0, y1) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


def file_fingerprints(path: str) -> Dict[str, str]:
    """sha1 of a file as stored, with LF endings and with CRLF endings.

    mission_planner.py hashes the raw bytes. The same map saved on Windows
    (CRLF) and checked out on the robot (LF, .gitattributes) must still match,
    so the stack accepts any of the three.
    """
    with open(path, 'rb') as f:
        raw = f.read()
    lf = raw.replace(b'\r\n', b'\n')
    crlf = lf.replace(b'\n', b'\r\n')
    return {'raw': hashlib.sha1(raw).hexdigest(), 'lf': hashlib.sha1(lf).hexdigest(),
            'crlf': hashlib.sha1(crlf).hexdigest()}


class Course:
    """V4 Course rebuilt from track_map.yaml (v1 or v2).

    features:       v2 only - track_features.yaml document (dict)
    mission_poses:  v2 only - {'P0': (x, y, a), ...} to resolve {mission_pose: P0}
    """

    def __init__(self, tm: Dict, features: Optional[Dict] = None,
                 mission_poses: Optional[Dict[str, Tuple[float, float, float]]] = None):
        self.tm = tm
        self.version = int(tm.get('version', 1))
        self._body_cache: Dict = {}
        if self.version == 1:
            self._init_v1(tm)
        elif self.version == 2:
            if features is None:
                raise ValueError('track_map.yaml v2 needs track_features.yaml (features=...)')
            self._init_v2(tm, features, mission_poses or {})
        else:
            raise ValueError(f'track_map.yaml: unknown version {self.version}')
        self.field = self._build_field()

    # ================================================================== v1
    def _init_v1(self, tm: Dict) -> None:
        self.sx = float(tm['course_length_m']) / 7.00
        self.sy = float(tm['course_width_m']) / 5.00
        ext = tm['extent_m']
        self.w = float(ext[0]) * self.sx
        self.h = float(ext[1]) * self.sy
        self.x0 = self.y0 = 0.0
        self.lane = float(tm['lane_width_m'])
        self.res = float(tm['field_resolution_m'])
        self.search = int(tm['field_search_cells'])
        self.nx = int(math.ceil(self.w / self.res)) + 1
        self.ny = int(math.ceil(self.h / self.res)) + 1

        # centrelines: sampled in drawing units, then scaled (V4 order)
        self.paths: List[np.ndarray] = []
        self.sections: Dict[str, np.ndarray] = {}
        for c in tm['centrelines']:
            if 'line' in c:
                p = sample_line(*c['line'])
            else:
                p = sample_arc(*c['arc'])
            p = p * np.array([self.sx, self.sy])
            self.paths.append(p)
            self.sections[c.get('id', f'line{len(self.paths)}')] = p
        self.areas = {k: self._rect(v) for k, v in (tm.get('areas') or {}).items()}
        drivable = list(tm.get('drivable_areas', []))
        self.rects = [self.areas[k] for k in drivable]
        self.area_polys = {k: rect_poly(*v) for k, v in self.areas.items()}
        self.area_kind = {k: ('drivable' if k in drivable else
                              'bay' if k.endswith('_bay') else 'area') for k in self.areas}
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
        self.drivable_polys: List[np.ndarray] = []          # v2 only
        rb = tm['roundabout']
        cx, cy, r = float(rb['x']) * self.sx, float(rb['y']) * self.sy, float(rb['radius_m'])
        self.ring = (cx, cy, r)
        # V4 flares: east, mirrored (west), rotated (north)
        self.exits = {'east': (cx + r, cy), 'west': (cx - r, cy), 'north': (cx, cy + r)}
        self.feat = tm
        # paint (V4 markings): borders + crossable dashes
        self.paint: List[Tuple[Tuple[float, float], Tuple[float, float], str, float]] = []
        self.crossable_rects: List[Tuple[float, float, float, float]] = []
        for m in tm.get('markings', []):
            for rct in self._marking_rects(m):
                x0, x1, y0, y1 = rct
                if m['kind'] == 'crossable':
                    self.crossable_rects.append(rct)
                wide = (x1 - x0) >= (y1 - y0)
                a = (x0, (y0 + y1) / 2) if wide else ((x0 + x1) / 2, y0)
                b = (x1, (y0 + y1) / 2) if wide else ((x0 + x1) / 2, y1)
                self.paint.append((a, b, 'dashed' if m['kind'] == 'crossable' else 'solid',
                                   min(x1 - x0, y1 - y0)))
        self.crossable_segments = np.zeros((0, 4))

    def _rect(self, r) -> Tuple[float, float, float, float]:
        x0, x1, y0, y1 = (float(v) for v in r)
        return (x0 * self.sx, x1 * self.sx, y0 * self.sy, y1 * self.sy)

    def _marking_rects(self, m) -> List[Tuple[float, float, float, float]]:
        """V4 paint rects of one v1 marking (dashes use the V4 float loops)."""
        if 'rect' in m:
            return [self._rect(m['rect'])]
        d = m['dashed']
        out = []
        v = float(d['from'])
        while v < float(d['to']):
            e = min(v + float(d['dash']), float(d['to']))
            a0, a1 = d['across']
            out.append(self._rect([v, e, a0, a1] if d['axis'] == 'x' else [a0, a1, v, e]))
            v += float(d['period'])
        return out

    # ================================================================== v2
    def _init_v2(self, tm: Dict, features: Dict, mission_poses: Dict) -> None:
        self.tpl = mg.template_from_yaml(tm)
        self.sx = self.sy = 1.0
        self.lane = float(tm['lane_width_m'])
        self.res = float(features.get('field_resolution_m', 0.01))
        self.search = int(features.get('field_search_cells', 31))
        self.sections = mg.centrelines(self.tpl, SAMPLE_STEP)
        self.paths = list(self.sections.values())
        ar = mg.areas(self.tpl)
        self.v2_areas = ar
        self.area_polys = {k: np.asarray(a['poly'], float) for k, a in ar.items()}
        self.area_kind = {k: a['kind'] for k, a in ar.items()}
        self.areas = {k: (float(p[:, 0].min()), float(p[:, 0].max()), float(p[:, 1].min()),
                          float(p[:, 1].max())) for k, p in self.area_polys.items()}
        self.rects = []
        self.flares = []
        self.drivable_polys = [self.area_polys[k] for k, a in ar.items() if a['kind'] == 'drivable']
        ring = mg.ring(self.tpl)
        if ring is None:
            raise ValueError('track_map.yaml v2: roundabout points are collinear')
        self.ring = ring
        self.exits = {k: tuple(map(float, v)) for k, v in self.tpl['roundabout'].items()}
        pts = np.vstack(self.paths + list(self.area_polys.values()))
        pad = 0.4
        self.x0 = math.floor((pts[:, 0].min() - pad) / self.res) * self.res
        self.y0 = math.floor((pts[:, 1].min() - pad) / self.res) * self.res
        self.w = pts[:, 0].max() + pad - self.x0
        self.h = pts[:, 1].max() + pad - self.y0
        self.nx = int(math.ceil(self.w / self.res)) + 1
        self.ny = int(math.ceil(self.h / self.res)) + 1
        self.paint = []
        segs = []
        for a in ar.values():
            for pa, pb, style in mg.paint_segments(a):
                self.paint.append((pa, pb, style, PAINT_WIDTH_M))
                if style == 'dashed':
                    segs.append([pa[0], pa[1], pb[0], pb[1]])
        self.crossable_rects = []
        self.crossable_segments = np.asarray(segs, float).reshape(-1, 4)
        self.feat = self._resolve_features(features, mission_poses)

    def _resolve_features(self, f: Dict, mission_poses: Dict) -> Dict:
        out = dict(f)
        for k, v in f.items():
            if isinstance(v, dict) and 'mission_pose' in v:
                ref = v['mission_pose']
                if ref not in mission_poses:
                    out[k] = {'unresolved': ref}
                    continue
                x, y, a = mission_poses[ref]
                out[k] = dict(v, x=x, y=y, a=a)
        t = f.get('tunnel') or {}
        if 'section' in t:
            c = self.tpl['corners'][t['section']]
            arc = mg.arc3_circle(c['start'], c['mid'], c['end'])
            if arc is None:
                raise ValueError(f'track_features.yaml tunnel.section {t["section"]} is not an arc')
            (cx, cy), r, a0, sweep = arc
            out['tunnel'] = dict(t, cx=cx, cy=cy, r=r, a0=a0, sweep=sweep,
                                 r_in=r - float(t['half_width_m']), r_out=r + float(t['half_width_m']))
        return out

    # ================================================================== field
    def _build_field(self) -> np.ndarray:
        res, n = self.res, self.search
        field = np.full((self.ny, self.nx), -10.0, np.float32)
        off = np.arange(-n, n + 1)
        for path in self.paths:
            for qx, qy in path:
                ix, iy = js_round((qx - self.x0) / res), js_round((qy - self.y0) / res)
                xs, ys = ix + off, iy + off
                okx = (xs >= 0) & (xs < self.nx)
                oky = (ys >= 0) & (ys < self.ny)
                if not okx.any() or not oky.any():
                    continue
                xs, ys = xs[okx], ys[oky]
                d = self.lane / 2 - np.hypot(xs[None, :] * res + self.x0 - qx,
                                             ys[:, None] * res + self.y0 - qy)
                blk = field[ys[0]:ys[-1] + 1, xs[0]:xs[-1] + 1]
                np.maximum(blk, d.astype(np.float32), out=blk)
        gx = self.x0 + np.arange(self.nx) * res
        gy = self.y0 + np.arange(self.ny) * res
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
        for poly in self.drivable_polys:
            # v2: signed distance like a V4 rect (+ inside, - outside), any polygon
            x0, y0 = poly.min(0) - 0.35
            x1, y1 = poly.max(0) + 0.35
            i0, i1 = max(0, int((x0 - self.x0) / res)), min(self.nx, int((x1 - self.x0) / res) + 2)
            j0, j1 = max(0, int((y0 - self.y0) / res)), min(self.ny, int((y1 - self.y0) / res) + 2)
            Xs, Ys = X[j0:j1, i0:i1], Y[j0:j1, i0:i1]
            d = np.min([_seg_dist(Xs, Ys, poly[k], poly[(k + 1) % len(poly)])
                        for k in range(len(poly))], axis=0)
            sd = np.where(_in_poly(Xs, Ys, poly), d, -d).astype(np.float32)
            np.maximum(field[j0:j1, i0:i1], sd, out=field[j0:j1, i0:i1])
        return field

    # ================================================================== query
    def clearance(self, x, y):
        """Signed distance to the road edge (>0 drivable). Scalars or arrays."""
        scalar = np.isscalar(x) and np.isscalar(y)
        xx = (np.asarray(x, float) - self.x0) / self.res
        yy = (np.asarray(y, float) - self.y0) / self.res
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

    # ================================================================== car body (V4)
    def body_samples(self, g, pad: float, kind: str = 'body') -> np.ndarray:
        """Car-frame sample points (rear axle origin) of a V4 body check.

        kind 'body': core.js Course.bodyClear (3.5 cm along the sides, 3 cm
        across the ends, then the 4 footprint corners).
        kind 'road': recovery.js roadClear (2.5 cm, then the 4 corners).
        """
        key = (kind, round(pad, 6), g.rear, g.front, g.w)
        if key in self._body_cache:
            return self._body_cache[key]
        pts = []
        if kind == 'body':
            x = -g.rear - pad
            while x < g.front + pad + 0.001:
                pts += [(x, -g.w / 2 - pad), (x, g.w / 2 + pad)]
                x += 0.035
            for xe in (-g.rear - pad, g.front + pad):
                y = -g.w / 2 - pad
                while y <= g.w / 2 + pad + 0.001:
                    pts.append((xe, y))
                    y += 0.03
        elif kind == 'road':
            xs, ys = (-g.rear - pad, g.front + pad), (-g.w / 2 - pad, g.w / 2 + pad)
            x = xs[0]
            while x <= xs[1]:
                pts += [(x, ys[0]), (x, ys[1])]
                x += 0.025
            y = ys[0]
            while y <= ys[1]:
                pts += [(xs[0], y), (xs[1], y)]
                y += 0.025
        else:
            raise ValueError(kind)
        pts += [(-g.rear - pad, -g.w / 2 - pad), (g.front + pad, -g.w / 2 - pad),
                (g.front + pad, g.w / 2 + pad), (-g.rear - pad, g.w / 2 + pad)]
        arr = np.asarray(pts, float)
        self._body_cache[key] = arr
        return arr

    def _body_values(self, X, Y, A, S) -> np.ndarray:
        X, Y, A = (np.atleast_1d(np.asarray(v, float)) for v in (X, Y, A))
        c, s = np.cos(A)[:, None], np.sin(A)[:, None]
        wx = X[:, None] + S[None, :, 0] * c - S[None, :, 1] * s
        wy = Y[:, None] + S[None, :, 0] * s + S[None, :, 1] * c
        return self.clearance(wx, wy)

    def body_clear_many(self, X, Y, A, g, pad: float = 0.0) -> np.ndarray:
        """Vectorised V4 bodyClear for poses (X, Y, A): bool array."""
        return (self._body_values(X, Y, A, self.body_samples(g, pad, 'body')) >= 0).all(axis=1)

    def body_clear(self, p, g, pad: float = 0.0) -> bool:
        return bool(self.body_clear_many(p[0], p[1], p[2], g, pad)[0])

    def body_margin_many(self, X, Y, A, g) -> np.ndarray:
        """V4 bodyMargin: min clearance of the 4 footprint corners."""
        S = np.array([(-g.rear, -g.w / 2), (g.front, -g.w / 2), (g.front, g.w / 2), (-g.rear, g.w / 2)])
        return self._body_values(X, Y, A, S).min(axis=1)

    def body_margin(self, p, g) -> float:
        return float(self.body_margin_many(p[0], p[1], p[2], g)[0])

    def road_clear_many(self, X, Y, A, g, tolerance: float = 0.0, pad: float = 0.005) -> np.ndarray:
        """V4 recovery.js roadClear: whole outline within the paint allowance."""
        return (self._body_values(X, Y, A, self.body_samples(g, pad, 'road')) >= -tolerance).all(axis=1)

    # ================================================================== paint / areas
    def crossable(self, x, y, pad: float = 0.025) -> np.ndarray:
        """True where a point lies on dashed (crossable) paint, +pad (V4 inRect(r, .025))."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        out = np.zeros(np.shape(x), bool)
        for (x0, x1, y0, y1) in self.crossable_rects:
            out |= (x >= x0 - pad) & (x <= x1 + pad) & (y >= y0 - pad) & (y <= y1 + pad)
        for sx0, sy0, sx1, sy1 in self.crossable_segments:
            out |= _seg_dist(x, y, (sx0, sy0), (sx1, sy1)) <= PAINT_WIDTH_M / 2 + pad
        return out

    def in_area(self, name: str, x: float, y: float, pad: float = 0.0) -> bool:
        if self.version == 1:
            x0, x1, y0, y1 = self.areas[name]
            return x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad
        poly = self.area_polys[name]
        if bool(_in_poly(np.array([x]), np.array([y]), poly)[0]):
            return True
        if pad <= 0:
            return False
        d = min(float(_seg_dist(np.array([x]), np.array([y]), poly[k], poly[(k + 1) % len(poly)])[0])
                for k in range(len(poly)))
        return d <= pad

    def section_of(self, x, y) -> Tuple[str, float]:
        """Nearest centreline section name and its distance."""
        best, bd = '', float('inf')
        for name, p in self.sections.items():
            d = float(np.min(np.hypot(p[:, 0] - x, p[:, 1] - y)))
            if d < bd:
                best, bd = name, d
        return best, bd

    # ================================================================== features
    def has_feature(self, key: str) -> bool:
        v = self.feat.get(key)
        return v is not None and not (isinstance(v, dict) and 'unresolved' in v)

    def pose(self, key: str) -> Tuple[float, float, float]:
        p = self.feat[key]
        if isinstance(p, dict) and 'unresolved' in p:
            raise KeyError(f'feature {key} refers to mission pose {p["unresolved"]}, '
                           'which is not in mission.yaml')
        return float(p['x']) * self.sx, float(p['y']) * self.sy, float(p.get('a', 0.0))

    def point(self, key: str, sub: Optional[str] = None) -> Tuple[float, float]:
        p = self.feat[key] if sub is None else self.feat[key][sub]
        return float(p['x']) * self.sx, float(p['y']) * self.sy

    def start_pose(self) -> Tuple[float, float, float]:
        return self.pose('start_pose')

    def surface(self, x: float, y: float) -> float:
        """Floor height (hill + bump), V4 Course.surface."""
        e, b = self.feat['elevation'], self.feat['speed_bump']
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

    def in_elevation(self, x: float, y: float, margin: float = 0.0) -> bool:
        e = self.feat['elevation']
        x, y = x / self.sx, y / self.sy
        return e['x0'] - margin < x < e['x1'] + margin and e['y0'] - margin < y < e['y1'] + margin

    def in_tunnel(self, x: float, y: float, margin: float = 0.0) -> bool:
        t = self.feat['tunnel']
        if 'sweep' in t:                                  # v2: tunnel along an arc section
            dx, dy = x - t['cx'], y - t['cy']
            r = math.hypot(dx, dy)
            if not (t['r_in'] - margin < r < t['r_out'] + margin):
                return False
            a = math.atan2(dy, dx)
            s = t['sweep']
            u = ((a - t['a0']) % (2 * math.pi)) if s > 0 else ((t['a0'] - a) % (2 * math.pi))
            slack = (float(t.get('end_margin_m', 0.0)) + margin) / max(t['r'], 1e-6)
            return u <= abs(s) + slack or u >= 2 * math.pi - slack
        xx, yy = x / self.sx - t['cx'], y / self.sy - t['cy']
        r = math.hypot(xx, yy)
        return xx <= t['x_max_rel'] + margin and yy <= t['y_max_rel'] + margin and \
            t['r_in'] - margin < r < t['r_out'] + margin


_CACHE: Dict[str, Course] = {}


def _sibling(path: str, name: str) -> str:
    return os.path.join(os.path.dirname(os.path.abspath(os.path.expanduser(path))), name)


def load_course(path: str, tm: Optional[Dict] = None, features: Optional[str] = None,
                mission: Optional[str] = None) -> Course:
    """Course for a track_map.yaml path (built once per file content).

    v2 maps also need track_features.yaml (and mission.yaml for poses such as
    the start). Pass their paths (nodes: data.track_features, data.mission);
    when not given, the files next to track_map.yaml are used.
    """
    import yaml
    with open(path, 'rb') as f:
        raw = f.read()
    doc = tm if tm is not None else yaml.safe_load(raw)
    key = hashlib.sha1(raw).hexdigest()
    feat_doc, poses = None, None
    if int(doc.get('version', 1)) == 2:
        fpath = features or _sibling(path, 'track_features.yaml')
        mpath = mission or _sibling(path, 'mission.yaml')
        with open(fpath, 'rb') as f:
            fraw = f.read()
        feat_doc = yaml.safe_load(fraw)
        key += hashlib.sha1(fraw).hexdigest()
        if os.path.isfile(mpath):
            with open(mpath, 'rb') as f:
                mraw = f.read()
            key += hashlib.sha1(mraw).hexdigest()
            md = yaml.safe_load(mraw) or {}
            poses = {p['id']: (float(p['x']), float(p['y']), math.radians(float(p['yaw_deg'])))
                     for p in md.get('poses', []) if 'id' in p}
    if key not in _CACHE:
        _CACHE[key] = Course(doc, feat_doc, poses)
    return _CACHE[key]


def course_from_params(p) -> Course:
    """Course for a CarbotNode (p = node.p): data.track_map + data.track_features + data.mission."""
    return load_course(str(p('data.track_map')), features=str(p('data.track_features')),
                       mission=str(p('data.mission')))


def map_sha1(path: str) -> str:
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()
