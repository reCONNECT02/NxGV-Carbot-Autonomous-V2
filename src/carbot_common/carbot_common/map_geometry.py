"""track_map.yaml VERSION 2 geometry (control points -> centrelines, areas, paint).

This is a verbatim port of section 3 ("GEOMETRY") of the team's offline tool
tools/map/map_builder.py, without its OpenCV editor. Both must build the same
road from the same control points, so:

  * keep the formulas identical to map_builder.py;
  * test_map_geometry.py imports tools/map/map_builder.py (when OpenCV is
    installed) and fails if the two ever disagree.

Pure numpy, no ROS, no OpenCV. Units: metres, radians, track frame.
"""
import math
from typing import Dict, List, Tuple

import numpy as np

PI = math.pi

# map_builder.py EDIT['min_lane_change_half_m']
MIN_LANE_CHANGE_HALF_M = 0.10

# map_builder.py FLARE: V4 flare polygon for one roundabout exit
# (distance past the ring radius, sideways), traced from the drawing.
FLARE = [(-0.10, 0.38), (0.18, 0.15), (0.50, 0.15), (0.50, -0.15), (0.18, -0.15), (-0.10, -0.38)]


# --------------------------------------------------------------------------- load
def template_from_yaml(d: Dict) -> Dict:
    """track_map.yaml v2 document -> map_builder template dict (same as
    mission_planner.load_map)."""
    if d.get('version') != 2:
        raise ValueError(f'track_map.yaml version {d.get("version")}: expected 2')
    return {
        'corners': {n: {k: tuple(v) for k, v in c.items()} for n, c in d['corners'].items()},
        'roundabout': {k: tuple(v) for k, v in d['roundabout'].items()},
        'lane_change': {k: tuple(v) for k, v in d['lane_change'].items()},
        'road_ends': {k: tuple(v) for k, v in d['road_ends'].items()},
        'parallel_bay': dict(d['parallel_bay']),
        'straights': {n: [tuple(a) for a in s] for n, s in d['straights'].items()},
        'areas': {n: {**a, 'along': [tuple(x) for x in a['along']], 'lateral': tuple(a['lateral'])}
                  for n, a in d['area_rules'].items()},
        'lane_width_m': float(d['lane_width_m']),
    }


# --------------------------------------------------------------------------- primitives
def circle_through(p1, p2, p3):
    """Centre and radius of the circle through 3 points (None if collinear)."""
    (ax, ay), (bx, by), (cx, cy) = p1, p2, p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    return (ux, uy), math.hypot(ax - ux, ay - uy)


def sample_line(a, b, step):
    n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    t = np.linspace(0, 1, n + 1)[:, None]
    return np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))


def arc3_circle(s, m, e):
    """(centre, radius, start angle, signed sweep) of the arc s -> m -> e, or None."""
    c = circle_through(s, m, e)
    if c is None:
        return None
    (cx, cy), r = c
    ang = lambda p: math.atan2(p[1] - cy, p[0] - cx)  # noqa: E731
    a0, am, a1 = ang(s), ang(m), ang(e)
    ccw = lambda a: (a - a0) % (2 * PI)  # noqa: E731
    sweep = ccw(a1) if ccw(am) < ccw(a1) else ccw(a1) - 2 * PI   # go the way that passes m
    return (cx, cy), r, a0, sweep


def sample_arc3(s, m, e, step):
    """Arc from s through m to e. Straight line if the 3 points are collinear."""
    c = arc3_circle(s, m, e)
    if c is None:
        return sample_line(s, e, step)
    (cx, cy), r, a0, sweep = c
    n = max(1, math.ceil(abs(sweep) * r / step))
    a = a0 + sweep * np.linspace(0, 1, n + 1)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])


def lane_change_geometry(tpl):
    """West end (lower lane), centre, east end (upper lane) of the lane change."""
    lc = tpl['lane_change']
    cx, cy = lc['centre']
    half = max(MIN_LANE_CHANGE_HALF_M, lc['length_handle'][0] - cx)
    y_low = tpl['roundabout']['east'][1]
    y_high = tpl['corners']['bottom_right']['start'][1]
    return (cx - half, y_low), (cx, cy), (cx + half, y_high)


def derived_points(tpl):
    w, _, e = lane_change_geometry(tpl)
    return {'lane_change.west': w, 'lane_change.east': e}


def get_point(tpl, path):
    node = tpl
    for k in path.split('.'):
        node = node[k]
    return node


def resolve(tpl, ref, derived=None):
    """('corners.top_left.start', dx, dy) -> (x, y)."""
    derived = derived if derived is not None else derived_points(tpl)
    base = derived[ref[0]] if ref[0] in derived else get_point(tpl, ref[0])
    return (base[0] + ref[1], base[1] + ref[2])


def straight_ends(tpl, name):
    d = derived_points(tpl)
    a, b = tpl['straights'][name]
    return resolve(tpl, a, d), resolve(tpl, b, d)


def sample_lane_change(tpl, step):
    """S-curve lower lane -> centre -> upper lane (map_builder.sample_lane_change)."""
    (x0, y0), (cx, cy), (x1, y1) = lane_change_geometry(tpl)
    ss = lambda u: u * u * (3 - 2 * u)  # noqa: E731

    def half(xa, xb, f):
        n = max(1, math.ceil(abs(xb - xa) / step))
        t = np.linspace(0, 1, n + 1)
        return xa + (xb - xa) * t, f(t)

    xa, ya = half(x0, cx, lambda t: y0 + (cy - y0) * 2 * ss(t / 2))
    xb, yb = half(cx, x1, lambda t: cy + (y1 - cy) * (2 * ss(0.5 + t / 2) - 1))
    return np.column_stack([np.concatenate([xa, xb[1:]]), np.concatenate([ya, yb[1:]])])


def centrelines(tpl, step=0.025) -> Dict[str, np.ndarray]:
    """{section name: Nx2 array} in the track frame, built from the control points."""
    out = {}
    for name, c in tpl['corners'].items():
        out[name] = sample_arc3(c['start'], c['mid'], c['end'], step)
    rb = tpl['roundabout']
    circ = circle_through(rb['west'], rb['north'], rb['east'])
    if circ is not None:
        (cx, cy), r = circ
        a = np.linspace(0, 2 * PI, math.ceil(2 * PI * r / step) + 1)
        out['roundabout'] = np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])
    for name in tpl['straights']:
        out[name] = sample_line(*straight_ends(tpl, name), step)
    out['lane_change'] = sample_lane_change(tpl, step)
    return out


def line_frame(tpl, line):
    """Origin, unit direction u, left normal n and length of a straight road."""
    a, b = straight_ends(tpl, line)
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.hypot(*(b - a)))
    u = (b - a) / max(L, 1e-9)
    return a, u, np.array([-u[1], u[0]]), L


def areas(tpl) -> Dict[str, Dict]:
    """Every area as {'kind', 'poly' (4x2), 'frame', 'along', 'lateral', 'paint', 'heading'}."""
    lane = float(tpl.get('lane_width_m', 0.30))
    out = {}
    for name, a in tpl['areas'].items():
        A, u, n, L = line_frame(tpl, a['line'])
        ref = {'start': 0.0, 'end': L, 'slide': tpl['parallel_bay']['along_m']}
        s0, s1 = sorted(ref[r] + off for r, off in a['along'])
        l0, l1 = a['lateral']
        corner = lambda s, l: A + u * s + n * l  # noqa: E731
        heading = None
        if a.get('heading') == 'along':
            heading = math.atan2(u[1], u[0])
        elif a.get('heading') == 'into':          # nose into the bay, away from the road
            d = n * (1 if (l0 + l1) > 0 else -1)
            heading = math.atan2(d[1], d[0])
        out[name] = {'kind': a['kind'], 'paint': a['paint'], 'frame': (A, u, n),
                     'along': (s0, s1), 'lateral': (l0, l1), 'heading': heading,
                     'poly': np.array([corner(s0, l0), corner(s1, l0), corner(s1, l1), corner(s0, l1)])}
    (x0, yl), _, (x1, yh) = lane_change_geometry(tpl)
    lo, hi = min(yl, yh) - lane / 2, max(yl, yh) + lane / 2
    A, u, n = np.array([x0, 0.0]), np.array([1.0, 0.0]), np.array([0.0, 1.0])
    out['lane_change'] = {'kind': 'drivable', 'paint': {'divider': 'dashed'}, 'frame': (A, u, n),
                          'along': (0.0, x1 - x0), 'lateral': (lo, hi), 'heading': None,
                          'poly': np.array([[x0, lo], [x1, lo], [x1, hi], [x0, hi]])}
    rb = tpl['roundabout']
    circ = circle_through(rb['west'], rb['north'], rb['east'])
    if circ is not None:
        (cx, cy), r = circ
        for k, p in rb.items():
            u = np.array([p[0] - cx, p[1] - cy])
            u = u / max(np.hypot(*u), 1e-9)
            n = np.array([-u[1], u[0]])
            C = np.array([cx, cy])
            poly = np.array([C + u * (r + da) + n * dl for da, dl in FLARE])
            out[f'flare_{k}'] = {'kind': 'drivable', 'paint': {}, 'frame': (C, u, n),
                                 'along': (r + FLARE[0][0], r + FLARE[2][0]), 'lateral': (-0.38, 0.38),
                                 'heading': None, 'poly': poly}
    return out


def paint_segments(area, dash=0.14, gap=0.06) -> List[Tuple[Tuple[float, float], Tuple[float, float], str]]:
    """Paint line segments [((xa, ya), (xb, yb), style)] of one area."""
    A, u, n = area['frame']
    s0, s1 = area['along']
    l0, l1 = area['lateral']
    near, far = (l0, l1) if abs(l0) <= abs(l1) else (l1, l0)
    P = lambda s, l: tuple(A + u * s + n * l)  # noqa: E731
    sides = {'start': (P(s0, l0), P(s0, l1)), 'end': (P(s1, l0), P(s1, l1)),
             'near': (P(s0, near), P(s1, near)), 'far': (P(s0, far), P(s1, far)),
             'divider': (P(s0, (l0 + l1) / 2), P(s1, (l0 + l1) / 2))}
    out = []
    for side, style in area['paint'].items():
        a, b = sides[side]
        if style == 'solid':
            out.append((a, b, style))
            continue
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        d = 0.0
        while d < L:
            e = min(d + dash, L)
            out.append(((a[0] + (b[0] - a[0]) * d / L, a[1] + (b[1] - a[1]) * d / L),
                        (a[0] + (b[0] - a[0]) * e / L, a[1] + (b[1] - a[1]) * e / L), style))
            d += dash + gap
    return out


def ring(tpl):
    """(cx, cy, r) of the roundabout centreline circle, or None."""
    rb = tpl['roundabout']
    c = circle_through(rb['west'], rb['north'], rb['east'])
    return None if c is None else (c[0][0], c[0][1], c[1])
