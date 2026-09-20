#!/usr/bin/env python3
"""map_builder.py -- RISA Bot, BLOCK 01 prior map: build track_map.yaml from a recorded lap.

How the map is made (calibration page "Build map"):
  1. Drive one lap. The fused UWB + IMU + encoder position is recorded (LOCKED).
  2. AUTO-FIT moves + rotates the whole competition map onto the lap.   DONE
  3. AUTO-FIT nudges each control point towards the lap.                TODO
  4. You drag anything still red, confirm, and track_map.yaml is saved.  DONE here
     as the `edit` window; the web page will do the same thing.

Everything you can move is a CONTROL POINT (handle in the edit window):
  * corner          start, mid, end              -> arc through the 3 points
  * roundabout      3 points, one per exit       -> circle through them
  * lane change     centre (where the 2 roads connect) + length handle
                    (stretches the connection); the S-curve runs from the lower
                    lane, through the centre, to the upper lane
  * road ends       free end of the start lane and of the perpendicular road
  * parallel bay    slide handle: moves the bay up/down along the parking road
Everything else hangs off those points: straights join them, the lane-change
area spans the two lanes, the perpendicular bay sits at the end of its road,
the parallel bay sits beside the parking road at the slide position.

Frames: `track` = the competition drawing (metres, x east, y north).
        `venue` = UWB coordinates. venue_transform maps track -> venue.

Run:
  python map_builder.py demo                      # synthetic lap, checks the fit works
  python map_builder.py show [lap.csv]            # picture of the fit (window + map_fit.png)
  python map_builder.py edit [lap.csv]            # drag the points, f = refit, s = save
fit, show and edit all write track_map.yaml (edit also saves when you close it);
the full path of the file is printed. demo writes nothing.
  python map_builder.py fit lap.csv [--init X Y YAW_DEG] [-o track_map.yaml]
No command (e.g. the VS Code Run button) = edit. No lap.csv = the synthetic demo lap. lap.csv: x,y per line (venue metres), header ok.
Needs: numpy, opencv-python, pyyaml.  No ROS.  Runs on Windows or the RDK X5.

-------------------------------------------------------------------------------
HANDOFF NOTES (for whoever continues this file)
-------------------------------------------------------------------------------
Project: RISA Bot, NxGV Driverless CarBot Challenge 26/27 (25-26 Sept 2026).
Target architecture: 16 blocks from the V4 JS simulator. This file = block 01.

Decided workflow (calibration web page "Build map"):
  * Drive one lap; the fused UWB + IMU + encoder + EKF position is recorded.
    The EKF must NOT use the map during this lap (no camera-to-map matching),
    otherwise the map would be fitted to a path already bent towards it.
  * The recorded lap is shown LOCKED on a grid; the map is laid over it.
  * Auto-fit first, manual touch-up second:
      stage 1  whole map move + rotate                        DONE (fit_rigid)
      stage 2  nudge each control point towards the lap,
               limited to +/-15 cm; uncovered points stay put  TODO
      stage 3  user drags handles, then confirms               DONE as OpenCV
               `edit` window (Editor class); web page TODO
  * Output file 1: track_map.yaml (venue_transform, control points, derived
    geometry, fit_report). Made once per venue. It holds ROAD GEOMETRY ONLY:
    no start pose, no goals, no checkpoints, no route.
  * Output file 2 (separate script mission_planner.py, TODO): reads
    track_map.yaml, user places start + goals, it computes checkpoints and the
    route (V4 hybridPlan) and saves mission.yaml with the map's fingerprint so
    race mode refuses a route computed on a different map.

Geometry rules:
  * Areas are rectangles in the frame of a road line (along, lateral), so they
    follow that road even when it is tilted: parallel bay/opening on the
    parking spur at the slide position; perpendicular zone/bay/apron anchored
    to the END of the perpendicular road; lane change spans both lanes.
  * Paint sides are named in that frame: start / end (along), near / far
    (lateral, relative to the road line), divider (middle).

Verified: default control points reproduce the V4 core.js centrelines and
drivable rects exactly; demo fit error < 2 mm / 0.02 deg from no guess and
from a 20 cm / 8 deg drag.

Roundabout flares (V4 junction mouths) are areas flare_west/north/east, built
from the ring radius and each exit direction; mission_planner.py needs them
to turn in and out of the ring.

Road region: road_field() = signed clearance raster (lane band around every
centreline + every drivable area incl. flares), 1 cm. mission_planner.py uses
it for the car-body fit check and the route search.

TODO (in order): stage 2 point fit; kink warning at corner/straight and lane-change joins;
attach light, gates, bump, hill, tunnel to control points; lap recorder
(CSV); web page; mission_planner.py.
-------------------------------------------------------------------------------
"""
import argparse
import copy
import math
import sys

import numpy as np

PI = math.pi

# =============================================================================
# 1. TUNABLES
# =============================================================================
CAR = dict(length=0.30, width=0.192, wheelbase=0.216, rear_overhang=0.042, min_turn_radius=0.40)
LANE_WIDTH = 0.30

FIT = dict(
    sample_step_m=0.01,       # centreline sampling used for matching
    field_res_m=0.02,         # distance-field cell size for the coarse search
    coarse_yaw_step_deg=5.0,  # coarse search (only when no initial guess is given)
    coarse_xy_step_m=0.10,
    coarse_points=250,        # lap points used in the coarse search
    drag_window_m=0.60,       # with a dragged guess: search +/- this around it ...
    drag_window_deg=20.0,     # ... and +/- this much rotation
    icp_iterations=60,
    icp_reject_m=0.15,        # ignore lap points further than this from the map
    covered_within_m=0.05,    # a centreline sample counts as "seen" if the lap passed this close
    covered_ratio=0.5,        # section is "fitted" when this share of it was seen
    ok_rms_m=0.04,            # section error above this is shown red
)

EDIT = dict(
    min_lane_change_half_m=0.10,   # length handle cannot go closer than this to the centre
    pick_radius_px=14,             # how close a click must be to grab a handle
)


# =============================================================================
# 2. TEMPLATE CONTROL POINTS (V4 simulator / rulebook drawing, track frame)
# =============================================================================
def _corner(cx, cy, r, a0, a1):
    """Corner points of a V4 arc: start, mid, end."""
    am = (a0 + a1) / 2
    p = lambda a: (cx + r * math.cos(a), cy + r * math.sin(a))  # noqa: E731
    return {'start': p(a0), 'mid': p(am), 'end': p(a1)}


TEMPLATE = {
    'corners': {
        'top_left':       _corner(1.00, 4.00, 0.75, PI / 2, PI),
        'tunnel_corner':  _corner(1.00, 1.50, 0.75, PI, 1.5 * PI),
        'top_right':      _corner(6.00, 4.00, 0.75, 0.0, PI / 2),
        'bottom_right':   _corner(6.00, 2.05, 0.75, -PI / 2, 0.0),
        'parking_corner': _corner(2.80, 3.15, 0.40, PI / 2, PI),
    },
    # one point per exit; the circle through them is the ring centreline
    'roundabout': {'west': (1.80, 0.80), 'north': (2.40, 1.40), 'east': (3.00, 0.80)},
    # Lane change (80 x 80 cm in V4): the lower lane comes from the roundabout east
    # exit, the upper lane is the start lane (y of the bottom-right corner start).
    # centre = where the two roads connect; length_handle = centre + half length.
    'lane_change': {'centre': (3.95, 1.05), 'length_handle': (4.35, 1.05)},
    # free road ends
    'road_ends': {'start_lane_end': (7.50, 1.30), 'perp_row_end': (3.91, 3.55)},
    # parallel bay position: distance of the bay centre along the parking spur
    'parallel_bay': {'along_m': 1.195},
    # straights: each end is a reference "group.name.point" (+ optional offset)
    'straights': {
        'top_straight':    [('corners.top_left.start', 0, 0),      ('corners.top_right.end', 0, 0)],
        'left_straight':   [('corners.top_left.end', 0, 0),        ('corners.tunnel_corner.start', 0, 0)],
        'gate_approach':   [('corners.tunnel_corner.end', 0, 0),   ('roundabout.west', 0.10, -0.05)],
        'right_straight':  [('corners.top_right.start', 0, 0),     ('corners.bottom_right.end', 0, 0)],
        'start_lane':      [('lane_change.east', 0, 0),            ('road_ends.start_lane_end', 0, 0)],
        'roundabout_east': [('roundabout.east', -0.05, 0),         ('lane_change.west', 0, 0)],
        'parking_spur':    [('roundabout.north', 0, -0.05),        ('corners.parking_corner.end', 0, 0)],
        'perp_row':        [('corners.parking_corner.start', 0, 0), ('road_ends.perp_row_end', 0, 0)],
    },
    # Areas = rectangles in the frame of a road line: `along` from a reference
    # ('start' | 'end' of the line, or 'slide' = parallel_bay.along_m) and
    # `lateral` to the LEFT of the line's direction (negative = right side).
    # V4 photo dimensions (cm): parallel bay 47 x 69; perpendicular bay 45 wide x
    # 47 deep with 17 cm free on the left and 24 cm on the right; 3 cm paint.
    'areas': {
        'parallel_opening': {'kind': 'drivable', 'line': 'parking_spur', 'paint': {},
                             'along': [('slide', -0.345), ('slide', 0.345)], 'lateral': (0.0, 0.62)},
        'parallel_bay': {'kind': 'bay', 'line': 'parking_spur', 'heading': 'along',
                         'paint': {'far': 'solid', 'start': 'solid', 'end': 'solid', 'near': 'dashed'},
                         'along': [('slide', -0.345), ('slide', 0.345)], 'lateral': (0.15, 0.62)},
        'perpendicular_apron': {'kind': 'drivable', 'line': 'perp_row', 'paint': {},
                                'along': [('start', 0.0), ('end', 0.15)], 'lateral': (-0.15, 0.15)},
        'perpendicular_zone': {'kind': 'drivable', 'line': 'perp_row',
                               'paint': {'start': 'solid', 'end': 'solid', 'far': 'solid'},
                               'along': [('end', -0.77), ('end', 0.15)], 'lateral': (-0.62, -0.15)},
        'perpendicular_bay': {'kind': 'bay', 'line': 'perp_row', 'heading': 'into',
                              'paint': {'start': 'dashed', 'end': 'dashed'},
                              'along': [('end', -0.57), ('end', -0.12)], 'lateral': (-0.62, -0.15)},
    },
    # No start / goal poses here on purpose: they belong to mission_planner.py.
}


# =============================================================================
# 3. GEOMETRY: control points -> centrelines and areas
# =============================================================================
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


def sample_arc3(s, m, e, step):
    """Arc from s through m to e. Straight line if the 3 points are collinear."""
    c = circle_through(s, m, e)
    if c is None:
        return sample_line(s, e, step)
    (cx, cy), r = c
    ang = lambda p: math.atan2(p[1] - cy, p[0] - cx)  # noqa: E731
    a0, am, a1 = ang(s), ang(m), ang(e)
    ccw = lambda a: (a - a0) % (2 * PI)  # noqa: E731
    sweep = ccw(a1) if ccw(am) < ccw(a1) else ccw(a1) - 2 * PI   # go the way that passes m
    n = max(1, math.ceil(abs(sweep) * r / step))
    a = a0 + sweep * np.linspace(0, 1, n + 1)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])


def lane_change_geometry(tpl):
    """West end (lower lane), centre, east end (upper lane) of the lane change."""
    lc = tpl['lane_change']
    cx, cy = lc['centre']
    half = max(EDIT['min_lane_change_half_m'], lc['length_handle'][0] - cx)
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


def set_point(tpl, path, value):
    keys = path.split('.')
    node = tpl
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = (float(value[0]), float(value[1]))


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
    """S-curve lower lane -> centre -> upper lane. With the centre in the middle it is
    one smoothstep; moving the centre shifts where the two roads meet."""
    (x0, y0), (cx, cy), (x1, y1) = lane_change_geometry(tpl)
    ss = lambda u: u * u * (3 - 2 * u)  # noqa: E731

    def half(xa, xb, f):
        n = max(1, math.ceil(abs(xb - xa) / step))
        t = np.linspace(0, 1, n + 1)
        return xa + (xb - xa) * t, f(t)

    xa, ya = half(x0, cx, lambda t: y0 + (cy - y0) * 2 * ss(t / 2))
    xb, yb = half(cx, x1, lambda t: cy + (y1 - cy) * (2 * ss(0.5 + t / 2) - 1))
    return np.column_stack([np.concatenate([xa, xb[1:]]), np.concatenate([ya, yb[1:]])])


def centrelines(tpl, step=0.025):
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


def parallel_slide_limits(tpl):
    half = max(abs(v) for _, v in tpl['areas']['parallel_bay']['along'])
    L = line_frame(tpl, 'parking_spur')[3]
    return half, max(half, L - half)


def areas(tpl):
    """Every area as {'kind', 'poly' (4x2 track frame), 'frame', 'along', 'lateral', 'paint', ...}."""
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
    # lane change area: spans both lanes (half a lane outside each) between its ends
    (x0, yl), _, (x1, yh) = lane_change_geometry(tpl)
    lo, hi = min(yl, yh) - LANE_WIDTH / 2, max(yl, yh) + LANE_WIDTH / 2
    A, u, n = np.array([x0, 0.0]), np.array([1.0, 0.0]), np.array([0.0, 1.0])
    out['lane_change'] = {'kind': 'drivable', 'paint': {'divider': 'dashed'}, 'frame': (A, u, n),
                          'along': (0.0, x1 - x0), 'lateral': (lo, hi), 'heading': None,
                          'poly': np.array([[x0, lo], [x1, lo], [x1, hi], [x0, hi]])}
    # roundabout junction mouths (V4 flares): tapered drivable area at each exit
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


# V4 flare polygon for one roundabout exit: (distance past the ring radius, sideways),
# traced from the drawing (east exit: (2.90, 1.18), (3.18, 0.95), (3.50, 0.95), ...).
FLARE = [(-0.10, 0.38), (0.18, 0.15), (0.50, 0.15), (0.50, -0.15), (0.18, -0.15), (-0.10, -0.38)]


def paint_segments(area, dash=0.14, gap=0.06):
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
        while d < L:                                   # dashes, like the V4 paint loops
            e = min(d + dash, L)
            out.append(((a[0] + (b[0] - a[0]) * d / L, a[1] + (b[1] - a[1]) * d / L),
                        (a[0] + (b[0] - a[0]) * e / L, a[1] + (b[1] - a[1]) * e / L), style))
            d += dash + gap
    return out


def road_field(tpl, res=0.01, pad=0.4):
    """Signed clearance raster (V4 Course.field idea): metres to the road edge,
    + inside the road. Road = lane band around every centreline + every drivable
    area (lane change, bays' openings, perpendicular zone/apron, roundabout flares).
    Returns (field float32 [ny, nx], x0, y0, res); cell (j, i) is at (x0 + i res, y0 + j res)."""
    import cv2
    pts = np.vstack(list(centrelines(tpl, res / 2).values()))
    x0, y0 = pts.min(0) - pad
    nx, ny = (np.ceil((pts.max(0) + pad - (x0, y0)) / res)).astype(int) + 1
    ij = lambda p: np.round((np.asarray(p) - (x0, y0)) / res).astype(np.int32)  # noqa: E731
    img = np.full((ny, nx), 255, np.uint8)
    q = ij(pts)
    img[q[:, 1], q[:, 0]] = 0
    field = LANE_WIDTH / 2 - cv2.distanceTransform(img, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    for a in areas(tpl).values():
        if a['kind'] != 'drivable':
            continue
        mask = np.zeros((ny, nx), np.uint8)
        cv2.fillPoly(mask, [ij(a['poly'])], 1)
        inside = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
        outside = cv2.distanceTransform(1 - mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
        field = np.maximum(field, np.where(mask > 0, inside, -outside))
    return field.astype(np.float32), float(x0), float(y0), res


def handles(tpl):
    """Every movable control point: [(id, (x, y) track frame, kind)].
    kind: free | lc_centre | lc_length | bay_slide"""
    out = []
    for n, c in tpl['corners'].items():
        for k in ('start', 'mid', 'end'):
            out.append((f'corners.{n}.{k}', c[k], 'free'))
    for k, p in tpl['roundabout'].items():
        out.append((f'roundabout.{k}', p, 'free'))
    out.append(('lane_change.centre', tpl['lane_change']['centre'], 'lc_centre'))
    lc = lane_change_geometry(tpl)
    out.append(('lane_change.length_handle', (lc[2][0], lc[1][1]), 'lc_length'))
    for k, p in tpl['road_ends'].items():
        out.append((f'road_ends.{k}', p, 'free'))
    A, u, _, _ = line_frame(tpl, 'parking_spur')
    out.append(('parallel_bay.slide', tuple(A + u * tpl['parallel_bay']['along_m']), 'bay_slide'))
    return out


def move_handle(tpl, hid, kind, q):
    """Move handle `hid` to track point q, respecting its constraint."""
    q = (float(q[0]), float(q[1]))
    if kind == 'free':
        set_point(tpl, hid, q)
    elif kind == 'lc_centre':                        # length handle travels with the centre
        c0, h0 = tpl['lane_change']['centre'], tpl['lane_change']['length_handle']
        tpl['lane_change']['centre'] = q
        tpl['lane_change']['length_handle'] = (h0[0] + q[0] - c0[0], q[1])
    elif kind == 'lc_length':                        # only stretches, along the road
        cx, cy = tpl['lane_change']['centre']
        tpl['lane_change']['length_handle'] = (max(q[0], cx + EDIT['min_lane_change_half_m']), cy)
    elif kind == 'bay_slide':                        # stays on the parking road
        A, u, _, _ = line_frame(tpl, 'parking_spur')
        lo, hi = parallel_slide_limits(tpl)
        tpl['parallel_bay']['along_m'] = float(np.clip(np.dot(np.asarray(q) - A, u), lo, hi))


# =============================================================================
# 4. 2-D RIGID TRANSFORMS  (x, y, yaw): track -> venue
# =============================================================================
def apply(T, pts):
    x, y, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])


def invert(T):
    x, y, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), -(-s * x + c * y), -yaw)


def best_rigid(src, dst):
    """Least-squares (x, y, yaw) with dst ~= R src + t (2-D Kabsch)."""
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    yaw = math.atan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    c, s = math.cos(yaw), math.sin(yaw)
    return (md[0] - (c * ms[0] - s * ms[1]), md[1] - (s * ms[0] + c * ms[1]), yaw)


# =============================================================================
# 5. AUTO-FIT, stage 1: move + rotate the whole map onto the recorded lap
# =============================================================================
class LineIndex:
    """All centreline samples with the section each belongs to, for nearest-point lookup."""

    def __init__(self, lines):
        self.names = list(lines)
        self.pts = np.vstack([lines[n] for n in self.names])
        self.owner = np.concatenate([np.full(len(lines[n]), i) for i, n in enumerate(self.names)])

    def nearest(self, q, chunk=400):
        idx = np.empty(len(q), dtype=np.int64)
        dist = np.empty(len(q))
        for k in range(0, len(q), chunk):
            d2 = ((q[k:k + chunk, None, :] - self.pts[None, :, :]) ** 2).sum(-1)
            j = d2.argmin(1)
            idx[k:k + chunk] = j
            dist[k:k + chunk] = np.sqrt(d2[np.arange(len(j)), j])
        return idx, dist


def distance_field(pts, res, pad=1.0):
    """Grid of distance (m) to the nearest centreline, for the fast coarse search."""
    import cv2
    x0, y0 = pts.min(0) - pad
    nx, ny = (np.ceil((pts.max(0) + pad - (x0, y0)) / res)).astype(int) + 1
    img = np.full((ny, nx), 255, np.uint8)
    ij = np.round((pts - (x0, y0)) / res).astype(int)
    img[ij[:, 1], ij[:, 0]] = 0
    d = cv2.distanceTransform(img, cv2.DIST_L2, 5) * res
    return d, x0, y0


def coarse_search(lap, lines, around=None):
    """Try many headings and positions, keep the best.

    around=None: search everywhere (a few seconds).
    around=T:    only near the guess T you dragged on the page (fast, and cannot
                 jump to a wrong-but-similar-looking part of the track).
    """
    pts = np.vstack(list(lines.values()))
    field, x0, y0 = distance_field(pts, FIT['field_res_m'])
    ny, nx = field.shape
    step = max(1, len(lap) // FIT['coarse_points'])
    centre = lap.mean(0)
    sub = lap[::step] - centre                        # lap centred on its own middle
    xy, dxy = FIT['coarse_xy_step_m'], FIT['drag_window_m']
    if around is None:
        xs = np.arange(pts[:, 0].min(), pts[:, 0].max(), xy)
        ys = np.arange(pts[:, 1].min(), pts[:, 1].max(), xy)
        yaws = np.arange(0, 360, FIT['coarse_yaw_step_deg'])
    else:
        gx, gy = apply(invert(around), centre[None, :])[0]     # where the guess puts the lap centre
        g_yaw = math.degrees(-around[2])
        xs = np.arange(gx - dxy, gx + dxy + 1e-9, xy / 2)
        ys = np.arange(gy - dxy, gy + dxy + 1e-9, xy / 2)
        yaws = g_yaw + np.arange(-FIT['drag_window_deg'], FIT['drag_window_deg'] + 1e-9, 1.0)
    best = (np.inf, None)
    for yaw in np.radians(yaws):
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.column_stack([c * sub[:, 0] - s * sub[:, 1], s * sub[:, 0] + c * sub[:, 1]])
        gx = ((rot[:, 0][None, None, :] + xs[None, :, None] - x0) / FIT['field_res_m']).astype(int)
        gy = ((rot[:, 1][None, None, :] + ys[:, None, None] - y0) / FIT['field_res_m']).astype(int)
        gx = np.clip(gx, 0, nx - 1)
        gy = np.clip(gy, 0, ny - 1)
        cost = np.minimum(field[gy, gx], FIT['icp_reject_m']).mean(-1)   # (len(ys), len(xs))
        k = np.unravel_index(cost.argmin(), cost.shape)
        if cost[k] < best[0]:
            best = (cost[k], (xs[k[1]], ys[k[0]], yaw))
    # candidate is "lap -> track"; turn it into "track -> venue"
    tx, ty, yaw = best[1]
    lap_to_track = (tx - (math.cos(yaw) * centre[0] - math.sin(yaw) * centre[1]),
                    ty - (math.sin(yaw) * centre[0] + math.cos(yaw) * centre[1]), yaw)
    return invert(lap_to_track)


def fit_rigid(lap, tpl=TEMPLATE, init=None):
    """Find venue_transform (track -> venue) that puts the template on the lap.

    lap:  Nx2 array of recorded positions, venue frame.
    init: (x, y, yaw) rough guess from dragging the map on the page, or None
          to search everywhere.
    Returns (T, report).
    """
    lines = centrelines(tpl, FIT['sample_step_m'])
    index = LineIndex(lines)
    T = coarse_search(lap, lines, around=tuple(init) if init is not None else None)
    reject = FIT['icp_reject_m']
    for it in range(FIT['icp_iterations']):
        in_track = apply(invert(T), lap)
        idx, dist = index.nearest(in_track)
        keep = dist < reject
        if keep.sum() < 10:
            raise RuntimeError('auto-fit failed: the lap does not overlap the map. Drag the map closer first.')
        T_new = best_rigid(index.pts[idx[keep]], lap[keep])
        moved = math.hypot(T_new[0] - T[0], T_new[1] - T[1]) + abs(T_new[2] - T[2])
        T = T_new
        reject = max(FIT['covered_within_m'], min(reject, 3 * np.median(dist[keep])))   # tighten as it converges
        if moved < 1e-6:
            break
    return T, section_report(lap, T, lines, index)


def section_report(lap, T, lines, index):
    """Per section: rms error of the lap points matched to it, and how much of it the lap covered."""
    in_track = apply(invert(T), lap)
    idx, dist = index.nearest(in_track)
    sec = index.owner[idx]
    report = {}
    for i, name in enumerate(index.names):
        mine = (sec == i) & (dist < FIT['icp_reject_m'])
        # coverage: share of this section's samples with a lap point nearby
        pts = lines[name]
        near = LineIndex({'lap': in_track[mine]}).nearest(pts)[1] if mine.any() else np.full(len(pts), np.inf)
        covered = float(np.mean(near < FIT['covered_within_m']))
        rms = float(np.sqrt(np.mean(dist[mine] ** 2))) if mine.any() else None
        fitted = covered >= FIT['covered_ratio']
        report[name] = {'rms_m': rms, 'covered': round(covered, 2), 'fitted': fitted,
                        'ok': fitted and rms is not None and rms <= FIT['ok_rms_m']}
    return report


# =============================================================================
# 6. SAVE
# =============================================================================
def to_yaml_dict(tpl, T, report):
    f = lambda v: round(float(v), 4)  # noqa: E731
    pt = lambda p: [f(p[0]), f(p[1])]  # noqa: E731
    lc = lane_change_geometry(tpl)
    return {
        'version': 2,
        'frame': 'track',
        'venue_transform': {'x': f(T[0]), 'y': f(T[1]), 'yaw_deg': round(math.degrees(float(T[2])), 3)},
        'lane_width_m': LANE_WIDTH,
        # ---- control points (what you moved)
        'corners': {n: {k: pt(v) for k, v in c.items()} for n, c in tpl['corners'].items()},
        'roundabout': {k: pt(v) for k, v in tpl['roundabout'].items()},
        'lane_change': {'centre': pt(tpl['lane_change']['centre']),
                        'length_handle': pt(tpl['lane_change']['length_handle'])},
        'road_ends': {k: pt(v) for k, v in tpl['road_ends'].items()},
        'parallel_bay': {'along_m': f(tpl['parallel_bay']['along_m'])},
        # ---- how the rest hangs off them
        'straights': {n: [list(a) for a in s] for n, s in tpl['straights'].items()},
        'area_rules': {n: {'kind': a['kind'], 'line': a['line'], 'along': [list(x) for x in a['along']],
                           'lateral': list(a['lateral']), 'paint': dict(a['paint']),
                           **({'heading': a['heading']} if 'heading' in a else {})}
                       for n, a in tpl['areas'].items()},
        # ---- resolved geometry (track frame), ready to use
        'derived': {
            'lane_change': {'west': pt(lc[0]), 'centre': pt(lc[1]), 'east': pt(lc[2])},
            'straight_ends': {n: [pt(p) for p in straight_ends(tpl, n)] for n in tpl['straights']},
            'areas': {n: {'kind': a['kind'], 'poly': [pt(p) for p in a['poly']],
                          **({'heading': f(a['heading'])} if a['heading'] is not None else {})}
                      for n, a in areas(tpl).items()},
        },
        'fit_report': {n: {k: (f(v) if isinstance(v, (float, np.floating)) else
                               bool(v) if isinstance(v, (bool, np.bool_)) else v) for k, v in r.items()}
                       for n, r in report.items()},
    }


def save_yaml(path, d):
    """Write the YAML and return its full path (printed so you can find the file)."""
    import os

    import yaml
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh, sort_keys=False)
    return os.path.abspath(path)


# =============================================================================
# 7. PICTURE OF THE FIT  (the calibration page will draw the same thing)
# =============================================================================
SHOW = dict(
    max_width_px=1250,        # picture is scaled to fit this ...
    max_height_px=760,        # ... and this (laptop screens)
    max_px_per_m=140,
    margin_px=50,
    colour_ok=(90, 200, 90),        # BGR
    colour_red=(60, 60, 235),
    colour_not_fitted=(150, 150, 150),
    colour_lap=(255, 200, 0),
    colour_handle=(255, 255, 255),
    colour_special=(0, 220, 255),   # lane change / road ends / bay slide handles
    colour_active=(0, 120, 255),
    lane_alpha=0.35,
)


def make_view(lap, T, tpl):
    """Fixed picture scale + origin so nothing jumps while you drag."""
    placed = [apply(T, p) for p in centrelines(tpl, 0.05).values()]
    allpts = np.vstack([lap] + placed)
    lo, hi = allpts.min(0) - 0.4, allpts.max(0) + 0.4
    m = SHOW['margin_px']
    s = min(SHOW['max_px_per_m'], (SHOW['max_width_px'] - 2 * m) / (hi[0] - lo[0]),
            (SHOW['max_height_px'] - 2 * m - 60) / (hi[1] - lo[1]))
    W = int((hi[0] - lo[0]) * s) + 2 * m
    H = int((hi[1] - lo[1]) * s) + 2 * m + 60
    return {'lo': lo, 'hi': hi, 's': s, 'm': m, 'W': W, 'H': H}


def to_px(view, p):
    p = np.atleast_2d(np.asarray(p, float))
    return np.column_stack([view['m'] + (p[:, 0] - view['lo'][0]) * view['s'],
                            view['H'] - view['m'] - (p[:, 1] - view['lo'][1]) * view['s']]).astype(np.int32)


def from_px(view, x, y):
    return np.array([view['lo'][0] + (x - view['m']) / view['s'],
                     view['lo'][1] + (view['H'] - view['m'] - y) / view['s']])


def draw_fit(lap, T, report, tpl=TEMPLATE, title='', view=None, active=None, help_text=''):
    """Recorded lap (venue frame, locked) with the fitted map and its handles on top."""
    import cv2
    view = view or make_view(lap, T, tpl)
    s, m, W, H, lo, hi = view['s'], view['m'], view['W'], view['H'], view['lo'], view['hi']
    px = lambda p: to_px(view, p)  # noqa: E731
    at = lambda p: px(apply(T, np.atleast_2d(np.asarray(p, float))))  # noqa: E731  track -> pixels
    img = np.full((H, W, 3), (34, 30, 28), np.uint8)

    for gx in np.arange(math.floor(lo[0] * 2) / 2, hi[0], 0.5):          # 0.5 m venue grid
        x = px([[gx, 0]])[0, 0]
        cv2.line(img, (x, 60), (x, H - m), (55, 55, 55), 1)
        cv2.putText(img, f'{gx:g}', (x - 10, H - m + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
    for gy in np.arange(math.floor(lo[1] * 2) / 2, hi[1], 0.5):
        y = px([[0, gy]])[0, 1]
        cv2.line(img, (m, y), (W - m, y), (55, 55, 55), 1)
        cv2.putText(img, f'{gy:g}', (6, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

    def colour(name):
        r = report.get(name)
        if r is None:
            return SHOW['colour_not_fitted']
        return SHOW['colour_ok'] if r['ok'] else (SHOW['colour_red'] if r['fitted'] else SHOW['colour_not_fitted'])

    lines = centrelines(tpl, 0.02)
    ars = areas(tpl)
    band = img.copy()
    lane_px = max(1, int(LANE_WIDTH * s))
    for name, p in lines.items():
        cv2.polylines(band, [at(p)], False, colour(name), lane_px, cv2.LINE_AA)
    for name, a in ars.items():
        if a['kind'] == 'drivable':
            cv2.fillPoly(band, [at(a['poly'])], colour('roundabout' if name.startswith('flare_') else name))
    img = cv2.addWeighted(band, SHOW['lane_alpha'], img, 1 - SHOW['lane_alpha'], 0)
    for name, p in lines.items():
        cv2.polylines(img, [at(p)], False, colour(name), 1, cv2.LINE_AA)

    paint_px = max(1, int(0.03 * s))
    for name, a in ars.items():
        if a['kind'] == 'bay':
            cv2.polylines(img, [at(a['poly'])], True, (240, 170, 60), 1, cv2.LINE_AA)
            c = at(a['poly'].mean(0))[0]
            cv2.putText(img, 'P', (c[0] - 5, c[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 170, 60), 1)
        for pa, pb, style in paint_segments(a):
            q = at([pa, pb])
            cv2.line(img, tuple(q[0]), tuple(q[1]), (255, 255, 255) if style == 'solid' else (200, 200, 200), paint_px)

    for q in px(lap):                                                     # recorded lap, locked
        cv2.circle(img, tuple(q), 1, SHOW['colour_lap'], -1)

    for hid, p, kind in handles(tpl):                                     # everything you can drag
        q = tuple(at(p)[0])
        col = SHOW['colour_active'] if hid == active else (
            SHOW['colour_handle'] if kind == 'free' and not hid.startswith('road_ends') else SHOW['colour_special'])
        if hid.startswith('roundabout'):
            cv2.drawMarker(img, q, col, cv2.MARKER_DIAMOND, 12, 2)
        elif kind == 'free' and not hid.startswith('road_ends'):
            cv2.circle(img, q, 4, col, -1)
        else:
            cv2.rectangle(img, (q[0] - 5, q[1] - 5), (q[0] + 5, q[1] + 5), col, -1)
    if active:
        cv2.putText(img, active, (m, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, SHOW['colour_active'], 1)

    fitted = [(n, r) for n, r in report.items() if r['fitted'] and r['rms_m'] is not None]
    cv2.putText(img, f"{title}  map->venue: x {T[0]:.3f} m  y {T[1]:.3f} m  yaw {math.degrees(T[2]):.2f} deg",
                (m, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    if fitted:
        wn, wr = max(fitted, key=lambda kv: kv[1]['rms_m'])
        info = f"sections OK {sum(r['ok'] for r in report.values())}/{len(report)}   worst: {wn} {wr['rms_m'] * 100:.1f} cm"
    else:
        info = 'no section fitted'
    cv2.putText(img, info + ('   |   ' + help_text if help_text else ''), (m, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
    y = 12
    for col, label in ((SHOW['colour_lap'], 'recorded lap (locked)'), (SHOW['colour_ok'], 'fitted, OK'),
                       (SHOW['colour_red'], 'fitted, too far off'), (SHOW['colour_not_fitted'], 'not driven')):
        cv2.rectangle(img, (W - 200, y), (W - 186, y + 10), col, -1)
        cv2.putText(img, label, (W - 180, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
        y += 15
    return img


def show(img, save_path=None, window=True):
    import cv2
    if save_path:
        cv2.imwrite(save_path, img)
        print(f'saved picture: {save_path}')
    if window:
        cv2.namedWindow('map_builder fit', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('map_builder fit', img)
        print('picture open: press any key in the window to close')
        cv2.waitKey(0)
        cv2.destroyAllWindows()


# =============================================================================
# 8. EDIT WINDOW: drag the handles (the web page will copy this behaviour)
# =============================================================================
class Editor:
    """Mouse: drag any handle.  Keys: f = refit whole map, r = reset points,
    s = save track_map.yaml, q / Esc = quit."""
    WINDOW = 'map_builder edit'
    HELP = 'drag points | f refit | r reset | s save | q quit'

    def __init__(self, lap, T, tpl, out_path, title=''):
        self.lap, self.T, self.out, self.title = lap, T, out_path, title
        self.tpl = copy.deepcopy(tpl)
        self.view = make_view(lap, T, self.tpl)
        self.active = None          # (id, kind) being dragged
        self.hover = None
        self.message = ''
        self.dirty = True
        self.update_report()

    def update_report(self):
        lines = centrelines(self.tpl, FIT['sample_step_m'])
        self.report = section_report(self.lap, self.T, lines, LineIndex(lines))

    def pick(self, x, y):
        best, best_d = None, EDIT['pick_radius_px']
        for hid, p, kind in handles(self.tpl):
            q = to_px(self.view, apply(self.T, np.atleast_2d(p)))[0]
            d = math.hypot(q[0] - x, q[1] - y)
            if d < best_d:
                best, best_d = (hid, kind), d
        return best

    def on_mouse(self, event, x, y, flags=0, param=None):
        import cv2
        if event == cv2.EVENT_LBUTTONDOWN:
            self.active = self.pick(x, y)
            self.dirty = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.active:
                q = apply(invert(self.T), from_px(self.view, x, y)[None, :])[0]    # pixel -> venue -> track
                move_handle(self.tpl, self.active[0], self.active[1], q)
                self.dirty = True
            else:
                h = self.pick(x, y)
                if h != self.hover:
                    self.hover, self.dirty = h, True
        elif event == cv2.EVENT_LBUTTONUP and self.active:
            self.active = None
            self.update_report()                     # colours update when you let go
            self.dirty = True

    def key(self, k):
        """Returns False to quit."""
        if k in (ord('q'), 27):
            return False
        if k == ord('f'):
            self.T, self.report = fit_rigid(self.lap, self.tpl, init=self.T)
            self.message = 'refitted'
        elif k == ord('r'):
            self.tpl = copy.deepcopy(TEMPLATE)
            self.update_report()
            self.message = 'points reset to the template'
        elif k == ord('s'):
            self.save()
        self.dirty = True
        return True

    def save(self):
        full = save_yaml(self.out, to_yaml_dict(self.tpl, self.T, self.report))
        self.message = f'saved {self.out}'
        print(f'saved {full}')

    def render(self):
        label = self.active[0] if self.active else (self.hover[0] if self.hover else None)
        return draw_fit(self.lap, self.T, self.report, self.tpl, self.title, self.view, label,
                        self.message or self.HELP)

    def run(self):
        import cv2
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.WINDOW, self.on_mouse)
        while True:
            if self.dirty:
                cv2.imshow(self.WINDOW, self.render())
                self.dirty = False
            k = cv2.waitKey(20) & 0xFF
            if k != 255 and not self.key(k):
                break
            if cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()
        self.save()                                  # closing the window always saves
        return self.tpl, self.T, self.report


# =============================================================================
# 9. COMMAND LINE
# =============================================================================
def print_report(T, report):
    print(f'venue_transform: x={T[0]:.3f} m  y={T[1]:.3f} m  yaw={math.degrees(T[2]):.2f} deg')
    for name, r in report.items():
        state = 'OK ' if r['ok'] else ('RED' if r['fitted'] else '-- ')
        rms = f"{r['rms_m'] * 100:5.1f} cm" if r['rms_m'] is not None else '   n/a  '
        print(f'  {state} {name:<16} error {rms}  covered {r["covered"] * 100:3.0f}%'
              + ('' if r['fitted'] else '  (not fitted: lap did not cover it)'))


def synthetic_lap(T, lateral_cm=2.0, noise_cm=1.5, seed=1):
    """A fake recorded lap: outer loop + lane change + roundabout, car wandering in the lane, UWB noise."""
    rng = np.random.default_rng(seed)
    lines = centrelines(TEMPLATE, 0.02)
    order = ['start_lane', 'lane_change', 'roundabout_east', 'roundabout', 'gate_approach', 'tunnel_corner',
             'left_straight', 'top_left', 'top_straight', 'top_right', 'right_straight', 'bottom_right']
    path = np.vstack([lines[n] for n in order])
    wander = lateral_cm / 100 * np.sin(np.linspace(0, 40, len(path)))[:, None]
    path = path + wander * np.array([[0.7, 0.7]]) + rng.normal(0, noise_cm / 100, path.shape)
    return apply(T, path)


DEMO_TRUTH = (1.23, -0.40, math.radians(1.8))


def load_lap(path):
    return np.loadtxt(path, delimiter=',', usecols=(0, 1), comments='#', skiprows=_header_rows(path))


def _header_rows(path):
    with open(path) as fh:
        first = fh.readline()
    try:
        [float(v) for v in first.split(',')[:2]]
        return 0
    except ValueError:
        return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description='RISA Bot prior map: fit the template to a recorded lap')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('demo', help='fit a synthetic lap and check the answer')
    for name, text in (('fit', 'fit a lap and save track_map.yaml'), ('show', 'picture of the fit'),
                       ('edit', 'drag the control points in a window')):
        p = sub.add_parser(name, help=text)
        p.add_argument('lap', nargs='?', help='CSV x,y in venue metres (default: synthetic demo lap)')
        p.add_argument('--init', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'))
        p.add_argument('-o', '--out', default='track_map.yaml')
        if name == 'show':
            p.add_argument('--save', default='map_fit.png')
            p.add_argument('--no-window', action='store_true', help='only save the PNG')
    argv = sys.argv[1:] if argv is None else argv
    if not argv:                     # VS Code Run button / double-click: open the editor on the demo lap
        print('no command given -> running: edit   (other commands: demo, show, fit; see --help)')
        argv = ['edit']
    a = ap.parse_args(argv)

    if a.cmd == 'demo':
        lap = synthetic_lap(DEMO_TRUTH)
        # a rough drag on the page: map 20 cm off and turned 8 deg about its own centre
        mc = np.array([3.8, 2.6])
        c8, s8 = math.cos(math.radians(8)), math.sin(math.radians(8))
        shift = mc - np.array([c8 * mc[0] - s8 * mc[1], s8 * mc[0] + c8 * mc[1]])
        drag = (shift[0] + 0.20, shift[1], math.radians(8))
        dc, ds = math.cos(DEMO_TRUTH[2]), math.sin(DEMO_TRUTH[2])
        init = (DEMO_TRUTH[0] + dc * drag[0] - ds * drag[1], DEMO_TRUTH[1] + ds * drag[0] + dc * drag[1],
                DEMO_TRUTH[2] + drag[2])
        for label, guess in (('no initial guess', None), ('rough drag (20 cm, 8 deg about map centre)', init)):
            T, report = fit_rigid(lap, init=guess)
            err = (math.hypot(T[0] - DEMO_TRUTH[0], T[1] - DEMO_TRUTH[1]) * 100, math.degrees(T[2] - DEMO_TRUTH[2]))
            print(f'\n[{label}] error vs truth: {err[0]:.2f} cm, {err[1]:.3f} deg')
            print_report(T, report)
        return 0

    lap = load_lap(a.lap) if a.lap else synthetic_lap(DEMO_TRUTH)
    title = a.lap or 'demo lap'
    init = (a.init[0], a.init[1], math.radians(a.init[2])) if a.init else None
    T, report = fit_rigid(lap, init=init)
    print_report(T, report)
    if a.cmd in ('fit', 'show'):
        print(f'saved {save_yaml(a.out, to_yaml_dict(TEMPLATE, T, report))}')
    if a.cmd == 'show':
        show(draw_fit(lap, T, report, title=title), a.save, not a.no_window)
    elif a.cmd == 'edit':
        print(Editor.__doc__)
        Editor(lap, T, TEMPLATE, a.out, title).run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
