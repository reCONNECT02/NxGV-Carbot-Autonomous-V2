#!/usr/bin/env python3
"""mission_planner.py -- RISA Bot, BLOCK 07: start / end poses of each leg + route.

Reads the track_map.yaml made by map_builder.py and walks you through the legs
one by one; each leg's END is the next leg's START:

    P0 --leg1--> P1 --leg2--> P2 --leg3--> P3
    start                                  finish

Guided steps:
  Step 1  place P0 (start)
  Step 2  place P1  -> route of leg1 is calculated and drawn
  Step 3  place P2  -> route of leg2
  Step 4  place P3  -> route of leg3
  Step 5  review all routes, press s to save mission.yaml
Enter (or n) = next step, only when the car fits and the leg has a route; if not,
the reason is shown in red at the top. Backspace (or b) = previous step. The route is recalculated the moment you let go of
the car (or rotate / flip it) - only if the pose really changed; a plain click
does nothing. Calculating runs in the background.

Run (same folder as map_builder.py and track_map.yaml):
  python mission_planner.py                          # track_map.yaml -> mission.yaml
  python mission_planner.py my_map.yaml -o my_mission.yaml
  python mission_planner.py --check                  # no window: plan all legs, print, save PNG + yaml

Window controls (only the car of the current step moves):
  drag the car            move it          drag the orange dot   rotate it
  [ ]  rotate 5 deg       , .  rotate 1 deg
  f    flip: turn the car round on the spot (nose-in <-> reverse-in)
  g    snap on/off: to the lane centre + lane direction, or centred in a bay
  r    put this pose back to its default
  Enter or n  next step    Backspace or b  previous step    s save (step 5)    q / Esc quit
  (click on the map window first so it has the keyboard, not VS Code)

Poses: REAR AXLE centre + heading (V4 convention). Green car = whole body on
the road, red = some part off it.

How a route is found (V4 core.js hybridPlan, ported):
  * car-like search: 4 cm steps at 5 steering values, whole body checked on the
    road at every step (V4 body sampling, 8 mm planning margin), costs as V4.
  * guided by a LANE GRAPH: every lane can be driven either way, no U-turns,
    the roundabout only clockwise. Its distance-to-goal is the search
    heuristic, so the search follows the road and picks the roundabout exit
    itself from where the end pose is. Clockwise ring rule as V4.
  * parking bays: the car drives to a spot on the bay's road (after the bay for
    parallel / reverse-in, before it for nose-in), then a manoeuvre with
    reverse (2.5 cm steps, V4 reverse costs) goes into the bay. Leaving a bay
    is the same in the other direction. Reverse parts are drawn orange.

-------------------------------------------------------------------------------
HANDOFF NOTES (for whoever continues this file)
-------------------------------------------------------------------------------
* Needs map_builder.py in the same folder: the road is rebuilt from the
  control points in track_map.yaml (map_builder.road_field, centrelines,
  areas incl. roundabout flares), so both scripts always agree on the road.
* mission.yaml stores the map file's sha1. Race mode must refuse to arm if it
  does not match the track_map.yaml it loads.
* Poses/routes are in the TRACK frame; track_map.yaml venue_transform converts
  to UWB coordinates.
* Roundabout exits are chosen HERE (lane-graph shortest legal path), stored
  per leg as roundabout_exits; the boom gate never changes them.
* The parking manoeuvre stored here is a preview: at race time block 11 plans
  it again from the OBSERVED bay (no pre-recorded motion).
* Parking = V4 parkingPlan order: docking straight (15/12/8 cm) + Reeds-Shepp,
  direct Reeds-Shepp, handoff extension, bounded reverse search with an RS
  finish. Forward-in docking is also tried (V4 only reversed in). The same
  function drives OUT of a bay (start in a bay).
* TODO: let the user force an exit per leg.
-------------------------------------------------------------------------------
"""
import argparse
import copy
import hashlib
import heapq
import math
import os
import sys
import threading
import time

import numpy as np

import map_builder as mb

PI = math.pi
TAU = 2 * PI

# =============================================================================
# 1. TUNABLES
# =============================================================================
LEGS = 3                  # P0 -> P1 -> P2 -> P3
FIT_PAD_M = 0.0           # extra margin the car body must keep from the road edge (pose check)
RECALC_DELAY_S = 0.0      # start recalculating this long after you let go (0 = immediately)
ROTATE_HANDLE_M = 0.30    # orange rotate handle distance in front of the pose

PLAN = dict(                                   # V4 core.js hybridPlan values
    step_m=0.04, reverse_step_m=0.025,
    xy_res_m=0.012, reverse_xy_res_m=0.014, heading_bins=180,
    curvature_fractions=(-1.0, -0.5, 0.0, 0.5, 1.0),       # x 1 / min turning radius
    margin_m=0.008, reverse_margin_m=0.006,                 # body must stay this far inside the road
    reverse_cost=1.07, gear_change_cost=0.08, steer_cost=0.0003, steer_change_cost=0.001,
    edge_cost=0.003,                                        # step * edge_cost / max(0.006, corner margin)
    heuristic_weight=1.25,
    goal_xy_m=0.025, goal_heading_rad=0.035,                # road legs
    park_goal_xy_m=0.02, park_goal_heading_rad=0.05,        # manoeuvres (V4: RS connector + 1 cm)
    road_max_nodes=40000, manoeuvre_max_nodes=20000,
    manoeuvre_box_m=0.7,                                    # manoeuvre search stays in this box around bay + handoff
    ring_band_m=0.20,                                       # clockwise rule within ring radius +/- this
    wrong_way_penalty_m=30.0,                               # heuristic for states against the lane graph
    graph_step_m=0.05, junction_radius_m=0.15, junction_max_turn_deg=100.0,
    # handoff spots on the bay's road, along-distance from the bay's ends (V4: parallel +13 cm past the bay)
    handoff_after_m=(0.13, 0.25, 0.40), handoff_before_m=(0.35, 0.50, 0.25),
)

DEFAULT_POSES = [
    ('start',                 (7.18, 1.30, PI)),        # V4 start, bottom-right, facing west
    ('traffic light stop',    (6.75, 3.80, -PI / 2)),   # V4 light goal
    ('parallel parking',      None),                    # None = centred in the bay (computed)
    ('perpendicular parking', None),
]
DEFAULT_BAYS = {2: 'parallel_bay', 3: 'perpendicular_bay'}

COLOURS = dict(
    road=(92, 92, 92), paint=(255, 255, 255), dashed=(200, 200, 200), bay=(240, 170, 60),
    ok=(90, 210, 90), bad=(60, 60, 235), selected=(0, 220, 255), rotate=(0, 140, 255),
    reverse=(0, 150, 255),
    legs=[(255, 170, 40), (230, 110, 230), (80, 220, 220), (120, 200, 120)],   # leg1, leg2, leg3, ...
    text=(230, 230, 230), dim=(150, 150, 150),
)

CAR_REAR = mb.CAR['rear_overhang']
CAR_FRONT = mb.CAR['length'] - mb.CAR['rear_overhang']
CAR_HALF_W = mb.CAR['width'] / 2
MIN_R = mb.CAR['min_turn_radius']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# =============================================================================
# 2. LOAD THE MAP
# =============================================================================
def load_map(path):
    """track_map.yaml -> (template for map_builder geometry, sha1 of the file)."""
    import yaml
    with open(path, 'rb') as fh:
        raw = fh.read()
    d = yaml.safe_load(raw)
    if d.get('version') != 2:
        raise SystemExit(f'{path}: version {d.get("version")}, expected 2. Re-save it with the current map_builder.py.')
    tpl = copy.deepcopy(mb.TEMPLATE)
    tpl['corners'] = {n: {k: tuple(v) for k, v in c.items()} for n, c in d['corners'].items()}
    tpl['roundabout'] = {k: tuple(v) for k, v in d['roundabout'].items()}
    tpl['lane_change'] = {k: tuple(v) for k, v in d['lane_change'].items()}
    tpl['road_ends'] = {k: tuple(v) for k, v in d['road_ends'].items()}
    tpl['parallel_bay'] = dict(d['parallel_bay'])
    tpl['straights'] = {n: [tuple(a) for a in s] for n, s in d['straights'].items()}
    tpl['areas'] = {n: {**a, 'along': [tuple(x) for x in a['along']], 'lateral': tuple(a['lateral'])}
                    for n, a in d['area_rules'].items()}
    return tpl, hashlib.sha1(raw).hexdigest()


# =============================================================================
# 3. ROAD: clearance raster, car body checks, snapping, bays
# =============================================================================
def _body_samples(pad):
    """V4 bodyClear sample points in the car frame (x forward from the rear axle)."""
    pts = []
    x = -CAR_REAR - pad
    while x < CAR_FRONT + pad + 0.001:
        pts += [(x, -CAR_HALF_W - pad), (x, CAR_HALF_W + pad)]
        x += 0.035
    for xe in (-CAR_REAR - pad, CAR_FRONT + pad):
        y = -CAR_HALF_W - pad
        while y <= CAR_HALF_W + pad + 0.001:
            pts.append((xe, y))
            y += 0.03
    pts += [(-CAR_REAR - pad, -CAR_HALF_W - pad), (CAR_FRONT + pad, -CAR_HALF_W - pad),
            (CAR_FRONT + pad, CAR_HALF_W + pad), (-CAR_REAR - pad, CAR_HALF_W + pad)]
    return np.asarray(pts)


CORNERS = np.array([(-CAR_REAR, -CAR_HALF_W), (CAR_FRONT, -CAR_HALF_W), (CAR_FRONT, CAR_HALF_W), (-CAR_REAR, CAR_HALF_W)])


class Road:
    """Where the car may be, rebuilt from the map's control points."""

    def __init__(self, tpl):
        self.tpl = tpl
        self.lines = mb.centrelines(tpl, 0.01)
        self.areas = mb.areas(tpl)
        self.field, self.x0, self.y0, self.res = mb.road_field(tpl)
        self.ny, self.nx = self.field.shape
        self.index = mb.LineIndex(self.lines)
        circ = mb.circle_through(*tpl['roundabout'].values())
        self.ring = (circ[0][0], circ[0][1], circ[1]) if circ else None
        self._samples = {}

    # ---- raster lookups
    def clearance(self, pts):
        """Metres to the road edge (+ inside) at points (..., 2), nearest cell."""
        pts = np.asarray(pts, float)
        i = np.rint((pts[..., 0] - self.x0) / self.res).astype(np.int64)
        j = np.rint((pts[..., 1] - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (j >= 0) & (i < self.nx) & (j < self.ny)
        out = np.full(i.shape, -10.0, np.float32)
        out[ok] = self.field[j[ok], i[ok]]
        return out

    def samples(self, pad):
        key = round(pad, 6)
        if key not in self._samples:
            self._samples[key] = _body_samples(pad)
        return self._samples[key]

    def body_ok(self, X, Y, A, pad):
        """Vectorised V4 bodyClear for poses (X, Y, A): bool array, and corner margins."""
        c, s = np.cos(A)[:, None], np.sin(A)[:, None]
        S = self.samples(pad)
        wx = X[:, None] + S[None, :, 0] * c - S[None, :, 1] * s
        wy = Y[:, None] + S[None, :, 0] * s + S[None, :, 1] * c
        ok = (self.clearance(np.stack([wx, wy], -1)) >= 0).all(1)
        cx = X[:, None] + CORNERS[None, :, 0] * c - CORNERS[None, :, 1] * s
        cy = Y[:, None] + CORNERS[None, :, 0] * s + CORNERS[None, :, 1] * c
        return ok, self.clearance(np.stack([cx, cy], -1)).min(1)

    # ---- pose helpers
    def lane_snap(self, x, y, yaw):
        """Nearest lane-centre point, heading along the lane (whichever way is closer to yaw)."""
        idx, _ = self.index.nearest(np.array([[x, y]]))
        i = int(idx[0])
        pts, owner = self.index.pts, self.index.owner
        j = i + 1 if i + 1 < len(pts) and owner[i + 1] == owner[i] else i - 1
        t = pts[j] - pts[i] if j > i else pts[i] - pts[j]
        a = math.atan2(t[1], t[0])
        if abs(wrap(a - yaw)) > PI / 2:
            a = wrap(a + PI)
        return float(pts[i][0]), float(pts[i][1]), a

    def bay_pose(self, name, yaw=None):
        """Rear-axle pose centring the body in a bay; keeps the car's facing if yaw given."""
        a = self.areas[name]
        cx, cy = a['poly'].mean(0)
        h = a['heading']
        if yaw is not None and abs(wrap(yaw - h)) > PI / 2:
            h = wrap(h + PI)
        shift = (CAR_FRONT - CAR_REAR) / 2
        return (float(cx - shift * math.cos(h)), float(cy - shift * math.sin(h)), float(h))

    def bay_at(self, x, y):
        import cv2
        for name, a in self.areas.items():
            if a['kind'] == 'bay' and cv2.pointPolygonTest(a['poly'].astype(np.float32), (float(x), float(y)), False) >= 0:
                return name
        return None

    def bay_of_pose(self, pose):
        """Bay the car body centre is in, or None."""
        c = body_corners(pose).mean(0)
        return self.bay_at(c[0], c[1])

    def handoff_candidates(self, bay, pose, arriving=True):
        """Spots on the bay's road where the road route hands over to the manoeuvre."""
        a = self.areas[bay]
        line = self.tpl['areas'][bay]['line']
        A, u, _, L = mb.line_frame(self.tpl, line)
        s0, s1 = a['along']
        hu = math.atan2(u[1], u[0])
        h = pose[2]
        n = a['frame'][2]
        lc = sum(a['lateral']) / 2
        if abs(math.cos(h - hu)) > 0.7:                      # parks along the road (parallel)
            after_first = True
        else:                                                # parks across the road (perpendicular)
            nose_in = (math.cos(h) * n[0] + math.sin(h) * n[1]) * np.sign(lc) > 0
            after_first = not nose_in
        after = [s1 + d for d in PLAN['handoff_after_m']]
        before = [s0 - d for d in PLAN['handoff_before_m']]
        order = after + before if after_first else before + after
        if not arriving:                                     # leaving: drive out forwards, past the bay first
            order = after + before
        out = []
        for s in order:
            for hh in (hu, wrap(hu + PI)):
                p = (float(A[0] + u[0] * s), float(A[1] + u[1] * s), hh)
                if fit_check(self, p, PLAN['margin_m'])[0]:
                    out.append(p)
        return out


def body_corners(pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return np.array([(x + u * c - v * s, y + u * s + v * c) for u, v in CORNERS])


def flip(pose):
    """Same car body facing the other way: 180 deg about the body centre."""
    x, y, a = pose
    d = CAR_FRONT - CAR_REAR
    return (x + d * math.cos(a), y + d * math.sin(a), wrap(a + PI))


def fit_check(road, pose, pad=FIT_PAD_M):
    """(whole body on the road?, smallest clearance of the body outline in m)."""
    corners = body_corners(pose)
    pts = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        n = max(1, math.ceil(np.hypot(*(b - a)) / 0.02))
        pts.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    m = float(road.clearance(np.vstack(pts)).min())
    return m >= pad, m


# =============================================================================
# 4. LANE GRAPH: which way the car may drive, and how far it is to the goal
# =============================================================================
class LaneGraph:
    """Nodes = centreline samples x 2 directions. Lanes can be driven either way
    (no U-turn), the roundabout only clockwise; section ends join nearby lanes."""

    def __init__(self, road):
        lines = mb.centrelines(road.tpl, PLAN['graph_step_m'])
        xs, ys, th, sec, first, last = [], [], [], [], [], []
        self.names = []
        closed = set()
        for si, (name, p) in enumerate(lines.items()):
            if name == 'roundabout':
                p = p[:-1]                                   # closed loop: drop the repeated point
                closed.add(si)
            self.names.append(name)
            t = np.gradient(p, axis=0) if len(p) > 1 else np.array([[1.0, 0.0]])
            k0 = len(xs)
            xs += list(p[:, 0])
            ys += list(p[:, 1])
            th += list(np.arctan2(t[:, 1], t[:, 0]))
            sec += [si] * len(p)
            first.append(k0)
            last.append(k0 + len(p) - 1)
        self.x, self.y, self.th = np.array(xs), np.array(ys), np.array(th)
        self.sec = np.array(sec)
        n = len(xs)
        ring = self.names.index('roundabout') if 'roundabout' in self.names else -1
        # node 2k = forward along the samples, 2k+1 = backward; ring samples run
        # counter-clockwise, so only the backward (clockwise) ring nodes are legal
        self.allowed = np.ones(2 * n, bool)
        if ring >= 0:
            self.allowed[2 * np.where(self.sec == ring)[0]] = False
        self.heading = np.empty(2 * n)
        self.heading[0::2] = self.th
        self.heading[1::2] = self.th + PI
        rev = [[] for _ in range(2 * n)]                     # reverse adjacency: to -> [(from, cost)]

        def link(a, b, cost):
            if self.allowed[a] and self.allowed[b]:
                rev[b].append((a, cost))

        for si in range(len(self.names)):
            k0, k1 = first[si], last[si]
            ks = list(range(k0, k1 + 1))
            pairs = list(zip(ks[:-1], ks[1:])) + ([(k1, k0)] if si in closed else [])
            for a, b in pairs:
                d = math.hypot(xs[b] - xs[a], ys[b] - ys[a])
                link(2 * a, 2 * b, d)
                link(2 * b + 1, 2 * a + 1, d)
        # junctions: a section end joins any other lane within junction_radius
        R, maxturn = PLAN['junction_radius_m'], math.radians(PLAN['junction_max_turn_deg'])
        ends = {}
        for si in range(len(self.names)):
            if si in closed:
                continue
            ends[first[si]] = (2 * first[si] + 1, 2 * first[si])      # (leaving node, entering node)
            ends[last[si]] = (2 * last[si], 2 * last[si] + 1)
        entering_nodes = {v[1] for v in ends.values()}
        leaving_nodes = {v[0] for v in ends.values()}
        for k, (leave, enter) in ends.items():
            d = np.hypot(self.x - xs[k], self.y - ys[k])
            for m in np.where((d < R) & (self.sec != sec[k]))[0]:
                for node in (2 * m, 2 * m + 1):
                    turn_out = abs(wrap(self.heading[node] - self.heading[leave]))
                    if turn_out <= maxturn and node not in leaving_nodes:
                        link(leave, node, d[m] + 0.01)
                    turn_in = abs(wrap(self.heading[enter] - self.heading[node]))
                    if turn_in <= maxturn and node not in entering_nodes:
                        link(node, enter, d[m] + 0.01)
        self.rev = rev
        # nearest graph sample for every raster cell near the road (heuristic lookup)
        step = 2
        gi = np.arange(0, road.nx, step)
        gj = np.arange(0, road.ny, step)
        near = np.full((len(gj), len(gi)), -1, np.int32)
        J, I = np.meshgrid(gj, gi, indexing='ij')
        on = road.field[J, I] > -0.12
        cells = np.column_stack([road.x0 + I[on] * road.res, road.y0 + J[on] * road.res])
        pts = np.column_stack([self.x, self.y])
        idx = np.empty(len(cells), np.int32)
        for c0 in range(0, len(cells), 2000):
            d2 = ((cells[c0:c0 + 2000, None, :] - pts[None, :, :]) ** 2).sum(-1)
            idx[c0:c0 + 2000] = d2.argmin(1)
        near[on] = idx
        self.near, self.near_step, self.road = near, step, road

    def node_for(self, x, y, yaw):
        """Graph node nearest (x, y) whose direction is closest to yaw."""
        k = self.nearest_sample(x, y)
        if k < 0:
            return -1
        a, b = 2 * k, 2 * k + 1
        cand = [n for n in (a, b) if self.allowed[n]]
        if not cand:
            return -1
        return min(cand, key=lambda n: abs(wrap(self.heading[n] - yaw)))

    def nearest_sample(self, x, y):
        r = self.road
        i = int(round((x - r.x0) / r.res / self.near_step))
        j = int(round((y - r.y0) / r.res / self.near_step))
        if 0 <= j < self.near.shape[0] and 0 <= i < self.near.shape[1]:
            return int(self.near[j, i])
        return -1

    def cost_to_go(self, goal_node):
        """Dijkstra backwards from the goal node: metres along legal lanes."""
        ctg = np.full(len(self.rev), np.inf)
        ctg[goal_node] = 0.0
        heap = [(0.0, goal_node)]
        while heap:
            c, n = heapq.heappop(heap)
            if c > ctg[n]:
                continue
            for m, w in self.rev[n]:
                if c + w < ctg[m]:
                    ctg[m] = c + w
                    heapq.heappush(heap, (c + w, m))
        return ctg

    def heuristic(self, goal):
        """h(x, y, yaw) for the road search towards goal pose."""
        g = self.node_for(*goal)
        if g < 0:
            return None
        ctg = self.cost_to_go(g)
        gx, gy, ga = goal
        pen = PLAN['wrong_way_penalty_m']

        def h(x, y, a):
            k = self.nearest_sample(x, y)
            if k < 0:
                return pen + math.hypot(gx - x, gy - y)
            fwd, bwd = 2 * k, 2 * k + 1
            n = fwd if math.cos(a - self.th[k]) >= 0 else bwd
            c = ctg[n]
            if not np.isfinite(c):
                c = pen
            d = math.hypot(gx - x, gy - y)
            if d < 0.3:                                  # close to the goal: also line up the heading
                return d + 0.12 * abs(wrap(ga - a))
            return c + math.hypot(self.x[k] - x, self.y[k] - y)
        return h


# =============================================================================
# 5. CAR-LIKE SEARCH (V4 core.js hybridPlan)
# =============================================================================
class Cancelled(Exception):
    pass


def hybrid_search(road, start, goal, heuristic, reverse=False, bounds=None, max_nodes=60000, cancel=None,
                  connector=None):
    """Search car-like paths from start to goal pose. Returns (path, expanded, reason);
    path = [(x, y, a, dir)], dir -1 = reversing."""
    P = PLAN
    step = P['reverse_step_m'] if reverse else P['step_m']
    xy = P['reverse_xy_res_m'] if reverse else P['xy_res_m']
    pad = P['reverse_margin_m'] if reverse else P['margin_m']
    tol_xy = P['park_goal_xy_m'] if reverse else P['goal_xy_m']
    tol_a = P['park_goal_heading_rad'] if reverse else P['goal_heading_rad']
    bins = P['heading_bins']
    ks = np.array(P['curvature_fractions']) / MIN_R
    dirs = np.array([1, -1] if reverse else [1])
    D = np.repeat(dirs, len(ks)).astype(float)
    K = np.tile(ks, len(dirs))
    w = P['heuristic_weight']
    gx, gy, ga = goal
    ring = road.ring

    X, Y, A, G, DIRS, KS, PARENT = [start[0]], [start[1]], [start[2]], [0.0], [1], [0.0], [-1]
    key = lambda x, y, a, d: (round(x / xy), round(y / xy), round((wrap(a) + PI) / TAU * bins), d)  # noqa: E731
    seen = {key(*start, 1): 0.0}
    heap = [(heuristic(*start) * w, 0)]
    expanded = 0
    best = -1
    while heap and expanded < max_nodes:
        _, i = heapq.heappop(heap)
        expanded += 1
        if cancel and expanded % 400 == 0 and cancel():
            raise Cancelled()
        x, y, a = X[i], Y[i], A[i]
        if math.hypot(x - gx, y - gy) < tol_xy and abs(wrap(a - ga)) < tol_a:
            best = i
            break
        if connector and expanded % 12 == 0 and math.hypot(x - gx, y - gy) < 0.75:
            tail = connector((x, y, a))
            if tail:
                best, finish = i, tail
                break
        # all successors at once (and their half-step midpoints)
        d = D * step
        a2 = a + d * K
        with np.errstate(divide='ignore', invalid='ignore'):
            straight = np.abs(K) < 1e-9
            nx_ = np.where(straight, x + d * math.cos(a), x + (np.sin(a2) - math.sin(a)) / K)
            ny_ = np.where(straight, y + d * math.sin(a), y + (-np.cos(a2) + math.cos(a)) / K)
            am = a + d / 2 * K
            mx = np.where(straight, x + d / 2 * math.cos(a), x + (np.sin(am) - math.sin(a)) / K)
            my = np.where(straight, y + d / 2 * math.sin(a), y + (-np.cos(am) + math.cos(a)) / K)
        if bounds is not None:
            inb = (nx_ >= bounds[0]) & (nx_ <= bounds[1]) & (ny_ >= bounds[2]) & (ny_ <= bounds[3])
        else:
            inb = np.ones(len(K), bool)
        ok_n, margin = road.body_ok(nx_, ny_, a2, pad)
        ok_m, _ = road.body_ok(mx, my, am, pad)
        ok = inb & ok_n & ok_m
        for c in np.where(ok)[0]:
            n_x, n_y, n_a, dr, kk = float(nx_[c]), float(ny_[c]), float(wrap(a2[c])), int(D[c]), float(K[c])
            if ring is not None:                             # V4: roundabout only clockwise
                rx, ry = n_x - ring[0], n_y - ring[1]
                rr = math.hypot(rx, ry)
                if abs(rr - ring[2]) < P['ring_band_m'] and (math.cos(n_a) * ry - math.sin(n_a) * rx) * dr < -0.05:
                    continue
            cost = (G[i] + step * (P['reverse_cost'] if dr < 0 else 1.0) + abs(kk) * P['steer_cost']
                    + (P['gear_change_cost'] if DIRS[i] != dr else 0.0) + abs(KS[i] - kk) * P['steer_change_cost']
                    + step * P['edge_cost'] / max(0.006, float(margin[c])))
            kid = key(n_x, n_y, n_a, dr)
            if kid in seen and seen[kid] <= cost:
                continue
            seen[kid] = cost
            X.append(n_x)
            Y.append(n_y)
            A.append(n_a)
            G.append(cost)
            DIRS.append(dr)
            KS.append(kk)
            PARENT.append(i)
            heapq.heappush(heap, (cost + heuristic(n_x, n_y, n_a) * w, len(X) - 1))
    if best < 0:
        why = 'search limit reached' if expanded >= max_nodes else 'no way through'
        return [], expanded, f'no feasible path ({why}, {expanded} nodes)'
    finish = locals().get('finish')
    chain = []
    while best >= 0:
        chain.append(best)
        best = PARENT[best]
    chain.reverse()
    path = [(X[chain[0]], Y[chain[0]], A[chain[0]], DIRS[chain[1]] if len(chain) > 1 else 1)]
    for p, q in zip(chain[:-1], chain[1:]):                  # 4 points per step, like V4
        for j in range(1, 5):
            d = DIRS[q] * step * j / 4
            kk = KS[q]
            if abs(kk) < 1e-9:
                path.append((X[p] + d * math.cos(A[p]), Y[p] + d * math.sin(A[p]), A[p], DIRS[q]))
            else:
                a2 = A[p] + d * kk
                path.append((X[p] + (math.sin(a2) - math.sin(A[p])) / kk,
                             Y[p] + (-math.cos(a2) + math.cos(A[p])) / kk, wrap(a2), DIRS[q]))
    if finish:
        path += finish[1:]
    return path, expanded, 'ok'


# =============================================================================
# 5b. REEDS-SHEPP (V4 reeds-shepp.js, from PythonRobotics, MIT: A. Sakai, V. Patel)
#     Exact forward/reverse arc+straight connections between two poses.
# =============================================================================
def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _acos(v):
    return math.acos(max(-1.0, min(1.0, v)))


def _asin(v):
    return math.asin(max(-1.0, min(1.0, v)))


_H = PI / 2
_m = wrap


def _f1(x, y, p):
    u, t = _polar(x - math.sin(p), y - 1 + math.cos(p))
    v = _m(p - t)
    if 0 <= t <= PI and 0 <= v <= PI:
        return [t, u, v], 'LSL'


def _f2(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r * r >= 4:
        u = math.sqrt(r * r - 4)
        t = _m(b + math.atan2(2, u))
        v = _m(t - p)
        if t >= 0 and v >= 0:
            return [t, u, v], 'LSR'


def _f3(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r <= 4:
        a = _acos(r / 4)
        t = _m(a + b + _H)
        u = _m(PI - 2 * a)
        v = _m(p - t - u)
        return [t, -u, v], 'LRL'


def _f4(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r <= 4:
        a = _acos(r / 4)
        t = _m(a + b + _H)
        u = _m(PI - 2 * a)
        v = _m(-p + t + u)
        return [t, -u, -v], 'LRL'


def _f5(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if 1e-8 < r <= 4:
        u = _acos(1 - r * r / 8)
        a = _asin(2 * math.sin(u) / r)
        t = _m(-a + b + _H)
        v = _m(t - u - p)
        return [t, u, -v], 'LRL'


def _f6(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r <= 2:
        a = _acos((r + 2) / 4)
        t = _m(b + a + _H)
        u = _m(a)
        v = _m(p - t + 2 * u)
        if t >= 0 and u >= 0 and v >= 0:
            return [t, u, -u, -v], 'LRLR'


def _f7(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    u2 = (20 - r * r) / 16
    if 0 <= u2 <= 1 and r > 1e-8:
        u = math.acos(u2)
        a = _asin(2 * math.sin(u) / r)
        t = _m(b + a + _H)
        v = _m(t - p)
        if t >= 0 and v >= 0:
            return [t, -u, -u, v], 'LRLR'


def _f8(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r >= 2:
        q = math.sqrt(r * r - 4)
        u = q - 2
        a = math.atan2(2, q)
        t = _m(b + a + _H)
        v = _m(t - p + _H)
        if t >= 0 and v >= 0:
            return [t, -_H, -u, -v], 'LRSL'


def _f9(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 2:
        t = _m(b + _H)
        u = r - 2
        v = _m(p - t - _H)
        if t >= 0 and v >= 0:
            return [t, -_H, -u, -v], 'LRSR'


def _f10(x, y, p):
    r, b = _polar(x - math.sin(p), y - 1 + math.cos(p))
    if r >= 2:
        q = math.sqrt(r * r - 4)
        u = q - 2
        a = math.atan2(q, 2)
        t = _m(b - a + _H)
        v = _m(t - p - _H)
        if t >= 0 and v >= 0:
            return [t, u, _H, -v], 'LSRL'


def _f11(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 2:
        t = _m(b)
        u = r - 2
        v = _m(p - t - _H)
        if t >= 0 and v >= 0:
            return [t, u, _H, -v], 'LSLR'


def _f12(x, y, p):
    r, b = _polar(x + math.sin(p), y - 1 - math.cos(p))
    if r >= 4:
        q = math.sqrt(r * r - 4)
        u = q - 4
        a = math.atan2(2, q)
        t = _m(b + a + _H)
        v = _m(t - p)
        if t >= 0 and v >= 0:
            return [t, -_H, -u, -_H, v], 'LRSLR'


RS_FAMILIES = (_f1, _f2, _f3, _f4, _f5, _f6, _f7, _f8, _f9, _f10, _f11, _f12)


def rs_candidates(start, goal, r=MIN_R, step=0.006):
    """All Reeds-Shepp connections start -> goal, cheapest first (V4 candidates)."""
    sx, sy, sa = start
    dx, dy = goal[0] - sx, goal[1] - sy
    c, s = math.cos(sa), math.sin(sa)
    x, y = (dx * c + dy * s) / r, (-dx * s + dy * c) / r
    phi = wrap(goal[2] - sa)
    out = []
    for f in RS_FAMILIES:
        for variant in range(4):
            flip, reflect = variant in (1, 3), variant >= 2
            q = f(-x if flip else x, -y if reflect else y, -phi if flip != reflect else phi)
            if not q:
                continue
            lengths = [v * r * (-1 if flip else 1) for v in q[0]]
            types = [({'L': 'R', 'R': 'L'}.get(t, t) if reflect else t) for t in q[1]]
            px_, py_, pa_ = sx, sy, sa
            xs, ys, as_, ds = [], [], [], []
            for d, t in zip(lengths, types):
                if abs(d) < 1e-8:
                    continue
                k = 1 / r if t == 'L' else (-1 / r if t == 'R' else 0.0)
                n = math.ceil(abs(d) / step)
                sarr = d * np.arange(0, n + 1) / n
                if k == 0:
                    X = px_ + sarr * math.cos(pa_)
                    Y = py_ + sarr * math.sin(pa_)
                    A = np.full(n + 1, pa_)
                else:
                    A = pa_ + sarr * k
                    X = px_ + (np.sin(A) - math.sin(pa_)) / k
                    Y = py_ + (-np.cos(A) + math.cos(pa_)) / k
                xs.append(X)
                ys.append(Y)
                as_.append(A)
                ds.append(np.full(n + 1, 1 if d > 0 else -1))
                px_, py_, pa_ = float(X[-1]), float(Y[-1]), float(A[-1])
            if not xs or math.hypot(px_ - goal[0], py_ - goal[1]) > 1e-5 or abs(wrap(pa_ - goal[2])) > 1e-5:
                continue
            out.append((sum(abs(v) for v in lengths), np.concatenate(xs), np.concatenate(ys),
                        np.concatenate(as_), np.concatenate(ds), ''.join(types)))
    out.sort(key=lambda o: o[0])
    return out


def rs_plan(road, start, goal, pad=0.003):
    """First collision-free Reeds-Shepp connection (V4 CarbotRS.plan) or None."""
    for cost, X, Y, A, D, _ in rs_candidates(start, goal):
        ok, _ = road.body_ok(X, Y, A, pad)
        if ok.all():
            return [(float(x), float(y), float(wrap(a)), int(d)) for x, y, a, d in zip(X, Y, A, D)], cost
    return None, None


def straight(a, b, heading, direction, step=0.006):
    n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, heading, direction) for i in range(n + 1)]


def path_clear(road, path, pad=0.003):
    P = np.array([p[:3] for p in path])
    return bool(road.body_ok(P[:, 0], P[:, 1], P[:, 2], pad)[0].all())


def bicycle(p, d, k):
    x, y, a = p
    if abs(k) < 1e-9:
        return (x + d * math.cos(a), y + d * math.sin(a), a)
    a2 = a + d * k
    return (x + (math.sin(a2) - math.sin(a)) / k, y + (-math.cos(a2) + math.cos(a)) / k, wrap(a2))


def parking_plan(road, start, goal, bay, cancel=None):
    """V4 core.js parkingPlan: docking straight + Reeds-Shepp, then direct RS, then
    handoff extension + RS, then bounded reverse search with RS connector."""
    # 1) docking straight into the goal (V4: 15 / 12 / 8 cm, reversing in); forward-in tried too
    for tail in (0.15, 0.12, 0.08):
        for sgn in (1, -1):
            pre = bicycle(goal, sgn * tail, 0.0)
            end = straight(pre, goal, goal[2], -sgn)
            if not path_clear(road, end):
                continue
            rs, _ = rs_plan(road, start, pre)
            if rs:
                return rs + end[1:], f'{int(tail * 100)} cm docking straight'
    # 2) direct connection
    rs, _ = rs_plan(road, start, goal)
    if rs:
        return rs, 'direct'
    # 3) move the handoff a little first
    best = None
    for d in (-0.10, -0.05, 0.05, 0.10, 0.15, 0.20):
        mid = bicycle(start, d, 0.0)
        approach = straight(start, mid, start[2], 1 if d > 0 else -1)
        if not path_clear(road, approach):
            continue
        rs, cost = rs_plan(road, mid, goal)
        if rs and (best is None or cost + abs(d) < best[0]):
            best = (cost + abs(d), approach + rs[1:])
    if best:
        return best[1], 'handoff extension'
    # 4) bounded reverse search, trying a Reeds-Shepp finish on the way
    poly = road.areas[bay]['poly']
    b = PLAN['manoeuvre_box_m']
    xs = np.r_[poly[:, 0], start[0], goal[0]]
    ys = np.r_[poly[:, 1], start[1], goal[1]]
    path, n, why = hybrid_search(road, start, goal, euclid_heuristic(goal), reverse=True,
                                 bounds=(xs.min() - b, xs.max() + b, ys.min() - b, ys.max() + b),
                                 max_nodes=PLAN['manoeuvre_max_nodes'], cancel=cancel,
                                 connector=lambda p: rs_plan(road, p, goal)[0])
    return (path, 'search + Reeds-Shepp') if path else (None, why)


def euclid_heuristic(goal):
    gx, gy, ga = goal
    return lambda x, y, a: math.hypot(gx - x, gy - y) + 0.12 * abs(wrap(ga - a))


# =============================================================================
# 6. ONE LEG = road part (+ bay manoeuvres)
# =============================================================================
def path_length(path):
    return float(sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(path[:-1], path[1:])))


def plan_road(road, graph, start, goal, cancel=None):
    h = graph.heuristic(goal)
    if h is None:
        return None, 'end pose is not near a lane'
    path, n, why = hybrid_search(road, start, goal, h, max_nodes=PLAN['road_max_nodes'], cancel=cancel)
    return ({'kind': 'road', 'path': path, 'nodes': n} if path else None), why


def plan_manoeuvre(road, start, goal, bay, cancel=None):
    path, how = parking_plan(road, start, goal, bay, cancel)
    return ({'kind': 'manoeuvre', 'path': path, 'how': how, 'bay': bay} if path else None), how


def plan_leg(road, graph, p_start, p_end, cancel=None):
    """Plan one leg. Returns dict: ok, pieces, length_m, sections, roundabout_exits, reason, seconds."""
    t0 = time.time()
    pieces = []
    cur = p_start
    start_bay = road.bay_of_pose(p_start)
    if start_bay:                                             # leave the bay first
        for cand in road.handoff_candidates(start_bay, p_start, arriving=False):
            piece, why = plan_manoeuvre(road, p_start, cand, start_bay, cancel)
            if piece:
                pieces.append(piece)
                cur = cand
                break
        else:
            return {'ok': False, 'reason': f'cannot drive out of {start_bay}', 'seconds': time.time() - t0}
    end_bay = road.bay_of_pose(p_end)
    reason = ''
    if end_bay:
        for cand in road.handoff_candidates(end_bay, p_end, arriving=True):
            piece_m, why = plan_manoeuvre(road, cand, p_end, end_bay, cancel)   # cheap check first
            if not piece_m:
                reason = f'cannot manoeuvre into {end_bay}: {why}'
                continue
            piece_r, why = plan_road(road, graph, cur, cand, cancel)
            if piece_r:
                pieces += [piece_r, piece_m]
                break
            reason = why
        else:
            return {'ok': False, 'reason': reason or f'no handoff spot next to {end_bay}', 'seconds': time.time() - t0}
    else:
        piece, why = plan_road(road, graph, cur, p_end, cancel)
        if not piece:
            return {'ok': False, 'reason': why, 'seconds': time.time() - t0}
        pieces.append(piece)
    allpts = [p for pc in pieces for p in pc['path']]
    sections, exits = route_sections(road, graph, allpts)
    return {'ok': True, 'pieces': pieces, 'length_m': path_length(allpts), 'sections': sections,
            'roundabout_exits': exits, 'reason': 'ok', 'seconds': time.time() - t0}


def route_sections(road, graph, pts):
    """Sections the route passes, in order, and the roundabout exit(s) it takes."""
    names = []
    for x, y, _, _ in pts[::4]:
        k = graph.nearest_sample(x, y)
        if k >= 0:
            n = graph.names[graph.sec[k]]
            if not names or names[-1] != n:
                names.append(n)
    # which straight leaves the ring at which exit point
    arm_exit = {}
    for ex, p in road.tpl['roundabout'].items():
        best = min(road.tpl['straights'], key=lambda s: min(math.dist(p, e) for e in mb.straight_ends(road.tpl, s)))
        arm_exit[best] = ex
    exits = [arm_exit.get(nxt, nxt) for cur, nxt in zip(names[:-1], names[1:]) if cur == 'roundabout']
    return names, exits


# =============================================================================
# 7. POSES AND LEGS
# =============================================================================
def pose_label(i, n_legs):
    if i == 0:
        return 'P0  start of leg1'
    if i == n_legs:
        return f'P{i}  end of leg{i} (finish)'
    return f'P{i}  end of leg{i} = start of leg{i + 1}'


def default_pose(road, i):
    _, p = DEFAULT_POSES[i] if i < len(DEFAULT_POSES) else ('extra', None)
    if p is None:
        bay = DEFAULT_BAYS.get(i)
        p = road.bay_pose(bay) if bay else DEFAULT_POSES[0][1]
    return tuple(float(v) for v in p)


def default_poses(road, n_legs=LEGS):
    return [default_pose(road, i) for i in range(n_legs + 1)]


def mission_dict(map_path, map_sha1, poses, road, routes):
    n_legs = len(poses) - 1
    r4 = lambda v: round(float(v), 4)  # noqa: E731
    legs = []
    for i in range(n_legs):
        r = routes[i] or {}
        leg = {'id': f'leg{i + 1}', 'start': f'P{i}', 'end': f'P{i + 1}', 'ok': bool(r.get('ok'))}
        if r.get('ok'):
            leg.update({
                'length_m': r4(r['length_m']),
                'roundabout_exits': r['roundabout_exits'],
                'sections': r['sections'],
                'checkpoints': checkpoints(r),
                'pieces': [{'kind': pc['kind'], **({'bay': pc['bay']} if 'bay' in pc else {}),
                            'points': [[r4(x), r4(y), r4(a), int(d)] for x, y, a, d in pc['path'][::2]]}
                           for pc in r['pieces']],
            })
        legs.append(leg)
    return {
        'version': 2,
        'map': {'file': os.path.basename(map_path), 'sha1': map_sha1},
        'frame': 'track',
        'pose_convention': 'rear axle centre + heading',
        'point_format': '[x, y, yaw_rad, dir]  dir -1 = reversing',
        'poses': [{'id': f'P{i}', 'role': pose_label(i, n_legs).split('  ', 1)[1],
                   'x': r4(p[0]), 'y': r4(p[1]), 'yaw_deg': round(math.degrees(p[2]), 2),
                   'fits': bool(fit_check(road, p)[0]), 'margin_m': r4(fit_check(road, p)[1])}
                  for i, p in enumerate(poses)],
        'legs': legs,
    }


def checkpoints(route):
    """A pose where the route enters each new section (plus the handoffs)."""
    out = []
    pts = [p for pc in route['pieces'] for p in pc['path']]
    return [{'section': s} for s in route['sections']] if not pts else _cp(route, pts, out)


def _cp(route, pts, out):
    total = len(pts)
    for pc in route['pieces'][:-1]:
        x, y, a, _ = pc['path'][-1]
        out.append({'section': 'handoff', 'x': round(x, 4), 'y': round(y, 4), 'yaw_deg': round(math.degrees(a), 1)})
    step = max(1, total // max(1, len(route['sections'])))
    for s_i, s in enumerate(route['sections']):
        x, y, a, _ = pts[min(total - 1, s_i * step)]
        out.append({'section': s, 'x': round(x, 4), 'y': round(y, 4), 'yaw_deg': round(math.degrees(a), 1)})
    return out


def load_poses(path, map_sha1):
    import yaml
    d = yaml.safe_load(open(path))
    if d['map']['sha1'] != map_sha1:
        print(f'WARNING: {path} was planned on a different version of the map. Poses loaded; routes will be recalculated.')
    return [(p['x'], p['y'], math.radians(p['yaw_deg'])) for p in d['poses']]


# =============================================================================
# 8. PICTURE
# =============================================================================
def make_view(tpl, max_w=1250, max_h=560, margin=50):
    pts = np.vstack(list(mb.centrelines(tpl, 0.05).values()))
    lo, hi = pts.min(0) - 0.4, pts.max(0) + 0.4
    s = min(140, (max_w - 2 * margin) / (hi[0] - lo[0]), (max_h - 2 * margin - 60) / (hi[1] - lo[1]))
    return {'lo': lo, 'hi': hi, 's': s, 'm': margin,
            'W': max(760, int((hi[0] - lo[0]) * s) + 2 * margin), 'H': int((hi[1] - lo[1]) * s) + 2 * margin + 60}


def draw_map(road, view):
    import cv2
    s, m, W, H, lo, hi = view['s'], view['m'], view['W'], view['H'], view['lo'], view['hi']
    px = lambda p: mb.to_px(view, p)  # noqa: E731
    img = np.full((H, W, 3), (34, 30, 28), np.uint8)
    for gx in np.arange(math.floor(lo[0] * 2) / 2, hi[0], 0.5):
        x = px([[gx, 0]])[0, 0]
        cv2.line(img, (x, 60), (x, H - m), (55, 55, 55), 1)
        cv2.putText(img, f'{gx:g}', (x - 10, H - m + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
    for gy in np.arange(math.floor(lo[1] * 2) / 2, hi[1], 0.5):
        y = px([[0, gy]])[0, 1]
        cv2.line(img, (m, y), (W - m, y), (55, 55, 55), 1)
        cv2.putText(img, f'{gy:g}', (6, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
    # road = clearance raster >= 0 (exactly what the planner uses)
    road_mask = (road.field >= 0).astype(np.uint8)[::-1]
    h_px = int(road.ny * road.res * s)
    w_px = int(road.nx * road.res * s)
    small = cv2.resize(road_mask, (w_px, h_px), interpolation=cv2.INTER_NEAREST)
    ox, oy = px([[road.x0, road.y0 + road.ny * road.res]])[0]
    y0, x0 = max(0, oy), max(0, ox)
    y1, x1 = min(H, oy + h_px), min(W, ox + w_px)
    sub = small[y0 - oy:y1 - oy, x0 - ox:x1 - ox]
    img[y0:y1, x0:x1][sub > 0] = COLOURS['road']
    paint_px = max(1, int(0.03 * s))
    for a in road.areas.values():
        if a['kind'] == 'bay':
            cv2.polylines(img, [px(a['poly'])], True, COLOURS['bay'], 1, cv2.LINE_AA)
        for pa, pb, style in mb.paint_segments(a):
            q = px([pa, pb])
            cv2.line(img, tuple(q[0]), tuple(q[1]), COLOURS['paint' if style == 'solid' else 'dashed'], paint_px)
    for p in road.lines.values():
        cv2.polylines(img, [px(p)], False, (125, 125, 125), 1, cv2.LINE_AA)
    return img


def rotate_handle(pose):
    return (pose[0] + ROTATE_HANDLE_M * math.cos(pose[2]), pose[1] + ROTATE_HANDLE_M * math.sin(pose[2]))


def draw_route(img, view, route, leg, width):
    import cv2
    if not route or not route.get('ok'):
        return
    base = COLOURS['legs'][leg % len(COLOURS['legs'])]
    for pc in route['pieces']:
        pts = pc['path']
        for (x0, y0, _, d0), (x1, y1, _, _) in zip(pts[:-1], pts[1:]):
            col = COLOURS['reverse'] if d0 < 0 else base
            a, b = mb.to_px(view, [(x0, y0), (x1, y1)])
            cv2.line(img, tuple(a), tuple(b), col, width, cv2.LINE_AA)
        if pc['kind'] == 'road' and len(route['pieces']) > 1:
            q = mb.to_px(view, [pts[-1][:2]])[0]
            cv2.circle(img, tuple(q), 4, COLOURS['text'], 1)


def leg_status(i, route, busy, stale):
    name = f'leg{i + 1}'
    if busy:
        return f'{name}: calculating...', COLOURS['selected']
    if route is None:
        return f'{name}: not calculated yet', COLOURS['dim']
    if not route.get('ok'):
        return f"{name}: NO ROUTE - {route['reason']}", COLOURS['bad']
    ex = ', '.join(route['roundabout_exits']) or 'no roundabout'
    rev = ' + parking manoeuvre' if any(pc['kind'] == 'manoeuvre' for pc in route['pieces']) else ''
    txt = f"{name}: {route['length_m']:.2f} m  roundabout exit: {ex}{rev}  ({route['seconds']:.1f} s)"
    return (txt + ('  [pose moved - will recalculate]' if stale else '')), \
        COLOURS['legs'][i % len(COLOURS['legs'])] if not stale else COLOURS['dim']


def draw_scene(base, road, view, poses, routes, step, busy, stale, snap, message, n_legs):
    import cv2
    img = base.copy()
    cur_leg = step - 1 if 1 <= step <= n_legs else None
    for i, r in enumerate(routes):
        if r and i != cur_leg and (i < step or step > n_legs):
            draw_route(img, view, r, i, 2 if step > n_legs else 1)
    if cur_leg is not None:
        draw_route(img, view, routes[cur_leg], cur_leg, 3)
    for i, p in enumerate(poses):
        if i > step and step <= n_legs:
            continue                                         # not placed yet
        ok, _ = fit_check(road, p)
        col = COLOURS['ok'] if ok else COLOURS['bad']
        active = (i == step)
        cv2.polylines(img, [mb.to_px(view, body_corners(p))], True, COLOURS['selected'] if active else col,
                      3 if active else 2)
        a, b = mb.to_px(view, [p[:2], rotate_handle(p)])
        cv2.arrowedLine(img, tuple(a), tuple(b), col, 2, tipLength=0.3)
        if active:
            cv2.circle(img, tuple(b), 7, COLOURS['rotate'], -1)
        cv2.putText(img, f'P{i}', (a[0] + 8, a[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    # header
    if step <= n_legs:
        title = f'Step {step + 1}/{n_legs + 2}: place P{step} ({pose_label(step, n_legs).split("  ", 1)[1]})'
    else:
        title = f'Step {n_legs + 2}/{n_legs + 2}: review, press s to save'
    cv2.putText(img, title, (view['m'], 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOURS['selected'], 2)
    if 1 <= step <= n_legs:                                  # result of the leg being placed, where you look
        text, col = leg_status(step - 1, routes[step - 1], busy[step - 1], stale[step - 1])
        cv2.putText(img, text, (view['m'], 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    elif step > n_legs:
        ok = [bool(r and r.get('ok')) and not stale[i] for i, r in enumerate(routes)]
        text = '   '.join(f"leg{i + 1} {'OK' if k else 'NO ROUTE'}" for i, k in enumerate(ok))
        cv2.putText(img, text, (view['m'], 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    COLOURS['ok'] if all(ok) else COLOURS['bad'], 1)
    cv2.putText(img, f"snap {'ON' if snap else 'off'} | drag car = move | orange dot = rotate | [ ] , . rotate | f flip",
                (view['m'], 42), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOURS['text'], 1)
    cv2.putText(img, "g snap | r reset | Enter / n next step | Backspace / b back | s save | q quit",
                (view['m'], 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOURS['text'], 1)
    if message:
        cv2.putText(img, message, (view['m'], 104), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOURS['bad'], 1)
    # panel below the map
    lines = []
    for i, p in enumerate(poses):
        if i > step and step <= n_legs:
            lines.append((f'  {pose_label(i, n_legs)}   (later step)', COLOURS['dim']))
            continue
        ok, margin = fit_check(road, p)
        lines.append((f"{'>' if i == step else ' '} {pose_label(i, n_legs):<36} x {p[0]:5.2f}  y {p[1]:5.2f}  "
                      f"{math.degrees(p[2]):6.1f} deg   {'fits' if ok else 'OFF ROAD'} ({margin * 100:+.1f} cm)",
                      COLOURS['ok'] if ok else COLOURS['bad']))
    for i in range(n_legs):
        lines.append(leg_status(i, routes[i], busy[i], stale[i]))
    if message:
        lines.append((message, COLOURS['selected']))
    panel = np.full((18 * len(lines) + 16, img.shape[1], 3), (26, 24, 22), np.uint8)
    y = 20
    for text, col in lines:
        cv2.putText(panel, text, (view['m'], y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        y += 18
    return np.vstack([img, panel])


# =============================================================================
# 9. GUIDED WINDOW
# =============================================================================
class Guide:
    WINDOW = 'mission_planner'

    def __init__(self, road, graph, poses, save_fn):
        self.road, self.graph, self.save_fn = road, graph, save_fn
        self.poses = [tuple(p) for p in poses]
        self.n_legs = len(poses) - 1
        self.routes = [None] * self.n_legs
        self.stale = [True] * self.n_legs
        self.busy = [False] * self.n_legs
        self.version = [0] * self.n_legs
        self.due = [None] * self.n_legs            # time a recalculation should start
        self.lock = threading.Lock()
        self.view = make_view(road.tpl)
        self.base = draw_map(road, self.view)
        self.step, self.drag, self.snap = 0, None, True
        self.drag_from = None
        self.message, self.dirty = '', True

    # ---- routes
    def pose_changed(self, i):
        """Pose i moved: the legs touching it are stale; recalc the current one after the delay."""
        for leg in (i - 1, i):
            if 0 <= leg < self.n_legs:
                self.stale[leg] = True
                self.version[leg] += 1
        self.schedule_current(RECALC_DELAY_S)

    def schedule_current(self, delay):
        legs = [self.step - 1] if 1 <= self.step <= self.n_legs else (
            range(self.n_legs) if self.step > self.n_legs else [])
        for leg in legs:
            if self.stale[leg]:
                self.due[leg] = time.time() + delay

    def start_due(self):
        for leg in range(self.n_legs):
            if self.due[leg] is not None and time.time() >= self.due[leg] and not self.busy[leg]:
                self.due[leg] = None
                self.busy[leg] = True
                v = self.version[leg]
                a, b = self.poses[leg], self.poses[leg + 1]
                threading.Thread(target=self._work, args=(leg, v, a, b), daemon=True).start()
                self.dirty = True

    def _work(self, leg, v, a, b):
        try:
            r = plan_leg(self.road, self.graph, a, b, cancel=lambda: self.version[leg] != v)
        except Cancelled:
            r = None
        except Exception as e:                                # never kill the window
            r = {'ok': False, 'reason': f'error: {e}', 'seconds': 0.0}
        with self.lock:
            self.busy[leg] = False
            if r is not None and self.version[leg] == v:
                self.routes[leg] = r
                self.stale[leg] = False
            elif self.stale[leg]:
                self.due[leg] = time.time()                   # pose moved meanwhile: go again
            self.dirty = True

    # ---- editing the current pose
    def place(self, i, x, y, yaw):
        if self.snap:
            bay = self.road.bay_at(x, y)
            if bay:
                self.poses[i] = self.road.bay_pose(bay, yaw)
                return
            x, y, yaw = self.road.lane_snap(x, y, yaw)
        self.poses[i] = (float(x), float(y), float(yaw))

    def on_mouse(self, event, x, y, flags=0, param=None):
        import cv2
        if self.step > self.n_legs or y >= self.view['H']:
            return
        i = self.step
        q = mb.from_px(self.view, x, y)
        p = self.poses[i]
        if event == cv2.EVENT_LBUTTONDOWN:
            tip = mb.to_px(self.view, [rotate_handle(p)])[0]
            if math.hypot(tip[0] - x, tip[1] - y) < 12:
                self.drag = ('rotate',)
            elif cv2.pointPolygonTest(body_corners(p).astype(np.float32), (float(q[0]), float(q[1])), False) >= 0:
                self.drag = ('move', q[0] - p[0], q[1] - p[1])
            self.drag_from = p                                # to tell a real move from a plain click
        elif event == cv2.EVENT_MOUSEMOVE and self.drag:
            if self.drag[0] == 'move':
                self.place(i, q[0] - self.drag[1], q[1] - self.drag[2], p[2])
            else:
                self.poses[i] = (p[0], p[1], math.atan2(q[1] - p[1], q[0] - p[0]))
            if self.poses[i] != self.drag_from:               # really moved: old route no longer valid
                for leg in (i - 1, i):
                    if 0 <= leg < self.n_legs:
                        self.stale[leg] = True
                        self.version[leg] += 1                # stops any calculation still running
                        self.due[leg] = None
            self.dirty = True
        elif event == cv2.EVENT_LBUTTONUP and self.drag:
            self.drag = None
            if self.poses[i] != self.drag_from:               # let go after a real change: calculate now
                self.pose_changed(i)
            self.dirty = True

    def key(self, k):
        if k in (ord('q'), 27):
            return False
        c = chr(k) if 32 <= k < 127 else ''
        i = self.step
        editable = i <= self.n_legs
        if k in (13, 10) or c == 'n':                         # Enter or n: next step
            self.next_step()
        elif k == 8 or c == 'b':                              # Backspace or b: previous step
            if self.step > 0:
                self.step -= 1
                self.message = ''
                self.schedule_current(0)
        elif editable and c in ('[', ']', ',', '.'):
            x, y, a = self.poses[i]
            self.poses[i] = (x, y, wrap(a + math.radians({'[': 5, ']': -5, ',': 1, '.': -1}[c])))
            self.pose_changed(i)
        elif editable and c == 'f':
            self.poses[i] = flip(self.poses[i])
            self.pose_changed(i)
        elif editable and c == 'r':
            self.poses[i] = default_pose(self.road, i)
            self.pose_changed(i)
        elif c == 'g':
            self.snap = not self.snap
        elif c == 's':
            if self.step <= self.n_legs:
                self.message = 'finish all steps first (Enter), then save on the review step'
            else:
                self.message = self.save_fn(self.poses, self.routes, self.stale)
                print(self.message)
        self.dirty = True
        return True

    def next_step(self):
        i = self.step
        if i > self.n_legs:
            self.message = 'review step: press s to save, Backspace to go back'
            return
        if not fit_check(self.road, self.poses[i])[0]:
            self.message = f'P{i} does not fit on the road - move it first'
            return
        if i >= 1:
            leg = i - 1
            if self.busy[leg] or self.due[leg] is not None:
                self.message = f'wait: leg{i} is still being calculated'
                return
            if self.stale[leg] or not (self.routes[leg] and self.routes[leg]['ok']):
                self.message = f'leg{i} has no route yet - move P{i} until one is found'
                return
        self.step += 1
        self.message = f'step {self.step + 1}: ' + ('place P%d' % self.step if self.step <= self.n_legs
                                                     else 'review, press s to save')
        self.schedule_current(0)

    def render(self):
        with self.lock:
            return draw_scene(self.base, self.road, self.view, self.poses, self.routes, self.step,
                              self.busy, self.stale, self.snap, self.message, self.n_legs)

    def run(self):
        import cv2
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.WINDOW, self.on_mouse)
        last = 0.0
        while True:
            self.start_due()
            if self.dirty or (any(self.busy) and time.time() - last > 0.3):
                # clear the flag BEFORE drawing: a route that finishes while we draw sets it
                # again and is shown on the next pass (before, it could be lost until the next click)
                self.dirty, last = False, time.time()
                cv2.imshow(self.WINDOW, self.render())
            k = cv2.waitKeyEx(20)
            if k != -1:
                if k in (0x250000, 0x270000):                 # ignore arrow keys (Windows codes)
                    continue
                if not self.key(k & 0xFF):
                    break
            if cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()
        return self.poses


# =============================================================================
# 10. COMMAND LINE
# =============================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description='Place the start / end pose of each leg and see its route')
    ap.add_argument('map', nargs='?', default='track_map.yaml')
    ap.add_argument('-o', '--out', default='mission.yaml')
    ap.add_argument('--legs', type=int, default=LEGS)
    ap.add_argument('--check', action='store_true', help='no window: plan every leg, print, save PNG + yaml')
    a = ap.parse_args(argv)

    if not os.path.exists(a.map):
        raise SystemExit(f'{a.map} not found. Make it first:  python map_builder.py fit lap.csv   '
                         f'(or  python map_builder.py fit  for the demo lap)')
    print('loading map and building the lane graph ...')
    tpl, sha1 = load_map(a.map)
    road = Road(tpl)
    graph = LaneGraph(road)
    poses = default_poses(road, a.legs)
    if os.path.exists(a.out):
        loaded = load_poses(a.out, sha1)
        if len(loaded) == a.legs + 1:
            poses = loaded
            print(f'continuing from {a.out}')

    def save(ps, routes, stale):
        import yaml
        bad = [f'P{i}' for i, p in enumerate(ps) if not fit_check(road, p)[0]]
        if bad:
            return f"NOT saved: car does not fit at {', '.join(bad)}"
        missing = [f'leg{i + 1}' for i, r in enumerate(routes) if stale[i] or not (r and r['ok'])]
        if missing:
            return f"NOT saved: no up-to-date route for {', '.join(missing)}"
        with open(a.out, 'w') as fh:
            yaml.safe_dump(mission_dict(a.map, sha1, ps, road, routes), fh, sort_keys=False)
        return f'saved {os.path.abspath(a.out)}'

    if a.check:
        import cv2
        routes = []
        for i in range(a.legs):
            r = plan_leg(road, graph, poses[i], poses[i + 1])
            routes.append(r)
            print(leg_status(i, r, False, False)[0])
        view = make_view(tpl)
        img = draw_scene(draw_map(road, view), road, view, poses, routes, a.legs + 1,
                         [False] * a.legs, [False] * a.legs, True, '', a.legs)
        cv2.imwrite('mission_routes.png', img)
        print('saved picture: mission_routes.png')
        print(save(poses, routes, [False] * a.legs))
        return 0

    print(__doc__.split('Window controls')[1].split('Poses:')[0])
    Guide(road, graph, poses, save).run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
