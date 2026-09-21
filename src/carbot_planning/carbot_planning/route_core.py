"""BLOCK 07 core (no ROS): the global route.

Two ways to get the route, depending on the mission.yaml version:

v2 (default): tools/map/mission_planner.py already computed every piece with
    the V4 hybridPlan + a lane graph (it picked the roundabout exits). Here we
    only CHECK it: the map fingerprint must match, every road point must keep
    the whole body on the road, and the roundabout exits must be the ones in
    mission_rules.yaml roundabout_visits. Nothing is replanned silently.
v1 (V4 reference): plan each leg through its checkpoints with a faithful port
    of V4 core.js hybridPlan / buildMission (clockwise roundabout rule).

Every route is resampled to 1 cm (the V4 path density), so all the V4
index-based parameters downstream (corridor every-5th sample, guide 120
points, closest() window -12..+130) keep their meaning.

Point format everywhere: [x, y, yaw, dir] (dir +1 forward, -1 reverse).
"""
import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

PI = math.pi
TAU = 2 * PI


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def js_round(v: float) -> int:
    return int(math.floor(v + 0.5))


def bicycle(x, y, a, d, k):
    """core.js bicycle: advance by arc length d at curvature k."""
    if abs(k) < 1e-9:
        return x + d * math.cos(a), y + d * math.sin(a), a
    a2 = a + d * k
    return (x + (math.sin(a2) - math.sin(a)) / k, y + (-math.cos(a2) + math.cos(a)) / k, wrap(a2))


# --------------------------------------------------------------------------- config
@dataclass
class PlanCfg:
    step: float = 0.04
    reverse_step: float = 0.025
    xy: float = 0.012
    reverse_xy: float = 0.014
    angles: int = 180
    curvature_fractions: Tuple[float, ...] = (-1.0, -0.5, 0.0, 0.5, 1.0)
    max_nodes: int = 120000
    margin: float = 0.008
    reverse_margin: float = 0.006
    goal_xy: float = 0.025
    goal_a: float = 0.035
    reverse_goal_xy: float = 0.01
    reverse_goal_a: float = 0.02
    h_heading: float = 0.12
    h_inflation: float = 1.25
    cost_reverse: float = 1.07
    cost_curvature: float = 0.0003
    cost_direction_change: float = 0.08
    cost_curvature_change: float = 0.001
    cost_clearance: float = 0.003
    clearance_floor: float = 0.006
    ring_inner: float = 0.40
    ring_outer: float = 0.80
    ring_min_dot: float = -0.05

    @classmethod
    def from_params(cls, p) -> 'PlanCfg':
        return cls(step=float(p('step_m')), reverse_step=float(p('reverse_step_m')),
                   xy=float(p('xy_resolution_m')), reverse_xy=float(p('reverse_xy_resolution_m')),
                   angles=int(p('heading_bins')),
                   curvature_fractions=tuple(float(v) for v in p('curvature_fractions')),
                   max_nodes=int(p('max_nodes')), margin=float(p('planning_margin_m')),
                   reverse_margin=float(p('reverse_planning_margin_m')),
                   goal_xy=float(p('goal_tolerance_m')), goal_a=float(p('goal_tolerance_rad')),
                   reverse_goal_xy=float(p('reverse_goal_tolerance_m')),
                   reverse_goal_a=float(p('reverse_goal_tolerance_rad')),
                   h_heading=float(p('heuristic_heading_weight')),
                   h_inflation=float(p('heuristic_inflation')),
                   cost_reverse=float(p('cost_reverse_factor')), cost_curvature=float(p('cost_curvature')),
                   cost_direction_change=float(p('cost_direction_change')),
                   cost_curvature_change=float(p('cost_curvature_change')),
                   cost_clearance=float(p('cost_clearance')), clearance_floor=float(p('clearance_floor_m')),
                   ring_inner=float(p('clockwise_ring_inner_m')), ring_outer=float(p('clockwise_ring_outer_m')),
                   ring_min_dot=float(p('clockwise_min_dot')))


# --------------------------------------------------------------------------- hybrid A* (V4)
@dataclass
class PlanResult:
    path: np.ndarray            # N x 4
    expanded: int
    reason: str


def hybrid_plan(start, goal, course, g, cfg: PlanCfg, reverse: bool = False,
                bounds: Optional[Sequence[float]] = None, clockwise: bool = False,
                max_nodes: Optional[int] = None, connector=None,
                time_budget_s: Optional[float] = None) -> PlanResult:
    """Faithful port of V4 core.js hybridPlan.

    start/goal: (x, y, a). Returns the path sampled at step/4 (1 cm forward).

    connector (phase 5, parking fallback only): V4 reverse mode tries a live
    Reeds-Shepp connection every 12th expansion within 0.75 m of the goal.
    connector(pose) -> N x 4 path (starting at pose) or None. Default None =
    the phase-4 behaviour. time_budget_s: stop the search after this long
    (the V4 node limit still applies).
    """
    import time as _time
    t_end = None if time_budget_s is None else _time.monotonic() + float(time_budget_s)
    conn = None
    step = cfg.reverse_step if reverse else cfg.step
    xy = cfg.reverse_xy if reverse else cfg.xy
    angles = cfg.angles
    max_nodes = cfg.max_nodes if max_nodes is None else max_nodes
    r = g.r
    ks = [f / r for f in cfg.curvature_fractions]
    dirs = (1, -1) if reverse else (1,)
    margin = cfg.reverse_margin if reverse else cfg.margin
    gx, gy, ga = goal
    cx, cy = course.ring[0], course.ring[1]

    def key(x, y, a, d):
        return (js_round(x / xy), js_round(y / xy), js_round((wrap(a) + PI) / TAU * angles), d)

    def heur(x, y, a):
        return math.hypot(gx - x, gy - y) + cfg.h_heading * abs(wrap(ga - a))

    # node: (x, y, a, g, parent_id, dir, k)
    nodes = [(start[0], start[1], start[2], 0.0, -1, 1, 0.0)]
    heap = [(heur(*start), 0, 0)]
    seen = {key(start[0], start[1], start[2], 1): 0.0}
    counter = 1
    best = -1
    exp = 0
    goal_xy = cfg.reverse_goal_xy if reverse else cfg.goal_xy
    goal_a = cfg.reverse_goal_a if reverse else cfg.goal_a
    while heap and exp < max_nodes:
        exp += 1
        _, _, nid = heapq.heappop(heap)
        px, py, pa, pg, _, pdir, pk = nodes[nid]
        if t_end is not None and (exp & 63) == 0 and _time.monotonic() > t_end:
            break
        if reverse and connector is not None and math.hypot(px - gx, py - gy) < 0.75 and exp % 12 == 0:
            r = connector((px, py, pa))
            if r is not None and len(r):
                best, conn = nid, np.asarray(r, float)[:, :4]
                break
        if math.hypot(px - gx, py - gy) < goal_xy and abs(wrap(pa - ga)) < goal_a:
            best = nid
            break
        cand = []
        for d in dirs:
            for k in ks:
                nx, ny, na = bicycle(px, py, pa, d * step, k)
                if bounds is not None and (nx < bounds[0] or nx > bounds[1] or ny < bounds[2] or ny > bounds[3]):
                    continue
                hx, hy, ha = bicycle(px, py, pa, d * step / 2, k)
                cand.append((nx, ny, na, hx, hy, ha, d, k))
        if not cand:
            continue
        C = np.asarray(cand)
        ok = course.body_clear_many(np.r_[C[:, 0], C[:, 3]], np.r_[C[:, 1], C[:, 4]],
                                    np.r_[C[:, 2], C[:, 5]], g, margin)
        n = len(cand)
        ok = ok[:n] & ok[n:]
        margins = course.body_margin_many(C[:, 0], C[:, 1], C[:, 2], g)
        for i in range(n):
            if not ok[i]:
                continue
            nx, ny, na, _, _, _, d, k = cand[i]
            d = int(d)
            if clockwise:
                rx, ry = nx - cx, ny - cy
                rr = math.hypot(rx, ry)
                if cfg.ring_inner < rr < cfg.ring_outer:
                    dot = (math.cos(na) * ry - math.sin(na) * rx) * d
                    if dot < cfg.ring_min_dot:
                        continue
            cost = (pg + step * (cfg.cost_reverse if d < 0 else 1.0) + abs(k) * cfg.cost_curvature
                    + (cfg.cost_direction_change if pdir != d else 0.0) + abs(pk - k) * cfg.cost_curvature_change
                    + step * cfg.cost_clearance / max(cfg.clearance_floor, float(margins[i])))
            kk = key(nx, ny, na, d)
            if kk in seen and seen[kk] <= cost:
                continue
            seen[kk] = cost
            nodes.append((nx, ny, na, cost, nid, d, k))
            heapq.heappush(heap, (cost + heur(nx, ny, na) * cfg.h_inflation, counter, len(nodes) - 1))
            counter += 1
    if best < 0:
        return PlanResult(np.zeros((0, 4)), exp, 'No feasible path found within bounded search')
    chain = []
    while best >= 0:
        chain.append(nodes[best])
        best = nodes[best][4]
    chain.reverse()
    first_dir = chain[1][5] if len(chain) > 1 else 1
    out = [(start[0], start[1], start[2], first_dir)]
    for i in range(1, len(chain)):
        a, b = chain[i - 1], chain[i]
        for j in range(1, 5):
            x, y, h = bicycle(a[0], a[1], a[2], b[5] * step * j / 4, b[6])
            out.append((x, y, h, b[5]))
    if conn is not None:
        return PlanResult(np.vstack([np.asarray(out, float), conn]), exp, 'Hybrid search + Reeds-Shepp connection')
    return PlanResult(np.asarray(out, float), exp, 'Validated sampled full footprint')


# --------------------------------------------------------------------------- geometry helpers
def resample(points: np.ndarray, step: float = 0.01) -> np.ndarray:
    """Uniform arc-length resampling, one gear at a time (V4 splitGears: the cusp
    pose ends one gear and starts the next, so it appears twice with both dirs)."""
    P = np.asarray(points, float).reshape(-1, 4)
    if len(P) < 2:
        return P.copy()
    segs, cur = [], [P[0]]
    for p in P[1:]:
        if cur[-1][3] != p[3]:
            segs.append(np.asarray(cur))
            last = cur[-1].copy()
            last[3] = p[3]
            cur = [last]
        cur.append(p)
    segs.append(np.asarray(cur))
    return np.vstack([_resample_one(s, step) for s in segs])


def _resample_one(seg: np.ndarray, step: float) -> np.ndarray:
    d = np.r_[0.0, np.cumsum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])))]
    if d[-1] < 1e-9:
        return seg[:1].copy()
    n = max(1, int(math.ceil(d[-1] / step)))
    s = np.linspace(0.0, d[-1], n + 1)
    yaw = np.unwrap(seg[:, 2])
    x = np.interp(s, d, seg[:, 0])
    y = np.interp(s, d, seg[:, 1])
    a = np.interp(s, d, yaw)
    a = np.arctan2(np.sin(a), np.cos(a))
    return np.column_stack([x, y, a, np.full(len(s), seg[0, 3])])


def path_length(points: np.ndarray) -> float:
    P = np.asarray(points)
    return float(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])).sum()) if len(P) > 1 else 0.0


def section_labels(course, pts: np.ndarray) -> List[str]:
    """Nearest map section for every point."""
    names, stacks = [], []
    for n, p in course.sections.items():
        names += [n] * len(p)
        stacks.append(p)
    S = np.vstack(stacks)
    out = []
    for i in range(0, len(pts), 400):
        q = pts[i:i + 400, :2]
        d = (q[:, None, 0] - S[None, :, 0]) ** 2 + (q[:, None, 1] - S[None, :, 1]) ** 2
        out += [names[j] for j in d.argmin(1)]
    return out


def section_spans(labels: List[str]) -> List[Dict]:
    """Consecutive runs of the same section: [{section, start, end}] (end inclusive)."""
    spans = []
    for i, s in enumerate(labels):
        if spans and spans[-1]['section'] == s:
            spans[-1]['end'] = i
        else:
            spans.append({'section': s, 'start': i, 'end': i})
    return spans


def roundabout_visits(course, pts: np.ndarray, band: float = 0.25) -> List[Dict]:
    """Where a route passes the roundabout: [{enter, leave, entry, exit}] with the
    entry/exit named after the nearest roundabout control point (west/north/east)."""
    cx, cy, r = course.ring
    dist = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    near = dist < r + band
    visits, i, n = [], 0, len(pts)
    names = list(course.exits)
    ang = {k: math.atan2(v[1] - cy, v[0] - cx) for k, v in course.exits.items()}

    def nearest(j):
        a = math.atan2(pts[j, 1] - cy, pts[j, 0] - cx)
        return min(names, key=lambda k: abs(wrap(a - ang[k])))

    while i < n:
        if near[i]:
            j = i
            while j + 1 < n and near[j + 1]:
                j += 1
            ring_pts = np.abs(dist[i:j + 1] - r) < 0.08
            if ring_pts.any():                   # actually drove on the ring
                visits.append({'enter': i, 'leave': j, 'entry': nearest(i), 'exit': nearest(j)})
            i = j + 1
        else:
            i += 1
    return visits


def validate(course, g, pts: np.ndarray, tolerance: float) -> Tuple[bool, int, float]:
    """(ok, first failing index or -1, min body margin): V4 bodyClear with pad = -tolerance."""
    if len(pts) == 0:
        return False, 0, -1.0
    ok = course.body_clear_many(pts[:, 0], pts[:, 1], pts[:, 2], g, -tolerance)
    m = float(course.body_margin_many(pts[:, 0], pts[:, 1], pts[:, 2], g).min())
    bad = np.flatnonzero(~ok)
    return bool(ok.all()), int(bad[0]) if len(bad) else -1, m


# --------------------------------------------------------------------------- v1 buildMission
def checkpoint_pose(course, c: Dict) -> Tuple[float, float, float]:
    if 'ref' in c:
        return course.pose(str(c['ref']))
    return float(c['x']) * course.sx, float(c['y']) * course.sy, float(c['a'])


def build_v1(course, mission, g, cfg: PlanCfg, log=print) -> List[Tuple[int, np.ndarray, str]]:
    """V4 buildMission: every enabled leg through its checkpoints, starting at the
    start pose and continuing from where the previous leg ended.
    -> [(leg index, points, reason)] (points empty if the leg cannot be planned)."""
    start = course.start_pose()
    out = []
    for lg in mission.legs:
        if not lg.enabled or not lg.checkpoints:
            continue
        leg_pts, reason = [], 'ok'
        for c in lg.checkpoints:
            goal = checkpoint_pose(course, c)
            r = hybrid_plan(start, goal, course, g, cfg, clockwise=lg.clockwise)
            log(f'  {lg.id} -> {c.get("name", "?")}: {len(r.path)} pts, {r.expanded} expanded')
            if not len(r.path):
                reason = (f'Route cannot fit at ({goal[0]:.2f},{goal[1]:.2f}): {r.reason}. Keep the '
                          'radius or correct the geometry; the car will not be forced through.')
                leg_pts = []
                break
            leg_pts.append(r.path if not leg_pts else r.path[1:])
            last = r.path[-1]
            start = (last[0], last[1], last[2])
        out.append((lg.index, np.vstack(leg_pts) if leg_pts else np.zeros((0, 4)), reason))
        if reason != 'ok':
            break
    return out


# --------------------------------------------------------------------------- the whole route
@dataclass
class RouteResult:
    ok: bool
    reason: str
    source: str
    pieces: List[Dict]          # {index, leg, leg_id, kind, bay, points (N x 4), end_behaviour, sections}
    visits: List[Dict]          # {visit, piece, enter, leave, entry, exit, label, leg_id}
    warnings: List[str]


def plan_mission(course, mission, g, cfg: PlanCfg, map_fingerprints: Dict[str, str],
                 tolerance: float, densify: float = 0.01, log=print) -> RouteResult:
    """Block 07: route for the whole mission, or ok=False with the reason."""
    warnings: List[str] = []
    fail = lambda why, src='': RouteResult(False, why, src, [], [], warnings)  # noqa: E731
    pieces: List[Dict] = []
    if mission.version == 2:
        src = 'mission.yaml v2 (tools/map/mission_planner.py), checked'
        if mission.map_sha1 not in map_fingerprints.values():
            return fail(f'mission.yaml was planned on a different track_map.yaml (sha1 '
                        f'{mission.map_sha1[:10]} vs {map_fingerprints["raw"][:10]}). Re-run '
                        'tools/map/mission_planner.py on this map.', src)
        if not mission.planned:
            bad = [lg.id for lg in mission.legs if lg.enabled]
            return fail(f'mission.yaml has legs without a route ({bad}); re-run mission_planner.py', src)
        for p in mission.pieces:
            pts = resample(p.points, densify)
            lg = mission.legs[p.leg]
            if p.kind == 'road':
                ok, bad, m = validate(course, g, pts, tolerance)
                if not ok:
                    q = pts[bad]
                    return fail(f'{lg.id} road piece leaves the road at ({q[0]:.2f}, {q[1]:.2f}) '
                                f'(body check, tolerance {tolerance * 100:.1f} cm): map and mission disagree', src)
            else:
                ok, bad, m = validate(course, g, pts, tolerance)
                if not ok:
                    q = pts[bad]
                    warnings.append(f'{lg.id} {p.bay} manoeuvre preview touches the edge at '
                                    f'({q[0]:.2f}, {q[1]:.2f}); block 11 replans it from the observed bay')
            pieces.append({'leg': p.leg, 'leg_id': lg.id, 'kind': p.kind, 'bay': p.bay, 'points': pts})
    else:
        src = 'mission.yaml v1: V4 hybridPlan through the checkpoints'
        legs = build_v1(course, mission, g, cfg, log)
        by_leg = {li: (pts, why) for li, pts, why in legs}
        for p in mission.pieces:
            lg = mission.legs[p.leg]
            if p.kind == 'road':
                pts, why = by_leg.get(p.leg, (np.zeros((0, 4)), 'not planned'))
                if not len(pts):
                    return fail(f'{lg.id}: {why}', src)
                pieces.append({'leg': p.leg, 'leg_id': lg.id, 'kind': 'road', 'bay': '',
                               'points': resample(pts, densify)})
            else:
                pieces.append({'leg': p.leg, 'leg_id': lg.id, 'kind': 'manoeuvre', 'bay': p.bay,
                               'points': np.zeros((0, 4))})
    if not pieces:
        return fail('mission has no enabled legs', src)
    # leg end behaviours on the last piece of each leg
    for i, p in enumerate(pieces):
        last = i + 1 >= len(pieces) or pieces[i + 1]['leg'] != p['leg']
        p['end_behaviour'] = mission.legs[p['leg']].end_behaviour if last else ''
        p['index'] = i
        p['sections'] = section_labels(course, p['points']) if len(p['points']) else []
    # roundabout exits actually taken, in order
    visits = []
    for p in pieces:
        if p['kind'] != 'road' or not len(p['points']):
            continue
        for v in roundabout_visits(course, p['points']):
            visits.append(dict(v, visit=len(visits) + 1, piece=p['index'], leg_id=p['leg_id']))
    planned = mission.visits()
    if len(planned) != len(visits):
        return fail(f'route visits the roundabout {len(visits)} times, mission rules expect '
                    f'{len(planned)} (roundabout_visits)', src)
    for v, pv in zip(visits, planned):
        v['label'] = pv['exit']
        if v['exit'] != pv['direction'] or v['leg_id'] != pv['leg_id']:
            return fail(f'roundabout visit {v["visit"]}: route leaves by {v["exit"]} on {v["leg_id"]}, '
                        f'mission rules plan {pv["direction"]} on {pv["leg_id"]}', src)
    for lg in mission.legs:
        mine = [v['exit'] for v in visits if v['leg_id'] == lg.id]
        if lg.declared_exits and lg.declared_exits != mine:
            warnings.append(f'{lg.id}: mission.yaml says exits {lg.declared_exits}, route check found {mine}')
    return RouteResult(True, 'ok', src, pieces, visits, warnings)
