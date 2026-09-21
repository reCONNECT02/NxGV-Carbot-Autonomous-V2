"""BLOCK 11 core (no ROS): compute the manoeuvre (V4 core.js parkingPlan +
app.js bayObservation / transition phase 2).

parking_plan(start, goal)       V4 core.js parkingPlan, stage order unchanged:
  1. docking straight: Reeds-Shepp to a pre-pose 15 / 12 / 8 cm ahead of the goal,
     then a straight reverse into the goal (shortest feasible connector within
     the first tail that works);
  2. direct Reeds-Shepp;
  3. handoff extensions: drive -10..+20 cm straight first, then Reeds-Shepp
     (cheapest total wins);
  4. bounded hybrid A* reverse search with the live Reeds-Shepp connector.
  Every connection is checked with the V4 bodyClear footprint (pad 3 mm) on the
  PRIOR map. Everything tried is kept for the GUI audit (grey / blue).
  Difference from V4, on purpose: stage 1 only runs when the goal is INSIDE the
  bay (parking in). Un-parking (leg 3) goes straight to stage 2.

observe_bay(...)                V4 app.js bayObservation, generalised to the v2
  bay polygons: paint cells from local memory (<= 24 s old) within 3.5 cm of
  the bay's START, END and FAR edges (the three V4 edges), >= 3 samples each,
  medians -> the observed bay is shifted by (along, lateral) and the goal with
  it. Not in V4: a shift larger than max_shift_m is rejected, and after
  fallback_after_s without a valid observation the map bay is used (V4 holds
  forever; a hold forever = manual intervention = 0 marks).

ParkingSession                  V4 transition() phase 2: the plan is split at
  the gear changes and published ONE SECTION AT A TIME. When the tracker
  reports `arrived` at the end of a section:
    * more sections: if the car stopped > 2.5 cm from the cusp, replan the rest
      from the estimated pose (<= 4 times); hold 0.4 s; next section;
    * last section: terminal heading error > 0.045 rad -> replan a correction
      from the estimated pose (<= 2 times); else DONE.
  Planning always starts from the ACTUAL estimated pose (never a replay).
"""
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import reeds_shepp as rs
from .route_core import PlanCfg, bicycle, hybrid_plan, wrap
from .tracker_core import split_gears


# --------------------------------------------------------------------------- config
@dataclass
class ParkingCfg:
    docking_tails: Sequence[float] = (0.15, 0.12, 0.08)
    docking_step: float = 0.006
    handoff_extensions: Sequence[float] = (-0.10, -0.05, 0.05, 0.10, 0.15, 0.20)
    clearance_pad: float = 0.003
    rs_step: float = 0.006
    fallback_enabled: bool = True
    fallback_max_nodes: int = 250000
    fallback_margin: float = 0.45
    fallback_time_budget_s: float = 6.0
    fallback_bounds_v1: Sequence[float] = (-0.02, 2.65, 1.70, 3.70)
    edge_tolerance: float = 0.035
    min_samples: int = 3
    observation_max_age_s: float = 24.0
    max_shift: float = 0.10
    fallback_after_s: float = 3.0
    cusp_replan_distance: float = 0.025
    max_cusp_replans: int = 4
    heading_tolerance: float = 0.045
    max_corrections: int = 2
    parked_heading_tolerance: float = 0.05
    parked_margin: float = 0.008
    gear_hold_s: float = 0.4
    stopped_speed: float = 0.01
    section_arrive_m: float = 0.08
    arrived_after_publish_s: float = 0.15
    replan_retry_s: float = 1.0
    reverse_time_unpark: bool = True
    plan_radius_factor: float = 1.08         # = YAML default; 1.0 = V4 (exactly the minimum turning radius)

    @classmethod
    def from_params(cls, p) -> 'ParkingCfg':
        return cls(docking_tails=tuple(float(v) for v in p('docking_tails_m')),
                   docking_step=float(p('docking_step_m')),
                   handoff_extensions=tuple(float(v) for v in p('handoff_extensions_m')),
                   clearance_pad=float(p('clearance_pad_m')), rs_step=float(p('rs_step_m')),
                   fallback_enabled=bool(p('fallback_enabled')),
                   fallback_max_nodes=int(p('fallback_max_nodes')), fallback_margin=float(p('fallback_margin_m')),
                   fallback_time_budget_s=float(p('fallback_time_budget_s')),
                   fallback_bounds_v1=tuple(float(v) for v in p('fallback_bounds')),
                   edge_tolerance=float(p('bay_observation.edge_tolerance_m')),
                   min_samples=int(p('bay_observation.min_samples')),
                   observation_max_age_s=float(p('bay_observation.max_age_s')),
                   max_shift=float(p('bay_observation.max_shift_m')),
                   fallback_after_s=float(p('bay_observation.fallback_after_s')),
                   cusp_replan_distance=float(p('cusp_replan_distance_m')),
                   max_cusp_replans=int(p('max_cusp_replans')),
                   heading_tolerance=float(p('heading_tolerance_rad')), max_corrections=int(p('max_corrections')),
                   parked_heading_tolerance=float(p('parked_heading_tolerance_rad')),
                   parked_margin=float(p('parked_margin_m')), gear_hold_s=float(p('gear_hold_s')),
                   stopped_speed=float(p('stopped_speed_mps')), section_arrive_m=float(p('section_arrive_m')),
                   arrived_after_publish_s=float(p('arrived_after_publish_s')),
                   replan_retry_s=float(p('replan_retry_s')),
                   reverse_time_unpark=bool(p('reverse_time_unpark')),
                   plan_radius_factor=float(p('plan_radius_factor')))


# --------------------------------------------------------------------------- plan
@dataclass
class Audit:
    id: int
    points: np.ndarray          # N x 4
    cost: float
    valid: bool
    reject: str
    stage: str
    types: str
    selected: bool = False


@dataclass
class ParkResult:
    path: np.ndarray            # N x 4 ([] = none)
    stage: str
    reason: str
    cost: float = 0.0
    types: str = ''
    evaluated: List[Audit] = field(default_factory=list)


def sample_line(a, b, step: float = 0.006) -> np.ndarray:
    """core.js sampleLine: n = ceil(dist / step), points i = 0..n (x, y)."""
    n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    i = np.arange(n + 1, dtype=float)
    return np.column_stack([a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n])


def _length(P: np.ndarray) -> float:
    if len(P) < 2:
        return 0.0
    return float(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])).sum())


def _clear(course, g, P: np.ndarray, pad: float) -> bool:
    return len(P) == 0 or bool(course.body_clear_many(P[:, 0], P[:, 1], P[:, 2], g, pad).all())


def fallback_bounds(course, start, goal, bay_poly, cfg: ParkingCfg):
    """V4 bounds [bay.x0 - 0.02, 2.65, 1.70, 3.70] on the V4 course (v1); on a v2 map
    the box around start, goal and the bay, grown by fallback_margin."""
    if getattr(course, 'version', 2) == 1 and 'parallel_bay' in getattr(course, 'areas', {}):
        b = cfg.fallback_bounds_v1
        return [course.areas['parallel_bay'][0] + b[0], b[1] * course.sx, b[2] * course.sy, b[3] * course.sy]
    pts = [start[:2], goal[:2]] + ([tuple(p) for p in bay_poly] if bay_poly is not None else [])
    P = np.asarray(pts, float)
    m = cfg.fallback_margin
    return [P[:, 0].min() - m, P[:, 0].max() + m, P[:, 1].min() - m, P[:, 1].max() + m]


def reverse_time(path: np.ndarray) -> np.ndarray:
    """The same poses driven the other way: order reversed, every direction flipped
    (headings unchanged). A kinematically feasible path stays feasible."""
    P = np.asarray(path, float)[::-1].copy()
    P[:, 3] = -P[:, 3]
    return P


def parking_plan(start, goal, course, g, cfg: ParkingCfg, docking: bool = True,
                 bay_poly: Optional[np.ndarray] = None, plan_cfg: Optional[PlanCfg] = None,
                 unpark_from_bay: bool = False) -> ParkResult:
    """V4 core.js parkingPlan (see module docstring). start / goal = (x, y, a).

    cfg.plan_radius_factor > 1 plans every connection at that multiple of the
    minimum turning radius (not V4): the tracker then keeps steering in reserve
    to correct lag instead of saturating on a minimum-radius arc.

    unpark_from_bay (not in V4, which never leaves a bay): before the slow hybrid
    fallback, plan the time-reversed problem - from the road goal INTO the bay
    at the car's actual pose, with the full V4 parking search - and drive that
    path backwards (stage 'reversed parking search')."""
    pad = cfg.clearance_pad
    evaluated: List[Audit] = []
    if cfg.plan_radius_factor != 1.0:
        import dataclasses
        g = dataclasses.replace(g, r=g.r * cfg.plan_radius_factor)

    def audit(allc, prefix: np.ndarray, suffix: np.ndarray, stage: str) -> None:
        pre_ok = _clear(course, g, prefix, pad)
        suf_ok = _clear(course, g, suffix, pad)
        extra = _length(prefix) + _length(suffix)
        for q in allc:
            pts = np.vstack([P for P in (prefix, rs.to_path4(q.points), suffix) if len(P)])
            valid = q.valid and pre_ok and suf_ok
            evaluated.append(Audit(len(evaluated), pts, q.cost + extra, valid,
                                   '' if valid else 'Footprint leaves drivable area', stage, q.types))

    def finish(r: ParkResult) -> ParkResult:
        r.evaluated = evaluated
        for a in evaluated:
            a.selected = len(r.path) > 0 and len(a.points) == len(r.path) and bool(np.allclose(a.points, r.path))
        return r

    # 1. docking straight (reverse the last tail metres straight into the goal)
    if docking:
        for tail in cfg.docking_tails:
            pre = bicycle(goal[0], goal[1], goal[2], tail, 0.0)
            win, allc = rs.plan(start, pre, course, g, pad, cfg.rs_step)
            xy = sample_line(pre, goal, cfg.docking_step)
            end = np.column_stack([xy, np.full(len(xy), goal[2]), np.full(len(xy), -1.0)])
            stage = f'{round(tail * 100)} cm docking straight'
            audit(allc, np.zeros((0, 4)), end, stage)
            if win is not None and _clear(course, g, end, pad):
                return finish(ParkResult(np.vstack([rs.to_path4(win.points), end]), stage,
                                         'Live Reeds-Shepp connection', win.cost + tail, win.types))
    # 2. direct
    win, allc = rs.plan(start, goal, course, g, pad, cfg.rs_step)
    audit(allc, np.zeros((0, 4)), np.zeros((0, 4)), 'direct')
    if win is not None:
        return finish(ParkResult(rs.to_path4(win.points), 'direct', 'Live Reeds-Shepp connection',
                                 win.cost, win.types))
    # 3. handoff extensions
    best = None
    for d in cfg.handoff_extensions:
        mid = bicycle(start[0], start[1], start[2], d, 0.0)
        xy = sample_line(start, mid, cfg.docking_step)
        approach = np.column_stack([xy, np.full(len(xy), start[2]), np.full(len(xy), 1.0 if d > 0 else -1.0)])
        if not _clear(course, g, approach, pad):
            continue
        q, allc = rs.plan(mid, goal, course, g, pad, cfg.rs_step)
        audit(allc, approach, np.zeros((0, 4)), 'handoff extension')
        if q is not None and (best is None or q.cost + abs(d) < best.cost):
            best = ParkResult(np.vstack([approach, rs.to_path4(q.points)]), 'handoff extension',
                              'Live Reeds-Shepp connection', q.cost + abs(d), q.types)
    if best is not None:
        return finish(best)
    # 3b. un-park: time-reversed parking search (see docstring)
    if unpark_from_bay and cfg.reverse_time_unpark:
        inner = parking_plan(goal, start, course, g, cfg, True, bay_poly, plan_cfg, False)
        for a in inner.evaluated:
            evaluated.append(Audit(len(evaluated), reverse_time(a.points), a.cost, a.valid, a.reject,
                                   'reversed ' + a.stage, a.types))
        if len(inner.path):
            return finish(ParkResult(reverse_time(inner.path), f'reversed parking search ({inner.stage})',
                                     inner.reason, inner.cost, inner.types))
    # 4. bounded hybrid fallback
    if not cfg.fallback_enabled:
        return finish(ParkResult(np.zeros((0, 4)), 'none', 'No collision-free analytic connection'))

    def connector(p):
        w, _ = rs.plan(p, goal, course, g, pad, cfg.rs_step)
        return None if w is None else rs.to_path4(w.points)

    r = hybrid_plan(start, goal, course, g, plan_cfg or PlanCfg(), reverse=True,
                    bounds=fallback_bounds(course, start, goal, bay_poly, cfg),
                    max_nodes=cfg.fallback_max_nodes, connector=connector,
                    time_budget_s=cfg.fallback_time_budget_s)
    return finish(ParkResult(r.path, 'hybrid search fallback', r.reason, _length(r.path)))


# --------------------------------------------------------------------------- bay
@dataclass
class Bay:
    """A bay as a frame: origin A, along unit u, left normal n, along (s0, s1),
    lateral (l0, l1); 'far' = the lateral edge away from the road line."""
    name: str
    A: np.ndarray
    u: np.ndarray
    n: np.ndarray
    s0: float
    s1: float
    l0: float
    l1: float
    poly: np.ndarray

    @property
    def far(self) -> float:
        return self.l1 if abs(self.l1) >= abs(self.l0) else self.l0

    @classmethod
    def from_course(cls, course, name: str) -> Optional['Bay']:
        areas = getattr(course, 'v2_areas', None) or {}
        a = areas.get(name)
        if a is None:
            return None
        A, u, n = (np.asarray(v, float) for v in a['frame'])
        (s0, s1), (l0, l1) = a['along'], a['lateral']
        return cls(name, A, u, n, float(s0), float(s1), float(min(l0, l1)), float(max(l0, l1)),
                   np.asarray(a['poly'], float))

    def local(self, P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        d = np.asarray(P, float)[:, :2] - self.A
        return d @ self.u, d @ self.n

    def contains(self, x: float, y: float) -> bool:
        s, l = self.local(np.array([[x, y]]))
        return bool(self.s0 <= s[0] <= self.s1 and self.l0 <= l[0] <= self.l1)


@dataclass
class BayObservation:
    valid: bool
    count: List[int]
    shift_along: float = 0.0
    shift_lateral: float = 0.0
    goal: Optional[Tuple[float, float, float]] = None
    reason: str = ''


def _median(v: List[float]) -> float:
    s = sorted(v)
    return s[len(s) // 2]                     # V4: g.sort()[floor(n/2)]


def observe_bay(bay: Bay, paint_xy: np.ndarray, prior_goal, cfg: ParkingCfg) -> BayObservation:
    """V4 bayObservation on a v2 bay (see module docstring). paint_xy = N x 2
    track-frame centres of memory PAINT cells that are young enough."""
    tol = cfg.edge_tolerance
    if paint_xy is None or not len(paint_xy):
        return BayObservation(False, [0, 0, 0], reason='no remembered paint')
    s, l = bay.local(paint_xy)
    lo_l, hi_l = bay.l0, bay.l1
    in_l = (l > lo_l) & (l < hi_l)
    in_s = (s > bay.s0) & (s < bay.s1)
    g_start = s[(np.abs(s - bay.s0) < tol) & in_l].tolist()
    g_end = s[(np.abs(s - bay.s1) < tol) & in_l].tolist()
    g_far = l[(np.abs(l - bay.far) < tol) & in_s].tolist()
    count = [len(g_far), len(g_start), len(g_end)]            # V4 order: left(far), bottom, top
    if any(c < cfg.min_samples for c in count):
        return BayObservation(False, count, reason=f'edge sample counts {count[0]}/{count[1]}/{count[2]}')
    da = (_median(g_start) + _median(g_end)) / 2 - (bay.s0 + bay.s1) / 2
    dl = _median(g_far) - bay.far
    if math.hypot(da, dl) > cfg.max_shift:
        return BayObservation(False, count, da, dl,
                              reason=f'observed bay {math.hypot(da, dl) * 100:.1f} cm from the map: rejected')
    gx = prior_goal[0] + da * bay.u[0] + dl * bay.n[0]
    gy = prior_goal[1] + da * bay.u[1] + dl * bay.n[1]
    return BayObservation(True, count, da, dl, (gx, gy, prior_goal[2]))


def parked_check(pose, goal, bay: Optional[Bay], g, cfg: ParkingCfg) -> Tuple[bool, float]:
    """V4 course.parked on the ESTIMATED pose: heading within 0.05 rad and the whole
    footprint inside the bay shrunk by 8 mm. -> (ok, heading error rad)."""
    err = abs(wrap(pose[2] - goal[2]))
    if bay is None:
        return err < cfg.parked_heading_tolerance, err
    c, s = math.cos(pose[2]), math.sin(pose[2])
    corners = [(-g.rear, -g.w / 2), (g.front, -g.w / 2), (g.front, g.w / 2), (-g.rear, g.w / 2)]
    P = np.array([(pose[0] + x * c - y * s, pose[1] + x * s + y * c) for x, y in corners])
    ss, ll = bay.local(P)
    m = cfg.parked_margin
    inside = bool(((ss >= bay.s0 + m) & (ss <= bay.s1 - m) & (ll >= bay.l0 + m) & (ll <= bay.l1 - m)).all())
    return inside and err < cfg.parked_heading_tolerance, err


# --------------------------------------------------------------------------- session
IDLE, OBSERVING, PLANNING, DRIVING, GEAR_HOLD, DONE = 'IDLE', 'OBSERVING', 'PLANNING', 'DRIVING', 'GEAR_HOLD', 'DONE'


@dataclass
class SessionOut:
    publish: Optional[np.ndarray] = None        # new section to publish (N x 4; empty = clear)
    audit: Optional[ParkResult] = None           # new plan for the GUI
    events: List[Tuple[str, str]] = field(default_factory=list)


class ParkingSession:
    """One manoeuvre piece (V4 app.js transition phase 2). Drive with update()."""

    def __init__(self, cfg: ParkingCfg, course, g, plan_cfg: Optional[PlanCfg] = None,
                 planner: Optional[Callable] = None):
        self.cfg, self.course, self.g, self.plan_cfg = cfg, course, g, plan_cfg
        self.planner = planner or (lambda s, gl, dock, poly, unpark=False: parking_plan(
            s, gl, course, g, cfg, dock, poly, plan_cfg, unpark))
        self.key = None
        self.state = IDLE
        self.reason = ''
        self.stage = ''
        self.goal = self.prior_goal = ()
        self.bay = None
        self.docking = False
        self.parts = []
        self.cusp_replans = self.corrections = 0
        self.observation = None
        self.parked_ok = self.heading_err = None
        self.done_t = -1e9

    # ------------------------------------------------------------------ start
    def start(self, key, goal, bay_name: str, t: float) -> SessionOut:
        """New manoeuvre piece: goal = the piece's end pose (mission_planner)."""
        self.key = key
        self.prior_goal = tuple(float(v) for v in goal)
        self.goal = self.prior_goal
        self.bay = Bay.from_course(self.course, bay_name) if bay_name else None
        self.docking = self.bay is not None and self.bay.contains(goal[0], goal[1])
        self.unpark = False              # set from the pose at the first plan
        self.t0 = t
        self.parts: List[np.ndarray] = []
        self.current: Optional[np.ndarray] = None
        self.published_t = -1e9
        self.hold_until = -1e9
        self.cusp_replans = 0
        self.corrections = 0
        self.last_try = -1e9
        self.stage = ''
        self.observation: Optional[BayObservation] = None
        self.parked_ok = None
        self.heading_err = None
        self.done_t = -1e9
        self.state = OBSERVING if self.docking else PLANNING
        self.reason = 'Waiting for the car to stop'
        return SessionOut(publish=np.zeros((0, 4)))     # clear the previous piece's path at once

    # ------------------------------------------------------------------ helpers
    def _plan(self, pose, t, out: SessionOut, event: str, detail: str) -> bool:
        self.last_try = t
        self.unpark = not self.docking and self.bay is not None and self.bay.contains(pose[0], pose[1])
        r = self.planner(tuple(pose), self.goal, self.docking, self.bay.poly if self.bay else None, self.unpark)
        out.audit = r
        if not len(r.path):
            self.reason = f'NO PARKING PATH: {r.reason}'
            out.events.append(('HOLD', self.reason))
            return False
        self.stage = r.stage
        self.parts = split_gears(r.path)
        out.events.append((event, f'{detail} {r.stage}: {r.reason}; {len(r.evaluated)} candidates, '
                                  f'{len(self.parts)} gear sections.'))
        return True

    def _next_part(self, t: float, out: SessionOut, hold: bool) -> None:
        self.current = self.parts.pop(0)
        if hold:
            self.state, self.hold_until = GEAR_HOLD, t + self.cfg.gear_hold_s
            self.reason = 'Speed zero before the next forward/reverse section'
        else:
            self._publish(t, out)

    def _publish(self, t: float, out: SessionOut) -> None:
        out.publish = self.current
        self.published_t = t
        self.state = DRIVING
        d = 'reverse' if self.current[-1, 3] < 0 else 'forward'
        self.reason = f'Section ({d}), {len(self.parts)} more'

    # ------------------------------------------------------------------ cycle
    def update(self, t: float, pose, speed: float, arrived_t: float,
               paint_xy: Optional[Callable[[], np.ndarray]] = None) -> SessionOut:
        out = SessionOut()
        c = self.cfg
        if self.state in (IDLE, DONE) or pose is None:
            return out
        if self.state == OBSERVING:
            if abs(speed) > c.stopped_speed:
                return out
            obs = observe_bay(self.bay, paint_xy() if paint_xy else None, self.prior_goal, c)
            self.observation = obs
            if obs.valid:
                self.goal = obs.goal
                out.events.append(('LIVE PARKING SEARCH', f'Three bay edges supported by '
                                   f'{obs.count[0]}/{obs.count[1]}/{obs.count[2]} image-derived samples; '
                                   f'bay shift {obs.shift_along * 100:+.1f} / {obs.shift_lateral * 100:+.1f} cm. '
                                   'Current estimated pose -> observed bay goal.'))
                self.state = PLANNING
            elif 0 <= c.fallback_after_s < t - self.t0:
                out.events.append(('BAY NOT OBSERVED', f'{obs.reason}: using the MAP bay goal'))
                self.state = PLANNING
            else:
                self.reason = f'BAY OBSERVATION HOLD: need visible/remembered bay tape; {obs.reason}'
                return out
        if self.state == PLANNING:
            if abs(speed) > c.stopped_speed or t - self.last_try < c.replan_retry_s:
                return out
            if self._plan(pose, t, out, 'PARKING PATH SELECTED', 'Planned from the estimated pose:'):
                self._next_part(t, out, hold=False)
            return out
        if self.state == GEAR_HOLD:
            if t >= self.hold_until:
                self._publish(t, out)
            return out
        # DRIVING: wait for the tracker to report the end of this section
        end = self.current[-1]
        if arrived_t < self.published_t + c.arrived_after_publish_s or \
                math.hypot(pose[0] - end[0], pose[1] - end[1]) > c.section_arrive_m:
            return out
        if self.parts:
            if math.hypot(pose[0] - end[0], pose[1] - end[1]) > c.cusp_replan_distance and \
                    self.cusp_replans < c.max_cusp_replans:
                if self._plan(pose, t, out, 'PARKING REPLAN', 'Stopped beyond the planned cusp; recomputed '
                              'the remaining parking from the estimated pose:'):
                    self.cusp_replans += 1
            out.events.append(('GEAR CHANGE', 'Speed zero before the next forward/reverse section.'))
            self._next_part(t, out, hold=True)
            return out
        err = abs(wrap(pose[2] - self.goal[2]))
        if err > c.heading_tolerance and self.corrections < c.max_corrections:
            if self._plan(pose, t, out, 'PARKING REPLAN', f'Terminal heading error {math.degrees(err):.1f} deg '
                          'outside tolerance; live correction:'):
                self.corrections += 1
                self._next_part(t, out, hold=True)
                return out
        self.parked_ok, self.heading_err = parked_check(pose, self.goal, self.bay if self.docking else None,
                                                        self.g, c)
        self.state, self.done_t = DONE, t
        if self.docking:
            out.events.append(('PARKING PASSED' if self.parked_ok else 'PARKING CHECK FAILED',
                               f'Estimated footprint inside {self.bay.name}={self.parked_ok}; heading error '
                               f'{math.degrees(self.heading_err):.2f} deg'))
        else:
            out.events.append(('MANOEUVRE COMPLETE', f'Reached the handoff pose (heading error '
                               f'{math.degrees(self.heading_err):.2f} deg)'))
        self.reason = 'Done'
        return out

    def state_dict(self) -> Dict:
        return {'key': str(self.key), 'state': self.state, 'reason': self.reason, 'stage': self.stage,
                'goal': list(self.goal) if self.state != IDLE else [],
                'prior_goal': list(self.prior_goal) if self.state != IDLE else [],
                'bay': self.bay.name if self.state != IDLE and self.bay else '',
                'docking': bool(self.docking) if self.state != IDLE else False,
                'done': self.state == DONE, 'done_t': self.done_t if self.state == DONE else -1.0,
                'cusp_replans': self.cusp_replans if self.state != IDLE else 0,
                'corrections': self.corrections if self.state != IDLE else 0,
                'sections_left': len(self.parts) if self.state != IDLE else 0,
                'parked_ok': self.parked_ok if self.state == DONE else None,
                'heading_error_deg': math.degrees(self.heading_err) if self.heading_err is not None else None,
                'observation': None if self.state == IDLE or self.observation is None else
                {'valid': self.observation.valid, 'count': self.observation.count,
                 'shift_along_m': self.observation.shift_along, 'shift_lateral_m': self.observation.shift_lateral,
                 'reason': self.observation.reason}}
