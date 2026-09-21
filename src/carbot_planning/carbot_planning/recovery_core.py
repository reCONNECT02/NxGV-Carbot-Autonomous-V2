"""BLOCK 12 core (no ROS): make room, then rejoin (faithful port of V4 recovery.js).

When a lane-driving problem (no feasible local candidate / tracking error > 10 cm /
recoverable branch hold) lasts problem_dwell_s (0.45 s) and recovery is
permitted (ROAD, no TRAFFIC / GATE / GEAR hold), the Recovery machine:

  BRAKE  stop (encoder < 3 mm/s); wait for fresh LiDAR (0.35 s) and camera (0.30 s)
  search Reeds-Shepp connections from the estimated pose to up to 5 route poses
         (first 25 cm ahead, then every 15 cm, body clear with 8 mm pad); a
         candidate must reverse FIRST then drive forward (exactly 2 gear
         sections), reverse 3..20 cm, length <= 1.6 m, footprint within the
         paint allowance, no LiDAR return in the swept footprint (+2 cm), and
         >= 55 % of the rear road observed (camera / memory).
         cost = length + 0.7 * reverse + 150 * 0.006 * sum(outside margin)
  WAIT   nothing valid: rescan every 2 s
  ALIGN  0.4 s: pre-steer to the first segment's curvature, speed 0
  TRACK  V4 Controller at 3 cm/s (parking tolerances), one gear section at a
         time, 18-point look-ahead re-checked every cycle (-> BRAKE if blocked)
  done   rejoined the route: the lane planner resumes.

At most 3 attempts per place (reset after 15 cm away from the last finish).
A safety veto or a mission hold pauses recovery (zero request); it can never
bypass them. UWB is not an input: only the local estimate is used.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import reeds_shepp as rs
from .corridor_core import closest
from .evidence import PAINT, ROAD, Evidence
from .tracker_core import Tracker, TrackerCfg, split_gears


@dataclass
class RecoveryCfg:
    enabled: bool = True
    reverse_max: float = 0.20
    reverse_min: float = 0.03
    problem_dwell_s: float = 0.45
    max_attempts: int = 3
    reset_attempts_after_m: float = 0.15
    rescan_period_s: float = 2.0
    align_s: float = 0.4
    max_goals: int = 5
    first_goal_m: float = 0.25
    goal_spacing_m: float = 0.15
    goal_clearance_m: float = 0.008
    max_cost: float = 1.6
    cost_reverse_weight: float = 0.7
    cost_outside_weight: float = 150.0
    outside_step_m: float = 0.006
    rear_evidence_min_ratio: float = 0.55
    rear_sample_every: int = 5
    rear_offset_m: float = 0.025
    rear_lateral_fraction: float = 0.35
    evidence_max_age_s: float = 3.0
    lidar_max_age_s: float = 0.35
    camera_max_age_s: float = 0.30
    lookahead_points: int = 18
    stopped_speed: float = 0.003
    recovery_speed: float = 0.03
    line_tolerance: float = 0.05
    road_pad_m: float = 0.005
    road_step_m: float = 0.025
    obstacle_pad_m: float = 0.02
    rs_step_m: float = 0.006

    @classmethod
    def from_params(cls, p) -> 'RecoveryCfg':
        return cls(enabled=bool(p('enabled')), reverse_max=float(p('reverse_max_m')),
                   reverse_min=float(p('reverse_min_m')), problem_dwell_s=float(p('problem_dwell_s')),
                   max_attempts=int(p('max_attempts')), reset_attempts_after_m=float(p('reset_attempts_after_m')),
                   rescan_period_s=float(p('rescan_period_s')), align_s=float(p('align_s')),
                   max_goals=int(p('max_goals')), first_goal_m=float(p('first_goal_m')),
                   goal_spacing_m=float(p('goal_spacing_m')), goal_clearance_m=float(p('goal_clearance_m')),
                   max_cost=float(p('max_cost')), cost_reverse_weight=float(p('cost_reverse_weight')),
                   cost_outside_weight=float(p('cost_outside_weight')), outside_step_m=float(p('outside_step_m')),
                   rear_evidence_min_ratio=float(p('rear_evidence_min_ratio')),
                   rear_sample_every=int(p('rear_sample_every')), rear_offset_m=float(p('rear_offset_m')),
                   rear_lateral_fraction=float(p('rear_lateral_fraction')),
                   evidence_max_age_s=float(p('evidence_max_age_s')), lidar_max_age_s=float(p('lidar_max_age_s')),
                   camera_max_age_s=float(p('camera_max_age_s')), lookahead_points=int(p('lookahead_points')),
                   stopped_speed=float(p('stopped_speed_mps')), recovery_speed=float(p('limits.recovery_speed_mps')),
                   line_tolerance=float(p('limits.line_tolerance_m')), road_pad_m=float(p('road_pad_m')),
                   road_step_m=float(p('road_step_m')), obstacle_pad_m=float(p('obstacle_pad_m')),
                   rs_step_m=float(p('rs_step_m')))


# --------------------------------------------------------------------------- checks (V4 helpers)
def _road_samples(g, pad: float, step: float) -> np.ndarray:
    """recovery.js roadClear sampling: long sides every 2.5 cm, short sides every 2.5 cm, corners."""
    xs, ys = (-g.rear - pad, g.front + pad), (-g.w / 2 - pad, g.w / 2 + pad)
    pts = []
    x = xs[0]
    while x <= xs[1]:
        pts += [(x, ys[0]), (x, ys[1])]
        x += step
    y = ys[0]
    while y <= ys[1]:
        pts += [(xs[0], y), (xs[1], y)]
        y += step
    pts += [(a, b) for a in xs for b in ys]
    return np.asarray(pts, float)


def road_clear_many(course, g, P: np.ndarray, tolerance: float, pad: float = 0.005, step: float = 0.025) -> np.ndarray:
    """V4 recovery.roadClear per pose: clearance >= -tolerance at every body sample."""
    P = np.asarray(P, float)
    if not len(P):
        return np.zeros(0, bool)
    S = _road_samples(g, pad, step)
    c, s = np.cos(P[:, 2])[:, None], np.sin(P[:, 2])[:, None]
    X = P[:, 0, None] + S[None, :, 0] * c - S[None, :, 1] * s
    Y = P[:, 1, None] + S[None, :, 0] * s + S[None, :, 1] * c
    d = np.asarray(course.clearance(X.ravel(), Y.ravel())).reshape(X.shape)
    return (d >= -tolerance).all(1)


def obstacles_clear_many(g, P: np.ndarray, hits: Optional[np.ndarray], pad: float = 0.02) -> np.ndarray:
    """V4 recovery.obstaclesClear: no LiDAR return inside the footprint + pad."""
    P = np.asarray(P, float)
    if hits is None or not len(hits) or not len(P):
        return np.ones(len(P), bool)
    c, s = np.cos(P[:, 2])[:, None], np.sin(P[:, 2])[:, None]
    dx, dy = hits[None, :, 0] - P[:, 0, None], hits[None, :, 1] - P[:, 1, None]
    qx, qy = dx * c + dy * s, -dx * s + dy * c
    inside = (qx > -g.rear - pad) & (qx < g.front + pad) & (np.abs(qy) < g.w / 2 + pad)
    return ~inside.any(1)


def rear_evidence(ev: Evidence, course, g, P: np.ndarray, cfg: RecoveryCfg) -> Tuple[bool, float]:
    """V4 recovery.rearEvidence: behind the rear bumper at 3 lateral points of every
    5th REVERSE pose, the road must be known (camera or memory) for >= 55 %."""
    rows = [p for i, p in enumerate(P) if i % cfg.rear_sample_every == 0 and p[3] == -1]
    if not rows:
        return False, 0.0
    W = []
    for x, y, a, _ in (r[:4] for r in rows):
        c, s = math.cos(a), math.sin(a)
        for ly in (-g.w * cfg.rear_lateral_fraction, 0.0, g.w * cfg.rear_lateral_fraction):
            lx = -g.rear - cfg.rear_offset_m
            W.append((x + lx * c - ly * s, y + lx * s + ly * c))
    W = np.asarray(W)
    kind, _ = ev.query(W[:, 0], W[:, 1], cfg.evidence_max_age_s)
    clr = np.asarray(course.clearance(W[:, 0], W[:, 1]))
    known = (kind == ROAD) | ((kind == PAINT) & (clr >= -cfg.line_tolerance))
    ratio = float(known.mean())
    return ratio >= cfg.rear_evidence_min_ratio, ratio


@dataclass
class RecCandidate:
    id: int
    points: np.ndarray          # N x 5 (RS: x, y, a, dir, k)
    valid: bool
    reject: str
    cost: float
    reverse: float
    goal_index: int
    types: str
    rear_ratio: float = 0.0


def goals_along(route: np.ndarray, start, index: int, course, g, cfg: RecoveryCfg) -> List[Tuple[np.ndarray, int]]:
    """V4: from the closest route point, the first pose >= 25 cm ahead then every
    15 cm, body clear with 8 mm pad, at most 5."""
    k, _ = closest(route, start[0], start[1], index)
    goals, travelled, nxt = [], 0.0, cfg.first_goal_m
    last = route[k]
    for i in range(k + 1, len(route)):
        q = route[i]
        travelled += math.hypot(q[0] - last[0], q[1] - last[1])
        last = q
        if travelled >= nxt:
            if course.body_clear(tuple(q[:3]), g, cfg.goal_clearance_m):
                goals.append((q, i))
            nxt += cfg.goal_spacing_m
            if len(goals) >= cfg.max_goals:
                break
    return goals


def search(start, route: np.ndarray, index: int, course, g, cfg: RecoveryCfg, ev: Evidence,
           hits: Optional[np.ndarray]) -> Tuple[List[RecCandidate], Optional[RecCandidate]]:
    """V4 recovery.search. -> (evaluated sorted valid-first then by cost, winner)."""
    out: List[RecCandidate] = []
    for goal, gi in goals_along(route, start, index, course, g, cfg):
        for q in rs.candidates(start, tuple(goal[:3]), g.r, cfg.rs_step_m):
            P = q.points
            reverse = q.reverse_length
            parts = split_gears(P[:, :4])
            reason, ratio = '', 0.0
            if P[0, 3] != -1 or P[-1, 3] != 1:
                reason = 'Recovery must reverse first, then rejoin forward'
            elif reverse < cfg.reverse_min or reverse > cfg.reverse_max or len(parts) != 2:
                reason = 'Reverse distance / gear-change limit'
            elif q.cost > cfg.max_cost:
                reason = 'Recovery manoeuvre too long'
            elif not road_clear_many(course, g, P, cfg.line_tolerance, cfg.road_pad_m, cfg.road_step_m).all():
                reason = 'Footprint exceeds paint allowance'
            elif not obstacles_clear_many(g, P, hits, cfg.obstacle_pad_m).all():
                reason = 'LiDAR obstacle in swept footprint'
            else:
                ok, ratio = rear_evidence(ev, course, g, P, cfg)
                if not ok:
                    reason = 'Insufficient observed rear road'
            margins = np.asarray(course.body_margin_many(P[:, 0], P[:, 1], P[:, 2], g))
            outside = float(np.maximum(0.0, -margins).sum()) * cfg.outside_step_m
            out.append(RecCandidate(len(out), P, not reason, reason,
                                    q.cost + reverse * cfg.cost_reverse_weight + outside * cfg.cost_outside_weight,
                                    reverse, gi, q.types, ratio))
    out.sort(key=lambda c: (-int(c.valid), c.cost))
    return out, next((c for c in out if c.valid), None)


# --------------------------------------------------------------------------- the machine
@dataclass
class RecInputs:
    t: float
    pose: Optional[Tuple[float, float, float]]
    speed: float                           # /odom (V4 encoder)
    route: Optional[np.ndarray]            # active road piece N x 4
    problem: str = ''                      # '' = none
    hard_hold: str = ''                    # safety veto / mission hold while active
    permitted: bool = True                 # ROAD and no TRAFFIC / GATE / GEAR hold
    lidar_age: float = 1e9
    camera_age: float = 1e9
    hits: Optional[np.ndarray] = None
    ev: Optional[Evidence] = None


@dataclass
class RecOut:
    request: Optional[Dict] = None         # {speed, steer, arrived, reason} or None (inactive)
    events: List[Tuple[str, str]] = field(default_factory=list)
    evaluated: Optional[List[RecCandidate]] = None   # new search for the GUI


class Recovery:

    def __init__(self, cfg: RecoveryCfg, course, g, tracker_cfg: Optional[TrackerCfg] = None):
        self.cfg, self.course, self.g = cfg, course, g
        tc = tracker_cfg or TrackerCfg()
        self.tcfg = TrackerCfg(**{**tc.__dict__, 'parking_speed': cfg.recovery_speed})
        self.active = False
        self.state = 'IDLE'
        self.reason = ''
        self.problem_since = None
        self.attempts = 0
        self.completed = 0
        self.evaluated: List[RecCandidate] = []
        self.selected: Optional[RecCandidate] = None
        self.last_finish_pose = None
        self.route_index = 0
        self.parts: List[np.ndarray] = []
        self.current = None
        self.ctrl = Tracker(self.tcfg, g)
        self.retry_at = -1e9
        self.align_until = -1e9

    def _zero(self, reason: Optional[str] = None) -> Dict:
        if reason is not None:
            self.reason = reason
        return {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': self.reason}

    def _start(self, i: RecInputs, out: RecOut) -> None:
        if self.last_finish_pose is not None and \
                math.hypot(i.pose[0] - self.last_finish_pose[0], i.pose[1] - self.last_finish_pose[1]) > \
                self.cfg.reset_attempts_after_m:
            self.attempts = 0
        self.active, self.state = True, 'BRAKE'
        self.reason = 'Stopping before reverse-and-rejoin search'
        out.events.append(('RECOVERY BRAKE', self.reason))

    def update(self, i: RecInputs) -> RecOut:
        out = RecOut()
        c = self.cfg
        if i.route is not None and len(i.route) and i.pose is not None:
            self.route_index, _ = closest(i.route, i.pose[0], i.pose[1], self.route_index)
        if not c.enabled or i.pose is None:
            return out
        if not self.active:
            if not i.problem or not i.permitted or i.hard_hold:
                self.problem_since = None
                return out
            if self.problem_since is None:
                self.problem_since = i.t
            if i.t - self.problem_since < c.problem_dwell_s:
                return out
            self._start(i, out)
        if not i.hard_hold and not i.problem and self.state in ('WAIT', 'BRAKE'):
            self.active, self.state, self.problem_since = False, 'IDLE', None
            out.events.append(('RECOVERY CANCELLED', 'A forward candidate is feasible again; normal planning resumes'))
            return out
        if i.hard_hold:
            out.request = self._zero('Recovery paused: ' + i.hard_hold)
            return out
        if i.lidar_age > c.lidar_max_age_s or i.camera_age > c.camera_max_age_s:
            out.request = self._zero('Waiting for fresh rear camera and LiDAR')
            return out
        if self.state in ('BRAKE', 'WAIT'):
            if abs(i.speed) > c.stopped_speed:
                out.request = self._zero()
                return out
            if self.state == 'WAIT' and i.t < self.retry_at:
                out.request = self._zero()
                return out
            if self.attempts >= c.max_attempts:
                out.request = self._zero('Recovery limit reached here; clear the obstruction or reset')
                return out
            if i.route is None or not len(i.route) or i.ev is None:
                out.request = self._zero('No route / evidence for the recovery search')
                return out
            ev, win = search(i.pose, i.route, self.route_index, self.course, self.g, c, i.ev, i.hits)
            self.evaluated, out.evaluated = ev, ev
            if win is None:
                self.state, self.retry_at = 'WAIT', i.t + c.rescan_period_s
                self.reason = 'No checked reverse-and-rejoin route; rescanning every 2 s'
                out.events.append(('RECOVERY WAIT', self.reason))
                out.request = self._zero()
                return out
            self.attempts += 1
            self.selected = win
            self.parts = split_gears(win.points)          # keeps the k column (N x 5)
            self.current = self.parts.pop(0)
            self.ctrl = Tracker(self.tcfg, self.g)
            self.ctrl.gear = int(self.current[0, 3])
            self.state, self.align_until = 'ALIGN', i.t + c.align_s
            out.events.append(('RECOVERY PATH SELECTED', f'{round(win.reverse * 100)} cm reverse, then forward '
                               f'rejoin; {len(ev)} analytic connections evaluated.'))
            out.request = self._zero()
            return out
        if self.state == 'ALIGN':
            if i.t < self.align_until:
                k = float(self.current[1, 4]) if len(self.current) > 1 else 0.0
                r = self._zero()
                r['steer'] = math.atan(self.g.wb * k)
                out.request = r
                return out
            self.state = 'TRACK'
        req = self.ctrl.update(self.current[:, :4], i.pose, i.speed, i.t, True)
        req['speed'] = max(-c.recovery_speed, min(c.recovery_speed, req['speed']))
        look = self.current[self.ctrl.index:min(len(self.current), self.ctrl.index + c.lookahead_points)]
        if len(look) and (not road_clear_many(self.course, self.g, look, c.line_tolerance, c.road_pad_m,
                                              c.road_step_m).all()
                          or not obstacles_clear_many(self.g, look, i.hits, c.obstacle_pad_m).all()):
            self.state = 'BRAKE'
            out.request = self._zero('Recovery clearance changed; stopping to recompute')
            return out
        self.reason = 'Reversing slowly along checked path' if self.current[-1, 3] < 0 else \
            'Driving forward onto the mission route'
        if req['arrived']:
            if self.parts:
                self.current = self.parts.pop(0)
                self.ctrl.reset()
                self.ctrl.gear = int(self.current[0, 3])
                self.state, self.align_until = 'ALIGN', i.t + c.align_s
                out.events.append(('RECOVERY GEAR CHANGE', 'Stopped before changing from reverse to forward'))
                out.request = self._zero()
                return out
            self.active, self.state, self.problem_since = False, 'IDLE', None
            self.completed += 1
            self.last_finish_pose = tuple(i.pose)
            self.route_index = max(0, self.selected.goal_index - 2)
            out.events.append(('RECOVERY COMPLETE', 'Rejoined the forward route; local lane planner resumes'))
            out.request = {'speed': 0.0, 'steer': 0.0, 'arrived': True, 'reason': 'RECOVERY COMPLETE'}
            return out
        out.request = dict(req, arrived=False, reason=self.reason)
        return out
