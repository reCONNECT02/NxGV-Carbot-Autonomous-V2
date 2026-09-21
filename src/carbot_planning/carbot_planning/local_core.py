"""BLOCK 10 core (no ROS): choose the next movement (V4 local-planner.js).

Nine lateral offsets of the corridor guide are rolled out 55 x 1 cm with the
bicycle model and the servo lag (the same pure-pursuit law the tracker uses).
Every 2nd step the swept body is checked against the prior-map road (the
paint allowance applies only in the relaxed pass), the camera footprint
evidence (paint under the body blocks in the first 20 cm), and - in the
relaxed pass, as in V4 - LiDAR obstacles. The lowest-cost valid candidate
wins: grey = tried, blue = selected. If every strict candidate fails and
the paint allowance is > 0, nine more are tried with it (ids 9..17).

Differences from V4, on purpose:
  * live camera support is looked up with the pose at the camera stamp
    (V4 used the current pose: in the simulator both are the same frame);
  * obstacle_check_strict (YAML, default false = V4) can add the LiDAR check to
    the strict pass as well.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .corridor_core import closest
from .evidence import PAINT, ROAD, Evidence
from .route_core import bicycle


@dataclass
class LocalCfg:
    offsets: Sequence[float] = (-0.06, -0.04, -0.02, -0.008, 0.0, 0.008, 0.02, 0.04, 0.06)
    steps: int = 55
    ds: float = 0.01
    lookahead: float = 0.095
    speed_min: float = 0.055
    speed_max: float = 0.12
    check_every: int = 2
    body_pad: float = 0.005
    c_tracking: float = 900.0
    c_unseen_memory: float = 0.006
    c_unseen: float = 0.014
    c_clearance: float = 0.00012
    c_clearance_floor: float = 0.003
    c_low_margin: float = 0.03
    c_low_margin_weight: float = 140.0
    c_offset: float = 25.0
    c_outside: float = 1000.0
    c_paint: float = 0.07
    c_unknown: float = 0.003
    paint_block_steps: int = 20
    paint_block_cells: int = 5
    relaxed_retry: bool = True
    end_points: int = 5
    end_distance: float = 0.035
    footprint_inset: float = 0.012
    footprint_step: float = 0.035
    crossable_pad: float = 0.025
    evidence_max_age: float = 3.0
    memory_cost_max_age: float = 25.0
    obstacle_strict: bool = False
    obstacle_pad: float = 0.02
    steer_lag: float = 0.12
    steer_rate: float = math.pi
    line_tolerance: float = 0.05

    @classmethod
    def from_params(cls, p) -> 'LocalCfg':
        return cls(offsets=tuple(float(v) for v in p('offsets_m')), steps=int(p('rollout_steps')),
                   ds=float(p('rollout_ds_m')), lookahead=float(p('lookahead_m')),
                   speed_min=float(p('rollout_speed_min_mps')), speed_max=float(p('rollout_speed_max_mps')),
                   check_every=int(p('check_every')), body_pad=float(p('body_pad_m')),
                   c_tracking=float(p('cost.tracking')), c_unseen_memory=float(p('cost.unseen_memory')),
                   c_unseen=float(p('cost.unseen')), c_clearance=float(p('cost.clearance')),
                   c_clearance_floor=float(p('cost.clearance_floor_m')), c_low_margin=float(p('cost.low_margin_m')),
                   c_low_margin_weight=float(p('cost.low_margin_weight')), c_offset=float(p('cost.offset')),
                   c_outside=float(p('cost.outside')), c_paint=float(p('cost.paint')),
                   c_unknown=float(p('cost.unknown')), paint_block_steps=int(p('paint_block_steps')),
                   paint_block_cells=int(p('paint_block_cells')), relaxed_retry=bool(p('relaxed_retry')),
                   end_points=int(p('end_points')), end_distance=float(p('end_distance_m')),
                   footprint_inset=float(p('footprint.inset_m')), footprint_step=float(p('footprint.step_m')),
                   crossable_pad=float(p('footprint.crossable_pad_m')),
                   evidence_max_age=float(p('evidence_max_age_s')),
                   memory_cost_max_age=float(p('memory_cost_max_age_s')),
                   obstacle_strict=bool(p('obstacle_check_strict')), obstacle_pad=float(p('obstacle_pad_m')),
                   steer_lag=float(p('vehicle.steering_lag_s')),
                   steer_rate=math.radians(float(p('vehicle.steering_rate_deg_s'))),
                   line_tolerance=float(p('limits.line_tolerance_m')))


@dataclass
class Candidate:
    id: int
    offset: float
    points: np.ndarray          # N x 4 (x, y, a, dir)
    cost: float
    valid: bool
    relaxed: bool
    seen: int
    road_blocked: int
    paint_blocked: int
    obstacle_blocked: int
    min_clear: float
    command_steer: float
    reject: str = ''


def _footprint_samples(g, inset: float, step: float) -> np.ndarray:
    pts, x = [], -g.rear + inset
    while x <= g.front - inset:
        for y in (-g.w / 2 + inset, 0.0, g.w / 2 - inset):
            pts.append((x, y))
        x += step
    return np.asarray(pts)


class LocalPlanner:

    def __init__(self, cfg: LocalCfg, g):
        self.cfg = cfg
        self.g = g
        self.fp = _footprint_samples(g, cfg.footprint_inset, cfg.footprint_step)

    # -------------------------------------------------------------- checks
    def _obstacle_blocked(self, P: np.ndarray, hits: Optional[np.ndarray]) -> np.ndarray:
        """V4 recovery.obstaclesClear for poses P (K x 3); hits = world M x 2."""
        if hits is None or not len(hits):
            return np.zeros(len(P), bool)
        g, pad = self.g, self.cfg.obstacle_pad
        c, s = np.cos(P[:, 2])[:, None], np.sin(P[:, 2])[:, None]
        dx, dy = hits[None, :, 0] - P[:, 0, None], hits[None, :, 1] - P[:, 1, None]
        qx, qy = dx * c + dy * s, -dx * s + dy * c
        inside = (qx > -g.rear - pad) & (qx < g.front + pad) & (np.abs(qy) < g.w / 2 + pad)
        return inside.any(1)

    def _footprint(self, P: np.ndarray, ev: Evidence, course):
        """V4 guidance.footprintEvidence -> (road, paint, unknown) per pose."""
        S = self.fp
        c, s = np.cos(P[:, 2])[:, None], np.sin(P[:, 2])[:, None]
        wx = P[:, 0, None] + S[None, :, 0] * c - S[None, :, 1] * s
        wy = P[:, 1, None] + S[None, :, 0] * s + S[None, :, 1] * c
        kind, _ = ev.query(wx.ravel(), wy.ravel(), self.cfg.evidence_max_age)
        kind = kind.reshape(wx.shape)
        paint = kind == PAINT
        if paint.any():
            cross = course.crossable(wx[paint], wy[paint], self.cfg.crossable_pad)
            p2 = paint.copy()
            p2[paint] = ~cross
            paint = p2
        unknown = kind == 0
        return paint.sum(1), unknown.sum(1)

    # -------------------------------------------------------------- rollouts
    def rollouts(self, start, reference: np.ndarray, steer0: float, speed_meas: float,
                 ev: Evidence, course, hits: Optional[np.ndarray], tolerance: float = 0.0) -> List[Candidate]:
        cfg, g = self.cfg, self.g
        out = []
        speed = max(cfg.speed_min, min(cfg.speed_max, abs(speed_meas)))
        ds = cfg.ds
        dt = ds / speed
        lag = cfg.steer_lag
        n_ref = len(reference)
        for cid, offset in enumerate(cfg.offsets):
            px, py, pa = start
            pts = [(px, py, pa, 1)]
            steer = steer0
            index = 0
            initial = 0.0
            checks = []                                    # (n, x, y, a, hit error)
            for n in range(cfg.steps):
                index, err = closest(reference, px, py, index)
                j = index
                while j < n_ref - 1 and math.hypot(reference[j, 0] - px, reference[j, 1] - py) < cfg.lookahead:
                    j += 1
                q = reference[j]
                tx, ty = q[0] - offset * math.sin(q[2]), q[1] + offset * math.cos(q[2])
                c, s = math.cos(pa), math.sin(pa)
                rx, ry = (tx - px) * c + (ty - py) * s, -(tx - px) * s + (ty - py) * c
                desired = max(-g.max_steer, min(g.max_steer,
                                                math.atan(g.wb * 2 * ry / max(0.002, rx * rx + ry * ry))))
                if n == 0:
                    initial = desired
                rate = max(-cfg.steer_rate, min(cfg.steer_rate, (desired - steer) / lag))
                step = (desired - steer) * (1 - math.exp(-dt / lag))
                steer += max(-abs(rate * dt), min(abs(rate * dt), step))
                px, py, pa = bicycle(px, py, pa, ds, math.tan(steer) / g.wb)
                pts.append((px, py, pa, 1))
                if n % cfg.check_every == 0:
                    checks.append((n, px, py, pa, err))
                if index > n_ref - cfg.end_points and \
                        math.hypot(px - reference[-1, 0], py - reference[-1, 1]) < cfg.end_distance:
                    break
            C = np.asarray(checks)
            P = C[:, 1:4]
            ns, errs = C[:, 0], C[:, 4]
            margin = course.body_margin_many(P[:, 0], P[:, 1], P[:, 2], g)
            min_clear = float(min(1.0, margin.min()))
            road_bad = ~course.road_clear_many(P[:, 0], P[:, 1], P[:, 2], g, tolerance, cfg.body_pad)
            live = ev.support(P[:, 0], P[:, 1])            # V4: current grid, no age test (safety owns staleness)
            mem = ev.memory(P[:, 0], P[:, 1], cfg.memory_cost_max_age, check_uncertainty=False)
            if (tolerance > 0 or cfg.obstacle_strict):
                obst = self._obstacle_blocked(P, hits)
            else:
                obst = np.zeros(len(P), bool)
            paint, unknown = self._footprint(P, ev, course)
            paint_bad = (ns < cfg.paint_block_steps) & (paint > cfg.paint_block_cells) & (tolerance == 0)
            cost = float(np.sum(paint * cfg.c_paint + unknown * cfg.c_unknown))
            cost += float(np.sum(errs ** 2 * cfg.c_tracking
                                 + np.where(live == 1, 0.0, np.where(mem == ROAD, cfg.c_unseen_memory, cfg.c_unseen))
                                 + cfg.c_clearance / np.maximum(cfg.c_clearance_floor, margin)
                                 + np.maximum(0.0, cfg.c_low_margin - margin) * cfg.c_low_margin_weight))
            cost += abs(offset) * cfg.c_offset + max(0.0, -min_clear) * cfg.c_outside
            rb, pb, ob = int(road_bad.sum()), int(paint_bad.sum()), int(obst.sum())
            blocked = rb + pb + ob
            reasons = [r for r, v in (('Swept body exceeds road allowance', rb),
                                      ('Camera paint overlaps footprint', pb),
                                      ('LiDAR obstacle in footprint', ob)) if v]
            out.append(Candidate(cid, offset, np.asarray(pts), cost, blocked == 0, tolerance > 0,
                                 int((live == 1).sum()), rb, pb, ob, min_clear, initial, '; '.join(reasons)))
        out.sort(key=lambda q: (not q.valid, q.cost))
        return out

    def candidates(self, start, reference, steer0, speed_meas, ev, course, hits) -> List[Candidate]:
        """V4 candidates(): strict pass, then the relaxed pass if nothing is valid."""
        strict = self.rollouts(start, reference, steer0, speed_meas, ev, course, hits, 0.0)
        if any(q.valid for q in strict) or not (self.cfg.line_tolerance > 0) or not self.cfg.relaxed_retry:
            return strict
        relaxed = self.rollouts(start, reference, steer0, speed_meas, ev, course, hits, self.cfg.line_tolerance)
        for q in relaxed:
            q.id += len(self.cfg.offsets)
        for q in strict:
            q.reject = q.reject or 'Strict pass'
        both = strict + relaxed
        both.sort(key=lambda q: (not q.valid, q.cost))
        return both
