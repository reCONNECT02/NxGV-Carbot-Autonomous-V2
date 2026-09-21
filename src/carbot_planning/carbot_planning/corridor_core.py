"""BLOCK 09 core (no ROS): find the lane we should follow (V4 guidance.js).

corridor(): walk the active route 0.9 m ahead (every 5th 1 cm point). At each
sample look sideways for a PAINT cell whose inner neighbour is ROAD, on both
sides. Only a plausible lane width (23-38 cm) counts. The measured lane middle
minus the prior-map middle, clamped to +-2.5 cm and halved, is that sample's
shift; the MEDIAN shift moves the whole guide (a common offset avoids a
sawtooth target at camera occlusion gaps). The car need not sit on the exact
geometric centreline: the route keeps its useful offset through tight turns.

branch_check(): near a roundabout exit the route takes (V4: west and north)
and in the lane change, the intended branch must be seen as road, and the
UWB-aided course position must agree with the local one; otherwise HOLD.
"""
import math
from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np

from .evidence import PAINT, ROAD, Evidence


@dataclass
class CorridorCfg:
    horizon_m: float = 0.9
    sample_every: int = 5
    edge_min: float = 0.055
    edge_max: float = 0.26
    edge_step: float = 0.009
    inside_probe: float = 0.027
    edge_correction: float = 0.0045
    width_min: float = 0.23
    width_max: float = 0.38
    prior_max: float = 0.32
    prior_step: float = 0.005
    shift_clamp: float = 0.025
    shift_gain: float = 0.5
    guide_points: int = 120
    live_max_age: float = 0.30
    evidence_max_age: float = 2.0
    camera_min_observed: int = 3
    lane_locked_min_observed: int = 3
    # branch
    near_radius: float = 0.35
    lane_change_margin: float = 0.1
    look_min: float = 0.16
    look_max: float = 0.40
    min_points: int = 5
    min_seen_ratio: float = 0.35
    agree_min: float = 0.25
    agree_sigma_mult: float = 3.0
    visual_rank_min: float = 0.25

    @classmethod
    def from_params(cls, p) -> 'CorridorCfg':
        return cls(horizon_m=float(p('horizon_m')), sample_every=int(p('sample_every')),
                   edge_min=float(p('edge_search_min_m')), edge_max=float(p('edge_search_max_m')),
                   edge_step=float(p('edge_search_step_m')), inside_probe=float(p('inside_probe_m')),
                   edge_correction=float(p('edge_correction_m')), width_min=float(p('lane_width_min_m')),
                   width_max=float(p('lane_width_max_m')), prior_max=float(p('prior_search_max_m')),
                   prior_step=float(p('prior_search_step_m')), shift_clamp=float(p('shift_clamp_m')),
                   shift_gain=float(p('shift_gain')), guide_points=int(p('guide_points')),
                   live_max_age=float(p('live_max_age_s')), evidence_max_age=float(p('evidence_max_age_s')),
                   camera_min_observed=int(p('camera_corridor_min_observed')),
                   lane_locked_min_observed=int(p('lane_locked_min_observed')),
                   near_radius=float(p('branch.near_radius_m')),
                   lane_change_margin=float(p('branch.lane_change_margin_m')),
                   look_min=float(p('branch.look_min_m')), look_max=float(p('branch.look_max_m')),
                   min_points=int(p('branch.min_points')), min_seen_ratio=float(p('branch.min_seen_ratio')),
                   agree_min=float(p('branch.agree_min_m')), agree_sigma_mult=float(p('branch.agree_sigma_mult')),
                   visual_rank_min=float(p('branch.visual_rank_min')))


def _frange(a: float, b: float, step: float) -> np.ndarray:
    """JS `for (d = a; d <= b; d += step)` values (float accumulation kept)."""
    out, d = [], a
    while d <= b:
        out.append(d)
        d += step
    return np.asarray(out)


def _frange_lt(a: float, b: float, step: float) -> np.ndarray:
    out, d = [], a
    while d < b:
        out.append(d)
        d += step
    return np.asarray(out)


def closest(path: np.ndarray, x: float, y: float, frm: int = 0):
    """V4 vehicle.js closest(): search from-12 .. from+130."""
    i0 = max(0, frm - 12)
    i1 = min(len(path), frm + 130)
    if i1 <= i0:
        return frm, float('inf')
    d = (path[i0:i1, 0] - x) ** 2 + (path[i0:i1, 1] - y) ** 2
    k = int(np.argmin(d))
    return i0 + k, float(math.sqrt(d[k]))


@dataclass
class CorridorResult:
    guide: np.ndarray                     # N x 4 shifted route
    centres: List[tuple] = field(default_factory=list)   # (x, y, a, width)
    left_edge: List[tuple] = field(default_factory=list)
    right_edge: List[tuple] = field(default_factory=list)
    offset: float = 0.0
    observed: int = 0
    mode: str = 'CAMERA ROAD + ROUTE BRANCH'
    first: int = 0


class CorridorFinder:

    def __init__(self, cfg: CorridorCfg):
        self.cfg = cfg
        self._d = _frange(cfg.edge_min, cfg.edge_max, cfg.edge_step)
        self._prior = _frange_lt(0.02, cfg.prior_max, cfg.prior_step)
        self.index = 0

    def reset(self) -> None:
        self.index = 0

    def corridor(self, path: np.ndarray, pose, ev: Evidence, course) -> CorridorResult:
        c = self.cfg
        first, _ = closest(path, pose[0], pose[1], self.index)
        self.index = first
        # samples: every 5th point within 0.9 m of travel (V4 loop order)
        samples, travelled, prev = [], 0.0, None
        for j in range(first, len(path)):
            q = path[j]
            if prev is not None:
                travelled += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
            if travelled >= c.horizon_m:
                break
            if j % c.sample_every != 0 and j != first:
                continue
            samples.append(j)
        shifts, centres, left, right = [], [], [], []
        if samples:
            Q = path[samples]
            nx, ny = -np.sin(Q[:, 2]), np.cos(Q[:, 2])
            D = self._d
            edges = {}
            for sign in (-1, 1):
                # candidate edge points and their inside probes, all at once
                wx = Q[:, 0, None] + sign * D[None, :] * nx[:, None]
                wy = Q[:, 1, None] + sign * D[None, :] * ny[:, None]
                ix = wx - sign * c.inside_probe * nx[:, None]
                iy = wy - sign * c.inside_probe * ny[:, None]
                k_edge, _ = ev.query(wx.ravel(), wy.ravel(), c.evidence_max_age)
                k_in, _ = ev.query(ix.ravel(), iy.ravel(), c.evidence_max_age)
                hit = ((k_edge == PAINT) & (k_in == ROAD)).reshape(wx.shape)
                has = hit.any(1)
                idx = hit.argmax(1)
                edges[sign] = (has, sign * (D[idx] - c.edge_correction), wx[np.arange(len(Q)), idx],
                               wy[np.arange(len(Q)), idx])
            # prior-map lane edges (first d where the map clearance becomes negative)
            P = self._prior
            expected = {}
            for sign in (-1, 1):
                px = Q[:, 0, None] + sign * P[None, :] * nx[:, None]
                py = Q[:, 1, None] + sign * P[None, :] * ny[:, None]
                neg = course.clearance(px, py) < 0
                anyn = neg.any(1)
                kk = neg.argmax(1)
                # V4 loop ends at d >= 0.32 when nothing is found: the value after the loop
                after = P[-1] + c.prior_step
                expected[sign] = sign * np.where(anyn, P[kk], after)
            for n in range(len(Q)):
                hl, dl, lxw, lyw = (edges[-1][0][n], edges[-1][1][n], edges[-1][2][n], edges[-1][3][n])
                hr, dr, rxw, ryw = (edges[1][0][n], edges[1][1][n], edges[1][2][n], edges[1][3][n])
                if not (hl and hr):
                    continue
                width = dr - dl
                if not (c.width_min < width < c.width_max):
                    continue
                mid = (dl + dr) / 2
                prior_mid = (expected[-1][n] + expected[1][n]) / 2
                shift = float(np.clip(mid - prior_mid, -c.shift_clamp, c.shift_clamp)) * c.shift_gain
                shifts.append(shift)
                q = Q[n]
                centres.append((q[0] + mid * nx[n], q[1] + mid * ny[n], q[2], width))
                # -1 side = right of the route direction, +1 = left
                right.append((lxw, lyw))
                left.append((rxw, ryw))
        ss = sorted(shifts)
        offset = ss[len(ss) // 2] if ss else 0.0
        seg = path[first:min(len(path), first + c.guide_points)].copy()
        seg[:, 0] -= offset * np.sin(seg[:, 2])
        seg[:, 1] += offset * np.cos(seg[:, 2])
        age = ev.live_age()
        mode = ('REMEMBERED CORRIDOR' if age > c.live_max_age else
                'CAMERA CORRIDOR' if len(ss) >= c.camera_min_observed else 'CAMERA ROAD + ROUTE BRANCH')
        return CorridorResult(seg, centres, left, right, offset, len(ss), mode, first)


@dataclass
class BranchResult:
    hold: bool
    label: str
    reason: str = ''


def branch_check(pose, guide: np.ndarray, ev: Evidence, course, exits: Sequence[str],
                 global_offset=(0.0, 0.0), global_sigma: float = 0.0, visual_rank: float = 1.0,
                 cfg: CorridorCfg = CorridorCfg()) -> BranchResult:
    """V4 guidance.branchCheck. `exits` = roundabout exits the route takes."""
    x, y = pose[0], pose[1]
    near = any(math.hypot(x - course.exits[e][0], y - course.exits[e][1]) < cfg.near_radius
               for e in exits if e in course.exits)
    if 'lane_change' in course.areas:
        near = near or course.in_area('lane_change', x, y, cfg.lane_change_margin)
    if not near:
        return BranchResult(False, 'Following current corridor')
    if len(guide):
        d = np.hypot(guide[:, 0] - x, guide[:, 1] - y)
        sel = (d >= cfg.look_min) & (d <= cfg.look_max)        # V4: skip d < .16 || d > .40
        total = int(sel.sum())
        seen = int((ev.query(guide[sel, 0], guide[sel, 1], 2.0)[0] == ROAD).sum()) if total else 0
    else:
        total = seen = 0
    agreement = math.hypot(*global_offset) < max(cfg.agree_min, cfg.agree_sigma_mult * global_sigma)
    if total > cfg.min_points and seen / total < cfg.min_seen_ratio:
        return BranchResult(True, 'Branch unconfirmed', 'Intended branch has insufficient local road evidence')
    if not agreement and visual_rank < cfg.visual_rank_min:
        return BranchResult(True, 'Route ambiguous',
                            'Course location conflicts with local branch; waiting for landmark agreement')
    return BranchResult(False, 'Route + visible opening agree' if agreement else
                        'Visual landmark confirms route; UWB disagrees')
