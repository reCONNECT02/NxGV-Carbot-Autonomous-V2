"""Blocks 05 and 06, pure Python/numpy (no ROS). Port of V4 vehicle.js Estimator.

BLOCK 05  LocalEstimator  (steers the car; UWB never touches it)
  V4 update():
    ds = encoder distance;  odom.x += ds cos(odom.a);  odom.y += ds sin(odom.a)
    odom.a = wrap(odom.a + 0.25 * wrap(imu_yaw - odom.a))
    sigma  = hypot(sigma, |ds| * 0.025, sqrt(dt) * 0.00025);  no packet: sigma += dt * 0.004
  V4 visualUpdate(): camera road/paint edges near the car are matched to the
    prior-map boundary (clearance == 0). A damped weighted least-squares step of
    at most 1.5 mm (and at most 12 % of the solution) moves `transform`, so the
    pose can never jump. pose = odom + transform.
  Real-car differences (V4 has a perfect world-frame yaw sensor):
    * /imu/rpy yaw is relative to power-on; the offset to the track frame is
      taken when the estimator is (re)seeded: heading = imu_yaw + imu_offset.
    * if /imu/rpy goes stale, heading follows the /odom yaw increments instead.
    * the camera grid is older than "now": it is registered at the pose at the
      grid's timestamp (pose history), the correction applies to the transform.

BLOCK 06  GlobalEstimator  (route identity only, never steering)
  V4: globalPose = pose + globalOffset, scalar variance, one 2-D UWB fix per
  update gated at chi2 13.82. Here (UWB_Handoff section 11): one EKF update per
  fresh anchor RANGE, each innovation-gated (1 dof), predicted at the pose at
  that range's own measurement time, 2x2 covariance. The V4 whole-fix update
  is kept as an option (use_per_range_updates: false). A well-conditioned
  visual landmark pulls the offset toward zero exactly as V4 does.
  Added safety net (not in V4): if every range is gated out for `reject_streak`
  updates in a row, the variance is inflated once so the filter can re-acquire
  instead of locking out UWB for the rest of the run.
"""
import math
from bisect import bisect_left
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- frames
@dataclass
class TrackToVenue:
    """p_venue = R(yaw) p_track + [x, y]   (uwb.yaml track_to_venue)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @staticmethod
    def from_yaml(doc: Dict) -> 'TrackToVenue':
        t = doc['track_to_venue']
        return TrackToVenue(float(t['x_m']), float(t['y_m']), math.radians(float(t['yaw_deg'])))

    @property
    def R(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[c, -s], [s, c]])

    def to_venue(self, x: float, y: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * x - s * y + self.x, s * x + c * y + self.y

    def to_track(self, x: float, y: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.x, y - self.y
        return c * dx + s * dy, -s * dx + c * dy


# --------------------------------------------------------------------------- pose history
class PoseHistory:
    """Time-ordered (t, x, y, a) samples with interpolation."""

    def __init__(self, keep_s: float):
        self.keep = float(keep_s)
        self.t: List[float] = []
        self.p: List[Tuple[float, float, float]] = []

    def clear(self) -> None:
        self.t.clear()
        self.p.clear()

    def add(self, t: float, x: float, y: float, a: float) -> None:
        if self.t and t < self.t[-1]:
            self.clear()                      # clock went back (bag restart)
        if self.t and t == self.t[-1]:
            self.p[-1] = (x, y, a)
        else:
            self.t.append(t)
            self.p.append((x, y, a))
        cut = bisect_left(self.t, t - self.keep)
        if cut > 0:
            del self.t[:cut]
            del self.p[:cut]

    def latest(self) -> Optional[Tuple[float, Tuple[float, float, float]]]:
        return (self.t[-1], self.p[-1]) if self.t else None

    def at(self, t: float, max_gap: float) -> Optional[Tuple[float, float, float]]:
        if not self.t or t < self.t[0] - max_gap or t > self.t[-1] + max_gap:
            return None
        if t <= self.t[0]:
            return self.p[0]
        if t >= self.t[-1]:
            return self.p[-1]
        i = bisect_left(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        f = (t - t0) / max(t1 - t0, 1e-9)
        a0, a1 = self.p[i - 1], self.p[i]
        return (a0[0] + f * (a1[0] - a0[0]), a0[1] + f * (a1[1] - a0[1]),
                wrap(a0[2] + f * wrap(a1[2] - a0[2])))


# --------------------------------------------------------------------------- block 05
@dataclass
class LocalCfg:
    heading_blend: float = 0.25
    sigma_initial: float = 0.003
    sigma_idle_growth: float = 0.004
    sigma_per_distance: float = 0.025
    sigma_per_sqrt_s: float = 0.00025
    sigma_floor: float = 0.004
    sigma_visual_decay: float = 0.94
    vis_max_radius: float = 0.70
    vis_max_edge_distance: float = 0.045
    vis_gradient_step: float = 0.012
    vis_grad_min: float = 0.75
    vis_grad_max: float = 1.2
    vis_min_rows: int = 8
    vis_prior_weight: float = 0.8
    vis_weight_scale: float = 0.015
    vis_step_cap: float = 0.12
    vis_step_gain: float = 0.0015
    vis_sample_stride: int = 2

    @staticmethod
    def from_params(p) -> 'LocalCfg':
        """p(name) -> value (CarbotNode.p or a dict getter)."""
        return LocalCfg(
            float(p('heading_blend')), float(p('sigma.initial_m')),
            float(p('sigma.idle_growth_m_per_s')), float(p('sigma.per_distance')),
            float(p('sigma.per_sqrt_s')), float(p('sigma.floor_m')), float(p('sigma.visual_decay')),
            float(p('visual.max_radius_m')), float(p('visual.max_edge_distance_m')),
            float(p('visual.gradient_step_m')), float(p('visual.gradient_norm_min')),
            float(p('visual.gradient_norm_max')), int(p('visual.min_rows')),
            float(p('visual.prior_weight')), float(p('visual.weight_scale_m')),
            float(p('visual.step_cap')), float(p('visual.step_gain')),
            int(p('visual.sample_stride')))


@dataclass
class VisualResult:
    matches: int
    rank: float
    step_x: float
    step_y: float
    applied: bool


class LocalEstimator:

    def __init__(self, cfg: LocalCfg, x: float, y: float, a: float):
        self.c = cfg
        self.reset(x, y, a)

    def reset(self, x: float, y: float, a: float) -> None:
        self.ox, self.oy, self.oa = float(x), float(y), float(a)   # V4 odom (track frame)
        self.tx = self.ty = 0.0                                      # V4 transform
        self.sigma = self.c.sigma_initial
        self.distance = 0.0
        self.imu_offset: Optional[float] = None                      # imu yaw -> track heading
        self.visual_matches = 0
        self.visual_rank = 0.0
        self.visual_updates = 0

    # ---- V4 update() ----------------------------------------------------------
    def predict(self, ds: float, dt: float, imu_yaw: Optional[float], odom_dyaw: float) -> None:
        """ds: signed wheel distance since the last packet; imu_yaw: IMU yaw (rad,
        its own zero) or None if stale; odom_dyaw: /odom yaw increment (fallback)."""
        self.distance += abs(ds)
        self.ox += ds * math.cos(self.oa)
        self.oy += ds * math.sin(self.oa)
        if imu_yaw is not None:
            if self.imu_offset is None:
                self.imu_offset = wrap(self.oa - imu_yaw)
            target = wrap(imu_yaw + self.imu_offset)
            self.oa = wrap(self.oa + self.c.heading_blend * wrap(target - self.oa))
        else:
            self.oa = wrap(self.oa + odom_dyaw)
        self.sigma = math.hypot(self.sigma, abs(ds) * self.c.sigma_per_distance,
                                math.sqrt(max(dt, 0.0)) * self.c.sigma_per_sqrt_s)

    def idle(self, dt: float) -> None:
        """No fresh sensor packet (V4: sigma += dt * 0.004)."""
        self.sigma += dt * self.c.sigma_idle_growth

    def realign_imu(self) -> None:
        """Take a new IMU-to-track offset at the next IMU sample (after a reset)."""
        self.imu_offset = None

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.ox + self.tx, self.oy + self.ty, self.oa

    @property
    def odom(self) -> Tuple[float, float, float]:
        return self.ox, self.oy, self.oa

    # ---- V4 visualUpdate() ----------------------------------------------------
    def edge_rows(self, kind: np.ndarray, grown: np.ndarray, lx: np.ndarray, ly: np.ndarray,
                  pose: Tuple[float, float, float], course) -> np.ndarray:
        """kind/grown: (rows, cols) with row along +x, col along +y (LocalGrid);
        lx: row centre x (rows,), ly: col centre y (cols,) in base_link.
        -> (N, 3) rows of (gx, gy, d)."""
        c = self.c
        rows, cols = kind.shape
        stride = max(1, c.vis_sample_stride)
        R = np.arange(1, rows - 1)
        C = np.arange(1, cols - 1, stride)
        rr, cc = np.meshgrid(R, C, indexing='ij')
        rr, cc = rr.ravel(), cc.ravel()
        px, py = lx[rr], ly[cc]
        sel = (grown[rr, cc] > 0) & (np.hypot(px, py) <= c.vis_max_radius)
        rr, cc, px, py = rr[sel], cc[sel], px[sel], py[sel]
        out = []
        cx, sx = math.cos(pose[2]), math.sin(pose[2])
        for dr, dc in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            nr, nc = rr + dr, cc + dc
            paint = kind[nr, nc] == 2
            if not paint.any():
                continue
            ex = (px[paint] + lx[nr[paint]]) / 2
            ey = (py[paint] + ly[nc[paint]]) / 2
            wx = pose[0] + ex * cx - ey * sx
            wy = pose[1] + ex * sx + ey * cx
            d = course.clearance(wx, wy)
            near = np.abs(d) <= c.vis_max_edge_distance
            if not near.any():
                continue
            wx, wy, d = wx[near], wy[near], d[near]
            gx, gy = course.gradient(wx, wy, c.vis_gradient_step)
            nrm = np.hypot(gx, gy)
            good = (nrm >= c.vis_grad_min) & (nrm <= c.vis_grad_max)
            if good.any():
                out.append(np.stack([gx[good], gy[good], d[good]], axis=1))
        return np.concatenate(out) if out else np.zeros((0, 3))

    def visual_update(self, kind, grown, lx, ly, pose_at_stamp, course) -> VisualResult:
        c = self.c
        rows = self.edge_rows(kind, grown, lx, ly, pose_at_stamp, course)
        self.visual_matches = len(rows)
        if len(rows) < c.vis_min_rows:
            self.visual_rank = 0.0
            return VisualResult(len(rows), 0.0, 0.0, 0.0, False)
        gx, gy, d = rows[:, 0], rows[:, 1], rows[:, 2]
        w = np.minimum(1.0, c.vis_weight_scale / np.maximum(0.001, np.abs(d)))
        xx = c.vis_prior_weight + float(np.sum(w * gx * gx))
        xy = float(np.sum(w * gx * gy))
        yy = c.vis_prior_weight + float(np.sum(w * gy * gy))
        bx = -float(np.sum(w * gx * d))
        by = -float(np.sum(w * gy * d))
        det = xx * yy - xy * xy
        dx = (yy * bx - xy * by) / det
        dy = (xx * by - xy * bx) / det
        scale = min(c.vis_step_cap, c.vis_step_gain / max(1e-5, math.hypot(dx, dy)))
        self.tx += dx * scale
        self.ty += dy * scale
        self.visual_rank = 4 * det / (xx + yy) ** 2
        self.sigma = max(c.sigma_floor, self.sigma * c.sigma_visual_decay)
        self.visual_updates += 1
        return VisualResult(len(rows), self.visual_rank, dx * scale, dy * scale, True)


# --------------------------------------------------------------------------- block 06
@dataclass
class GlobalCfg:
    initial_variance: float = 1e-4
    growth_per_s: float = 8e-6
    growth_per_m: float = 2e-5
    range_sigma: float = 0.05
    range_gate_chi2: float = 6.63
    position_gate_chi2: float = 13.82
    fix_sigma: float = 0.05
    landmark_min_rank: float = 0.25
    landmark_sigma: float = 0.025
    reject_streak: int = 15
    reacquire_variance: float = 0.04
    max_variance: float = 1.0

    @staticmethod
    def from_params(p) -> 'GlobalCfg':
        return GlobalCfg(
            float(p('initial_variance_m2')), float(p('growth_per_s_m2')), float(p('growth_per_m_m2')),
            float(p('range_sigma_m')), float(p('range_gate_chi2')), float(p('position_gate_chi2')),
            float(p('fix_sigma_m')), float(p('visual_landmark_min_rank')),
            float(p('visual_landmark_sigma_m')), int(p('reacquire.reject_streak')),
            float(p('reacquire.inflate_variance_m2')), float(p('max_variance_m2')))


@dataclass
class RangeUpdate:
    anchor: str
    innovation: float
    mahalanobis2: float
    accepted: bool
    gain: float


@dataclass
class GlobalEstimator:
    c: GlobalCfg
    off: np.ndarray = field(default_factory=lambda: np.zeros(2))
    P: np.ndarray = field(default_factory=lambda: np.eye(2))
    accepted: int = 0
    rejected: int = 0
    streak: int = 0
    reacquires: int = 0
    residual: float = 0.0
    last_gain: float = 0.0
    last_accepted: bool = False

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.off = np.zeros(2)
        self.P = np.eye(2) * self.c.initial_variance
        self.streak = 0

    @property
    def sigma(self) -> float:
        return math.sqrt(max(float(np.trace(self.P)) / 2.0, 0.0))

    def predict(self, dt: float, ds: float) -> None:
        q = self.c.growth_per_s * max(dt, 0.0) + self.c.growth_per_m * abs(ds)
        self.P = self.P + np.eye(2) * q
        self._cap()

    def _cap(self) -> None:
        tr = float(np.trace(self.P)) / 2.0
        if tr > self.c.max_variance:
            self.P *= self.c.max_variance / tr

    def _reject(self) -> None:
        self.rejected += 1
        self.streak += 1
        if self.streak >= self.c.reject_streak:
            self.P = self.P + np.eye(2) * self.c.reacquire_variance
            self._cap()
            self.reacquires += 1
            self.streak = 0

    def range_update(self, anchor: str, local_xy: Tuple[float, float], anchor_xy: Tuple[float, float],
                     r: float, t2v: TrackToVenue) -> Optional[RangeUpdate]:
        """One flattened range to one anchor. local_xy: block-05 pose (track) at the
        range's measurement time. Returns None if geometry is degenerate."""
        g = np.array(local_xy) + self.off
        v = t2v.R @ g + np.array([t2v.x, t2v.y])
        diff = v - np.array(anchor_xy)
        h = float(np.hypot(*diff))
        if h < 0.05:
            return None
        H = (diff / h) @ t2v.R                     # dh/doffset, shape (2,)
        H = H.reshape(1, 2)
        Rr = self.c.range_sigma ** 2
        S = float((H @ self.P @ H.T)[0, 0]) + Rr
        innov = r - h
        m2 = innov * innov / S
        self.residual = abs(innov)
        if m2 > self.c.range_gate_chi2:
            self.last_accepted = False
            self.last_gain = 0.0
            self._reject()
            return RangeUpdate(anchor, innov, m2, False, 0.0)
        K = (self.P @ H.T) / S                     # (2, 1)
        A = np.eye(2) - K @ H
        self.off = self.off + K.ravel() * innov
        self.P = A @ self.P @ A.T + K @ K.T * Rr   # Joseph form
        self.accepted += 1
        self.streak = 0
        self.last_accepted = True
        self.last_gain = float(np.linalg.norm(K))
        return RangeUpdate(anchor, innov, m2, True, self.last_gain)

    def fix_update(self, local_xy: Tuple[float, float], fix_track_xy: Tuple[float, float]) -> bool:
        """V4 whole-fix update (use_per_range_updates: false)."""
        g = np.array(local_xy) + self.off
        e = np.array(fix_track_xy) - g
        R = np.eye(2) * self.c.fix_sigma ** 2
        S = self.P + R
        m2 = float(e @ np.linalg.solve(S, e))
        self.residual = float(np.hypot(*e))
        if m2 > self.c.position_gate_chi2:
            self.last_accepted = False
            self.last_gain = 0.0
            self._reject()
            return False
        K = self.P @ np.linalg.inv(S)
        A = np.eye(2) - K
        self.off = self.off + K @ e
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.accepted += 1
        self.streak = 0
        self.last_accepted = True
        self.last_gain = float(np.trace(K) / 2)
        return True

    def landmark_update(self, rank: float) -> bool:
        """V4: a well-conditioned visual match pulls the offset toward zero."""
        if rank <= self.c.landmark_min_rank:
            return False
        R = np.eye(2) * self.c.landmark_sigma ** 2
        K = self.P @ np.linalg.inv(self.P + R)
        A = np.eye(2) - K
        self.off = A @ self.off
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True

    def pose(self, local: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return local[0] + float(self.off[0]), local[1] + float(self.off[1]), local[2]


def covariance_ellipse(P: np.ndarray, k: float = 2.0) -> Tuple[float, float, float]:
    """(semi-major, semi-minor, angle) of the k-sigma ellipse of a 2x2 covariance."""
    vals, vecs = np.linalg.eigh(P)
    vals = np.maximum(vals, 0.0)
    i = int(np.argmax(vals))
    return (k * math.sqrt(vals[i]), k * math.sqrt(vals[1 - i]),
            math.atan2(vecs[1, i], vecs[0, i]))


def odom_increment(prev: Optional[Tuple[float, float, float]], cur: Tuple[float, float, float]
                   ) -> Tuple[float, float]:
    """Signed distance and yaw change between two /odom poses. The base
    servo_controller integrates x += v cos(yaw) dt, so projecting the step onto
    the heading recovers the signed wheel distance exactly."""
    if prev is None:
        return 0.0, 0.0
    dx, dy = cur[0] - prev[0], cur[1] - prev[1]
    a = cur[2]
    return dx * math.cos(a) + dy * math.sin(a), wrap(cur[2] - prev[2])


def grid_axes(rows: int, cols: int, res: float, x0: float, y0: float
              ) -> Tuple[np.ndarray, np.ndarray]:
    """LocalGrid cell centres: row -> x, col -> y."""
    return x0 + (np.arange(rows) + 0.5) * res, y0 + (np.arange(cols) + 0.5) * res


def summarise_updates(updates: Sequence[RangeUpdate]) -> Dict[str, float]:
    ok = [u for u in updates if u.accepted]
    return {'n': len(updates), 'accepted': len(ok),
            'mean_abs_innov': float(np.mean([abs(u.innovation) for u in updates])) if updates else 0.0}
