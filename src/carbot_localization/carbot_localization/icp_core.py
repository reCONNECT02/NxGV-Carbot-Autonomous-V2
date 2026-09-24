"""ICP-style correction of the LOCAL estimate from the front camera's road edges (pure numpy, no ROS).

What exists (V4 vehicle.js Estimator.visualUpdate, estimator_core.LocalEstimator.visual_update): camera
road/paint edges are matched to the PRIOR MAP boundary, translation only, at most 1.5 mm per frame; the
heading comes from the IMU. This module adds a second, independent correction that needs no prior map:

  * EdgeMemory keeps the last few seconds of camera edge points in the ODOM frame (wheel + IMU dead
    reckoning, never corrected: same rule as block 04 LocalMemory, V4 `estimate.odom`), each with its
    time and the travelled distance at observation. A point is only used while
        age <= mem_max_age_s   and   unc_base + unc_per_m * travel_since <= unc_limit     (V4 guidance.evidence)
  * register() is a robust point-to-LINE ICP (3 degrees of freedom: dx, dy, dtheta in the car frame):
    the current frame's edge points against the remembered edges seen at least mem_min_age_s ago. Straight
    lane edges only constrain sideways position and heading; along-track motion is unobservable, so the
    normal equations are solved with an eigenvalue cut-off (degenerate directions get NO correction).
  * IcpCorrector turns the registration into a SMOOTH, BOUNDED step: it applies only the share of the
    measured residual that accumulated since its last update (interval / mean point age), times a gain,
    clamped per update (max_step_m, max_step_rad) and in total (max_total_m, max_total_rad). Poor fits
    (few inliers, high residual, implausible size) are rejected and counted. So the estimate NEVER jumps.

Nothing here reads UWB. The correction is applied by the caller to LocalEstimator (position transform +
heading trim); UWB stays in block 06 and never reaches the steering pose. Off by default (icp.enabled).
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- configuration
@dataclass
class IcpCfg:
    enabled: bool
    every_n_grids: int            # run the registration on every n-th road grid (grids arrive ~8 Hz)
    add_every_n_grids: int        # store every n-th grid's edges in the memory
    max_radius_m: float           # only edge cells this close to the car
    sample_stride: int            # grid column stride when extracting edges
    max_points: int               # cap on the current frame's points
    normal_k: int                 # neighbours used for the local line normal
    mem_max_age_s: float          # V4 drive max age
    mem_min_age_s: float          # younger points are (nearly) the same frame: no information
    mem_max_points: int           # cap on the reference set (voxel-thinned)
    mem_voxel_m: float
    unc_base_m: float             # V4: base + per_m * travel <= limit
    unc_per_m: float
    unc_limit_m: float
    max_iter: int
    pair_max_dist_m: float        # correspondences further apart are outliers
    trim_fraction: float          # keep the best share of the correspondences
    huber_m: float                # residual scale of the robust weight
    min_inliers: int
    min_inlier_ratio: float
    max_rms_m: float
    min_ref_points: int
    eig_min_ratio: float          # normal-matrix eigenvalues below this x the largest are unobservable (no step)
    rot_arm_m: float              # lever arm that makes dtheta comparable with metres in the normal matrix
    max_fit_translation_m: float  # a registration larger than this is implausible: rejected
    max_fit_rotation_rad: float
    gain: float                   # share of the (time-scaled) residual applied per update
    max_step_m: float             # per-update translation cap (the pose never jumps)
    max_step_rad: float
    max_total_m: float            # total ICP correction bound since the last reset
    max_total_rad: float
    min_speed_mps: float          # no update below this measured speed (nothing has moved)

    @staticmethod
    def from_params(p) -> 'IcpCfg':
        """p(name) -> value (CarbotNode.p or a dict getter); every key must exist."""
        g = lambda k: p('icp.' + k)   # noqa: E731
        return IcpCfg(
            bool(g('enabled')), int(g('every_n_grids')), int(g('memory.add_every_n_grids')),
            float(g('max_radius_m')), int(g('sample_stride')), int(g('max_points')), int(g('normal_k')),
            float(g('memory.max_age_s')), float(g('memory.min_age_s')), int(g('memory.max_points')),
            float(g('memory.voxel_m')), float(g('memory.uncertainty_base_m')),
            float(g('memory.uncertainty_per_m')), float(g('memory.uncertainty_limit_m')),
            int(g('max_iter')), float(g('pair_max_dist_m')), float(g('trim_fraction')), float(g('huber_m')),
            int(g('min_inliers')), float(g('min_inlier_ratio')), float(g('max_rms_m')),
            int(g('min_ref_points')), float(g('eig_min_ratio')), float(g('rot_arm_m')),
            float(g('max_fit_translation_m')), float(g('max_fit_rotation_rad')), float(g('gain')),
            float(g('max_step_m')), float(g('max_step_rad')), float(g('max_total_m')),
            float(g('max_total_rad')), float(g('min_speed_mps')))


# --------------------------------------------------------------------------- edge extraction
def edge_points(kind: np.ndarray, grown: np.ndarray, lx: np.ndarray, ly: np.ndarray,
                max_radius: float, stride: int) -> np.ndarray:
    """(N, 2) edge points in base_link: midpoint between a grown-ROAD cell and a touching PAINT cell.

    kind/grown: (rows, cols), row along +x, col along +y (LocalGrid); lx (rows,), ly (cols,) cell centres.
    Same selection as LocalEstimator.edge_rows (paint cell next to grown road within max_radius)."""
    rows, cols = kind.shape
    R = np.arange(1, rows - 1)
    C = np.arange(1, cols - 1, max(1, int(stride)))
    rr, cc = np.meshgrid(R, C, indexing='ij')
    rr, cc = rr.ravel(), cc.ravel()
    px, py = lx[rr], ly[cc]
    sel = (grown[rr, cc] > 0) & (np.hypot(px, py) <= max_radius)
    rr, cc, px, py = rr[sel], cc[sel], px[sel], py[sel]
    out = []
    for dr, dc in ((0, -1), (0, 1), (-1, 0), (1, 0)):
        nr, nc = rr + dr, cc + dc
        paint = kind[nr, nc] == 2
        if paint.any():
            out.append(np.stack([(px[paint] + lx[nr[paint]]) / 2, (py[paint] + ly[nc[paint]]) / 2], axis=1))
    return np.concatenate(out) if out else np.zeros((0, 2))


def sqdist(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(N, M) squared distances via one matrix product (3x cheaper than broadcasting differences)."""
    d2 = (a * a).sum(axis=1)[:, None] + (b * b).sum(axis=1)[None, :] - 2.0 * (a @ b.T)
    return np.maximum(d2, 0.0)


def thin(pts: np.ndarray, max_points: int) -> np.ndarray:
    """Deterministic even subsample down to max_points (order kept)."""
    n = len(pts)
    if n <= max_points:
        return pts
    idx = np.linspace(0, n - 1, max_points).astype(int)
    return pts[idx]


def normals(pts: np.ndarray, k: int) -> np.ndarray:
    """(N, 2) unit normals of the local line through each point's k nearest neighbours (PCA; sign arbitrary)."""
    n = len(pts)
    if n == 0:
        return np.zeros((0, 2))
    k = max(2, min(int(k), n))
    d2 = sqdist(pts, pts)
    nb = np.argpartition(d2, k - 1, axis=1)[:, :k]
    nbp = pts[nb]                                        # (n, k, 2)
    c = nbp - nbp.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', c, c)
    w, v = np.linalg.eigh(cov)                           # ascending
    return v[:, :, 0]                                    # smallest-variance direction = the line normal


# --------------------------------------------------------------------------- memory of edge points
class EdgeMemory:
    """Edge points (odom frame) with time and travelled distance; a private, low-rate sibling of block 04."""

    def __init__(self, capacity: int = 4000):
        self.cap = int(capacity)
        self.x = np.zeros(0)
        self.y = np.zeros(0)
        self.nx = np.zeros(0)
        self.ny = np.zeros(0)
        self.t = np.zeros(0)
        self.dist = np.zeros(0)

    def __len__(self) -> int:
        return len(self.t)

    def clear(self) -> None:
        self.__init__(self.cap)

    def add(self, pts_base: np.ndarray, nrm_base: np.ndarray, pose: Tuple[float, float, float],
            t: float, distance: float) -> None:
        """pts/normals in base_link, pose = ODOM pose (x, y, a) at time t, distance = travelled path length."""
        if not len(pts_base):
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        ox = pose[0] + pts_base[:, 0] * c - pts_base[:, 1] * s
        oy = pose[1] + pts_base[:, 0] * s + pts_base[:, 1] * c
        nx = nrm_base[:, 0] * c - nrm_base[:, 1] * s
        ny = nrm_base[:, 0] * s + nrm_base[:, 1] * c
        n = len(ox)
        self.x = np.concatenate([self.x, ox])
        self.y = np.concatenate([self.y, oy])
        self.nx = np.concatenate([self.nx, nx])
        self.ny = np.concatenate([self.ny, ny])
        self.t = np.concatenate([self.t, np.full(n, float(t))])
        self.dist = np.concatenate([self.dist, np.full(n, float(distance))])
        if len(self.t) > self.cap:
            cut = len(self.t) - self.cap
            for name in ('x', 'y', 'nx', 'ny', 't', 'dist'):
                setattr(self, name, getattr(self, name)[cut:])

    def expire(self, now: float, max_age: float) -> None:
        keep = self.t >= now - max_age
        if not keep.all():
            for name in ('x', 'y', 'nx', 'ny', 't', 'dist'):
                setattr(self, name, getattr(self, name)[keep])

    def reference(self, pose: Tuple[float, float, float], now: float, distance: float, cfg: IcpCfg):
        """Usable remembered points in the base_link frame of `pose` (the current ODOM pose):
        (pts (M,2), normals (M,2), mean_age_s). Age window + V4 motion-uncertainty rule + voxel thinning."""
        if not len(self.t):
            return np.zeros((0, 2)), np.zeros((0, 2)), 0.0
        age = now - self.t
        unc = cfg.unc_base_m + cfg.unc_per_m * np.abs(distance - self.dist)
        ok = (age >= cfg.mem_min_age_s) & (age <= cfg.mem_max_age_s) & (unc <= cfg.unc_limit_m)
        if not ok.any():
            return np.zeros((0, 2)), np.zeros((0, 2)), 0.0
        dx, dy = self.x[ok] - pose[0], self.y[ok] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        bx, by = c * dx + s * dy, -s * dx + c * dy            # odom -> base_link
        bnx, bny = c * self.nx[ok] + s * self.ny[ok], -s * self.nx[ok] + c * self.ny[ok]
        near = np.hypot(bx, by) <= cfg.max_radius_m + cfg.pair_max_dist_m + 0.3
        bx, by, bnx, bny, ag = bx[near], by[near], bnx[near], bny[near], age[ok][near]
        if not len(bx):
            return np.zeros((0, 2)), np.zeros((0, 2)), 0.0
        key = np.round(bx / cfg.mem_voxel_m).astype(np.int64) * 100003 + np.round(by / cfg.mem_voxel_m).astype(np.int64)
        _, first = np.unique(key, return_index=True)
        first.sort()
        if len(first) > cfg.mem_max_points:
            first = first[np.linspace(0, len(first) - 1, cfg.mem_max_points).astype(int)]
        pts = np.stack([bx[first], by[first]], axis=1)
        nrm = np.stack([bnx[first], bny[first]], axis=1)
        return pts, nrm, float(ag[first].mean())


# --------------------------------------------------------------------------- registration
@dataclass
class IcpResult:
    ok: bool
    reason: str
    dx: float = 0.0               # correction in the CAR frame that maps the current points onto the memory
    dy: float = 0.0
    dth: float = 0.0
    rms_m: float = 0.0
    inliers: int = 0
    ratio: float = 0.0
    constrained: int = 0          # observable degrees of freedom (0..3); straight edges give 2 (sideways + heading)
    iterations: int = 0
    ref_points: int = 0
    mean_age_s: float = 0.0


def register(src: np.ndarray, ref: np.ndarray, ref_n: np.ndarray, cfg: IcpCfg) -> IcpResult:
    """Robust point-to-line ICP of `src` (current frame, base_link) onto `ref` (+ line normals), 3 DOF."""
    n = len(src)
    if n < cfg.min_inliers or len(ref) < cfg.min_ref_points:
        return IcpResult(False, f'too few points (cur {n}, ref {len(ref)})', ref_points=len(ref))
    L = cfg.rot_arm_m
    T = np.zeros(3)                                       # dx, dy, dth (accumulated, applied to src each iteration)
    cur = src.copy()
    res = IcpResult(False, 'no iteration', ref_points=len(ref))
    for it in range(1, cfg.max_iter + 1):
        d2 = sqdist(cur, ref)
        j = d2.argmin(axis=1)
        dist = np.sqrt(d2[np.arange(n), j])
        q, nn = ref[j], ref_n[j]
        r = ((cur - q) * nn).sum(axis=1)                  # signed distance of each point to the matched line
        ok = dist <= cfg.pair_max_dist_m
        if ok.sum() < cfg.min_inliers:
            return IcpResult(False, f'few correspondences ({int(ok.sum())})', iterations=it, ref_points=len(ref))
        # trimmed: keep the best `trim_fraction` of the valid pairs
        order = np.argsort(np.abs(r) + np.where(ok, 0.0, 1e9))
        keep = np.zeros(n, bool)
        keep[order[:max(cfg.min_inliers, int(cfg.trim_fraction * ok.sum()))]] = True
        keep &= ok
        w = np.where(keep, np.minimum(1.0, cfg.huber_m / np.maximum(np.abs(r), 1e-9)), 0.0)
        perp = np.stack([-cur[:, 1], cur[:, 0]], axis=1)
        J = np.stack([nn[:, 0], nn[:, 1], (nn * perp).sum(axis=1) / L], axis=1)     # unknowns: dx, dy, L*dth
        H = (J * w[:, None]).T @ J
        g = (J * w[:, None]).T @ r
        ev, V = np.linalg.eigh(H)
        observable = ev > cfg.eig_min_ratio * max(float(ev.max()), 1e-12)
        inv = np.where(observable, 1.0 / np.maximum(ev, 1e-12), 0.0)                # unobservable: no step
        delta = -(V * inv) @ (V.T @ g)
        step = np.array([delta[0], delta[1], delta[2] / L])
        c, s = math.cos(step[2]), math.sin(step[2])
        cur = np.stack([c * cur[:, 0] - s * cur[:, 1] + step[0], s * cur[:, 0] + c * cur[:, 1] + step[1]], axis=1)
        # compose: T <- step o T  (rotation about the car origin, translation after)
        cT, sT = math.cos(step[2]), math.sin(step[2])
        T = np.array([cT * T[0] - sT * T[1] + step[0], sT * T[0] + cT * T[1] + step[1], T[2] + step[2]])
        inl = keep & ok
        rms = float(np.sqrt(np.mean(r[inl] ** 2))) if inl.any() else 1.0
        res = IcpResult(True, '', float(T[0]), float(T[1]), float(wrap(T[2])), rms, int(inl.sum()),
                        float(inl.sum()) / n, int(observable.sum()), it, len(ref))
        if np.hypot(step[0], step[1]) < 2e-4 and abs(step[2]) < 2e-5:      # converged to 0.2 mm / 0.001 deg
            break
    # residual after the last step, on the final correspondences
    d2 = sqdist(cur, ref)
    j = d2.argmin(axis=1)
    dist = np.sqrt(d2[np.arange(n), j])
    r = ((cur - ref[j]) * ref_n[j]).sum(axis=1)
    inl = dist <= cfg.pair_max_dist_m
    if inl.sum() >= 1:
        cut = np.quantile(np.abs(r[inl]), cfg.trim_fraction)
        inl &= np.abs(r) <= max(cut, 1e-9)
    res.inliers = int(inl.sum())
    res.ratio = float(inl.sum()) / n
    res.rms_m = float(np.sqrt(np.mean(r[inl] ** 2))) if inl.any() else 1.0
    return res


# --------------------------------------------------------------------------- corrector
@dataclass
class IcpStep:
    applied: bool
    reason: str
    dx: float = 0.0               # applied, car frame
    dy: float = 0.0
    dth: float = 0.0
    result: Optional[IcpResult] = None


class IcpCorrector:
    """Memory + registration + slew/bounds. The caller applies the returned car-frame step to its estimate."""

    def __init__(self, cfg: IcpCfg):
        self.c = cfg
        self.mem = EdgeMemory()
        self.n_grids = 0
        self.last_t: Optional[float] = None
        self.net = np.zeros(2)                            # net position correction applied so far (track frame)
        self.total_th = 0.0                               # net heading trim applied so far
        self.attempts = 0
        self.accepted = 0
        self.rejected = 0
        self.last_reason = ''
        self.last_result: Optional[IcpResult] = None
        self.max_step_m = 0.0
        self.max_step_rad = 0.0

    def reset(self) -> None:
        self.mem.clear()
        self.n_grids = 0
        self.last_t = None
        self.net = np.zeros(2)
        self.total_th = 0.0

    def on_grid(self, kind, grown, lx, ly, odom_pose: Tuple[float, float, float], t: float,
                distance: float, speed: float, heading: Optional[float] = None) -> Optional[IcpStep]:
        """One camera road grid. odom_pose = UNCORRECTED odom pose at the grid stamp; heading = the corrected
        heading (odom + trim), used to rotate the applied step into the track frame for the total bound.
        Returns a step (possibly not applied, with the reason) when a registration was attempted, else None."""
        c = self.c
        self.n_grids += 1
        do_reg = self.n_grids % c.every_n_grids == 0
        do_add = self.n_grids % c.add_every_n_grids == 0
        if not (do_reg or do_add):
            return None                       # extraction is the per-grid cost: only do it on grids that are used
        pts = thin(edge_points(kind, grown, lx, ly, c.max_radius_m, c.sample_stride), c.max_points)
        step = None
        if do_reg:
            step = self._register(pts, odom_pose, t, distance, speed,
                                  odom_pose[2] if heading is None else heading)
        if do_add and len(pts) >= c.min_inliers:
            self.mem.add(pts, normals(pts, c.normal_k), odom_pose, t, distance)
        self.mem.expire(t, c.mem_max_age_s)
        return step

    def _register(self, pts, odom_pose, t, distance, speed, heading) -> IcpStep:
        c = self.c
        self.attempts += 1
        if abs(speed) < c.min_speed_mps:
            return self._reject('standing still')
        ref, ref_n, mean_age = self.mem.reference(odom_pose, t, distance, c)
        r = register(pts, ref, ref_n, c)
        r.mean_age_s = mean_age
        self.last_result = r
        if not r.ok:
            return self._reject(r.reason, r)
        if r.inliers < c.min_inliers or r.ratio < c.min_inlier_ratio:
            return self._reject(f'weak fit ({r.inliers} inliers, {r.ratio:.2f})', r)
        if r.rms_m > c.max_rms_m:
            return self._reject(f'residual {r.rms_m * 1000:.1f} mm', r)
        if math.hypot(r.dx, r.dy) > c.max_fit_translation_m or abs(r.dth) > c.max_fit_rotation_rad:
            return self._reject('implausible size', r)
        interval = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        share = 1.0 if interval <= 0.0 else min(1.0, interval / max(mean_age, 1e-3))
        dx, dy, dth = (v * share * c.gain for v in (r.dx, r.dy, r.dth))
        m = math.hypot(dx, dy)
        if m > c.max_step_m:
            dx, dy = dx * c.max_step_m / m, dy * c.max_step_m / m
        dth = max(-c.max_step_rad, min(c.max_step_rad, dth))
        # total bound: never let the ICP alone move the estimate further than max_total_* (a step that
        # brings the net correction back toward zero is always allowed)
        ch, sh = math.cos(heading), math.sin(heading)
        cand = self.net + np.array([ch * dx - sh * dy, sh * dx + ch * dy])
        if np.hypot(*cand) > c.max_total_m and np.hypot(*cand) > np.hypot(*self.net):
            dx = dy = 0.0
            cand = self.net
        if abs(self.total_th + dth) > c.max_total_rad and abs(self.total_th + dth) > abs(self.total_th):
            dth = 0.0
        if dx == 0.0 and dy == 0.0 and dth == 0.0:
            return self._reject('total bound reached', r)
        self.net = cand
        self.total_th += dth
        self.accepted += 1
        self.max_step_m = max(self.max_step_m, math.hypot(dx, dy))
        self.max_step_rad = max(self.max_step_rad, abs(dth))
        self.last_reason = ''
        return IcpStep(True, '', dx, dy, dth, r)

    def _reject(self, why: str, r: Optional[IcpResult] = None) -> IcpStep:
        self.rejected += 1
        self.last_reason = why
        return IcpStep(False, why, result=r)
