"""Calibration math for wizard steps 3 (intrinsics) and 4 (extrinsics + IPM).

No ROS imports. The command-line tools (calib_intrinsics, calib_extrinsics)
and the phase-8 wizard both call these functions, so the numbers are the same
whichever way a student runs the step.

Step 3  A hand-held chessboard (calibration_steps.yaml step 3 target) is
        shown to one camera in many poses. Both lens models are fitted:
          plumb_bob    (standard, k1 k2 p1 p2 k3)
          equidistant  (fisheye, k1..k4)
        and the fisheye model is kept only if it is clearly better
        (rms < prefer_fisheye_ratio x pinhole rms). Nobody has to know in
        advance whether a lens counts as "fisheye".

Step 4  Chessboards lie flat on the floor at measured positions in base_link
        (step 4 target.boards). With the intrinsics known, solvePnP gives each
        camera's pose. OpenCV's corner order is ambiguous (180 deg turn, and a
        mirrored order gives a camera "under the floor"), so every ordering is
        tried and the physically valid one closest to the nominal (CAD) mount wins.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .camera_model import Intrinsics, Mount, pixels_to_ground


# --------------------------------------------------------------------------- chessboard
def board_points(cols: int, rows: int, square: float) -> np.ndarray:
    """Board-frame corners (N,3), OpenCV row-major order (x along cols)."""
    g = np.zeros((rows * cols, 3), np.float32)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return g


def detect_chessboard(img: np.ndarray, cols: int, rows: int, fast: bool = False) -> Optional[np.ndarray]:
    """Inner corners (N,2) float32 in OpenCV order, or None.

    fast=True (live capture on the RDK): classic detector with FAST_CHECK on a
    half-size image, corners refined on the full image. Otherwise the more
    robust sector-based detector first (offline / extrinsics)."""
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern = (int(cols), int(rows))
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 1e-3)
    if fast:
        half = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        ok, c = cv2.findChessboardCorners(half, pattern, flags=flags)
        if not ok:
            return None
        c = cv2.cornerSubPix(gray, (c * 2.0 + 0.5).astype(np.float32), (5, 5), (-1, -1), crit)
        return c.reshape(-1, 2).astype(np.float32)
    if hasattr(cv2, 'findChessboardCornersSB'):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, c = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        if ok:
            return c.reshape(-1, 2).astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if not ok:
        return None
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), crit)
    return c.reshape(-1, 2).astype(np.float32)


def sharpness(img: np.ndarray, corners: np.ndarray) -> float:
    """Variance of the Laplacian inside the board's bounding box (motion blur check)."""
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    x, y, w, h = cv2.boundingRect(corners.astype(np.float32))
    roi = gray[max(y, 0):y + h, max(x, 0):x + w]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0


# --------------------------------------------------------------------------- view selection
@dataclass
class ViewCollector:
    """Keeps only views that add something: new position, size or tilt.
    Also tracks which thirds of the image have been covered by corners,
    because distortion is only measured where the board has been."""
    width: int
    height: int
    min_novelty: float = 0.12
    views: List[np.ndarray] = field(default_factory=list)
    features: List[np.ndarray] = field(default_factory=list)
    coverage: np.ndarray = field(default_factory=lambda: np.zeros((3, 3), int))

    @staticmethod
    def feature(c: np.ndarray, w: int, h: int, cols: int, rows: int) -> np.ndarray:
        diag = math.hypot(w, h)
        centre = c.mean(axis=0)
        grid = c.reshape(rows, cols, 2)
        top = np.linalg.norm(grid[0, -1] - grid[0, 0])
        bottom = np.linalg.norm(grid[-1, -1] - grid[-1, 0])
        left = np.linalg.norm(grid[-1, 0] - grid[0, 0])
        right = np.linalg.norm(grid[-1, -1] - grid[0, -1])
        size = math.sqrt(cv2.contourArea(cv2.convexHull(c.astype(np.float32)))) / diag
        return np.array([centre[0] / w, centre[1] / h, 2 * size,
                         (top - bottom) / max(top + bottom, 1e-6),
                         (left - right) / max(left + right, 1e-6)])

    def offer(self, corners: np.ndarray, cols: int, rows: int) -> bool:
        f = self.feature(corners, self.width, self.height, cols, rows)
        if self.features and min(np.linalg.norm(f - g) for g in self.features) < self.min_novelty:
            return False
        self.views.append(corners)
        self.features.append(f)
        for u, v in corners:
            self.coverage[min(int(3 * v / self.height), 2), min(int(3 * u / self.width), 2)] += 1
        return True

    def coverage_cells(self) -> int:
        return int((self.coverage > 0).sum())

    def coverage_text(self) -> str:
        return '\n'.join(' '.join('##' if n else '..' for n in row) for row in self.coverage)


# --------------------------------------------------------------------------- intrinsics
@dataclass
class IntrinsicsFit:
    intr: Intrinsics
    rms_px: float
    per_view_px: List[float]


def fit_pinhole(obj: Sequence[np.ndarray], img: Sequence[np.ndarray], size: Tuple[int, int]
                ) -> IntrinsicsFit:
    rms, K, D, rv, tv = cv2.calibrateCamera([o.astype(np.float32) for o in obj],
                                            [i.reshape(-1, 1, 2).astype(np.float32) for i in img],
                                            size, None, None)
    per = []
    for o, i, r, t in zip(obj, img, rv, tv):
        p, _ = cv2.projectPoints(o.astype(np.float64), r, t, K, D)
        per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i) ** 2, axis=1)))))
    return IntrinsicsFit(Intrinsics(size[0], size[1], K, D.ravel()[:5], 'plumb_bob', 'calibrated'),
                         float(rms), per)


def fit_fisheye(obj: Sequence[np.ndarray], img: Sequence[np.ndarray], size: Tuple[int, int]
                ) -> Optional[IntrinsicsFit]:
    o = [x.reshape(-1, 1, 3).astype(np.float64) for x in obj]
    i = [x.reshape(-1, 1, 2).astype(np.float64) for x in img]
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-7)
    for flags in (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
                  | cv2.fisheye.CALIB_CHECK_COND,
                  cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW):
        try:
            rms, K, D, rv, tv = cv2.fisheye.calibrate(o, i, size, K, D, flags=flags, criteria=crit)
            break
        except cv2.error:
            K, D = np.zeros((3, 3)), np.zeros((4, 1))
    else:
        return None
    per = []
    for oo, ii, r, t in zip(o, i, rv, tv):
        p, _ = cv2.fisheye.projectPoints(oo, r, t, K, D)
        per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - ii.reshape(-1, 2)) ** 2, axis=1)))))
    return IntrinsicsFit(Intrinsics(size[0], size[1], K, D.ravel(), 'equidistant', 'calibrated'),
                         float(rms), per)


def calibrate_intrinsics(views: Sequence[np.ndarray], cols: int, rows: int, square: float,
                         size: Tuple[int, int], prefer_fisheye_ratio: float = 0.85,
                         drop_worst_px: float = 2.0) -> Dict:
    """Fit both models, drop views that fit badly (>drop_worst_px) once, choose.
    Returns {'chosen': IntrinsicsFit, 'pinhole': fit, 'fisheye': fit|None, 'used_views': [...]}"""
    obj = board_points(cols, rows, square)
    idx = list(range(len(views)))
    for _ in range(2):
        pin = fit_pinhole([obj] * len(idx), [views[k] for k in idx], size)
        keep = [k for k, e in zip(idx, pin.per_view_px) if e <= drop_worst_px]
        if len(keep) == len(idx) or len(keep) < 6:
            break
        idx = keep
    pin = fit_pinhole([obj] * len(idx), [views[k] for k in idx], size)
    fish = fit_fisheye([obj] * len(idx), [views[k] for k in idx], size)
    chosen = fish if (fish is not None and fish.rms_px < prefer_fisheye_ratio * pin.rms_px) else pin
    chosen.intr.meta.update({'rms_px': round(chosen.rms_px, 4), 'views': len(idx),
                             'pinhole_rms_px': round(pin.rms_px, 4),
                             'fisheye_rms_px': None if fish is None else round(fish.rms_px, 4),
                             'hfov_deg': round(chosen.intr.hfov_deg(), 2)})
    return {'chosen': chosen, 'pinhole': pin, 'fisheye': fish, 'used_views': idx}


# --------------------------------------------------------------------------- extrinsics
@dataclass
class FloorBoard:
    """A chessboard lying on the floor. centre = centre of the INNER-corner grid
    (= the centre of the printed board), yaw = direction of the board's
    `cols` axis in base_link."""
    name: str
    roles: List[str]
    cols: int
    rows: int
    square: float
    cx: float
    cy: float
    yaw: float

    @classmethod
    def from_yaml(cls, d: Dict) -> 'FloorBoard':
        roles = d.get('roles') or [d['role']]
        cols, rows = d['inner_corners']
        return cls(d.get('name', roles[0]), list(roles), int(cols), int(rows), float(d['square_m']),
                   float(d['centre_m'][0]), float(d['centre_m'][1]), math.radians(float(d['yaw_deg'])))

    def ground_points(self) -> np.ndarray:
        """(N,3) base_link corners in OpenCV row-major order (identity ordering)."""
        u = (np.arange(self.cols) - (self.cols - 1) / 2.0) * self.square
        v = (np.arange(self.rows) - (self.rows - 1) / 2.0) * self.square
        uu, vv = np.meshgrid(u, v)                   # rows x cols
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = self.cx + uu * c - vv * s
        y = self.cy + uu * s + vv * c
        return np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])

    def orderings(self) -> List[Tuple[str, np.ndarray]]:
        g = self.ground_points().reshape(self.rows, self.cols, 3)
        return [('identity', g.reshape(-1, 3)), ('rot180', g[::-1, ::-1].reshape(-1, 3)),
                ('flip_rows', g[::-1, :].reshape(-1, 3)), ('flip_cols', g[:, ::-1].reshape(-1, 3))]


@dataclass
class MountFit:
    role: str
    mount: Mount
    ordering: str
    reproj_px: float
    ground_rms_m: float
    ground_max_m: float
    nominal_dpos_m: float
    nominal_dang_deg: float
    ground_err_per_corner: np.ndarray


def solve_mount(role: str, intr: Intrinsics, corners: np.ndarray, board: FloorBoard,
                nominal: Mount) -> Optional[MountFit]:
    norm = intr.normalize(corners).astype(np.float64)
    ok_px = np.all(np.isfinite(norm), axis=1)
    if ok_px.sum() < 6:
        return None
    best = None
    for name, g in board.orderings():
        obj = g[ok_px].astype(np.float64)
        img = norm[ok_px].reshape(-1, 1, 2)
        try:
            flag = getattr(cv2, 'SOLVEPNP_IPPE', cv2.SOLVEPNP_ITERATIVE)
            ok, rvec, tvec = cv2.solvePnP(obj, img, np.eye(3), None, flags=flag)
            if not ok:
                continue
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, np.eye(3), None, rvec, tvec) \
                if hasattr(cv2, 'solvePnPRefineLM') else (rvec, tvec)
        except cv2.error:
            continue
        R, _ = cv2.Rodrigues(rvec)                 # base -> optical
        C = (-R.T @ tvec).ravel()
        if C[2] <= 0:
            continue                               # camera under the floor: mirrored ordering
        depth = (obj @ R.T + tvec.ravel())[:, 2]
        if np.any(depth <= 0):
            continue
        mount = Mount.from_r_base_opt(R.T, C)
        dpos, dang = mount.difference(nominal)
        uv, _ok = intr.project(mount.base_to_optical(obj))
        reproj = float(np.sqrt(np.nanmean(np.sum((uv - corners[ok_px]) ** 2, axis=1))))
        gnd, gok = pixels_to_ground(intr, mount, corners[ok_px])
        err = np.linalg.norm(gnd - obj[:, :2], axis=1)
        err = np.where(gok, err, np.nan)
        fit = MountFit(role, mount, name, reproj, float(np.sqrt(np.nanmean(err ** 2))),
                       float(np.nanmax(err)), dpos, dang, err)
        score = dpos + math.radians(dang) * 0.3    # metres-equivalent distance to nominal
        if best is None or score < best[0]:
            best = (score, fit)
    return None if best is None else best[1]


def seam_error(fits: Dict[str, MountFit], intr: Dict[str, Intrinsics],
               corners: Dict[Tuple[str, str], np.ndarray], boards: Sequence[FloorBoard]) -> Dict:
    """For boards seen by two cameras: distance between where each camera puts
    the same corner on the ground. corners[(board, role)] = detected pixels."""
    out = {}
    for b in boards:
        seen = [r for r in b.roles if (b.name, r) in corners and r in fits]
        for i in range(len(seen)):
            for j in range(i + 1, len(seen)):
                a, c = seen[i], seen[j]
                ga, oka = pixels_to_ground(intr[a], fits[a].mount, corners[(b.name, a)])
                gc, okc = pixels_to_ground(intr[c], fits[c].mount, corners[(b.name, c)])
                if len(ga) != len(gc):
                    continue
                # corner order of two cameras may differ: compare sorted-by-nearest
                d = np.linalg.norm(ga[:, None, :] - gc[None, :, :], axis=2)
                e = np.nanmin(d, axis=1)
                out[f'{b.name}:{a}-{c}'] = float(np.nanmax(e))
    return out
