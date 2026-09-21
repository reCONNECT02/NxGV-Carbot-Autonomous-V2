"""Calibration step 11: fit uwb.yaml track_to_venue from UWB ranges (pure numpy).

Unknowns (x, y, yaw) of  p_venue = R(yaw) p_track + [x, y].
Data: samples (tag position in TRACK frame at the range time, anchor id,
flattened corrected range). Two sources:
  lap     tag position = block-05 local pose (+ tag lever arm) while the car is
          pushed / driven slowly around the track
  points  the car is parked at 2+ named map poses (start, checkpoints)
Solver: yaw grid search (every 10 deg) + Gauss-Newton with a Huber loss on
the range residuals |R p + t - A| - r. Ranges are used directly (not 2-D fixes),
so a missing anchor or a multipath burst costs nothing.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np


@dataclass
class Sample:
    tx: float          # tag position in track frame
    ty: float
    anchor: str
    r: float           # flattened, offset-corrected range (m)


@dataclass
class FitResult:
    x: float
    y: float
    yaw: float
    rms_m: float           # RMS of inlier range residuals
    inlier_frac: float
    n: int
    median_abs_m: float
    track_extent_m: float  # size of the area covered by the samples


def _residuals(theta, P, A, r):
    x, y, yaw = theta
    c, s = math.cos(yaw), math.sin(yaw)
    vx = c * P[:, 0] - s * P[:, 1] + x
    vy = s * P[:, 0] + c * P[:, 1] + y
    dx, dy = vx - A[:, 0], vy - A[:, 1]
    h = np.maximum(np.hypot(dx, dy), 1e-6)
    res = h - r
    ux, uy = dx / h, dy / h
    dyaw = ux * (-s * P[:, 0] - c * P[:, 1]) + uy * (c * P[:, 0] - s * P[:, 1])
    J = np.stack([ux, uy, dyaw], axis=1)
    return res, J


def _gauss_newton(theta, P, A, r, huber, iters=30):
    theta = np.array(theta, float)
    for _ in range(iters):
        res, J = _residuals(theta, P, A, r)
        a = np.abs(res)
        w = np.where(a <= huber, 1.0, huber / np.maximum(a, 1e-9))
        JW = J * w[:, None]
        H = JW.T @ J + np.eye(3) * 1e-6
        step = np.linalg.solve(H, -JW.T @ res)
        theta += step
        theta[2] = math.atan2(math.sin(theta[2]), math.cos(theta[2]))
        if np.linalg.norm(step[:2]) < 1e-5 and abs(step[2]) < 1e-6:
            break
    res, _ = _residuals(theta, P, A, r)
    a = np.abs(res)
    cost = float(np.sum(np.where(a <= huber, 0.5 * a * a, huber * (a - 0.5 * huber))))
    return theta, cost, res


def fit_track_to_venue(samples: Sequence[Sample], anchors: Dict[str, Tuple[float, float]],
                       huber_m: float = 0.10, inlier_m: float = 0.25,
                       yaw_step_deg: float = 10.0) -> FitResult:
    """anchors: id -> (x, y) venue."""
    rows = [s for s in samples if s.anchor in anchors]
    if len(rows) < 6:
        raise ValueError(f'need at least 6 ranges, got {len(rows)}')
    P = np.array([[s.tx, s.ty] for s in rows])
    A = np.array([anchors[s.anchor] for s in rows])
    r = np.array([s.r for s in rows])
    if len({s.anchor for s in rows}) < 2:
        raise ValueError('ranges to at least 2 anchors are needed')
    centre_a = np.mean(np.array(list(anchors.values())), axis=0)
    centre_p = P.mean(axis=0)
    best = None
    for yaw in np.radians(np.arange(0.0, 360.0, yaw_step_deg)):
        c, s = math.cos(yaw), math.sin(yaw)
        t0 = centre_a - np.array([c * centre_p[0] - s * centre_p[1], s * centre_p[0] + c * centre_p[1]])
        theta, cost, res = _gauss_newton((t0[0], t0[1], yaw), P, A, r, huber_m)
        if best is None or cost < best[1]:
            best = (theta, cost, res)
    theta, _, res = best
    inl = np.abs(res) <= inlier_m
    rms = float(np.sqrt(np.mean(res[inl] ** 2))) if inl.any() else float('inf')
    ext = float(np.hypot(*(P.max(axis=0) - P.min(axis=0))))
    return FitResult(float(theta[0]), float(theta[1]), float(theta[2]), rms,
                     float(inl.mean()), len(rows), float(np.median(np.abs(res))), ext)


def tag_position(pose: Tuple[float, float, float], lever: Tuple[float, float]) -> Tuple[float, float]:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + lever[0] * c - lever[1] * s, pose[1] + lever[0] * s + lever[1] * c


def start_pose_error(fix_venue: Tuple[float, float], t2v, start_pose: Tuple[float, float, float],
                     lever: Tuple[float, float]) -> float:
    """Distance between the raw UWB fix and where the tag should be if the car
    sits on the map start pose (race preflight 'start pose matches map')."""
    fx, fy = t2v.to_track(*fix_venue)
    ex, ey = tag_position(start_pose, lever)
    return math.hypot(fx - ex, fy - ey)


def samples_to_dicts(samples: List[Sample]) -> List[Dict]:
    return [{'tx': s.tx, 'ty': s.ty, 'anchor': s.anchor, 'r': s.r} for s in samples]


def samples_from_dicts(rows: List[Dict]) -> List[Sample]:
    return [Sample(float(d['tx']), float(d['ty']), str(d['anchor']), float(d['r'])) for d in rows]
