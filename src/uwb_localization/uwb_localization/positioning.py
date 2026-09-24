"""UWB 2-D positioning -- Haffiz's method (pure Python, no ROS).

Source: tools/uwb/haffiz/turtle_uwb_visualizer.py (+ _noEKF.py), committed unchanged
as the reference. This module is the SAME math, made configurable and shared by
every consumer, so the car, the wizard, the CLIs and the sandbox all compute the
identical position:

  uwb_ranges node          -> /carbot/uwb/raw_fix (solver) + /carbot/uwb/position (filtered)
  calibration step 10      -> Verify stage (median of the filtered fixes)
  calibration step 11      -> UWB lap for the map fit (map_builder.fit_rigid)
  tools/uwb/uwb_xy.py, uwb_turtle.py, record_lap, sandbox

Pipeline per tag report (common.yaml `/**` uwb_positioning):
  1. ranges  = every link in the report (offset-corrected, height-flattened;
               with range_offset_m 0 and z 0 these are Haffiz's raw R values)
  2. solver  linear   (Haffiz default) closed-form trilateration
             nlls     (Haffiz noEKF)   soft_l1 least squares from the last solution
             pairwise (old uwb_xy.py)  average of pairwise circle intersections
  3. filter  cv_kf          (Haffiz default) 2-D constant-velocity Kalman filter,
                            state [x, y, vx, vy], Mahalanobis gate 16 (~4 sigma)
             moving_average (Haffiz noEKF)   mean of the last N solver fixes
             none           solver output as is

Faithfulness: with filter cv_kf, reacquire_after_rejects 0 and the same time
steps, the output equals Haffiz's ExtendedKalmanFilter2D.process() to machine
precision (test_positioning.py runs his class, copied verbatim, side by side).
Two deliberate differences, both YAML:
  * time step = the report's estimated measurement time (latency-compensated,
    uwb_core.RangeProcessor), not the callback's wall clock;
  * reacquire_after_rejects (> 0): after that many CONSECUTIVE gated fixes the
    filter re-initialises on the measurement. Haffiz's filter waits for its
    covariance to grow, which can take many seconds after a > ~1 m jump
    (e.g. the tag rebooted mid-track). 0 = exactly his behaviour.

This position is the "UWB/coarse" estimate. It feeds block 06 (global_pose) and
calibration only. It NEVER reaches the servo and never writes block 05.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .uwb_core import SOLVERS, AnchorSet, RangeProcessor, parse_report, trilaterate

FILTERS = ('cv_kf', 'moving_average', 'none')

# every key below `uwb_positioning` in common.yaml (flattened with dots); missing = loud error
KEYS = ('solver', 'filter', 'min_anchors', 'skip_repeat_reports',
        'cv_kf.process_noise', 'cv_kf.measurement_noise_m', 'cv_kf.gate_mahalanobis2',
        'cv_kf.initial_variance_m2', 'cv_kf.reacquire_after_rejects',
        'moving_average.window', 'nlls.f_scale_m', 'nlls.initial_xy_m')


class PositioningConfigError(ValueError):
    pass


def flatten(d: Dict, prefix: str = '') -> Dict:
    out = {}
    for k, v in (d or {}).items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


@dataclass
class PositioningCfg:
    solver: str = 'linear'
    filter: str = 'cv_kf'
    min_anchors: int = 3
    skip_repeat_reports: bool = False
    process_noise: float = 0.2            # Haffiz: ExtendedKalmanFilter2D(process_noise=0.2, ...)
    measurement_noise_m: float = 0.25     # Haffiz: measurement_noise=0.25
    gate_mahalanobis2: float = 16.0       # Haffiz: mahalanobis_dist > 16.0 -> reject
    initial_variance_m2: float = 1.0      # Haffiz: P = eye(4) * 1.0
    reacquire_after_rejects: int = 0      # 0 = Haffiz (never re-initialise)
    ma_window: int = 10                   # Haffiz noEKF: MovingAverageFilter(window_size=10)
    nlls_f_scale_m: float = 1.0           # scipy least_squares default f_scale
    nlls_initial_xy_m: Tuple[float, float] = (1.5, 2.5)   # Haffiz noEKF: self.last_pos = [1.5, 2.5]

    @staticmethod
    def from_dict(d: Dict) -> 'PositioningCfg':
        """d = the `uwb_positioning` section, nested (YAML) or flattened (params_under)."""
        f = flatten(d) if any(isinstance(v, dict) for v in (d or {}).values()) else dict(d or {})
        miss = [k for k in KEYS if k not in f]
        if miss:
            raise PositioningConfigError('common.yaml uwb_positioning: missing ' + ', '.join(miss))
        c = PositioningCfg(
            str(f['solver']), str(f['filter']), int(f['min_anchors']), bool(f['skip_repeat_reports']),
            float(f['cv_kf.process_noise']), float(f['cv_kf.measurement_noise_m']),
            float(f['cv_kf.gate_mahalanobis2']), float(f['cv_kf.initial_variance_m2']),
            int(f['cv_kf.reacquire_after_rejects']), int(f['moving_average.window']),
            float(f['nlls.f_scale_m']), tuple(float(v) for v in f['nlls.initial_xy_m']))
        c.check()
        return c

    def check(self) -> None:
        if self.solver not in SOLVERS:
            raise PositioningConfigError(f'uwb_positioning.solver {self.solver!r}: use {" | ".join(SOLVERS)}')
        if self.filter not in FILTERS:
            raise PositioningConfigError(f'uwb_positioning.filter {self.filter!r}: use {" | ".join(FILTERS)}')
        if self.min_anchors < 3:
            raise PositioningConfigError('uwb_positioning.min_anchors must be >= 3 (2-D needs 3 ranges)')
        if self.process_noise <= 0 or self.measurement_noise_m <= 0 or self.initial_variance_m2 <= 0:
            raise PositioningConfigError('uwb_positioning.cv_kf noises / variance must be > 0')
        if self.gate_mahalanobis2 <= 0 or self.ma_window < 1 or self.nlls_f_scale_m <= 0:
            raise PositioningConfigError('uwb_positioning: gate, window and f_scale must be > 0')
        if len(self.nlls_initial_xy_m) != 2:
            raise PositioningConfigError('uwb_positioning.nlls.initial_xy_m needs [x, y]')

    def as_dict(self) -> Dict:
        return {'solver': self.solver, 'filter': self.filter, 'min_anchors': self.min_anchors,
                'skip_repeat_reports': self.skip_repeat_reports,
                'cv_kf': {'process_noise': self.process_noise, 'measurement_noise_m': self.measurement_noise_m,
                          'gate_mahalanobis2': self.gate_mahalanobis2,
                          'initial_variance_m2': self.initial_variance_m2,
                          'reacquire_after_rejects': self.reacquire_after_rejects},
                'moving_average': {'window': self.ma_window},
                'nlls': {'f_scale_m': self.nlls_f_scale_m, 'initial_xy_m': list(self.nlls_initial_xy_m)}}


# --------------------------------------------------------------------------- filters
class CvKalman:
    """Haffiz ExtendedKalmanFilter2D (constant-velocity model, state [x, y, vx, vy]).

    Line for line the same predict / update / gate; `process(x, y, t)` takes the
    measurement time instead of calling time.time(). Differences only when
    reacquire_after_rejects > 0 (see the module docstring)."""

    def __init__(self, process_noise: float, measurement_noise: float, gate: float = 16.0,
                 initial_variance: float = 1.0, reacquire_after: int = 0):
        self.q_var = float(process_noise)
        self.R = np.eye(2) * (float(measurement_noise) ** 2)
        self.gate = float(gate)
        self.p0 = float(initial_variance)
        self.reacquire_after = int(reacquire_after)
        self.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * self.p0
        self.is_initialized = False
        self.last_time: Optional[float] = None
        self.streak = 0
        self.accepted = self.rejected = self.reacquires = 0
        self.last_accepted = True
        self.last_m2 = 0.0

    def initialize(self, x: float, y: float, t: float) -> None:
        # Haffiz: state reset, P left as it is (eye * 1.0 from the constructor)
        self.x = np.array([[x], [y], [0.0], [0.0]])
        self.last_time = t
        self.is_initialized = True

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        Q = np.array([[0.25 * dt ** 4, 0, 0.5 * dt ** 3, 0],
                      [0, 0.25 * dt ** 4, 0, 0.5 * dt ** 3],
                      [0.5 * dt ** 3, 0, dt ** 2, 0],
                      [0, 0.5 * dt ** 3, 0, dt ** 2]]) * self.q_var
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, zx: float, zy: float) -> Tuple[float, float]:
        z = np.array([[zx], [zy]])
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        # .item(): Haffiz writes float(1x1 array), which numpy >= 2 refuses (his try/except
        # would then drop every fix silently). Same value on numpy 1.x.
        m2 = (y.T @ np.linalg.inv(S) @ y).item()
        self.last_m2 = m2
        if m2 > self.gate:                         # reject, trust the prediction
            self.last_accepted = False
            self.rejected += 1
            self.streak += 1
            return float(self.x[0, 0]), float(self.x[1, 0])
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.last_accepted = True
        self.accepted += 1
        self.streak = 0
        return float(self.x[0, 0]), float(self.x[1, 0])

    def process(self, x: float, y: float, t: float) -> Tuple[float, float]:
        if not self.is_initialized:
            self.initialize(x, y, t)
            self.last_accepted = True
            return x, y
        dt = t - self.last_time
        self.last_time = t
        self.predict(dt)
        out = self.update(x, y)
        if not self.last_accepted and self.reacquire_after > 0 and self.streak >= self.reacquire_after:
            self.x = np.array([[x], [y], [0.0], [0.0]])
            self.P = np.eye(4) * self.p0
            self.streak = 0
            self.reacquires += 1
            out = (x, y)
        return out

    @property
    def cov_xy(self) -> np.ndarray:
        return self.P[:2, :2].copy()

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2, 0]), float(self.x[3, 0])


class MovingAverage:
    """Haffiz MovingAverageFilter (noEKF variant)."""

    def __init__(self, window: int):
        self.bx: Deque[float] = deque(maxlen=int(window))
        self.by: Deque[float] = deque(maxlen=int(window))
        self.bt: Deque[float] = deque(maxlen=int(window))

    def reset(self) -> None:
        self.bx.clear()
        self.by.clear()
        self.bt.clear()

    def filter(self, x: float, y: float, t: float) -> Tuple[float, float]:
        self.bx.append(x)
        self.by.append(y)
        self.bt.append(t)
        return float(np.mean(self.bx)), float(np.mean(self.by))

    def velocity(self) -> Optional[Tuple[float, float]]:
        if len(self.bt) < 3 or self.bt[-1] - self.bt[0] <= 1e-6:
            return None
        dt = self.bt[-1] - self.bt[0]
        return (self.bx[-1] - self.bx[0]) / dt, (self.by[-1] - self.by[0]) / dt


# --------------------------------------------------------------------------- pipeline
@dataclass
class Fix:
    t: float                                   # measurement time (s)
    raw: Tuple[float, float]                   # solver output (venue, m)
    xy: Tuple[float, float]                    # filtered position (venue, m)
    cov: Tuple[float, float, float]            # filtered covariance xx, xy, yy (m^2)
    vel: Optional[Tuple[float, float]]         # venue-frame velocity (m/s), None if unknown
    accepted: bool                             # filter accepted this measurement (always True without a gate)
    n_anchors: int
    anchors: Tuple[str, ...] = ()


class Positioner:
    """ranges -> Fix, one call per tag report. Owns the solver state (nlls start point)
    and the filter state. reset() after a tag reboot or a localisation reset."""

    def __init__(self, anchors: AnchorSet, cfg: PositioningCfg):
        cfg.check()
        self.anchors, self.cfg = anchors, cfg
        self.kf = CvKalman(cfg.process_noise, cfg.measurement_noise_m, cfg.gate_mahalanobis2,
                           cfg.initial_variance_m2, cfg.reacquire_after_rejects)
        self.ma = MovingAverage(cfg.ma_window)
        self.nlls_x0 = list(cfg.nlls_initial_xy_m)
        self.solved = self.unsolved = 0

    def reset(self) -> None:
        self.kf.reset()
        self.ma.reset()
        self.nlls_x0 = list(self.cfg.nlls_initial_xy_m)

    def solve(self, ranges: Dict[str, float]) -> Optional[Tuple[float, float]]:
        use = {a: r for a, r in ranges.items() if a in self.anchors.anchors and math.isfinite(r)}
        if len(use) < self.cfg.min_anchors:
            return None
        p = trilaterate(self.anchors, use, self.cfg.solver, self.nlls_x0, self.cfg.nlls_f_scale_m)
        if p is not None and self.cfg.solver == 'nlls':
            self.nlls_x0 = [p[0], p[1]]
        return p

    def update(self, ranges: Dict[str, float], t: float) -> Optional[Fix]:
        raw = self.solve(ranges)
        if raw is None:
            self.unsolved += 1
            return None
        self.solved += 1
        n = sum(1 for a in ranges if a in self.anchors.anchors)
        ids = tuple(sorted(a for a in ranges if a in self.anchors.anchors))
        r2 = self.cfg.measurement_noise_m ** 2
        if self.cfg.filter == 'cv_kf':
            fx, fy = self.kf.process(raw[0], raw[1], t)
            P = self.kf.cov_xy
            return Fix(t, raw, (fx, fy), (float(P[0, 0]), float(P[0, 1]), float(P[1, 1])),
                       self.kf.velocity, self.kf.last_accepted, n, ids)
        if self.cfg.filter == 'moving_average':
            fx, fy = self.ma.filter(raw[0], raw[1], t)
            var = r2 / max(len(self.ma.bx), 1)
            return Fix(t, raw, (fx, fy), (var, 0.0, var), self.ma.velocity(), True, n, ids)
        return Fix(t, raw, raw, (r2, 0.0, r2), None, True, n, ids)


def report_ranges(processed, skip_repeat_reports: bool) -> Tuple[Dict[str, float], float, bool]:
    """uwb_core.Processed -> (ranges for the solver, measurement time, any fresh range).

    Every link that is not too old / out of range is used, repeated sample_seq
    included: Haffiz solves every report with the latest range per anchor.
    Time = encode time minus the mean age of the ranges used."""
    use = [r for r in processed.ranges if r.reason in ('', 'repeat')]
    rng = {r.anchor: float(r.corrected_m) for r in use}
    fresh = any(r.reason == '' for r in use)
    if use:
        t = processed.encode_time - sum(r.age_ms for r in use) / (1000.0 * len(use))
    else:
        t = processed.encode_time
    if skip_repeat_reports and not fresh:
        return {}, t, False
    return rng, t, fresh


def positions_from_rows(anchors: AnchorSet, rows: Sequence[Dict], cfg: PositioningCfg,
                        on_fix: Optional[Callable[[Fix, Dict], None]] = None) -> List[Fix]:
    """Offline: raw tag rows [{'t': s, 'json': str}] (wizard feed, calib captures) -> fixes.
    Row time is the arrival time (no latency model offline)."""
    proc = RangeProcessor(anchors, 400, 0.05, 30.0, 'arrival')
    pos = Positioner(anchors, cfg)
    boot = None
    out: List[Fix] = []
    for row in rows:
        rep = parse_report(row['json'])
        if rep is None:
            continue
        if boot is not None and rep.boot_id != boot:
            pos.reset()
        boot = rep.boot_id
        res = proc.process(rep, float(row['t']))
        rng, t, _ = report_ranges(res, cfg.skip_repeat_reports)
        if not rng:
            continue
        f = pos.update(rng, t)
        if f is not None:
            out.append(f)
            if on_fix is not None:
                on_fix(f, row)
    return out


def cov_ellipse(cxx: float, cxy: float, cyy: float, k: float = 2.0) -> Tuple[float, float, float]:
    """(semi-major, semi-minor, angle rad) of the k-sigma ellipse."""
    tr, det = cxx + cyy, cxx * cyy - cxy * cxy
    disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
    l1, l2 = tr / 2.0 + disc, max(tr / 2.0 - disc, 0.0)
    ang = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    return k * math.sqrt(max(l1, 0.0)), k * math.sqrt(l2), ang


def rear_axle_from_tag(xy: Tuple[float, float], heading: float,
                       lever: Tuple[float, float]) -> Tuple[float, float]:
    """Tag antenna position -> rear-axle centre (base_link origin), same frame as xy.
    lever = uwb.yaml tag.mount_xy_m (tag in base_link: +x forward, +y left)."""
    c, s = math.cos(heading), math.sin(heading)
    return xy[0] - (lever[0] * c - lever[1] * s), xy[1] - (lever[0] * s + lever[1] * c)


def cfg_from_common_yaml(config_dir: str = '') -> PositioningCfg:
    """For tools that run outside the launch (calib_uwb, record_lap, tools/uwb, sandbox):
    read `/**` uwb_positioning from carbot_bringup config/params/common.yaml (+ the ACTIVE
    calibration session's params_overlay.yaml if it overrides it)."""
    import os

    import yaml
    from carbot_common.calib_tools import bringup_config_dir
    d = bringup_config_dir(config_dir)
    with open(os.path.join(d, 'params', 'common.yaml'), 'r', encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    sec = ((doc.get('/**') or {}).get('ros__parameters') or {}).get('uwb_positioning')
    if sec is None:
        raise PositioningConfigError(f'{d}/params/common.yaml has no /** uwb_positioning section')
    return PositioningCfg.from_dict(sec)
