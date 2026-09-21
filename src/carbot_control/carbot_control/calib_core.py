"""Calibration steps 7 + 8, pure analysis (no ROS). Used by calib_steering /
calib_speed, their --replay mode and the phase-8 wizard.

Step 7 servo_steering
  straight   raw drive with angular.z = 0: IMU yaw change / distance = residual
             curvature k. Lateral drift per metre of a constant-curvature run of
             length L is k L / 2 (pass: <= straight_drift_m_per_m). servo_center is
             an integer: correction = round(-k / k_per_unit), with k_per_unit taken
             from the full-lock circles (or the nominal 1 / (R * range) before them).
  circles    raw drive at full lock (angular.z -1 = physical LEFT, +1 = RIGHT in the
             base convention): R = distance / |yaw change|. Writes
             left/right_max_rad = atan(wheelbase / R) and min_turning_radius_m =
             max(R_left, R_right) (the planners assume one radius both ways).
             A left lock that turns right means the steering sign is reversed.
Step 8 speed_pid
  sweep      raw duty steps (alternating direction so the car stays in place):
             steady speed = mean /odom speed over the last `steady_s` of a step.
             Least squares  duty = static + duty_per_mps * |v|  over moving steps.
  step       closed loop (CALIBRATION source through the owner's PID): steady
             error and overshoot per target.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def plain(x):
    """numpy scalars / tuples -> plain Python (YAML safe_dump)."""
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    return x


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- step 7
@dataclass
class StraightResult:
    distance_m: float
    yaw_change_rad: float
    curvature: float
    drift_m_per_m: float

    def as_dict(self) -> Dict:
        return {'distance_m': self.distance_m, 'yaw_change_deg': math.degrees(self.yaw_change_rad),
                'curvature_1pm': self.curvature, 'drift_m_per_m': self.drift_m_per_m}


def straight_result(distance: float, yaw_change: float) -> StraightResult:
    L = max(abs(distance), 1e-6)
    k = yaw_change / L * (1 if distance >= 0 else -1)
    return StraightResult(distance, yaw_change, k, abs(k) * L / 2)


def centre_correction(curvature: float, k_per_unit_left: float, k_per_unit_right: float) -> int:
    """Integer servo_center change that cancels a residual curvature (+ = drifting LEFT).
    Base convention: a larger servo angle turns RIGHT, so a left drift needs +units."""
    if curvature > 0:
        return int(round(curvature / max(k_per_unit_right, 1e-6)))
    return -int(round(-curvature / max(k_per_unit_left, 1e-6)))


@dataclass
class CircleResult:
    side: str
    distance_m: float
    yaw_change_rad: float
    radius_m: float
    steer_max_rad: float
    turned_correct_way: bool

    def as_dict(self) -> Dict:
        return {'side': self.side, 'distance_m': self.distance_m, 'yaw_change_deg': math.degrees(self.yaw_change_rad),
                'radius_m': self.radius_m, 'steer_max_rad': self.steer_max_rad,
                'turned_correct_way': self.turned_correct_way}


def circle_result(side: str, distance: float, yaw_change: float, wheelbase: float) -> CircleResult:
    R = abs(distance) / max(abs(yaw_change), 1e-6)
    left_expected = side == 'left'
    correct = (yaw_change > 0) == left_expected if distance >= 0 else (yaw_change < 0) == left_expected
    return CircleResult(side, distance, yaw_change, R, math.atan(wheelbase / R), correct)


def steering_summary(left: CircleResult, right: CircleResult, straight: Optional[StraightResult],
                     pass_cfg: Dict) -> Dict:
    r_max = max(left.radius_m, right.radius_m)
    checks = {
        'direction': left.turned_correct_way and right.turned_correct_way,
        'min_radius': r_max <= float(pass_cfg['min_radius_m_max']),
        'straight': straight is not None and straight.drift_m_per_m <= float(pass_cfg['straight_drift_m_per_m']),
    }
    return {'passed': all(checks.values()), 'checks': checks, 'min_turning_radius_m': r_max,
            'left_max_rad': left.steer_max_rad, 'right_max_rad': right.steer_max_rad}


# --------------------------------------------------------------------------- step 8
def steady_speed(t: Sequence[float], v: Sequence[float], t_end: float, steady_s: float) -> float:
    T, V = np.asarray(t, float), np.asarray(v, float)
    m = (T > t_end - steady_s) & (T <= t_end)
    return float(V[m].mean()) if m.any() else 0.0


@dataclass
class FeedforwardFit:
    duty_per_mps: float
    static_duty: float
    min_moving_speed: float
    points: List[Tuple[float, float]]
    residual_mps: float
    ok: bool
    reason: str = ''


def fit_feedforward(samples: Sequence[Tuple[float, float]], moving_mps: float = 0.01) -> FeedforwardFit:
    """samples = (duty, steady speed) per sweep step (either direction)."""
    pts = [(abs(d), abs(v)) for d, v in samples if abs(v) > moving_mps and d * v > 0]
    if len(pts) < 3:
        return FeedforwardFit(0.0, 0.0, 0.0, pts, 0.0, False, f'only {len(pts)} moving steps (need 3)')
    D = np.array([p[0] for p in pts])
    V = np.array([p[1] for p in pts])
    A = np.column_stack([np.ones_like(V), V])
    (static, per), *_ = np.linalg.lstsq(A, D, rcond=None)
    pred_v = (D - static) / per if per > 0 else V * 0
    res = float(np.sqrt(np.mean((pred_v - V) ** 2)))
    ok = per > 0 and static >= 0
    return FeedforwardFit(float(per), float(max(0.0, static)), float(V.min()), pts, res, ok,
                          '' if ok else 'non-physical fit (check /odom sign and the motor)')


@dataclass
class StepMetrics:
    target: float
    steady: float
    steady_error: float
    overshoot_pct: float

    def as_dict(self) -> Dict:
        return self.__dict__.copy()


def step_metrics(t: Sequence[float], v: Sequence[float], target: float, t_end: float, steady_s: float) -> StepMetrics:
    V = np.asarray(v, float) * (1 if target >= 0 else -1)
    s = steady_speed(t, V, t_end, steady_s)
    peak = float(V.max()) if len(V) else 0.0
    tgt = abs(target)
    return StepMetrics(target, s * (1 if target >= 0 else -1), abs(tgt - s),
                       max(0.0, (peak - tgt) / max(tgt, 1e-6) * 100.0))


def pid_verdict(metrics: Sequence[StepMetrics], ff: FeedforwardFit, pass_cfg: Dict) -> Dict:
    err = max((m.steady_error for m in metrics), default=1e9)
    over = max((m.overshoot_pct for m in metrics), default=1e9)
    creep_limit = 1.5 * float(pass_cfg['min_creep_speed_mps'])
    checks = {'feedforward_fit': ff.ok,
              'steady_error': err <= float(pass_cfg['max_steady_error_mps']),
              'overshoot': over <= float(pass_cfg['max_overshoot_pct']),
              'creep': ff.ok and ff.min_moving_speed <= creep_limit}
    return {'passed': all(checks.values()), 'checks': checks, 'max_steady_error_mps': err,
            'max_overshoot_pct': over, 'min_moving_speed_mps': ff.min_moving_speed,
            'creep_limit_mps': creep_limit}


def retune(kp: float, ki: float, verdict: Dict) -> Tuple[float, float, str]:
    """One simple adjustment per failed verify run (then verify again)."""
    if not verdict['checks']['overshoot']:
        return kp * 0.7, ki * 0.8, 'overshoot: kp x0.7, ki x0.8'
    if not verdict['checks']['steady_error']:
        return kp, ki * 1.5, 'steady error: ki x1.5'
    return kp, ki, 'no change'
