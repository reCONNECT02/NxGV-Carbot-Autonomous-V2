"""BLOCKS 15 + 16 core (no ROS): one final writer.

Arbiter        V4 vehicle.js arbitrate() + app.js priority, extended with the
               race arming / e-stop latch:
                 DISARMED -> SAFETY_STOP (e-stop, stale or vetoing safety, mission
                 SAFETY_STOP) -> WATCHDOG (mission state stale, request older than
                 request_expiry_s) -> HOLD -> the source mission logic marked active.
SteeringMap    REP-103 steer_rad (+ left) <-> the base servo_controller's
               normalized /cmd_vel_auto angular.z (+ = RIGHT):
                 s = steer_rad + trim
                 angular.z = steer_sign * s / (left_max_rad if s >= 0 else right_max_rad)
               clamped to +-angular_limit. to_steer() is the exact inverse (tunnel
               bridge), so the servo receives what the base stack would send.
SpeedController  target m/s -> base duty (linear.x; x 255 = PWM):
                 duty = feedforward(v) + PID(v - v_measured), clamped to
                 [-duty_reverse_max, duty_max], slew-limited except towards 0
                 (a stop is always immediate). Integrator reset at stop and on a
                 direction change; anti-windup via integral_limit (duty units).
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

DRIVE_SOURCES = ('ROAD', 'TUNNEL', 'PARKING', 'RECOVERY')
CALIBRATION_SOURCES = ('CALIBRATION', 'CALIBRATION_RAW')


# --------------------------------------------------------------------------- steering
@dataclass
class SteeringMap:
    steer_sign: float = -1.0
    left_max_rad: float = 0.495
    right_max_rad: float = 0.495
    trim_rad: float = 0.0
    angular_limit: float = 1.0

    @classmethod
    def from_params(cls, p) -> 'SteeringMap':
        return cls(float(p('steering.steer_sign')), float(p('steering.left_max_rad')),
                   float(p('steering.right_max_rad')), float(p('steering.trim_rad')),
                   float(p('steering.angular_limit')))

    def to_angular(self, steer_rad: float) -> float:
        s = float(steer_rad) + self.trim_rad
        z = self.steer_sign * s / (self.left_max_rad if s >= 0 else self.right_max_rad)
        return max(-self.angular_limit, min(self.angular_limit, z))

    def to_steer(self, angular_z: float) -> float:
        """Inverse of to_angular (inside the angular limit)."""
        z = max(-self.angular_limit, min(self.angular_limit, float(angular_z)))
        s = self.steer_sign * z                          # sign(s) = physical side (+ left)
        s *= self.left_max_rad if s >= 0 else self.right_max_rad
        return s - self.trim_rad


# --------------------------------------------------------------------------- speed
@dataclass
class SpeedCfg:
    kp: float = 0.8
    ki: float = 0.4
    kd: float = 0.0
    integral_limit: float = 0.15
    duty_per_mps: float = 1.0
    static_duty: float = 0.08
    duty_max: float = 0.40
    duty_reverse_max: float = 0.30
    duty_slew_per_s: float = 1.5
    stop_speed: float = 0.004
    measured_max_age_s: float = 0.2

    @classmethod
    def from_params(cls, p) -> 'SpeedCfg':
        return cls(float(p('speed_pid.kp')), float(p('speed_pid.ki')), float(p('speed_pid.kd')),
                   float(p('speed_pid.integral_limit')), float(p('feedforward.duty_per_mps')),
                   float(p('feedforward.static_duty')), float(p('duty_max')), float(p('duty_reverse_max')),
                   float(p('duty_slew_per_s')), float(p('stop_speed_mps')), float(p('measured_max_age_s')))


def feedforward(v: float, cfg: SpeedCfg) -> float:
    if abs(v) <= cfg.stop_speed:
        return 0.0
    return math.copysign(cfg.static_duty + cfg.duty_per_mps * abs(v), v)


def base_duty_to_mps(duty: float, duty_per_mps: float, static_duty: float) -> float:
    """Inverse of feedforward() (tunnel bridge, speed_source base_duty; the PID closes the rest)."""
    if abs(duty) <= static_duty or duty_per_mps <= 0:
        return 0.0
    return (abs(duty) - static_duty) / duty_per_mps * (1 if duty > 0 else -1)


class SpeedController:

    def __init__(self, cfg: SpeedCfg):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_meas = None
        self.duty = 0.0
        self.last_sign = 0

    def update(self, target: float, measured: Optional[float], dt: float) -> float:
        c = self.cfg
        dt = max(1e-3, min(0.1, dt))
        if abs(target) <= c.stop_speed:
            self.reset()
            return 0.0
        sign = 1 if target > 0 else -1
        if sign != self.last_sign:
            self.integral = 0.0
            self.prev_meas = None
            self.last_sign = sign
        u = feedforward(target, c)
        if measured is not None:
            e = target - measured
            self.integral += e * dt
            if c.ki > 0:
                lim = c.integral_limit / c.ki
                self.integral = max(-lim, min(lim, self.integral))
            d = 0.0 if self.prev_meas is None else -(measured - self.prev_meas) / dt
            self.prev_meas = measured
            u += c.kp * e + c.ki * self.integral + c.kd * d
        u = max(-c.duty_reverse_max, min(c.duty_max, u))
        if sign > 0:
            u = max(0.0, u)            # never brake by reversing the motor in the same gear
        else:
            u = min(0.0, u)
        step = c.duty_slew_per_s * dt
        if u * self.duty < 0:
            u = 0.0                                   # direction change: pass through zero first
        elif abs(u) > abs(self.duty):
            u = self.duty + max(-step, min(step, u - self.duty))   # ramp up
        # else: less duty -> immediately (braking is never slew-limited)
        self.duty = u
        return u


# --------------------------------------------------------------------------- arbitration
@dataclass
class OwnerCfg:
    request_expiry_s: float = 0.2
    safety_max_age_s: float = 0.1
    mission_max_age_s: float = 0.5
    calibration_allowed: bool = False
    calibration_max_speed: float = 0.25
    calibration_duty_max: float = 0.35


@dataclass
class Req:
    source: str
    speed: float
    steer: float
    arrived: bool
    reason: str
    t: float                   # receive time


@dataclass
class Decision:
    winner: str
    reason: str
    speed: float = 0.0
    steer: float = 0.0
    raw: bool = False          # CALIBRATION_RAW: speed = duty, steer = angular.z
    request: Optional[Req] = None
    request_age: float = -1.0
    drive: bool = False


def arbitrate(t: float, armed: bool, estop: bool, safety: Optional[Tuple[bool, str, float]],
              mission: Optional[Tuple[str, str, float]], requests: Dict[str, Req],
              calibration: Optional[Req], cfg: OwnerCfg, manual: bool = False) -> Decision:
    """safety = (motion_allowed, reason, receive time); mission = (mode, active_source, receive time).
    manual = GUI Manual control (phase 7): the owner stops writing and releases base AUTO,
    so the base servo_controller drives from the joystick. The e-stop still wins."""
    if estop:
        return Decision('SAFETY_STOP', 'E-stop (counts as manual intervention = 0 marks)')
    if manual:
        return Decision('MANUAL', 'Manual control: controller drives through servo_controller')
    # calibration mode (steps 7/8): supervised, never while the race is armed
    if cfg.calibration_allowed and not armed and calibration is not None:
        age = t - calibration.t
        if age > cfg.request_expiry_s:
            return Decision('WATCHDOG', f'Calibration request {age * 1000:.0f} ms old', request=calibration,
                            request_age=age)
        raw = calibration.source == 'CALIBRATION_RAW'
        if raw:
            sp = max(-cfg.calibration_duty_max, min(cfg.calibration_duty_max, calibration.speed))
        else:
            sp = max(-cfg.calibration_max_speed, min(cfg.calibration_max_speed, calibration.speed))
        return Decision(calibration.source, calibration.reason or 'Calibration step', sp, calibration.steer,
                        raw, calibration, age, True)
    if not armed:
        return Decision('DISARMED', 'Not armed: waiting for START (race_supervisor)')
    if safety is None or t - safety[2] > cfg.safety_max_age_s:
        return Decision('SAFETY_STOP', 'Safety status stale' if safety else 'No safety status yet')
    if not safety[0]:
        return Decision('SAFETY_STOP', safety[1] or 'Safety veto')
    if mission is None or t - mission[2] > cfg.mission_max_age_s:
        return Decision('WATCHDOG', 'Mission state stale')
    mode, source, _ = mission
    if mode == 'SAFETY_STOP':
        return Decision('SAFETY_STOP', 'Mission reports SAFETY STOP')
    if source not in DRIVE_SOURCES:
        return Decision('HOLD', f'Mission {mode or "?"}: {source or "HOLD"}')
    r = requests.get(source)
    if r is None:
        return Decision('WATCHDOG', f'No {source} request yet')
    age = t - r.t
    if age > cfg.request_expiry_s:
        return Decision('WATCHDOG', f'Request older than {cfg.request_expiry_s * 1000:.0f} ms '
                                    f'({source} {age * 1000:.0f} ms)', request=r, request_age=age)
    return Decision(source, r.reason or 'Fresh authorised request', r.speed, r.steer, False, r, age, True)
