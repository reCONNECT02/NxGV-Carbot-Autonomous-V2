"""Block 15/16: arbitration priority, watchdog, steering map, speed controller."""
import math

import numpy as np
import pytest
from carbot_control.owner_core import (OwnerCfg, Req, SpeedCfg, SpeedController, SteeringMap, arbitrate,
                                       base_duty_to_mps, feedforward)

CFG = OwnerCfg(request_expiry_s=0.2, safety_max_age_s=0.1, mission_max_age_s=0.5)
OK = (True, '', 10.0)


def req(src, t, v=0.1, s=0.05):
    return Req(src, v, s, False, 'r', t)


def test_priority_order():
    reqs = {'ROAD': req('ROAD', 9.95), 'PARKING': req('PARKING', 9.95)}
    m = ('ROAD', 'ROAD', 9.99)
    assert arbitrate(10.0, False, False, OK, m, reqs, None, CFG).winner == 'DISARMED'
    assert arbitrate(10.0, True, True, OK, m, reqs, None, CFG).winner == 'SAFETY_STOP'      # e-stop first
    assert arbitrate(10.0, True, False, (False, 'camera_fresh: stale', 10.0), m, reqs, None, CFG).winner \
        == 'SAFETY_STOP'
    assert arbitrate(10.0, True, False, (True, '', 9.8), m, reqs, None, CFG).reason == 'Safety status stale'
    assert arbitrate(10.0, True, False, OK, ('ROAD', 'ROAD', 9.0), reqs, None, CFG).winner == 'WATCHDOG'
    assert arbitrate(10.0, True, False, OK, ('SAFETY_STOP', 'ROAD', 9.99), reqs, None, CFG).winner == 'SAFETY_STOP'
    assert arbitrate(10.0, True, False, OK, ('HOLD', 'HOLD', 9.99), reqs, None, CFG).winner == 'HOLD'
    d = arbitrate(10.0, True, False, OK, ('PARKING', 'PARKING', 9.99), reqs, None, CFG)
    assert d.winner == 'PARKING' and d.drive and d.speed == 0.1


def test_only_the_active_source_is_followed_and_watchdog():
    reqs = {'ROAD': req('ROAD', 9.95)}
    d = arbitrate(10.0, True, False, OK, ('TUNNEL', 'TUNNEL', 9.99), reqs, None, CFG)
    assert d.winner == 'WATCHDOG' and not d.drive                  # a ROAD request never drives TUNNEL
    d = arbitrate(10.0, True, False, OK, ('ROAD', 'ROAD', 9.99), {'ROAD': req('ROAD', 9.7)}, None, CFG)
    assert d.winner == 'WATCHDOG' and 'older than 200 ms' in d.reason


def test_calibration_source_only_in_calibrate_mode_and_never_armed():
    cal = Req('CALIBRATION_RAW', 0.9, 0.3, False, 'step 7', 9.99)
    c = OwnerCfg(calibration_allowed=True, calibration_duty_max=0.35)
    d = arbitrate(10.0, False, False, None, None, {}, cal, c)
    assert d.winner == 'CALIBRATION_RAW' and d.raw and d.speed == 0.35
    assert arbitrate(10.0, False, False, None, None, {}, cal, CFG).winner == 'DISARMED'   # race: ignored
    assert arbitrate(10.0, True, False, None, None, {}, cal, c).winner == 'SAFETY_STOP'   # armed: race rules
    assert arbitrate(10.0, False, True, None, None, {}, cal, c).winner == 'SAFETY_STOP'   # e-stop wins
    assert arbitrate(10.5, False, False, None, None, {}, cal, c).winner == 'WATCHDOG'


@pytest.mark.parametrize('m', [SteeringMap(), SteeringMap(-1.0, 0.45, 0.52, 0.01, 1.0)])
def test_steering_map_inverse_and_convention(m):
    for s in np.linspace(-0.4, 0.4, 41):
        assert abs(m.to_steer(m.to_angular(s)) - s) < 1e-12
    assert m.to_angular(0.2 - m.trim_rad) < 0                # REP-103 left -> base angular.z negative
    assert m.to_angular(-0.2 - m.trim_rad) > 0               # right -> positive (base: + = RIGHT)
    assert m.to_angular(5.0) == -m.angular_limit and m.to_angular(-5.0) == m.angular_limit
    # base tunnel follower values round-trip exactly (servo sees the same angular.z)
    for z in (-1.0, -0.37, 0.0, 0.25, 0.8):
        assert abs(m.to_angular(m.to_steer(z)) - z) < 1e-12


def test_duty_inverse():
    c = SpeedCfg(duty_per_mps=1.2, static_duty=0.07)
    for v in (0.03, 0.075, 0.15, -0.05):
        assert abs(base_duty_to_mps(feedforward(v, c), 1.2, 0.07) - v) < 1e-12


def _plant(duty, v, dt, k=0.9, static=0.06, tau=0.18):
    """First-order motor: steady v = k (duty - static) above the dead band."""
    target = 0.0 if abs(duty) <= static else math.copysign(k * (abs(duty) - static), duty)
    return v + (target - v) * min(1.0, dt / tau)


def test_speed_controller_tracks_with_wrong_feedforward():
    # feedforward uncalibrated (1.0 duty per m/s, 0.08 static) vs plant 0.9 / 0.06: the PID closes it
    sc = SpeedController(SpeedCfg())
    v, dt = 0.0, 0.02
    hist = []
    for i in range(400):
        d = sc.update(0.10, v, dt)
        v = _plant(d, v, dt)
        hist.append(v)
    assert abs(hist[-1] - 0.10) < 0.005
    assert max(hist) < 0.10 * 1.2                             # overshoot < 20 % (step 8 pass limit)
    assert sc.update(0.0, v, dt) == 0.0 and sc.integral == 0.0  # stop is immediate, integrator cleared


def test_slew_limits_ramp_up_not_braking():
    sc = SpeedController(SpeedCfg(duty_slew_per_s=1.0))
    d1 = sc.update(0.15, 0.0, 0.02)
    assert d1 <= 0.02 + 1e-12                                # 1.0 /s * 20 ms
    for _ in range(50):
        sc.update(0.15, 0.0, 0.02)
    high = sc.duty
    d2 = sc.update(0.02, 0.15, 0.02)                         # slow down: immediate
    assert d2 < high - 0.05
    assert sc.update(-0.05, 0.02, 0.02) == 0.0              # direction change passes through zero
    assert sc.update(-0.05, 0.0, 0.02) < 0


def test_manual_takeover_phase7():
    """GUI Manual control: owner stops driving (base drives from /joy); e-stop still wins."""
    reqs = {'ROAD': req('ROAD', 9.95)}
    m = ('ROAD', 'ROAD', 9.99)
    d = arbitrate(10.0, True, False, OK, m, reqs, None, CFG, manual=True)
    assert d.winner == 'MANUAL' and not d.drive and d.speed == 0.0
    assert arbitrate(10.0, True, True, OK, m, reqs, None, CFG, manual=True).winner == 'SAFETY_STOP'
    assert arbitrate(10.0, False, False, OK, m, reqs, None, CFG, manual=True).winner == 'MANUAL'
    assert arbitrate(10.0, True, False, OK, m, reqs, None, CFG).winner == 'ROAD'   # hand back
