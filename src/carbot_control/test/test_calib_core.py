"""Calibration steps 7/8 analysis + --replay (no ROS)."""
import json
import math

import pytest
from carbot_control import calib_core as cc

PASS7 = {'straight_drift_m_per_m': 0.02, 'min_radius_m_max': 0.45}
PASS8 = {'max_steady_error_mps': 0.01, 'max_overshoot_pct': 20.0, 'min_creep_speed_mps': 0.03}


def test_circle_radius_and_direction():
    L = cc.circle_result('left', 1.2566, math.pi, 0.216)           # R = 0.40
    assert abs(L.radius_m - 0.40) < 1e-3 and L.turned_correct_way
    assert abs(L.steer_max_rad - math.atan(0.216 / 0.40)) < 1e-3
    assert not cc.circle_result('left', 1.2, -math.pi, 0.216).turned_correct_way
    assert cc.circle_result('right', 1.2, -math.pi, 0.216).turned_correct_way


def test_straight_drift_and_centre_correction_sign():
    s = cc.straight_result(1.5, math.radians(3.0))                  # drifting LEFT
    assert s.curvature > 0 and abs(s.drift_m_per_m - s.curvature * 0.75) < 1e-12
    k = (1 / 0.4) / 50
    assert cc.centre_correction(s.curvature, k, k) > 0              # larger servo angle = right
    assert cc.centre_correction(-s.curvature, k, k) < 0
    assert cc.centre_correction(1e-5, k, k) == 0


def test_steering_summary():
    L = cc.circle_result('left', 1.2566, math.pi, 0.216)
    R = cc.circle_result('right', 1.3823, -math.pi, 0.216)          # 0.44
    good = cc.steering_summary(L, R, cc.straight_result(1.5, 0.005), PASS7)
    assert good['passed'] and abs(good['min_turning_radius_m'] - 0.44) < 1e-3
    assert not cc.steering_summary(L, R, cc.straight_result(1.5, 0.1), PASS7)['passed']
    wide = cc.circle_result('right', 1.8, -math.pi, 0.216)          # 0.57 m: fails the radius check
    assert not cc.steering_summary(L, wide, cc.straight_result(1.5, 0.0), PASS7)['checks']['min_radius']


def test_feedforward_fit_recovers_the_motor():
    # plant: v = 0.9 (duty - 0.06) above the dead band; alternating direction
    duties = [0.06, 0.08, 0.10, 0.12, 0.15, 0.18, 0.22]
    samples = [((d if i % 2 == 0 else -d), (0.0 if d <= 0.06 else 0.9 * (d - 0.06)) * (1 if i % 2 == 0 else -1))
               for i, d in enumerate(duties)]
    ff = cc.fit_feedforward(samples)
    assert ff.ok and abs(ff.duty_per_mps - 1 / 0.9) < 1e-6 and abs(ff.static_duty - 0.06) < 1e-6
    assert abs(ff.min_moving_speed - 0.018) < 1e-9
    assert not cc.fit_feedforward([(0.1, 0.0), (0.2, 0.0)]).ok


def test_step_metrics_verdict_and_retune():
    t = [i * 0.02 for i in range(150)]
    v = [min(0.08, 0.1 * (1 - math.exp(-x / 0.2))) for x in t]      # overshoots to 0.08 on target 0.075
    m = cc.step_metrics(t, v, 0.075, t[-1], 1.0)
    assert abs(m.steady - 0.08) < 1e-3 and abs(m.overshoot_pct - 6.67) < 0.1
    rev = cc.step_metrics(t, [-x for x in v], -0.075, t[-1], 1.0)
    assert abs(rev.steady + 0.08) < 1e-3 and rev.steady_error == pytest.approx(m.steady_error)
    ff = cc.FeedforwardFit(1.1, 0.06, 0.02, [], 0.0, True)
    ok = cc.pid_verdict([cc.StepMetrics(0.075, 0.074, 0.001, 5.0)], ff, PASS8)
    assert ok['passed']
    bad = cc.pid_verdict([cc.StepMetrics(0.075, 0.075, 0.0, 35.0)], ff, PASS8)
    assert not bad['passed'] and cc.retune(0.8, 0.4, bad)[0] == pytest.approx(0.56)
    slow = cc.pid_verdict([cc.StepMetrics(0.075, 0.06, 0.015, 0.0)], ff, PASS8)
    assert cc.retune(0.8, 0.4, slow) == (0.8, pytest.approx(0.6), 'steady error: ki x1.5')


def _session_args(tmp_path):
    return ['--data-root', str(tmp_path), '--session', 'test', '--yes']


def test_replay_writes_overlay(tmp_path):
    from carbot_common import calib_tools as ct
    from carbot_control import calib_speed, calib_steering
    cap = {'servo_center': 92, 'steer_sign': -1.0, 'left': {'distance': 1.2566, 'yaw': math.pi},
           'right': {'distance': 1.3823, 'yaw': -math.pi},
           'straight': [{'distance': 1.5, 'yaw': 0.005, 'servo_center': 92}]}
    f = tmp_path / 'steering.json'
    f.write_text(json.dumps(cap))
    assert calib_steering.main(_session_args(tmp_path) + ['--replay', str(f)]) == 0
    s = str(tmp_path / 'calibration' / 'test')
    assert ct.overlay_value(s, 'servo_controller', 'servo_center') == 92
    assert ct.overlay_value(s, 'command_owner', 'steering.left_max_rad') == \
        ct.overlay_value(s, 'tunnel_bridge', 'command_owner_steering.left_max_rad')
    sp = {'sweep': [[0.08, 0.018], [-0.10, -0.036], [0.12, 0.054], [-0.15, -0.081], [0.18, 0.108]],
          'pid_start': {'kp': 0.8, 'ki': 0.4, 'kd': 0.0, 'integral_limit': 0.15},
          'verify': [{'pid': {'kp': 0.8, 'ki': 0.4, 'kd': 0.0, 'integral_limit': 0.15},
                      'metrics': [{'target': 0.075, 'steady': 0.074, 'steady_error': 0.001, 'overshoot_pct': 8.0}]}]}
    g = tmp_path / 'speed.json'
    g.write_text(json.dumps(sp))
    assert calib_speed.main(_session_args(tmp_path) + ['--replay', str(g)]) == 0
    assert ct.overlay_value(s, 'command_owner', 'feedforward.static_duty') == pytest.approx(0.06, abs=1e-3)
    assert ct.overlay_value(s, 'tunnel_bridge', 'command_owner_feedforward.duty_per_mps') == \
        ct.overlay_value(s, 'command_owner', 'feedforward.duty_per_mps')
