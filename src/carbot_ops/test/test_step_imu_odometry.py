"""Step 6 (IMU + wheel odometry): distance / spin / drift logic, corrections, save / keep (no ROS).

The fake car below reproduces servo_controller: odom distance = ticks / ticks_per_meter (sign
flipped by a wrong odom_reverse_polarity), /imu/rpy yaw = normalise(raw * imu_yaw_scale).
"""
import copy
import json
import math
import os

import pytest
import yaml

from carbot_common import calib_tools as ct
from carbot_ops import wizard_core as wc
from carbot_ops.sensor_checks import ConfigError
from carbot_ops.step_imu_odometry import ImuOdometryStep, MotionRecorder, wrap_deg
from helpers import STEPS

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0,
       'page_watch_s': 8.0}
STEP6 = next(s for s in STEPS['steps'] if s['id'] == 'imu_odometry')
SERVO = 'servo_controller'


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeLink:
    """servo_controller parameter services. auto=False keeps replies pending until flush()."""

    def __init__(self, values, auto=True, fail=False):
        self.values, self.auto, self.fail = dict(values), auto, fail
        self.pending, self.sets = [], []

    def get(self, names, done):
        self._later(lambda: done(None if self.fail else {n: self.values.get(n) for n in names}))

    def set(self, values, done):
        def go():
            if not self.fail:
                self.values.update(values)
                self.sets.append(dict(values))
            done(not self.fail)
        self._later(go)

    def _later(self, fn):
        if self.auto:
            fn()
        else:
            self.pending.append(fn)

    def flush(self):
        p, self.pending = self.pending, []
        for fn in p:
            fn()


class Car:
    """Physical car + servo_controller output, feeding a MotionRecorder at 20 Hz."""

    def __init__(self, clock, rec, link, tpm_true=1050.0, polarity_true=False, imu_gain=1.0, imu_sign=1.0,
                 drift_deg_per_min=0.0):
        self.clock, self.rec, self.link = clock, rec, link
        self.tpm_true, self.pol_true = tpm_true, polarity_true
        self.gain, self.sign, self.drift = imu_gain, imu_sign, drift_deg_per_min
        self.x = 0.0                 # /odom x, integrated per increment like servo_controller
        self.raw_yaw = 37.0          # IMU raw reading, deg (arbitrary start)
        self._publish()              # servo_controller publishes from launch on

    def _publish(self, dticks=0.0):
        v = self.link.values
        sign = -1.0 if bool(v['odom_reverse_polarity']) != self.pol_true else 1.0
        self.x += sign * dticks / float(v['ticks_per_meter'])
        self.rec.on_odom(self.clock.t, self.x, 0.0, 0.0)
        self.rec.on_imu(self.clock.t, wrap_deg(wrap_deg(self.raw_yaw) * float(v['imu_yaw_scale'])))

    def step(self, dist_m=0.0, turn_deg=0.0, seconds=0.05):
        self.clock.t += seconds
        self.raw_yaw += self.sign * self.gain * turn_deg + self.drift * seconds / 60.0
        self._publish(dist_m * self.tpm_true)

    def push(self, metres, speed=0.25):
        n = max(1, int(metres / speed / 0.05))
        for _ in range(n):
            self.step(dist_m=metres / n)

    def turn(self, deg, rate=40.0):
        n = max(1, int(abs(deg) / rate / 0.05))
        for _ in range(n):
            self.step(turn_deg=deg / n)

    def wait(self, seconds):
        for _ in range(int(seconds / 0.05)):
            self.step()


START = {'ticks_per_meter': 1050.0, 'odom_reverse_polarity': False, 'imu_yaw_scale': 1.0}


def make(cfg=None, values=None, auto=True, **car):
    clock, rec = Clock(), MotionRecorder()
    link = FakeLink(values or START, auto=auto)
    step = ImuOdometryStep(copy.deepcopy(cfg or STEP6), rec, link, clock=clock)
    return step, Car(clock, rec, link, **car), link, clock


def op(step, name):
    return step.handle(name, {'op': name}, {})


def distance_run(step, car, metres=2.0):
    step.live({})                       # reads the servo values
    r = op(step, 'distance_start')
    assert r['ok'], r
    car.push(metres)
    car.wait(0.3)
    r = op(step, 'distance_stop')
    assert r['ok'], r
    return step.runs['distance'][-1]


def spin(step, car, deg=360.0):
    step.live({})
    r = op(step, 'spin_start')
    assert r['ok'], r
    car.wait(1.0)                       # settle window
    car.turn(deg)
    car.wait(0.3)
    r = op(step, 'spin_stop')
    assert r['ok'], r
    return step.runs['spin'][-1]


def drift(step, car, clock):
    err = step.start(clock.t, {})
    assert err is None, err
    res = None
    while res is None:
        car.wait(1.0)
        res = step.tick(clock.t, {})
    return res


# ---------------------------------------------------------------- config
def test_repo_yaml_builds():
    step, *_ = make()
    assert step.true_m == 2.0 and step.drift_s == 60


@pytest.mark.parametrize('where,key', [('procedure', 'min_rate_hz'), ('procedure', 'settle_s'),
                                       ('pass', 'max_spin_error_pct')])
def test_missing_key_is_config_error(where, key):
    cfg = copy.deepcopy(STEP6)
    del cfg[where][key]
    with pytest.raises(ConfigError, match=key):
        make(cfg)


# ---------------------------------------------------------------- distance
def test_distance_corrects_ticks_then_verifies():
    step, car, link, _ = make(tpm_true=1120.0)
    r1 = distance_run(step, car)
    assert r1['status'] == 'CORRECTED'
    assert r1['odom_m'] == pytest.approx(2.0 * 1120 / 1050, rel=1e-3)
    assert link.values['ticks_per_meter'] == pytest.approx(1120.0, rel=1e-3)
    r2 = distance_run(step, car)
    assert r2['status'] == 'PASS' and abs(r2['error_pct']) < 0.2
    assert step.values['ticks_per_meter'] == link.values['ticks_per_meter']


def test_negative_distance_toggles_polarity():
    step, car, link, _ = make(tpm_true=1050.0, polarity_true=True)
    r1 = distance_run(step, car)
    assert r1['status'] == 'CORRECTED' and r1['odom_m'] < 0
    assert link.values['odom_reverse_polarity'] is True
    assert distance_run(step, car)['status'] == 'PASS'


def test_far_too_short_changes_nothing():
    step, car, link, _ = make()
    r = distance_run(step, car, metres=0.2)
    assert r['status'] == 'FAIL' and link.sets == []


def test_no_odom_is_no_data():
    step, car, link, clock = make()
    step.live({})
    op(step, 'distance_start')
    clock.t += 10.0                      # nothing published
    op(step, 'distance_stop')
    assert step.runs['distance'][-1]['status'] == 'NO_DATA' and link.sets == []


def test_ops_wait_for_servo_values():
    step, car, link, _ = make(auto=False)
    r = op(step, 'distance_start')
    assert not r['ok'] and 'Reading' in r['message']
    link.flush()
    assert op(step, 'distance_start')['ok']


def test_unreachable_servo_is_reported():
    step, car, link, _ = make()
    link.fail = True
    step.live({})
    assert 'Cannot read' in step.link_error
    assert not op(step, 'distance_start')['ok']


def test_stop_without_start_and_double_start():
    step, car, *_ = make()
    step.live({})
    assert not op(step, 'distance_stop')['ok']
    assert op(step, 'distance_start')['ok']
    assert not op(step, 'spin_start')['ok']
    assert op(step, 'abort')['ok'] and step.active is None
    assert not op(step, 'nonsense')['ok']


# ---------------------------------------------------------------- spin
def test_spin_passes_with_good_imu_across_the_wrap():
    step, car, link, _ = make()
    r = spin(step, car)
    assert r['status'] == 'PASS' and r['yaw_change_deg'] == pytest.approx(360.0, abs=1.0)


def test_spin_sign_flip_is_corrected_then_verified():
    step, car, link, _ = make(imu_sign=-1.0)
    r1 = spin(step, car)
    assert r1['status'] == 'CORRECTED' and link.values['imu_yaw_scale'] == pytest.approx(-1.0, abs=1e-3)
    r2 = spin(step, car)
    assert r2['status'] == 'PASS'


def test_spin_scale_error_measured_at_unit_scale_and_verified():
    """IMU reads 5 % short. Correction -> scale 1/0.95; the verify spin must PASS even though
    servo_controller's published yaw (normalise(raw * scale)) jumps at the wrap."""
    step, car, link, _ = make(imu_gain=0.95)
    r1 = spin(step, car)
    assert r1['status'] == 'CORRECTED'
    assert link.values['imu_yaw_scale'] == pytest.approx(1 / 0.95, rel=2e-3)
    r2 = spin(step, car)
    assert r2['status'] == 'PASS', r2
    assert r2['measured_at_scale'] == 1.0 and r2['yaw_at_scale_deg'] == pytest.approx(360.0, abs=2.0)
    # the scale was put back after the measurement
    assert link.values['imu_yaw_scale'] == pytest.approx(1 / 0.95, rel=2e-3)


def test_spin_waits_for_unit_scale_then_settles():
    step, car, link, clock = make(values=dict(START, imu_yaw_scale=1.05), auto=False)
    step.live({})
    link.flush()
    assert op(step, 'spin_start')['ok']
    assert step.active['phase'] == 'arming'
    assert not op(step, 'spin_stop')['ok']
    link.flush()                         # unit scale applied
    assert step.active['phase'] == 'measuring' and link.values['imu_yaw_scale'] == 1.0
    car.wait(1.0)
    car.turn(360.0 / 1.05)               # true full turn for an IMU that reads 5 % long
    car.wait(0.3)
    op(step, 'spin_stop')
    link.flush()
    assert step.runs['spin'][-1]['status'] == 'PASS'
    assert link.values['imu_yaw_scale'] == 1.05


def test_abort_spin_restores_scale():
    step, car, link, _ = make(values=dict(START, imu_yaw_scale=0.97))
    step.live({})
    op(step, 'spin_start')
    assert link.values['imu_yaw_scale'] == 1.0
    op(step, 'abort')
    assert link.values['imu_yaw_scale'] == 0.97


def test_half_turn_changes_nothing():
    step, car, link, _ = make()
    r = spin(step, car, deg=120.0)
    assert r['status'] == 'FAIL' and link.sets == []


# ---------------------------------------------------------------- drift (RUN) + result
def test_drift_refused_until_distance_and_spin_pass():
    step, car, link, clock = make()
    step.live({})
    assert 'distance and spin' in step.start(clock.t, {})
    distance_run(step, car)
    assert 'spin' in step.start(clock.t, {})


def test_full_pass_result_and_values():
    step, car, link, clock = make(tpm_true=1120.0, imu_gain=0.95, drift_deg_per_min=0.5)
    distance_run(step, car)
    distance_run(step, car)
    spin(step, car)
    spin(step, car)
    res = drift(step, car, clock)
    assert res['passed'], res
    assert res['values'] == link.values
    assert res['tests']['drift']['drift_deg_per_min'] == pytest.approx(0.5, abs=0.05)
    assert res['warnings'] and 'jumps' in res['warnings'][0]      # |scale| != 1


def test_drift_too_high_fails():
    step, car, link, clock = make(drift_deg_per_min=5.0)
    distance_run(step, car)
    spin(step, car)
    res = drift(step, car, clock)
    assert not res['passed']
    assert [c['key'] for c in res['checks'] if not c['passed']] == ['drift']


def test_spin_test_off_needs_distance_only():
    cfg = copy.deepcopy(STEP6)
    cfg['procedure']['spin_test'] = False
    step, car, link, clock = make(cfg)
    distance_run(step, car)
    res = drift(step, car, clock)
    assert res['passed'] and 'spin' not in res['tests']


# ---------------------------------------------------------------- wizard: order, save, keep
def _wizard(tmp_path, step, clock):
    return wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'imu_odometry': step}, now=clock)


def _pass_earlier(tmp_path, w):
    from carbot_common import calibration_store as cs
    sess = w._ensure_session()
    for s in w.slots:
        if s.index < 6:
            cs.update_step(sess, s.id, 'PASS', '')
    w.refresh()
    return sess


def test_wizard_run_save_writes_overlay(tmp_path):
    step, car, link, clock = make(tpm_true=1100.0)
    w = _wizard(tmp_path, step, clock)
    sess = _pass_earlier(tmp_path, w)
    step.live({})
    r = w.action('imu_odometry', 'STEP', json.dumps({'op': 'distance_start'}), {})
    assert r['ok'], r
    car.push(2.0)
    w.action('imu_odometry', 'STEP', json.dumps({'op': 'distance_stop'}), {})
    distance_run(step, car)
    spin(step, car)
    assert w.action('imu_odometry', 'RUN', '', {})['ok']
    assert not w.action('imu_odometry', 'STEP', json.dumps({'op': 'distance_start'}), {})['ok']  # running
    done = None
    while done is None:
        car.wait(1.0)
        done = w.tick({})
    assert done.status == 'PASS'
    r = w.action('imu_odometry', 'SAVE', '', {})
    assert r['ok'], r
    ov = ct.load_yaml(os.path.join(sess, 'params_overlay.yaml'))[SERVO]['ros__parameters']
    assert ov['ticks_per_meter'] == pytest.approx(1100.0, rel=1e-3)
    assert ov['odom_reverse_polarity'] is False and ov['imu_yaw_scale'] == 1.0
    doc = yaml.safe_load(open(os.path.join(sess, '06_imu_odometry.yaml')))
    assert doc['data_files'] == ['params_overlay.yaml'] and doc['tests']['distance']['status'] == 'PASS'


def test_save_refused_when_values_changed_after_result(tmp_path):
    step, car, link, clock = make()
    distance_run(step, car)
    spin(step, car)
    res = drift(step, car, clock)
    step.values['ticks_per_meter'] = 999.0
    with pytest.raises(wc.StepRefused):
        step.save_data(str(tmp_path), res)


def test_keep_previous_copies_overlay_and_sets_live(tmp_path):
    src, dst = tmp_path / 'old', tmp_path / 'new'
    src.mkdir()
    dst.mkdir()
    ct.merge_overlay(str(src), SERVO, {'ticks_per_meter': 1111.0, 'odom_reverse_polarity': True,
                                       'imu_yaw_scale': -1.0})
    step, car, link, _ = make()
    step.keep_data(str(src), str(dst))
    ov = ct.load_yaml(os.path.join(dst, 'params_overlay.yaml'))[SERVO]['ros__parameters']
    assert ov == {'ticks_per_meter': 1111.0, 'odom_reverse_polarity': True, 'imu_yaw_scale': -1.0}
    assert link.values['imu_yaw_scale'] == -1.0 and link.values['ticks_per_meter'] == 1111.0


def test_keep_previous_refused_without_overlay(tmp_path):
    src = tmp_path / 'old'
    src.mkdir()
    ct.merge_overlay(str(src), SERVO, {'ticks_per_meter': 1111.0})
    step, *_ = make()
    with pytest.raises(wc.StepRefused, match='imu_yaw_scale'):
        step.keep_data(str(src), str(tmp_path))


def test_live_view_shape():
    step, car, link, _ = make()
    lv = step.live({})
    assert lv['values'] == START and lv['tests']['distance']['status'] == 'NOT_RUN'
    op(step, 'distance_start')
    car.push(0.5)
    lv = step.live({})
    assert lv['active']['test'] == 'distance' and lv['active']['distance_m'] > 0.4
    json.dumps(lv)


def test_recorder_ignores_samples_before_settle():
    rec = MotionRecorder()
    rec.on_imu(0.0, 170.0)
    rec.zero(ignore_until=1.0)
    rec.on_imu(0.5, -100.0)             # jump while settling: baseline only
    rec.on_imu(1.5, -90.0)
    assert rec.yaw == pytest.approx(10.0)
    assert math.isclose(wrap_deg(190.0), -170.0)


# --------------------------------------------------------------------------- delete runs
def delete(step, test, index=None):
    a = {'op': 'delete_run', 'test': test}
    if index is not None:
        a['index'] = index
    return step.handle('delete_run', a, {})


def test_delete_newest_corrected_run_undoes_its_correction():
    step, car, link, _ = make(tpm_true=1000.0)                      # odom reads 5 % long at 1050: a correction follows
    r = distance_run(step, car)
    assert r['status'] == 'CORRECTED' and link.values['ticks_per_meter'] != 1050.0
    out = delete(step, 'distance', 0)
    assert out['ok'] and out['invalidate_result'] and 'ticks_per_meter back to 1050' in out['message']
    assert link.values['ticks_per_meter'] == 1050.0 and step.runs['distance'] == []
    assert step.live({})['tests']['distance']['status'] == 'NOT_RUN'


def test_delete_older_run_leaves_the_live_value_and_says_so():
    step, car, link, _ = make(tpm_true=1000.0)
    distance_run(step, car)                                         # CORRECTED (first)
    second = distance_run(step, car)                                # verify run
    assert second['status'] == 'PASS'
    out = delete(step, 'distance', 0)                               # delete the OLD corrected run, not the newest
    assert out['ok'] and 'stays on' in out['message']
    assert link.values['ticks_per_meter'] != 1050.0 and len(step.runs['distance']) == 1
    assert step.live({})['tests']['distance']['status'] == 'PASS'   # the verify run is in charge again


def test_delete_when_live_value_changed_since_keeps_it():
    step, car, link, _ = make(tpm_true=1000.0)
    distance_run(step, car)
    link.values['ticks_per_meter'] = step.values['ticks_per_meter'] = 999.0      # set by hand afterwards + re-read
    out = delete(step, 'distance', 0)
    assert out['ok'] and 'left on' in out['message'] and link.values['ticks_per_meter'] == 999.0


def test_delete_spin_run_restores_the_scale():
    step, car, link, _ = make(imu_gain=1.1)
    r = spin(step, car)
    assert r['status'] == 'CORRECTED' and link.values['imu_yaw_scale'] != 1.0
    out = delete(step, 'spin', 0)
    assert out['ok'] and link.values['imu_yaw_scale'] == 1.0


def test_delete_refused_while_measuring_and_for_bad_input():
    step, car, link, _ = make()
    step.live({})
    assert not delete(step, 'distance', 0)['ok']                    # nothing to delete
    op(step, 'distance_start')
    out = delete(step, 'distance', 0)
    assert not out['ok'] and 'in progress' in out['message']
    op(step, 'abort')
    distance_run(step, car)
    assert not delete(step, 'nope', 0)['ok'] and not delete(step, 'distance', 7)['ok'] and not delete(step, 'distance')['ok']


def test_delete_all_runs_and_live_index():
    step, car, link, _ = make(tpm_true=1000.0)
    distance_run(step, car)
    distance_run(step, car)
    live = step.live({})['tests']['distance']
    assert [r['index'] for r in live['runs']] == [0, 1]
    out = step.handle('delete_all_runs', {'op': 'delete_all_runs', 'test': 'distance'}, {})
    assert out['ok'] and 'Values already set' in out['message'] and step.runs['distance'] == []


def test_deleting_a_run_drops_an_unsaved_step_result(tmp_path):
    step, car, link, clock = make(tpm_true=1100.0)
    w = _wizard(tmp_path, step, clock)
    _pass_earlier(tmp_path, w)
    distance_run(step, car)                                         # CORRECTED
    distance_run(step, car)                                         # PASS
    spin(step, car)
    assert w.action('imu_odometry', 'RUN', '', {})['ok']
    done = None
    while done is None:
        car.wait(1.0)
        done = w.tick({})
    assert done.status == 'PASS' and done.unsaved
    r = w.action('imu_odometry', 'STEP', json.dumps({'op': 'delete_run', 'test': 'distance', 'index': 1}), {})
    assert r['ok'], r
    slot = next(x for x in w.slots if x.id == 'imu_odometry')
    assert slot.status == 'PENDING' and slot.result is None and slot.unsaved is False
