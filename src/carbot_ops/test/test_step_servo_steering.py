"""Step 7 (servo centre + steering): supervised drive sequence, corrections, abort paths,
save / keep (no ROS). The simulated car drives from the requested CALIBRATION_RAW
commands like the real one: full lock = circles of radius RL / RR, straight = residual
curvature from the servo centre error; /imu/rpy = normalise(raw * imu_yaw_scale)."""
import copy
import json
import math
import os

import pytest

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_ops import wizard_core as wc
from carbot_ops.sensor_checks import ConfigError
from carbot_ops.step_imu_odometry import MotionRecorder, wrap_deg
from carbot_ops.step_servo_steering import ServoSteeringStep
from helpers import STEPS

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0, 'page_watch_s': 8.0}
STEP7 = next(s for s in STEPS['steps'] if s['id'] == 'servo_steering')
WB = 0.216
SERVO_START = {'servo_center': 90, 'servo_range_left': 50, 'servo_range_right': 70, 'imu_yaw_scale': 1.0}
OWNER_START = {'mode': 'calibrate', 'steering.steer_sign': -1.0}


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Link:
    def __init__(self, values, fail=False):
        self.values, self.fail, self.sets = dict(values), fail, []

    def get(self, names, done):
        done(None if self.fail else {n: self.values.get(n) for n in names})

    def set(self, values, done):
        if not self.fail:
            self.values.update(values)
            self.sets.append(dict(values))
        done(not self.fail)


class Drive:
    def __init__(self):
        self.cmd, self.log = None, []

    def command(self, source, speed, steer, reason):
        self.cmd = (source, speed, steer, reason)
        self.log.append(self.cmd)

    def stop(self):
        self.cmd = None


class Car:
    """RL / RR = full-lock radii, true_centre = servo_center that drives straight."""

    def __init__(self, clock, rec, servo, drive, RL=0.42, RR=0.44, true_centre=93, true_sign=-1.0,
                 imu_gain=1.0, raw_yaw=37.0, speed_per_duty=1.25, stuck=False, min_duty=0.0):
        self.clock, self.rec, self.servo, self.drive = clock, rec, servo, drive
        self.RL, self.RR, self.true_centre, self.true_sign = RL, RR, true_centre, true_sign
        self.gain, self.raw_yaw, self.k_speed, self.stuck = imu_gain, raw_yaw, speed_per_duty, stuck
        self.min_duty = min_duty          # static friction: the car only moves at or above this duty
        self.x = self.y = self.th = 0.0
        self.odom_on = True
        self.publish()

    def curvature(self, z):
        if z * self.true_sign > 0:
            return 1.0 / self.RL
        if z * self.true_sign < 0:
            return -1.0 / self.RR
        k_unit = (1.0 / self.RR) / self.servo.values['servo_range_right']
        return (self.true_centre - self.servo.values['servo_center']) * k_unit

    def publish(self):
        if self.odom_on:
            self.rec.on_odom(self.clock.t, self.x, self.y, self.th)
        self.rec.on_imu(self.clock.t, wrap_deg(wrap_deg(self.raw_yaw) * self.servo.values['imu_yaw_scale']))

    def step(self, dt=0.05):
        self.clock.t += dt
        c = self.drive.cmd
        if c is not None and not self.stuck:
            v = c[1] * self.k_speed if c[1] >= self.min_duty else 0.0
            ds = v * dt
            dth = ds * self.curvature(c[2])
            self.x += ds * math.cos(self.th + dth / 2)
            self.y += ds * math.sin(self.th + dth / 2)
            self.th += dth
            self.raw_yaw += math.degrees(dth) * self.gain
        self.publish()


def make(cfg=None, servo=None, owner=None, **car):
    clock, rec, drive = Clock(), MotionRecorder(), Drive()
    sl, ol = Link(servo or SERVO_START), Link(owner or OWNER_START)
    step = ServoSteeringStep(copy.deepcopy(cfg or STEP7), rec, sl, ol, drive, WB, clock=clock)
    return step, Car(clock, rec, sl, drive, **car), sl, drive, clock


def go(step):
    return step.handle('go', {'op': 'go'}, {})


def drive_until(step, car, phase='ready', limit_s=60.0):
    """Tick the step + car until it waits at `phase` again or finishes. -> result or None."""
    t_end = car.clock.t + limit_s
    while car.clock.t < t_end:
        car.step()
        res = step.tick(car.clock.t, {})
        if res is not None:
            return res
        if step.run and step.run['phase'] == phase:
            return None
    raise AssertionError(f'stuck in {step.run and step.run["phase"]}')


def full_run(step, car, max_segments=8):
    step.live({})
    assert step.start(car.clock.t, {}) is None
    for _ in range(max_segments):
        assert go(step)['ok']
        res = drive_until(step, car)
        if res is not None:
            return res
    raise AssertionError('never finished')


# ---------------------------------------------------------------- config
def test_repo_yaml_builds():
    step, *_ = make()
    assert step.duty == 0.16 and step.max_runs == 4


def test_missing_key_is_config_error():
    cfg = copy.deepcopy(STEP7)
    del cfg['procedure']['odom_timeout_s']
    with pytest.raises(ConfigError, match='odom_timeout_s'):
        make(cfg)


# ---------------------------------------------------------------- the drive sequence
def test_full_pass_corrects_centre_then_verifies():
    step, car, servo, drive, _ = make()
    res = full_run(step, car)
    assert res['passed'], res['summary']
    assert res['servo_center'] == 93 == servo.values['servo_center']
    assert res['left']['radius_m'] == pytest.approx(0.42, rel=0.03)
    assert res['right']['radius_m'] == pytest.approx(0.44, rel=0.03)
    assert res['min_turning_radius_m'] == pytest.approx(0.44, rel=0.03)
    assert res['steering']['left_max_rad'] == pytest.approx(math.atan(WB / 0.42), rel=0.03)
    assert res['steering']['steer_sign'] == -1.0
    assert len(res['straight_runs']) == 2 and res['straight_runs'][-1]['drift_m_per_m'] <= 0.02
    assert drive.cmd is None
    # left lock = steer_sign (-1), right = +1, straight = 0; all CALIBRATION_RAW at raw_duty
    driving = [c for c in drive.log if c[1] > 0]
    assert [c[2] for c in driving] == [-1.0, 1.0, 0.0, 0.0]
    assert {c[0] for c in drive.log} == {'CALIBRATION_RAW'} and {c[1] for c in driving} == {0.16}


def test_waits_for_go_before_every_segment():
    step, car, *_ = make()
    step.live({})
    step.start(car.clock.t, {})
    for _ in range(40):
        car.step()
        assert step.tick(car.clock.t, {}) is None
    assert step.run['phase'] == 'ready' and step.drive.cmd is None and car.x == 0.0
    go(step)
    drive_until(step, car, phase='driving')
    assert not go(step)['ok']                        # not while driving


def test_reversed_steering_flips_sign():
    step, car, *_ = make(true_sign=1.0)
    res = full_run(step, car)
    assert res['steering']['steer_sign'] == 1.0
    assert res['checks'][0]['passed'] and 'steer_sign' in res['note']


def test_non_unit_imu_scale_across_the_wrap():
    """IMU 5 % short, step 6 saved scale 1/0.95; the circles cross +-180 deg."""
    step, car, servo, *_ = make(servo=dict(SERVO_START, imu_yaw_scale=1 / 0.95), imu_gain=0.95, raw_yaw=150.0)
    res = full_run(step, car)
    assert res['passed'], res['summary']
    assert res['left']['radius_m'] == pytest.approx(0.42, rel=0.03)
    assert servo.values['imu_yaw_scale'] == pytest.approx(1 / 0.95)       # restored
    assert {'imu_yaw_scale': 1.0} in servo.sets


def test_wide_turns_fail_radius():
    step, car, *_ = make(RL=0.55, RR=0.60)
    res = full_run(step, car)
    assert not res['passed']
    assert [c['key'] for c in res['checks'] if not c['passed']] == ['min_radius']


def test_max_runs_stops_straight_runs():
    cfg = copy.deepcopy(STEP7)
    cfg['procedure']['max_runs'] = 1
    step, car, *_ = make(cfg)
    res = full_run(step, car)
    assert not res['passed'] and len(res['straight_runs']) == 1


# ---------------------------------------------------------------- abort paths
def test_cancel_stops_driving_and_restores_scale():
    step, car, servo, drive, _ = make(servo=dict(SERVO_START, imu_yaw_scale=0.97))
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    drive_until(step, car, phase='driving')
    assert drive.cmd is not None and servo.values['imu_yaw_scale'] == 1.0
    step.cancel()
    assert drive.cmd is None and servo.values['imu_yaw_scale'] == 0.97 and step.run is None


def test_odom_loss_aborts():
    step, car, servo, drive, _ = make()
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    drive_until(step, car, phase='driving')
    car.odom_on = False
    res = drive_until(step, car)
    assert res is not None and not res['passed'] and '/odom stopped' in res['summary']
    assert drive.cmd is None


def test_stuck_car_times_out():
    step, car, *_ = make(stuck=True)
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    res = drive_until(step, car)
    assert res is not None and not res['passed'] and 'not finished within' in res['summary']


def test_refused_outside_calibrate_mode():
    step, car, *_ = make(owner=dict(OWNER_START, mode='race'))
    step.live({})
    assert 'calibrate' in step.start(car.clock.t, {})


def test_go_needs_run():
    step, *_ = make()
    assert not go(step)['ok']


# ---------------------------------------------------------------- wizard, save, keep
def test_wizard_go_while_running_and_save(tmp_path):
    step, car, servo, drive, clock = make()
    w = wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'servo_steering': step}, now=clock)
    sess = w._ensure_session()
    for s in w.slots:
        if s.index < 7:
            cs.update_step(sess, s.id, 'PASS', '')
    w.refresh()
    step.live({})
    assert w.action('servo_steering', 'RUN', '', {})['ok']
    done = None
    for _ in range(8):
        r = w.action('servo_steering', 'STEP', json.dumps({'op': 'go'}), {})
        assert r['ok'], r
        while done is None and step.run and step.run['phase'] != 'ready':
            car.step()
            done = w.tick({})
        if done is None:
            car.step()
            done = w.tick({})
        if done is not None:
            break
    assert done is not None and done.status == 'PASS'
    assert w.action('servo_steering', 'SAVE', '', {})['ok']
    ov = ct.load_yaml(os.path.join(sess, 'params_overlay.yaml'))
    assert ov['servo_controller']['ros__parameters']['servo_center'] == 93
    st = ov['command_owner']['ros__parameters']['steering']
    assert st['steer_sign'] == -1.0 and st['trim_rad'] == 0.0 and st['angular_limit'] == 1.0
    assert ov['tunnel_bridge']['ros__parameters']['command_owner_steering'] == st
    assert ov['/**']['ros__parameters']['vehicle']['min_turning_radius_m'] == pytest.approx(0.44, rel=0.03)


def test_step_ops_still_refused_while_running_for_other_steps(tmp_path):
    class Plain(wc.StepImpl):
        def start(self, now, inputs):
            return None

        def handle(self, op, args, inputs):
            return {'ok': True, 'message': 'handled'}
    clock = Clock()
    w = wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'sensor_health': Plain({})}, now=clock)
    assert w.action('sensor_health', 'RUN', '', {})['ok']
    assert not w.action('sensor_health', 'STEP', json.dumps({'op': 'x'}), {})['ok']


def test_save_refused_when_centre_changed(tmp_path):
    step, car, servo, *_ = make()
    res = full_run(step, car)
    step.values['servo_center'] = 91
    with pytest.raises(wc.StepRefused):
        step.save_data(str(tmp_path), res)


def test_keep_previous_copies_all_and_sets_centre(tmp_path):
    src, dst = tmp_path / 'old', tmp_path / 'new'
    src.mkdir()
    dst.mkdir()
    steering = {'steer_sign': -1.0, 'left_max_rad': 0.47, 'right_max_rad': 0.45, 'trim_rad': 0.0, 'angular_limit': 1.0}
    ct.merge_overlay(str(src), 'servo_controller', {'servo_center': 94})
    ct.merge_overlay(str(src), 'command_owner', {f'steering.{k}': v for k, v in steering.items()})
    ct.merge_overlay(str(src), '/**', {'vehicle.min_turning_radius_m': 0.43})
    step, car, servo, *_ = make()
    step.keep_data(str(src), str(dst))
    ov = ct.load_yaml(os.path.join(dst, 'params_overlay.yaml'))
    assert ov['tunnel_bridge']['ros__parameters']['command_owner_steering'] == steering
    assert servo.values['servo_center'] == 94


def test_keep_previous_refused_without_steering(tmp_path):
    src = tmp_path / 'old'
    src.mkdir()
    ct.merge_overlay(str(src), 'servo_controller', {'servo_center': 94})
    step, *_ = make()
    with pytest.raises(wc.StepRefused, match='steer_sign'):
        step.keep_data(str(src), str(tmp_path))


def test_live_view_is_json():
    step, car, *_ = make()
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    drive_until(step, car, phase='driving')
    for _ in range(10):
        car.step()
        step.tick(car.clock.t, {})
    lv = step.live({})
    assert lv['run']['segment'] == 'left' and lv['run']['distance_m'] > 0
    json.dumps(lv)


# ---------------------------------------------------------------- stall boost (BACKLOG #63)
def test_normal_car_is_never_boosted():
    step, car, sl, drive, _ = make()
    res = full_run(step, car)
    assert res['passed']
    assert {c[1] for c in drive.log if c[1] > 0} == {step.duty}          # only the base duty was ever requested


def test_stalled_car_gets_more_duty_until_it_moves():
    step, car, sl, drive, _ = make(min_duty=0.23)                        # 0.16 does not move it
    step.live({})
    assert step.start(car.clock.t, {}) is None
    assert go(step)['ok']
    res = drive_until(step, car)
    assert res is None                                                    # the left circle finished, waits for Go
    duties = [c[1] for c in drive.log if c[1] > 0]
    assert duties[0] == step.duty and duties == sorted(duties) and duties[-1] >= 0.23
    assert all('stall boost' in c[3] for c in drive.log if c[1] > step.duty)
    assert max(duties) <= step.boost_max + 1e-9
    assert step.live({})['run']['boosts'] >= 4 and step.live({})['run']['base_duty'] == step.duty


def test_boost_is_capped_and_starts_over_each_segment():
    step, car, sl, drive, _ = make(min_duty=0.9)                          # never moves: above the cap
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    res = drive_until(step, car)
    assert res is not None and not res['passed'] and 'not finished within' in res['summary']
    assert max(c[1] for c in drive.log) <= step.boost_max + 1e-9
    step2, car2, sl2, drive2, _ = make(min_duty=0.23)
    step2.live({})
    step2.start(car2.clock.t, {})
    go(step2)
    drive_until(step2, car2)
    go(step2)                                                             # next segment: back to the base duty
    car2.step(); step2.tick(car2.clock.t, {})
    for _ in range(80):
        car2.step(); step2.tick(car2.clock.t, {})
        if step2.run['phase'] == 'driving':
            break
    assert [c[1] for c in drive2.log if c[1] > 0][-1] == step2.duty or step2.run['duty'] >= step2.duty


def test_stall_boost_keys_are_required_and_checked():
    bad = copy.deepcopy(STEP7)
    del bad['procedure']['stall_boost_step']
    with pytest.raises(ConfigError, match='stall_boost_step'):
        make(cfg=bad)
    worse = copy.deepcopy(STEP7)
    worse['procedure']['stall_boost_max_duty'] = worse['procedure']['raw_duty'] - 0.05
    with pytest.raises(ConfigError, match='stall_boost_max_duty'):
        make(cfg=worse)
