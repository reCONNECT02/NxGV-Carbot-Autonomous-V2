"""Step 7 (servo centre + steering limits) + wizard_drive plumbing, against SimCar (no ROS)."""
import copy
import json
import math
import os

import pytest
import yaml

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_control import calib_steering as cst
from carbot_ops import wizard_core as wc
from carbot_ops import wizard_drive as wd
from carbot_ops.step_servo_steering import ConfigError, ServoSteeringStep
from carbot_ops.wizard_core import StepRefused
from helpers import STEPS

STEP7 = next(s for s in STEPS['steps'] if s['id'] == 'servo_steering')
WB = 0.216
CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0}


class Rig:
    """SimCar + DriveMeter + DictParams around one ServoSteeringStep, on a fake clock."""

    def __init__(self, owner_sign=-1.0, true_sign=-1.0, centre=90, true_center=93, servo_vals=None, cfg=None):
        self.t = 100.0
        self.meter = wd.DriveMeter(3000)
        self.cmd = wd.DriveCommand(0.6)
        vals = {'servo_center': centre, 'servo_range_left': 50, 'servo_range_right': 70}
        if servo_vals is not None:
            vals = servo_vals
        self.servo = wd.DictParams(vals)
        self.owner = wd.DictParams({'mode': 'calibrate', 'steering.steer_sign': owner_sign})
        self.car = wd.SimCar(self.meter, self.cmd, self.servo, true_sign=true_sign, true_center=true_center)
        self.kit = wd.DriveKit(self.cmd, self.servo, self.owner, 1.0)
        self.step = ServoSteeringStep(cfg or STEP7, WB, self.kit)
        self.sim(0.2)                      # a few odom / IMU samples before Run

    def now(self):
        return self.t

    def inputs(self, **kw):
        d = {'drive': self.meter.snapshot(self.t, 0.5), 'drive_history': self.meter.history}
        d.update(kw)
        return d

    def sim(self, seconds, dt=0.05):
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            self.car.step(self.t, dt)

    def run(self, arg='', limit_s=80.0, tick=None, on_tick=None):
        err = self.step.start(self.t, self.inputs(argument=arg))
        assert err is None, err
        return self.until_done(limit_s, tick, on_tick)

    def until_done(self, limit_s=80.0, tick=None, on_tick=None):
        tick = tick or self.step.tick
        steps = 0
        while steps * 0.2 < limit_s:
            self.sim(0.2)
            if on_tick:
                on_tick(self)
            r = tick(self.t, self.inputs())
            steps += 1
            if r is not None:
                return r
        raise AssertionError('step never finished')


# ---------------------------------------------------------------- config
def test_repo_yaml_builds_and_writes_match_cli():
    Rig()
    assert STEP7['procedure']['settle_s'] > 0
    nodes = {n for n, _ in cst.OVERLAY_KEYS}
    assert nodes == {'servo_controller', 'command_owner', 'tunnel_bridge', '/**'}


def test_missing_key_is_loud():
    cfg = copy.deepcopy(STEP7)
    del cfg['procedure']['settle_s']
    with pytest.raises(ConfigError, match='settle_s'):
        ServoSteeringStep(cfg, WB, Rig().kit)
    with pytest.raises(ConfigError, match='wheelbase'):
        ServoSteeringStep(STEP7, 0.0, Rig().kit)


# ---------------------------------------------------------------- refusals
def test_start_refusals():
    r = Rig()
    assert 'feed' in r.step.start(r.t, {'argument': ''})
    assert 'Unknown run' in r.step.start(r.t, r.inputs(argument='spin'))
    assert 'Run circles first' in r.step.start(r.t, r.inputs(argument='straight'))
    r.meter.on_estop(r.t, True)
    assert 'STOP MOTORS' in r.step.start(r.t, r.inputs())
    r.meter.on_estop(r.t, False)
    r.t += 2.0                                   # no odom for 2 s
    assert '/odom' in r.step.start(r.t, r.inputs())


# ---------------------------------------------------------------- the procedure
def test_circles_then_straight_with_live_centre_correction():
    r = Rig()
    seen = []
    res = r.run('', on_tick=lambda rig: seen.append(rig.cmd.current(rig.t)))
    # never anything but the raw calibration source, full lock left (= steer_sign -1) then right
    srcs = {c[0] for c in seen if c}
    assert srcs == {wd.RAW}
    steers = [c[2] for c in seen if c and c[3] != 'stop']
    assert steers[0] == -1.0 and steers[-1] == 1.0
    assert r.cmd.current(r.t) is None            # stopped at the end
    assert not res['passed'] and res['next'] == 'straight' and 'still to do' in res['summary']
    assert res['left']['radius_m'] == pytest.approx(0.38, rel=0.03)
    assert res['right']['radius_m'] == pytest.approx(0.41, rel=0.03)
    assert res['checks'] == {'direction': True, 'min_radius': True, 'straight': False}
    assert res['min_turning_radius_m'] == pytest.approx(max(res['left']['radius_m'], res['right']['radius_m']))
    assert res['left_max_rad'] == pytest.approx(math.atan(WB / res['left']['radius_m']))

    live = r.step.live(r.inputs())
    assert live['circles']['left']['ok'] and live['next'] == 'straight'

    res = r.run('')                              # straight 1: centre 90, true 93 -> drifts left
    assert not res['passed'] and res['next'] == 'straight'
    assert r.servo.sets and r.servo.sets[-1]['servo_center'] > 90
    assert 'set LIVE' in res['note']
    new_c = r.servo.values['servo_center']
    res = r.run('straight')                      # straight 2 with the corrected centre
    assert res['passed'], res['summary']
    assert res['servo_center'] == new_c
    assert res['steering']['steer_sign'] == -1.0 and res['steering']['angular_limit'] == 1.0
    assert set(res['steering']) == set(cst.STEERING_KEYS)
    assert [c['passed'] for c in res['check_rows']] == [True, True, True]
    assert res['capture']['straight'][-1]['servo_center'] == new_c
    for k in ('left', 'right', 'straight_runs', 'checks', 'min_turning_radius_m', 'left_max_rad', 'right_max_rad',
              'step', 'wheelbase_m', 'servo_center', 'steering'):
        assert k in res                          # the CLI's keys
    yaml.safe_dump(res)                          # plain types only


def test_reversed_steering_flips_steer_sign():
    r = Rig(owner_sign=1.0, true_sign=-1.0)      # owner thinks +1 = left, the base says -1
    res = r.run('circles')
    assert res['checks']['direction'], res['check_rows']
    assert res['steering']['steer_sign'] == -1.0
    assert 'wrong way' in res['note']


def test_one_side_wrong_fails_direction():
    r = Rig()
    r.car.curvature = lambda z: (-1 / 0.4 if z else 0.0)      # both locks turn right
    res = r.run('circles')
    assert not res['checks']['direction'] and res['next'] == 'circles'


def test_estop_stops_at_once():
    r = Rig()

    def press(rig):
        if rig.step.phase == 'left' and rig.step.seg.progress()['yaw_deg'] > 30:
            rig.meter.on_estop(rig.t, True)
    res = r.run('circles', on_tick=press)
    assert res['aborted'] and 'STOP MOTORS' in res['summary']
    assert r.cmd.current(r.t) is None


def test_cancel_stops_the_command():
    r = Rig()
    assert r.step.start(r.t, r.inputs(argument='circles')) is None
    for _ in range(5):
        r.sim(0.2)
        r.step.tick(r.t, r.inputs())
    assert r.cmd.current(r.t) is not None
    r.step.cancel()
    assert r.cmd.current(r.t) is None and r.step.phase is None


def test_owner_not_accepting_aborts():
    r = Rig()
    r.car.blocked = True
    res = r.run('circles')
    assert res['aborted'] and 'command_owner did not take' in res['summary']


def test_owner_wrong_mode_and_missing_servo_params():
    r = Rig()
    r.owner.values['mode'] = 'race'
    res = r.run('circles')
    assert res['aborted'] and 'not calibrate' in res['summary']
    r = Rig(servo_vals={'servo_center': 90})
    res = r.run('circles')
    assert res['aborted'] and 'servo_range_left' in res['summary']


def test_failed_live_set_and_max_runs():
    r = Rig(true_center=99)

    class NoSet(wd.DictParams):
        def set(self, values, done):
            done(False)
    r.servo = r.kit.servo = r.step.kit.servo = NoSet(r.servo.values)
    r.car.servo = r.servo
    r.run('circles')
    for i in range(STEP7['procedure']['max_runs']):
        res = r.run('straight')
        assert not res['passed'] and 'failed' in res['note']
    assert res['next'] == 'circles'
    assert 'max_runs' in r.step.start(r.t, r.inputs(argument='straight'))


def test_command_expires_without_refresh():
    c = wd.DriveCommand(0.6)
    c.set(10.0, wd.RAW, 0.16, 0.0, 'x')
    assert c.current(10.5) is not None and c.current(10.7) is None
    with pytest.raises(ValueError):
        c.set(10.0, 'ROAD', 0.1, 0.0, 'x')


def test_meter_distance_sign_and_yaw_unwrap():
    m = wd.DriveMeter(10)
    m.on_odom(0.0, 0.0, 0.0, 0.0)
    m.on_odom(0.1, 1.0, 0.0, 0.5)
    m.on_odom(0.2, 0.5, 0.0, -0.5)
    assert m.dist == pytest.approx(0.5)
    for k, deg in enumerate((170, -175, -160)):   # crossing +-180 going left
        m.on_yaw_deg(k, deg)
    assert math.degrees(m.yaw) == pytest.approx(30)
    assert not m.on_imu_json(0, 'nope') and m.on_imu_json(0, json.dumps({'yaw': -160}))
    snap = m.snapshot(0.3, 0.5)
    assert snap['odom_ok'] and snap['owner_winner'] == ''


def test_segment_timeout_settles():
    r = Rig()
    seg = wd.Segment(wd.RAW, 0.16, 0.0, 't', lambda d, y, el: False, 1.0, 0.4, 1.0)
    seg.start(r.t, r.inputs())
    out = None
    while out is None:
        r.sim(0.2)
        out = seg.tick(r.t, r.inputs(), r.cmd)
    assert out['aborted'].startswith('timeout') and r.cmd.current(r.t) is None
    assert out['rows'] and out['rows'][0]['t'] >= 0


# ---------------------------------------------------------------- wizard flow, save, keep
def seed(root, upto=6):
    """A session where steps 1..upto passed (their pages are other agents' work)."""
    session = cs.open_session(str(root), '20260923_090000')
    for s in STEPS['steps'][:upto]:
        cs.update_step(session, s['id'], 'PASS', '')
    return session


def test_wizard_run_save_and_keep(tmp_path):
    r = Rig()
    session = seed(tmp_path)
    w = wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'servo_steering': r.step}, now=r.now)
    assert w.session == session
    for arg in ('circles', 'straight', 'straight'):
        a = w.action('servo_steering', 'RUN', arg, r.inputs())
        assert a['ok'], a
        done = r.until_done(tick=lambda now, inputs: w.tick(inputs))
        assert done.id == 'servo_steering'
    assert w.slot('servo_steering').status == 'PASS'
    a = w.action('servo_steering', 'SAVE', '', {})
    assert a['ok'], a
    ov = ct.load_yaml(os.path.join(session, 'params_overlay.yaml'))
    c = r.servo.values['servo_center']
    assert ov['servo_controller']['ros__parameters']['servo_center'] == c
    assert ov['command_owner']['ros__parameters']['steering']['steer_sign'] == -1.0
    assert set(ov['tunnel_bridge']['ros__parameters']['command_owner_steering']) == set(cst.STEERING_KEYS)
    assert ov['/**']['ros__parameters']['vehicle']['min_turning_radius_m'] > 0.3
    assert os.path.isfile(os.path.join(session, 'captures', 'steering.json'))
    doc = ct.load_yaml(os.path.join(session, '07_servo_steering.yaml'))
    assert doc['passed'] and doc['checks']['straight'] and doc['data_files']

    # the CLI replays the saved capture to the same numbers
    cap = json.load(open(os.path.join(session, 'captures', 'steering.json')))
    again = cst.analyse(cap, STEP7, WB)
    assert again['min_turning_radius_m'] == pytest.approx(doc['min_turning_radius_m'])

    # keep previous: copy every overlay key into a new session
    dst = cs.open_session(str(tmp_path), '20260923_100000')
    paths = r.step.keep_data(session, dst)
    assert any(p.endswith('steering.json') for p in paths)
    for node, key in cst.OVERLAY_KEYS:
        assert ct.overlay_value(dst, node, key) == ct.overlay_value(session, node, key)
    empty = cs.open_session(str(tmp_path), '20260923_110000')
    with pytest.raises(StepRefused, match='run this step again'):
        r.step.keep_data(empty, dst)


def test_save_refuses_incomplete(tmp_path):
    r = Rig()
    res = r.run('circles')
    with pytest.raises(StepRefused):
        r.step.save_data(str(tmp_path), res)


def test_cli_write_overlay_unchanged(tmp_path):
    """calib_steering main --replay still writes every overlay key (refactor guard)."""
    r = Rig()
    r.run('circles')
    r.run('straight')
    res = r.run('straight')
    cap_path = tmp_path / 'cap.json'
    cap_path.write_text(json.dumps(res['capture']))
    cfg_dir = ct.bringup_config_dir()
    rc = cst.main(['--replay', str(cap_path), '--data-root', str(tmp_path), '--session', 'cli', '--config-dir', cfg_dir])
    assert rc == 0
    s = os.path.join(cs.calibration_dir(str(tmp_path)), 'cli')
    for node, key in cst.OVERLAY_KEYS:
        assert ct.overlay_value(s, node, key) is not None
