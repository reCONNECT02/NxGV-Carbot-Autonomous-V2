"""Step 8 (speed feedforward + PID): supervised drive sequence, feedforward fit, retune,
abort paths, save / keep, CLI replay of the saved capture (no ROS).

The simulated car obeys the requested commands like the real stack: CALIBRATION_RAW =
base duty straight to the motor; CALIBRATION = m/s through command_owner's own
SpeedController (owner_core, feedforward + PID with the live parameter values). The
motor: steady speed = (|duty| - STATIC) / PER_MPS, first-order lag TAU."""
import copy
import json
import math
import os

import pytest

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_control import calib_speed as csp
from carbot_control import owner_core as oc
from carbot_ops import wizard_core as wc
from carbot_ops.sensor_checks import ConfigError
from carbot_ops.step_imu_odometry import MotionRecorder
from carbot_ops.step_speed_pid import SpeedPidStep
from helpers import STEPS

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0, 'page_watch_s': 8.0}
STEP8 = next(s for s in STEPS['steps'] if s['id'] == 'speed_pid')
OWNER_START = {'mode': 'calibrate', 'calibration.duty_max': 0.5, 'speed_pid.kp': 0.8, 'speed_pid.ki': 0.4, 'speed_pid.kd': 0.0,
               'speed_pid.integral_limit': 0.15, 'feedforward.duty_per_mps': 1.0, 'feedforward.static_duty': 0.08}
STATIC, PER_MPS = 0.045, 0.8
N_SEG = len(STEP8['procedure']['sweep_duties']) + len(STEP8['procedure']['step_targets_mps'])
STEP8_BRK = STEP8                                               # the repo YAML: breakaway ramp + kick ON
STEP8 = copy.deepcopy(STEP8_BRK)                                # the older tests below run without it
STEP8['procedure']['breakaway']['enabled'] = False


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
    def __init__(self, clock, rec, owner, drive, static=STATIC, per=PER_MPS, tau=0.2, stuck=False, breakaway=0.0):
        self.clock, self.rec, self.owner, self.drive = clock, rec, owner, drive
        self.static, self.per, self.tau, self.stuck = static, per, tau, stuck
        self.breakaway = breakaway                    # static friction: a stopped car needs this duty to start
        self.ctrl = oc.SpeedController(oc.SpeedCfg())
        self.x = self.v = 0.0
        self.odom_on = True
        self.publish()

    def publish(self):
        if self.odom_on:
            self.rec.on_odom(self.clock.t, self.x, 0.0, 0.0, self.v)

    def _cfg(self):
        o = self.owner.values
        return oc.SpeedCfg(kp=o['speed_pid.kp'], ki=o['speed_pid.ki'], kd=o['speed_pid.kd'],
                           integral_limit=o['speed_pid.integral_limit'], duty_per_mps=o['feedforward.duty_per_mps'],
                           static_duty=o['feedforward.static_duty'])

    def step(self, dt=0.02):
        self.clock.t += dt
        c = self.drive.cmd
        duty = 0.0
        if c is None:
            self.ctrl.reset()
        elif c[0] == 'CALIBRATION_RAW':
            duty = c[1]
        else:
            self.ctrl.cfg = self._cfg()
            duty = self.ctrl.update(c[1], self.v, dt)
        v_ss = 0.0 if self.stuck else math.copysign(max(0.0, abs(duty) - self.static) / self.per, duty)
        if abs(self.v) < 0.002 and abs(duty) < self.breakaway:
            v_ss = 0.0
        self.v += (v_ss - self.v) * min(1.0, dt / self.tau)
        self.x += self.v * dt
        self.publish()


def make(cfg=None, owner=None, **car):
    clock, rec, drive = Clock(), MotionRecorder(), Drive()
    ol = Link(owner or OWNER_START)
    step = SpeedPidStep(copy.deepcopy(cfg or STEP8), rec, ol, drive, clock=clock)
    return step, Car(clock, rec, ol, drive, **car), ol, drive, clock


def go(step):
    return step.handle('go', {'op': 'go'}, {})


def drive_until(step, car, phase='ready', limit_s=60.0, tick_every=1):
    t_end, n = car.clock.t + limit_s, 0
    while car.clock.t < t_end:
        car.step()
        n += 1
        if n % tick_every:
            continue
        res = step.tick(car.clock.t, {})
        if res is not None:
            return res
        if step.run and step.run['phase'] == phase:
            return None
    raise AssertionError(f'stuck in {step.run and step.run["phase"]}')


def full_run(step, car, max_segments=40, tick_every=1):
    step.live({})
    assert step.start(car.clock.t, {}) is None
    for n in range(max_segments):
        assert go(step)['ok']
        res = drive_until(step, car, tick_every=tick_every)
        if res is not None:
            return res, n + 1
    raise AssertionError('never finished')


# ---------------------------------------------------------------- config
def test_repo_yaml_builds():
    step, *_ = make()
    assert step.duties[0] == 0.06 and step.targets == [0.03, 0.075, 0.15] and step.max_runs == 3


def test_missing_key_is_config_error():
    cfg = copy.deepcopy(STEP8)
    del cfg['procedure']['stop_settle_s']
    with pytest.raises(ConfigError, match='stop_settle_s'):
        make(cfg)
    cfg = copy.deepcopy(STEP8)
    del cfg['pass']['max_overshoot_pct']
    with pytest.raises(ConfigError, match='max_overshoot_pct'):
        make(cfg)


def test_bad_lists_are_config_errors():
    cfg = copy.deepcopy(STEP8)
    cfg['procedure']['sweep_duties'] = []
    with pytest.raises(ConfigError, match='sweep_duties'):
        make(cfg)
    cfg = copy.deepcopy(STEP8)
    cfg['procedure']['steady_s'] = 5.0
    with pytest.raises(ConfigError, match='steady_s'):
        make(cfg)


# ---------------------------------------------------------------- the drive sequence
def test_full_pass_fits_feedforward_and_verifies():
    step, car, owner, drive, _ = make()
    res, n = full_run(step, car)
    assert res['passed'], res['summary']
    assert n == N_SEG                                          # one Go per segment, one verify run
    ff = res['feedforward']
    assert ff['static_duty'] == pytest.approx(STATIC, abs=0.005)
    assert ff['duty_per_mps'] == pytest.approx(PER_MPS, rel=0.05)
    assert owner.values['feedforward.static_duty'] == pytest.approx(ff['static_duty'])       # live
    assert owner.values['feedforward.duty_per_mps'] == pytest.approx(ff['duty_per_mps'])
    assert res['max_steady_error_mps'] <= 0.01 and res['max_overshoot_pct'] <= 20
    assert res['pid'] == {'kp': 0.8, 'ki': 0.4, 'kd': 0.0, 'integral_limit': 0.15}
    assert len(res['verify_runs']) == 1 and res['step'] == 'speed_pid'
    assert set(res['check_flags']) == {'feedforward_fit', 'steady_error', 'overshoot', 'creep'}
    assert drive.cmd is None
    # sweep = CALIBRATION_RAW at the YAML duties, alternating; verify = CALIBRATION at the targets
    moving = [c for c in drive.log if c[1] != 0.0]
    assert [c[0] for c in moving] == ['CALIBRATION_RAW'] * 7 + ['CALIBRATION'] * 3
    assert [c[1] for c in moving] == [0.06, -0.08, 0.10, -0.12, 0.15, -0.18, 0.22, 0.03, -0.075, 0.15]
    assert all(c[2] == 0.0 for c in drive.log)


def test_pass_at_wizard_tick_rate():
    step, car, *_ = make()
    res, _ = full_run(step, car, tick_every=10)                # 0.2 s ticks, like tick_hz 5
    assert res['passed'], res['summary']


def test_waits_for_go_before_every_segment():
    step, car, *_ = make()
    step.live({})
    step.start(car.clock.t, {})
    for _ in range(100):
        car.step()
        assert step.tick(car.clock.t, {}) is None
    assert step.run['phase'] == 'ready' and step.drive.cmd is None and car.x == 0.0
    go(step)
    drive_until(step, car, phase='stopping')
    assert not go(step)['ok']                                  # not while driving / stopping


def test_overshoot_retunes_then_passes():
    owner = dict(OWNER_START, **{'speed_pid.kp': 3.0, 'speed_pid.ki': 6.0})
    step, car, ol, *_ = make(owner=owner, tau=0.5)
    res, n = full_run(step, car)
    assert len(res['verify_runs']) >= 2, res['summary']
    assert res['retunes'] and 'overshoot' in res['retunes'][0]
    first = res['verify_runs'][0]['pid']
    assert first['kp'] == 3.0 and res['verify_runs'][1]['pid']['kp'] == pytest.approx(2.1)
    assert n == 7 + 3 * len(res['verify_runs'])
    if res['passed']:
        assert ol.values['speed_pid.kp'] == res['pid']['kp']


def test_failing_run_restores_start_values():
    cfg = copy.deepcopy(STEP8)
    cfg['procedure']['max_runs'] = 1
    cfg['pass']['max_overshoot_pct'] = 0.0                     # cannot pass
    cfg['pass']['max_steady_error_mps'] = 0.0
    step, car, owner, *_ = make(cfg)
    res, _ = full_run(step, car)
    assert not res['passed'] and len(res['verify_runs']) == 1
    assert owner.values == OWNER_START                         # put back
    assert {c['key']: c['passed'] for c in res['checks']}['feedforward_fit']


def test_stuck_car_fails_the_fit_and_stops_after_the_sweep():
    step, car, owner, drive, _ = make(stuck=True)
    res, n = full_run(step, car)
    assert not res['passed'] and n == 7 and res['verify_runs'] == []
    bad = [c['key'] for c in res['checks'] if not c['passed']]
    assert 'feedforward_fit' in bad and 'only 0 moving steps' in res['note']
    assert owner.sets == [] and drive.cmd is None


def test_creep_fails_without_a_pointless_retune():
    """The slowest duty already moves too fast: no kp / ki change can fix that."""
    step, car, *_ = make(static=0.01)
    res, n = full_run(step, car)
    assert not res['passed'] and len(res['verify_runs']) == 1 and n == N_SEG
    assert [c['key'] for c in res['checks'] if not c['passed']] == ['creep']
    assert 'creep' in res['note']


# ---------------------------------------------------------------- abort paths
def test_cancel_stops_driving_and_restores():
    step, car, owner, drive, _ = make()
    step.live({})
    step.start(car.clock.t, {})
    for _ in range(7):
        go(step)
        drive_until(step, car)
    assert step.run['kind'] == 'verify' and owner.values['feedforward.static_duty'] != 0.08
    go(step)
    drive_until(step, car, phase='driving')
    assert drive.cmd is not None
    step.cancel()
    assert drive.cmd is None and step.run is None and owner.values == OWNER_START
    assert car.rec.trace is None


def test_odom_loss_aborts():
    step, car, owner, drive, _ = make()
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    car.odom_on = False
    res = drive_until(step, car)
    assert res is not None and not res['passed'] and '/odom stopped' in res['summary']
    assert res['checks'][0]['key'] == 'drive' and drive.cmd is None


def test_owner_refusing_parameters_aborts():
    step, car, owner, drive, _ = make()
    step.live({})
    step.start(car.clock.t, {})
    owner.fail = True
    res = None
    for _ in range(7):
        go(step)
        res = drive_until(step, car)
    res = res or drive_until(step, car)
    assert res is not None and not res['passed'] and 'did not accept' in res['summary']


def test_refused_outside_calibrate_mode():
    step, car, *_ = make(owner=dict(OWNER_START, mode='race'))
    step.live({})
    assert 'calibrate' in step.start(car.clock.t, {})


def test_start_reads_first():
    step, car, *_ = make()
    assert 'Try again' in step.start(car.clock.t, {})
    assert step.start(car.clock.t, {}) is None


def test_go_needs_run():
    step, *_ = make()
    assert not go(step)['ok']


# ---------------------------------------------------------------- wizard, save, keep, replay
def test_wizard_go_while_running_save_and_cli_replay(tmp_path):
    step, car, owner, drive, clock = make()
    w = wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'speed_pid': step}, now=clock)
    sess = w._ensure_session()
    for s in w.slots:
        if s.index < 8:
            cs.update_step(sess, s.id, 'PASS', '')
    w.refresh()
    step.live({})
    assert w.action('speed_pid', 'RUN', '', {})['ok']
    done = None
    for _ in range(N_SEG + 2):
        r = w.action('speed_pid', 'STEP', json.dumps({'op': 'go'}), {})
        assert r['ok'], r
        while done is None and step.run and step.run['phase'] != 'ready':
            car.step()
            done = w.tick({})
        if done is not None:
            break
    assert done is not None and done.status == 'PASS', done and done.result.get('summary')
    assert w.action('speed_pid', 'SAVE', '', {})['ok']
    ov = ct.load_yaml(os.path.join(sess, 'params_overlay.yaml'))
    co = ov['command_owner']['ros__parameters']
    ff = done.result['feedforward']
    assert co['feedforward'] == {'duty_per_mps': round(ff['duty_per_mps'], 4), 'static_duty': round(ff['static_duty'], 4)}
    assert co['speed_pid'] == {'kp': 0.8, 'ki': 0.4, 'kd': 0.0, 'integral_limit': 0.15}
    assert ov['tunnel_bridge']['ros__parameters']['command_owner_feedforward'] == co['feedforward']
    # the capture is what `calib_speed --replay` reads: same verdict and fit
    cap = json.load(open(os.path.join(sess, 'captures', 'speed.json')))
    again = csp.analyse(cap, STEP8)
    assert again['passed'] and again['feedforward']['static_duty'] == pytest.approx(ff['static_duty'])
    assert again['pid'] == done.result['pid']


def test_cli_overlay_writer_matches_its_keys(tmp_path):
    ff = {'duty_per_mps': 0.812345, 'static_duty': 0.04321}
    pid = {'kp': 0.8, 'ki': 0.4, 'kd': 0.0, 'integral_limit': 0.15}
    csp.write_overlay(str(tmp_path), ff, pid)
    ov = ct.load_yaml(os.path.join(str(tmp_path), 'params_overlay.yaml'))
    got = {n: sorted(k for k in csp.OVERLAY_KEYS[n] if ct.overlay_value(str(tmp_path), n, k) is not None)
           for n in csp.OVERLAY_KEYS}
    assert got == {n: sorted(v) for n, v in csp.OVERLAY_KEYS.items()}
    assert ov['command_owner']['ros__parameters']['feedforward'] == {'duty_per_mps': 0.8123, 'static_duty': 0.0432}


def test_save_refused_when_owner_values_changed(tmp_path):
    step, car, *_ = make()
    res, _ = full_run(step, car)
    step.values['pid']['kp'] = 1.1
    with pytest.raises(wc.StepRefused):
        step.save_data(str(tmp_path), res)


def test_keep_previous_copies_all_and_sets_live(tmp_path):
    src, dst = tmp_path / 'old', tmp_path / 'new'
    src.mkdir()
    dst.mkdir()
    csp.write_overlay(str(src), {'duty_per_mps': 0.79, 'static_duty': 0.05},
                      {'kp': 0.6, 'ki': 0.5, 'kd': 0.0, 'integral_limit': 0.15})
    step, car, owner, *_ = make()
    step.keep_data(str(src), str(dst))
    ov = ct.load_yaml(os.path.join(dst, 'params_overlay.yaml'))
    assert ov['tunnel_bridge']['ros__parameters']['command_owner_feedforward'] == {'duty_per_mps': 0.79,
                                                                                 'static_duty': 0.05}
    assert ov['command_owner']['ros__parameters']['speed_pid']['kp'] == 0.6
    assert owner.values['speed_pid.kp'] == 0.6 and owner.values['feedforward.static_duty'] == 0.05


def test_keep_previous_refused_without_values(tmp_path):
    src = tmp_path / 'old'
    src.mkdir()
    ct.merge_overlay(str(src), 'command_owner', {'feedforward.duty_per_mps': 0.8})
    step, *_ = make()
    with pytest.raises(wc.StepRefused, match='static_duty'):
        step.keep_data(str(src), str(tmp_path))


def test_live_view_is_json():
    step, car, *_ = make()
    step.live({})
    step.start(car.clock.t, {})
    go(step)
    for _ in range(50):
        car.step()
        step.tick(car.clock.t, {})
    lv = step.live({})
    run = lv['run']
    assert run['kind'] == 'sweep' and run['phase'] == 'driving' and run['segment']['command'] == 0.06
    assert run['trace'] and lv['speed_mps'] > 0
    json.dumps(lv)
    for _ in range(9):
        go(step)
        if drive_until(step, car) is not None:
            break
    json.dumps(step.live({}))


# ---------------------------------------------------------------- breakaway ramp + kick
BREAKAWAY = 0.14


def test_breakaway_is_measured_both_ways_then_every_segment_is_kicked():
    step, car, owner, drive, _ = make(cfg=STEP8_BRK, breakaway=BREAKAWAY)
    res, n = full_run(step, car)
    assert res['passed'], res['summary']
    assert n == N_SEG + 2                                      # + forward and backward breakaway ramps
    bd = res['breakaway_duty']
    assert BREAKAWAY <= bd['forward'] <= BREAKAWAY + 0.04 and BREAKAWAY <= bd['backward'] <= BREAKAWAY + 0.04
    assert res['capture']['breakaway'] == bd
    assert 'breakaway' in res['summary']
    ff = res['feedforward']                                    # the 0.06 duty moved the car thanks to the kick
    assert ff['static_duty'] == pytest.approx(STATIC, abs=0.01)
    assert len(ff['points']) >= 6
    # a low sweep duty is preceded by a KICK (a bigger raw duty) and then drops straight to the segment value
    cmds = [c for c in drive.log if c[1] != 0.0]
    i = next(k for k, c in enumerate(cmds) if c[1] == 0.06 and k > 3)         # first sweep duty, after the ramps
    assert cmds[i - 1][0] == 'CALIBRATION_RAW' and cmds[i - 1][1] > bd['forward']
    assert drive.cmd is None


def test_closed_loop_step_is_kicked_too():
    step, car, owner, drive, _ = make(cfg=STEP8_BRK, breakaway=BREAKAWAY)
    full_run(step, car)
    seq = [c for c in drive.log if c[1] != 0.0]
    j = next(k for k, c in enumerate(seq) if c[0] == 'CALIBRATION')
    assert seq[j - 1][0] == 'CALIBRATION_RAW' and abs(seq[j - 1][1]) > BREAKAWAY - 0.01


def test_car_that_never_moves_aborts_the_breakaway_ramp():
    step, car, owner, drive, _ = make(cfg=STEP8_BRK, stuck=True)
    res, n = full_run(step, car)
    assert not res['passed'] and n == 1
    assert 'did not move at duty' in res['note'] or any('did not move' in c['why'] for c in res['checks'])
    assert max(abs(c[1]) for c in drive.log) <= STEP8_BRK['procedure']['breakaway']['max_duty'] + 1e-9
    assert drive.cmd is None and owner.sets == []


def test_breakaway_config_errors():
    cfg = copy.deepcopy(STEP8_BRK)
    del cfg['procedure']['breakaway']['step_s']
    with pytest.raises(ConfigError, match='step_s'):
        make(cfg)
    cfg = copy.deepcopy(STEP8_BRK)
    cfg['procedure']['breakaway']['max_duty'] = 0.01
    with pytest.raises(ConfigError, match='max_duty'):
        make(cfg)
    cfg = copy.deepcopy(STEP8_BRK)
    del cfg['procedure']['breakaway']
    with pytest.raises(ConfigError, match='breakaway'):
        make(cfg)


def test_live_view_shows_the_ramp():
    step, car, *_ = make(cfg=STEP8_BRK, breakaway=BREAKAWAY)
    step.live({})
    assert step.start(car.clock.t, {}) is None
    assert go(step)['ok']
    for _ in range(40):
        car.step()
        step.tick(car.clock.t, {})
    lv = step.live({})['run']
    assert lv['phase'] == 'ramping' and lv['duty'] > 0.06 and lv['kind'] == 'breakaway'
    json.dumps(step.live({}))


def test_breakaway_max_duty_above_the_owner_cap_is_refused():
    cfg = copy.deepcopy(STEP8_BRK)
    cfg['procedure']['breakaway']['max_duty'] = 0.60
    step, car, owner, drive, _ = make(cfg=cfg)
    step.live({})
    msg = step.start(car.clock.t, {})
    assert msg and 'calibration.duty_max' in msg and step.run is None
    cfg['procedure']['breakaway']['max_duty'] = 0.50            # equal to the cap is fine
    step, car, *_ = make(cfg=cfg)
    step.live({})
    assert step.start(car.clock.t, {}) is None
