"""Step 2 (camera identity): front-only (repo cameras.yaml) and 3-camera logic, save/keep data (no ROS)."""
import copy
import json
import os

import pytest
import yaml

from carbot_common import calibration_store as cs
from carbot_common.data import unconfirmed_roles
from carbot_ops import wizard_core as wc
from carbot_ops.sensor_checks import ConfigError
from carbot_ops.step_camera_identity import CameraIdentityStep, parse_argument
from carbot_ops.step_sensor_health import SensorHealthStep
from helpers import CAMERAS, REPO_CAMERAS, STEP1, STEPS, UWB, good

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0}
STEP2 = next(s for s in STEPS['steps'] if s['id'] == 'camera_identity')
CONFIRM = json.dumps({'confirm': True, 'swap': False})
SWAPPED = json.dumps({'confirm': True, 'swap': True})


def front_only():
    cams = copy.deepcopy(REPO_CAMERAS)
    for name, s in cams['sensors'].items():
        s['enabled'] = name == 'astra'
    return cams


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Feed:
    def __init__(self, snap=None):
        self.seq, self.snap = 0, snap or good()

    def next(self):
        self.seq += 1
        return {'snap': self.snap, 'health_seq': self.seq}


def make(root, cameras, clock=None):
    clock = clock or Clock()
    impls = {'sensor_health': SensorHealthStep(STEP1, cameras, UWB),
             'camera_identity': CameraIdentityStep(STEP2, cameras)}
    return wc.Wizard(STEPS, str(root), dict(CFG), impls, now=clock), clock


def run(w, clock, feed, step, arg=''):
    r = w.action(step, 'RUN', arg, feed.next())
    assert r['ok'], r
    done = None
    for _ in range(14):
        clock.t += 0.5
        done = w.tick(feed.next()) or done
    return done


def pass_step1(w, clock):
    run(w, clock, Feed(), 'sensor_health')
    assert w.action('sensor_health', 'SAVE', '', {})['ok']


def session_cameras(w):
    with open(os.path.join(w.session, 'data', 'cameras.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------- config
def test_repo_yaml_builds_and_repo_roles_unconfirmed():
    CameraIdentityStep(STEP2, REPO_CAMERAS)
    assert REPO_CAMERAS['roles_confirmed'] is False and REPO_CAMERAS['roles_confirmed_for'] == []
    assert set(REPO_CAMERAS['roles']) == {'front', 'left_rear', 'right_rear'}   # role keys never dropped
    assert unconfirmed_roles(REPO_CAMERAS) == [r for r, s in REPO_CAMERAS['roles'].items()
                                               if REPO_CAMERAS['sensors'][s]['enabled']]


@pytest.mark.parametrize('where,key', [('procedure', 'measure_s'), ('pass', 'max_image_age_s')])
def test_missing_step_key_is_config_error(where, key):
    cfg = copy.deepcopy(STEP2)
    del cfg[where][key]
    with pytest.raises(ConfigError, match=key):
        CameraIdentityStep(cfg, CAMERAS)


def test_missing_cameras_key_is_error():
    cams = copy.deepcopy(CAMERAS)
    del cams['roles_confirmed_for']
    with pytest.raises(ConfigError, match='roles_confirmed_for'):
        CameraIdentityStep(STEP2, cams)
    cams = copy.deepcopy(CAMERAS)
    del cams['sensors']['ov5647']['enabled']
    with pytest.raises(KeyError, match='enabled'):
        CameraIdentityStep(STEP2, cams)


def test_parse_argument():
    assert parse_argument(CONFIRM) == (False, '')
    assert parse_argument(SWAPPED) == (True, '')
    for bad in ('', 'x', '{}', '{"confirm": false}', '[1]'):
        assert parse_argument(bad)[0] is None


# ---------------------------------------------------------------- front-only
def test_front_only_live_view_skips_side_cameras():
    step = CameraIdentityStep(STEP2, front_only())
    lv = step.live({'snap': good(), 'health_seq': 1})
    by = {c['role']: c for c in lv['cameras']}
    assert by['front']['enabled'] and by['front']['state'] == 'live' and by['front']['preview'] == 'cam_front'
    assert by['left_rear']['state'] == 'off' and by['right_rear']['state'] == 'off'
    assert lv['sides_enabled'] is False


def test_front_only_confirm_save_writes_complete_cameras_yaml(tmp_path):
    cams = front_only()
    w, clock = make(tmp_path, cams)
    pass_step1(w, clock)
    done = run(w, clock, Feed(), 'camera_identity', CONFIRM)
    assert done.status == 'PASS', done.result
    assert done.result['confirmed_roles'] == ['front']
    assert set(done.result['skipped_roles']) == {'left_rear', 'right_rear'}
    assert 'skipped left side, right side' in done.result['summary']
    r = w.action('camera_identity', 'SAVE', '', {})
    assert r['ok'] and 'data/cameras.yaml' in r['message'], r
    saved = session_cameras(w)
    assert saved['roles'] == cams['roles']                      # side role keys kept, unchanged
    assert saved['roles_confirmed'] is True and saved['roles_confirmed_for'] == ['front']
    assert saved['sensors'] == cams['sensors'] and saved['mounts'] == cams['mounts']   # complete file
    assert unconfirmed_roles(saved) == []
    # a side camera switched on later is unconfirmed again
    saved['sensors'][saved['roles']['left_rear']]['enabled'] = True
    assert unconfirmed_roles(saved) == ['left_rear']
    assert cs.load_summary(w.session)['steps']['camera_identity']['status'] == 'PASS'


def test_swap_refused_when_sides_off(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    r = w.action('camera_identity', 'RUN', SWAPPED, Feed().next())
    assert not r['ok'] and 'nothing to swap' in r['message']


def test_run_needs_confirmation_argument(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    r = w.action('camera_identity', 'RUN', '', Feed().next())
    assert not r['ok'] and 'Confirm' in r['message']


def test_order_step1_first(tmp_path):
    w, _ = make(tmp_path, front_only())
    r = w.action('camera_identity', 'RUN', CONFIRM, Feed().next())
    assert not r['ok'] and 'Finish step 1' in r['message']


def test_unsaved_step1_pass_says_press_save(tmp_path):
    w, clock = make(tmp_path, front_only())
    run(w, clock, Feed(), 'sensor_health')                    # PASS, but Save not pressed
    r = w.action('camera_identity', 'RUN', CONFIRM, Feed().next())
    assert not r['ok'] and 'not saved yet' in r['message'] and 'press Save' in r['message']
    w.action('camera_identity', 'SELECT', '', {})
    blk = w.live({})['step']['blocked_by']
    assert blk['unsaved_pass'] is True and 'press Save' in blk['text']


def test_frozen_front_picture_fails(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    snap = good()
    snap['topics']['/camera/color/image_raw'] = {'hz': 0.0, 'age': 4.0, 'latency': -1}
    done = run(w, clock, Feed(snap), 'camera_identity', CONFIRM)
    assert done.status == 'FAIL'
    bad = [c for c in done.result['checks'] if not c['passed']]
    assert bad and bad[0]['key'] == 'stream_front' and bad[0]['fix']
    assert not w.action('camera_identity', 'SAVE', '', {})['ok']


def test_disabled_side_topics_do_not_matter(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    snap = good()
    for t in ('/cam_ov5647/image_raw', '/cam_imx219/image_raw'):
        snap['topics'].pop(t)
    assert run(w, clock, Feed(snap), 'camera_identity', CONFIRM).status == 'PASS'


def test_no_health_reports_fails(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    r = w.action('camera_identity', 'RUN', CONFIRM, {'snap': good(), 'health_seq': 5})
    assert r['ok']
    done = None
    for _ in range(10):
        clock.t += 0.5
        done = w.tick({'snap': good(), 'health_seq': 5}) or done
    assert done.status == 'FAIL' and 'system_monitor' in done.result['summary']


# ---------------------------------------------------------------- three cameras
def test_three_cameras_swap_saves_swapped_roles(tmp_path):
    w, clock = make(tmp_path, CAMERAS)
    pass_step1(w, clock)
    done = run(w, clock, Feed(), 'camera_identity', SWAPPED)
    assert done.status == 'PASS' and done.result['swapped']
    assert done.result['confirmed_roles'] == ['front', 'left_rear', 'right_rear']
    assert w.action('camera_identity', 'SAVE', '', {})['ok']
    saved = session_cameras(w)
    assert saved['roles']['left_rear'] == CAMERAS['roles']['right_rear']
    assert saved['roles']['right_rear'] == CAMERAS['roles']['left_rear']
    assert saved['roles']['front'] == 'astra'


def test_three_cameras_one_side_missing_fails(tmp_path):
    w, clock = make(tmp_path, CAMERAS)
    pass_step1(w, clock)
    snap = good()
    snap['topics'].pop('/cam_ov5647/image_raw')
    done = run(w, clock, Feed(snap), 'camera_identity', CONFIRM)
    assert done.status == 'FAIL'
    assert [c['key'] for c in done.result['checks'] if not c['passed']] == \
        ['stream_' + next(r for r, s in CAMERAS['roles'].items() if s == 'ov5647')]


# ---------------------------------------------------------------- save merges, keep previous
def test_save_keeps_keys_an_earlier_step_wrote(tmp_path):
    w, clock = make(tmp_path, front_only())
    pass_step1(w, clock)
    data = os.path.join(w.session, 'data')
    os.makedirs(data, exist_ok=True)
    other = front_only()
    other['sensors']['astra']['intrinsics_file'] = '/x/astra.yaml'          # as step 3 would
    cs.write_yaml(os.path.join(data, 'cameras.yaml'), other)
    run(w, clock, Feed(), 'camera_identity', CONFIRM)
    assert w.action('camera_identity', 'SAVE', '', {})['ok']
    saved = session_cameras(w)
    assert saved['sensors']['astra']['intrinsics_file'] == '/x/astra.yaml'
    assert saved['roles_confirmed_for'] == ['front']


def _passed_session(tmp_path, cams, clock):
    w, _ = make(tmp_path, cams, clock)
    pass_step1(w, clock)
    run(w, clock, Feed(), 'camera_identity', CONFIRM)
    assert w.action('camera_identity', 'SAVE', '', {})['ok']
    return os.path.basename(w.session)


def test_keep_previous_copies_roles(tmp_path):
    clock = Clock()
    first = _passed_session(tmp_path, front_only(), clock)
    cfg = dict(CFG, resume_max_age_h=0.0)                     # new wizard = new session
    impls = {'sensor_health': SensorHealthStep(STEP1, front_only(), UWB),
             'camera_identity': CameraIdentityStep(STEP2, front_only())}
    w2 = wc.Wizard(STEPS, str(tmp_path), cfg, impls, now=clock)
    pass_step1(w2, clock)
    assert w2.slot('camera_identity').previous == first
    r = w2.action('camera_identity', 'KEEP_PREVIOUS', '', {})
    assert r['ok'], r
    saved = session_cameras(w2)
    assert saved['roles_confirmed'] is True and saved['roles_confirmed_for'] == ['front']
    assert cs.load_summary(w2.session)['steps']['camera_identity']['status'] == 'KEPT_PREVIOUS'


def test_keep_previous_refused_when_a_side_camera_is_on_now(tmp_path):
    clock = Clock()
    _passed_session(tmp_path, front_only(), clock)
    cfg = dict(CFG, resume_max_age_h=0.0)
    impls = {'sensor_health': SensorHealthStep(STEP1, CAMERAS, UWB),
             'camera_identity': CameraIdentityStep(STEP2, CAMERAS)}     # all three on now
    w2 = wc.Wizard(STEPS, str(tmp_path), cfg, impls, now=clock)
    pass_step1(w2, clock)
    r = w2.action('camera_identity', 'KEEP_PREVIOUS', '', {})
    assert not r['ok'] and 'run this step again' in r['message']
    assert 'camera_identity' not in (cs.load_summary(w2.session).get('steps') or {})
    assert not os.path.isfile(os.path.join(w2.session, 'data', 'cameras.yaml'))
