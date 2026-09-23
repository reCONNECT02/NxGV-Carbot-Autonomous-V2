"""Wizard state machine on the repo's real 13-step YAML, in a temp data root (no ROS)."""
import copy
import os

import pytest
import yaml

from carbot_common import calibration_store as cs
from carbot_ops import wizard_core as wc
from carbot_ops.step_sensor_health import SensorHealthStep
from helpers import CAMERAS, STEP1, STEPS, UWB, good

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0}


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Feed:
    """Simulated SystemHealth arrivals."""

    def __init__(self, snap=None):
        self.seq, self.snap = 0, snap or good()

    def next(self):
        self.seq += 1
        return {'snap': self.snap, 'health_seq': self.seq}


def make(root, steps=STEPS, cfg=CFG, clock=None):
    clock = clock or Clock()
    impl = {'sensor_health': SensorHealthStep(STEP1, CAMERAS, UWB)}
    return wc.Wizard(steps, str(root), dict(cfg), impl, now=clock), clock


def run_step1(w, clock, feed):
    r = w.action('sensor_health', 'RUN', '', feed.next())
    assert r['ok'], r
    done = None
    for _ in range(12):
        clock.t += 0.5
        done = w.tick(feed.next()) or done
    return done


def test_thirteen_steps_placeholders_and_optional(tmp_path):
    w, _ = make(tmp_path)
    st = w.state()
    assert len(st['steps']) == 13 and st['session'] == ''
    # step 12 mission_planner is required since its page exists (BACKLOG #20); 13 practice stays optional
    assert [s['status'] for s in st['steps']][-2:] == ['PENDING', 'SKIPPED_OPTIONAL']
    r = w.action('camera_identity', 'RUN', '', {})
    assert not r['ok'] and 'no wizard page yet' in r['message']


def test_run_pass_save_writes_session_but_not_active(tmp_path):
    w, clock = make(tmp_path)
    done = run_step1(w, clock, Feed())
    assert done is not None and done.status == 'PASS'
    assert w.state()['steps'][0]['can_advance'] is False        # not saved yet
    r = w.action('sensor_health', 'SAVE', '', {})
    assert r['ok'] and '01_sensor_health.yaml' in r['message']
    sess = w.session
    doc = yaml.safe_load(open(os.path.join(sess, '01_sensor_health.yaml')))
    assert doc['passed'] and doc['n'] == 11
    assert cs.load_summary(sess)['steps']['sensor_health']['status'] == 'PASS'
    assert cs.active_session(str(tmp_path)) is None            # 10 required steps still open
    assert w.state()['steps'][0]['can_advance'] is True
    assert 'Race mode keeps using' in r['message']


def test_failed_run_cannot_be_saved_and_blocks_next(tmp_path):
    w, clock = make(tmp_path)
    bad = good()
    bad['topics']['/scan'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    done = run_step1(w, clock, Feed(bad))
    assert done.status == 'FAIL'
    lid = next(c for c in done.result['checks'] if c['key'] == 'lidar')
    assert not lid['passed'] and lid['fix']
    r = w.action('sensor_health', 'SAVE', '', {})
    assert not r['ok'] and 'Only a passing' in r['message']
    assert not w.state()['steps'][0]['can_advance']


def test_no_health_reports_fails_with_reason(tmp_path):
    w, clock = make(tmp_path)
    w.action('sensor_health', 'RUN', '', {'snap': {}, 'health_seq': 0})
    for _ in range(12):
        clock.t += 0.5
        done = w.tick({'snap': {}, 'health_seq': 0})
        if done:
            break
    assert done.status == 'FAIL' and 'system_monitor is not publishing' in done.result['summary']


def test_resave_keeps_old_file(tmp_path):
    w, clock = make(tmp_path)
    feed = Feed()
    run_step1(w, clock, feed)
    w.action('sensor_health', 'SAVE', '', {})
    run_step1(w, clock, feed)
    assert w.action('sensor_health', 'SAVE', '', {})['ok']
    files = [f for f in os.listdir(w.session) if f.startswith('01_sensor_health')]
    assert len(files) == 2


def test_order_is_enforced(tmp_path):
    class Dummy(wc.StepImpl):
        def tick(self, now, inputs):
            return {'passed': True, 'summary': 'ok'}
    w, _ = make(tmp_path)
    w.impls['camera_identity'] = Dummy({})
    r = w.action('camera_identity', 'RUN', '', {})
    assert not r['ok'] and 'Finish step 1' in r['message']


def test_cancel_and_single_runner(tmp_path):
    w, clock = make(tmp_path)
    feed = Feed()
    w.action('sensor_health', 'RUN', '', feed.next())
    assert not w.action('sensor_health', 'RUN', '', feed.next())['ok']
    r = w.cancel_running('STOP MOTORS pressed')
    assert r['ok'] and w.slot(1).status == 'PENDING' and w.running is None


def test_keep_previous_and_placeholder_refusal(tmp_path):
    w, clock = make(tmp_path)
    run_step1(w, clock, Feed())
    w.action('sensor_health', 'SAVE', '', {})
    first = w.session_name()
    w2, _ = make(tmp_path, cfg=dict(CFG, resume_max_age_h=0))    # force a NEW session
    assert w2.slot(1).previous == first
    r = w2.action('sensor_health', 'KEEP_PREVIOUS', '', {})
    assert r['ok'] and w2.slot(1).status == 'KEPT_PREVIOUS'
    assert os.path.isfile(os.path.join(w2.session, '01_sensor_health.yaml'))
    r = w2.action('camera_identity', 'KEEP_PREVIOUS', '', {})
    assert not r['ok'] and 'not available' in r['message']


def test_keep_previous_without_history(tmp_path):
    w, _ = make(tmp_path)
    r = w.action('sensor_health', 'KEEP_PREVIOUS', '', {})
    assert not r['ok'] and 'must pass now' in r['message']


def test_resume_unfinished_session(tmp_path):
    w, clock = make(tmp_path)
    run_step1(w, clock, Feed())
    w.action('sensor_health', 'SAVE', '', {})
    w2, _ = make(tmp_path)
    assert w2.session == w.session and 'Resumed' in w2.notice
    assert w2.slot(1).status == 'PASS' and w2.current == 2


def test_refresh_picks_up_terminal_tool_result(tmp_path):
    w, clock = make(tmp_path)
    run_step1(w, clock, Feed())
    w.action('sensor_health', 'SAVE', '', {})
    cs.update_step(w.session, 'camera_intrinsics', 'PASS', '03_camera_intrinsics.yaml')
    assert w.refresh()
    assert w.slot(3).status == 'PASS'
    w.action('imu_odometry', 'SELECT', '', {})
    assert w.live({})['step']['meta']['tool_cmd'] == \
        'ros2 run carbot_localization calib_odometry --session ' + w.session_name()


def test_active_only_when_all_required_pass(tmp_path):
    w, clock = make(tmp_path)
    run_step1(w, clock, Feed())
    w.action('sensor_health', 'SAVE', '', {})
    for s in w.slots[1:]:
        if s.required:
            cs.update_step(w.session, s.id, 'PASS', '')
    w.refresh()
    run_step1(w, clock, Feed())
    r = w.action('sensor_health', 'SAVE', '', {})
    assert 'now ACTIVE' in r['message']
    assert cs.active_session(str(tmp_path)) == w.session


def test_rollback(tmp_path):
    w, clock = make(tmp_path)
    run_step1(w, clock, Feed())
    w.action('sensor_health', 'SAVE', '', {})
    r = w.action('', 'ROLLBACK', w.session_name(), {})
    assert r['ok'] and 'refuse to arm' in r['message']
    assert not w.action('', 'ROLLBACK', 'nope', {})['ok']


def test_unwritable_root_gives_message(tmp_path):
    root = tmp_path / 'ro'
    root.mkdir()
    w, clock = make(root)
    run_step1(w, clock, Feed())
    os.chmod(root, 0o500)
    try:
        r = w.action('sensor_health', 'SAVE', '', {})
    finally:
        os.chmod(root, 0o700)
    if os.geteuid() != 0:
        assert not r['ok'] and 'Cannot write' in r['message']


def test_bad_yaml_is_config_error(tmp_path):
    bad = copy.deepcopy(STEPS)
    bad['steps'][3]['index'] = 9
    with pytest.raises(wc.ConfigError, match='indices'):
        make(tmp_path, steps=bad)
    with pytest.raises(wc.ConfigError, match='resume_max_age_h'):
        make(tmp_path, cfg={'session_format': 'x', 'allow_keep_previous': True})


def test_unknown_action_and_step(tmp_path):
    w, _ = make(tmp_path)
    assert not w.action('sensor_health', 'FLY', '', {})['ok']
    assert not w.action('nope', 'RUN', '', {})['ok']


def test_live_payload_shape(tmp_path):
    w, _ = make(tmp_path)
    live = w.live(Feed().next())
    assert live['step']['built'] and live['step']['live']['n'] == 11
    assert live['step']['meta']['instructions']
    w.action('camera_intrinsics', 'SELECT', '', {})
    live = w.live({})
    assert not live['step']['built'] and live['step']['blocked_by']['index'] == 1
