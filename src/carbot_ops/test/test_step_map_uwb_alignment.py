"""Step 11 (map-to-UWB alignment) without ROS: a simulated car goes around a lap
(MotionRecorder fed like /odom + /imu/rpy), a synthetic UWB tag reports ranges from
where the car really is in the venue frame, and the real wizard_core saves the result."""
import copy
import json
import math
import os

import pytest
import yaml

from carbot_common import calibration_store as cs
from carbot_ops import wizard_core as wc
from carbot_ops import wizard_uwb as wu
from carbot_ops.step_imu_odometry import MotionRecorder, wrap_deg
from carbot_ops.step_map_uwb_alignment import (WRITES, ConfigError, MapUwbAlignmentStep, basis, parse_argument,
                                               to_track)
from carbot_localization.alignment import tag_position
from helpers import DATA, STEPS, UWB

CONFIG_DIR = os.path.dirname(DATA)
CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0, 'page_watch_s': 8.0}
STEP11 = next(s for s in STEPS['steps'] if s['id'] == 'map_uwb_alignment')
TRUE = {'1782': (0.0, 0.0, 1.2), '1786': (7.5, 0.0, 1.2), '1783': (7.5, 4.83, 1.2)}
BIAS = {'1782': 0.95, '1786': 1.02, '1783': 0.88}
TAG = {'z_m': 0.15, 'mount_xy_m': [0.12, 0.0]}
T2V = {'x_m': 0.40, 'y_m': -0.25, 'yaw_deg': 2.0}          # the truth the fit must find


def step10_doc(bias=BIAS, tag=TAG):
    """data/uwb.yaml as step 10 saves it: surveyed anchors, measured offsets, tag mount."""
    d = copy.deepcopy(UWB)
    d['anchors'] = [{'id': a, 'xyz_m': list(v), 'range_offset_m': bias[a]} for a, v in TRUE.items()]
    d['tag'] = dict(d['tag'], **copy.deepcopy(tag))
    d['anchors_surveyed'] = d['offsets_calibrated'] = True
    return d


def to_venue(x, y, t=T2V):
    yaw = math.radians(t['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y + t['x_m'], s * x + c * y + t['y_m']


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """Wizard with step 11 only (index 1), the real UwbFeed + MotionRecorder, a simulated car."""

    def __init__(self, root, clock=None, resume=12.0, step10=True, cfg=None):
        self.clock = clock or Clock()
        self.motion = MotionRecorder()
        self.cfg = dict(cfg or STEP11, index=1)
        self.wiz = None
        self.step = MapUwbAlignmentStep(self.cfg, copy.deepcopy(UWB), CONFIG_DIR, self.motion,
                                        lambda: self.wiz.session if self.wiz else None, clock=self.clock)
        self.wiz = wc.Wizard({'steps': [self.cfg]}, str(root), dict(CFG, resume_max_age_h=resume),
                             {'map_uwb_alignment': self.step}, now=self.clock)
        self.feed = wu.UwbFeed(30.0, 3.0, clock=self.clock)
        self.tag = wu.SyntheticTag(TRUE, TAG['z_m'], BIAS, noise_m=0.01)
        # car: true track pose; odom has its own frame, the IMU its own zero
        self.pose = list(self.step.poses['start_pose'])
        self.odom = [2.0, -1.0, 0.7]
        self.imu0 = 37.0
        self.odom_scale = 1.0                               # 1.0 = step 6 calibrated the encoder
        if step10:
            self.write_step10()
        self.feed_sensors()

    def write_step10(self, doc=None):
        session = self.wiz._ensure_session()
        cs.write_yaml(os.path.join(session, 'data', 'uwb.yaml'), doc or step10_doc())

    def inputs(self):
        return {wu.INPUT_KEY: self.feed}

    def do(self, action, argument=''):
        return self.wiz.action('map_uwb_alignment', action, argument, self.inputs())

    def feed_sensors(self):
        t = self.clock.t
        self.motion.on_odom(t, self.odom[0], self.odom[1], self.odom[2])
        self.motion.on_imu(t, wrap_deg(math.degrees(self.pose[2]) + self.imu0))
        tx, ty = tag_position(tuple(self.pose), tuple(TAG['mount_xy_m']))
        self.tag.xy = list(to_venue(tx, ty))

    def move(self, ds, dth):
        self.pose[2] += dth
        self.pose[0] += ds * math.cos(self.pose[2])
        self.pose[1] += ds * math.sin(self.pose[2])
        self.odom[2] += dth
        self.odom[0] += ds * self.odom_scale * math.cos(self.odom[2])
        self.odom[1] += ds * self.odom_scale * math.sin(self.odom[2])

    def advance(self, seconds, motion=None, done=None, stop_at=None):
        """20 Hz sensors, 10 Hz tag, 5 Hz wizard tick. motion(t) -> (v m/s, w rad/s)."""
        for k in range(int(round(seconds * 20))):
            self.clock.t += 0.05
            if motion:
                v, w = motion(k * 0.05)
                self.move(v * 0.05, w * 0.05)
            self.feed_sensors()
            if k % 2 == 0:
                self.feed.add(self.tag.report())
            if k % 4 == 0:
                done = self.wiz.tick(self.inputs()) or done
            if done:
                return done
        return done


def lap_motion(v=0.3, straights=(4.0, 3.0, 4.0, 3.0), r=0.5):
    """Clockwise rounded rectangle from the start pose (facing west): straight, 90 deg right turn, ..."""
    segs = []
    for L in straights:
        segs.append((L / v, v, 0.0))
        segs.append((math.pi / 2 * r / v, v, -v / r))
    total = sum(s[0] for s in segs)

    def f(t):
        for d, vv, w in segs:
            if t < d:
                return vv, w
            t -= d
        return 0.0, 0.0
    return f, total


def run_lap(rig, **kw):
    motion, total = lap_motion(**kw)
    r = rig.do('RUN', json.dumps({'mode': 'lap'}))
    assert r['ok'], r
    assert rig.advance(total + 1.0, motion) is None            # still recording: the user presses Stop
    assert rig.do('STEP', json.dumps({'op': 'stop'}))['ok']
    return rig.advance(1.0)


def session_uwb(w):
    with open(os.path.join(w.session, 'data', 'uwb.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------- config
def test_repo_yaml_builds_and_writes_list_matches():
    step = MapUwbAlignmentStep(STEP11, UWB, CONFIG_DIR, MotionRecorder(), lambda: None)
    assert [w.replace('uwb.yaml ', '') for w in STEP11['writes']] == list(WRITES)
    assert {'start_pose', 'light_goal_pose'} <= set(step.poses)
    assert step.modes == ['lap', 'points']


@pytest.mark.parametrize('where,key', [('procedure', 'lap_min_s'), ('procedure', 'huber_m'),
                                       ('procedure', 'points_min_extent_m'), ('procedure', 'overlay_max_points'),
                                       ('pass', 'max_rms_m'), ('pass', 'min_inlier_frac')])
def test_missing_step_key_is_config_error(where, key):
    cfg = copy.deepcopy(STEP11)
    del cfg[where][key]
    with pytest.raises(ConfigError, match=key):
        MapUwbAlignmentStep(cfg, UWB, CONFIG_DIR, MotionRecorder(), lambda: None)


def test_bad_mode_list_is_config_error():
    cfg = copy.deepcopy(STEP11)
    cfg['procedure']['modes'] = ['lap', 'spiral']
    with pytest.raises(ConfigError, match='modes'):
        MapUwbAlignmentStep(cfg, UWB, CONFIG_DIR, MotionRecorder(), lambda: None)


def test_parse_argument():
    poses = {'start_pose': (1, 2, 3)}
    assert parse_argument('{"mode": "lap"}', poses) == ({'mode': 'lap'}, '')
    assert parse_argument('{"mode": "points", "pose": "start_pose"}', poses)[0] == {'mode': 'points',
                                                                                     'pose': 'start_pose'}
    for bad in ('', 'x', '{}', '{"mode": "fly"}', '{"mode": "points"}', '{"mode": "points", "pose": "P9"}'):
        assert parse_argument(bad, poses)[0] is None, bad


def test_to_track_inverts_track_to_venue():
    x, y = to_venue(1.5, 2.5)
    assert to_track(T2V, x, y) == pytest.approx((1.5, 2.5), abs=1e-9)


# ---------------------------------------------------------------- refusals
def test_refused_without_step10_session_data(tmp_path):
    rig = Rig(tmp_path, step10=False)
    r = rig.do('RUN', '{"mode": "lap"}')
    assert not r['ok'] and 'step 10' in r['message'] and 'No calibration session' in r['message']
    rig.wiz._ensure_session()
    r = rig.do('RUN', '{"mode": "lap"}')
    assert not r['ok'] and 'no data/uwb.yaml' in r['message'] and 'step 10' in r['message']
    rig.write_step10(dict(step10_doc(), offsets_calibrated=False))
    r = rig.do('RUN', '{"mode": "lap"}')
    assert not r['ok'] and 'offsets_calibrated' in r['message']
    live = rig.step.live(rig.inputs())
    assert live['ready'] is False and 'step 10' in live['blocked']


def test_lap_refused_without_odometry(tmp_path):
    rig = Rig(tmp_path)
    rig.clock.t += 5.0                                      # /odom and /imu/rpy went silent
    r = rig.do('RUN', '{"mode": "lap"}')
    assert not r['ok'] and '/odom' in r['message']


def test_mode_switched_off_in_yaml(tmp_path):
    cfg = copy.deepcopy(STEP11)
    cfg['procedure']['modes'] = ['points']
    rig = Rig(tmp_path, cfg=cfg)
    r = rig.do('RUN', '{"mode": "lap"}')
    assert not r['ok'] and 'switched off' in r['message']


# ---------------------------------------------------------------- lap
def test_lap_finds_the_transform_and_save_merges_only_track_to_venue(tmp_path):
    rig = Rig(tmp_path)
    before = session_uwb(rig.wiz)
    done = run_lap(rig)
    res = done.result
    assert res['passed'], res['summary']
    f = res['fit']
    assert f['x_m'] == pytest.approx(T2V['x_m'], abs=0.05)
    assert f['y_m'] == pytest.approx(T2V['y_m'], abs=0.05)
    assert f['yaw_deg'] == pytest.approx(T2V['yaw_deg'], abs=0.5)
    assert res['capture']['duration_s'] >= STEP11['procedure']['lap_min_s']
    assert res['track_to_venue'] == {'x_m': f['x_m'], 'y_m': f['y_m'], 'yaw_deg': f['yaw_deg'], 'aligned': True}
    assert res['map']['sha1'] and res['map']['file'].endswith('track_map.yaml')
    assert res['basis'] == basis(before)
    r = rig.do('SAVE')
    assert r['ok'], r
    after = session_uwb(rig.wiz)
    assert after['track_to_venue'] == res['track_to_venue']
    # step 10's keys survive untouched
    for k in ('anchors', 'tag', 'anchors_surveyed', 'offsets_calibrated', 'agent'):
        assert after[k] == before[k], k
    assert set(after) == set(before)
    saved = os.path.join(rig.wiz.session, '01_map_uwb_alignment.yaml')
    doc = yaml.safe_load(open(saved, encoding='utf-8'))
    assert doc['map']['sha1'] == res['map']['sha1'] and doc['track_to_venue']['aligned'] is True
    assert os.path.isfile(os.path.join(rig.wiz.session, 'captures', 'step11_lap_uwb.jsonl'))
    assert 'data/uwb.yaml' in doc['data_files']


def test_lap_too_short_fails(tmp_path):
    rig = Rig(tmp_path)
    motion, _ = lap_motion(v=0.6)
    assert rig.do('RUN', '{"mode": "lap"}')['ok']
    rig.advance(20.0, motion)
    rig.do('STEP', '{"op": "stop"}')
    res = rig.advance(1.0).result
    assert not res['passed']
    bad = {c['key'] for c in res['checks'] if not c['passed']}
    assert 'lap_time' in bad
    assert rig.do('SAVE')['ok'] is False


def test_lap_off_the_start_pose_gives_a_shifted_transform(tmp_path):
    """A rigid start error cannot be seen by the fit: it moves the transform. Hence 'start EXACTLY on
    the start pose' and the overlay check on the map."""
    rig = Rig(tmp_path)
    rig.pose[0] += 0.3
    res = run_lap(rig).result
    assert res['fit']['x_m'] == pytest.approx(T2V['x_m'] + 0.3, abs=0.06)


def test_uncalibrated_odometry_fails(tmp_path):
    rig = Rig(tmp_path)
    rig.odom_scale = 1.25                                   # encoder reads 25 % long (step 6 not done)
    res = run_lap(rig).result
    assert not res['passed']
    assert {'rms', 'inliers'} & {c['key'] for c in res['checks'] if not c['passed']}


def test_odometry_stopping_mid_lap_fails(tmp_path):
    rig = Rig(tmp_path)
    assert rig.do('RUN', '{"mode": "lap"}')['ok']
    rig.advance(3.0, lap_motion()[0])
    done = None
    for _ in range(20):                                      # tag keeps talking, odom / IMU silent
        rig.clock.t += 0.2
        rig.feed.add(rig.tag.report())
        done = rig.wiz.tick(rig.inputs()) or done
    assert done is not None and not done.result['passed']
    assert done.result['checks'][0]['key'] == 'inputs'


def test_live_view_during_lap_has_overlay_and_is_json(tmp_path):
    rig = Rig(tmp_path)
    assert rig.do('RUN', '{"mode": "lap"}')['ok']
    rig.advance(25.0, lap_motion()[0])
    live = rig.wiz.live(rig.inputs())['step']['live']
    json.dumps(live)
    run = live['run']
    assert run['mode'] == 'lap' and run['reports'] > 100 and run['extent_m'] > 3.0
    assert run['fit'] is not None and run['samples'] > 50
    assert run['fix_error_m'] < 0.3 and run['fix_frame'] == 'fit so far'
    ov = run['overlay']
    assert len(ov['path']) <= STEP11['procedure']['overlay_max_points'] and ov['fixes'] and ov['anchors']
    assert 'start_pose' in ov['poses']


# ---------------------------------------------------------------- points
def park(rig, name):
    x, y, a = rig.step.poses[name]
    rig.pose = [x, y, a]
    r = rig.do('RUN', json.dumps({'mode': 'points', 'pose': name}))
    assert r['ok'], r
    return rig.advance(STEP11['procedure']['points_seconds'] + 1.0)


def test_points_mode_needs_two_poses_then_passes(tmp_path):
    rig = Rig(tmp_path)
    first = park(rig, 'start_pose').result
    assert not first['passed'] and 'Point start_pose recorded (1 of 2)' in first['message']
    sx, sy, _ = rig.step.poses['start_pose']
    far = max(rig.step.poses, key=lambda k: math.hypot(rig.step.poses[k][0] - sx, rig.step.poses[k][1] - sy))
    res = park(rig, far).result
    assert res['passed'], (far, rig.step.poses, res['summary'])
    assert res['capture']['points'] == ['start_pose', far]
    assert res['fit']['x_m'] == pytest.approx(T2V['x_m'], abs=0.08)
    assert res['fit']['yaw_deg'] == pytest.approx(T2V['yaw_deg'], abs=1.5)
    assert rig.do('SAVE')['ok']
    assert session_uwb(rig.wiz)['track_to_venue']['aligned'] is True
    assert rig.do('STEP', '{"op": "clear_points"}')['ok']
    assert rig.step.points == []


# ---------------------------------------------------------------- stale data
def test_save_refused_when_step10_changed_after_the_fit(tmp_path):
    rig = Rig(tmp_path)
    assert run_lap(rig).result['passed']
    rig.write_step10(step10_doc(bias=dict(BIAS, **{'1782': 0.80})))
    r = rig.do('SAVE')
    assert not r['ok'] and 'stale' in r['message']
    assert session_uwb(rig.wiz)['track_to_venue']['aligned'] is False


def _aligned_session(tmp_path, clock):
    rig = Rig(tmp_path, clock=clock)
    assert run_lap(rig).result['passed']
    assert rig.do('SAVE')['ok']
    return rig.wiz.session_name(), session_uwb(rig.wiz)['track_to_venue']


def test_keep_previous_copies_track_to_venue(tmp_path):
    clock = Clock()
    first, t2v = _aligned_session(tmp_path, clock)
    clock.t += 5.0
    rig = Rig(tmp_path, clock=clock, resume=0.0)                 # new session, same step-10 data
    assert rig.wiz.slot('map_uwb_alignment').previous == first
    before = session_uwb(rig.wiz)
    r = rig.do('KEEP_PREVIOUS')
    assert r['ok'], r
    after = session_uwb(rig.wiz)
    assert after['track_to_venue'] == t2v
    assert {k: v for k, v in after.items() if k != 'track_to_venue'} == \
        {k: v for k, v in before.items() if k != 'track_to_venue'}


def test_keep_previous_refused_when_anchors_or_offsets_differ(tmp_path):
    clock = Clock()
    _aligned_session(tmp_path, clock)
    clock.t += 5.0
    rig = Rig(tmp_path, clock=clock, resume=0.0, step10=False)
    rig.write_step10(step10_doc(bias=dict(BIAS, **{'1786': 0.90})))
    r = rig.do('KEEP_PREVIOUS')
    assert not r['ok'] and 'stale' in r['message']
    assert session_uwb(rig.wiz)['track_to_venue']['aligned'] is False
    moved = step10_doc()
    moved['anchors'][1]['xyz_m'] = [7.4, 0.0, 1.2]
    rig.write_step10(moved)
    assert 'stale' in rig.do('KEEP_PREVIOUS')['message']


def test_keep_previous_refused_without_alignment(tmp_path):
    rig = Rig(tmp_path)
    src = tmp_path / 'old'
    (src / 'data').mkdir(parents=True)
    with pytest.raises(wc.StepRefused, match='no data/uwb.yaml'):
        rig.step.keep_data(str(tmp_path / 'nothing'), rig.wiz.session)
    cs.write_yaml(str(src / 'data' / 'uwb.yaml'), step10_doc())      # aligned false
    with pytest.raises(wc.StepRefused, match='aligned false'):
        rig.step.keep_data(str(src), rig.wiz.session)
