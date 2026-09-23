"""Step 5 page logic (step_lidar_camera) with the real camera model and repo YAML."""
import copy
import json
import math
import os

import numpy as np
import pytest
import yaml

from carbot_ops import step_lidar_camera as slc
from carbot_ops.wizard_core import StepRefused
from carbot_common import calib_tools as ct
from carbot_perception import camera_model as cm

from test_lidar_align import NOMINAL, fake_scan

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', '..', 'carbot_bringup', 'config', 'data')
TRUE_DELTA = 3.0
TARGETS = ((0.80, 0.26), (0.95, 0.0), (0.75, -0.24))


def load(name):
    with open(os.path.join(DATA, name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def step_cfg():
    return next(s for s in load('calibration_steps.yaml')['steps'] if s['id'] == 'lidar_camera')


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def make(mount=None, cams=None):
    cams = cams or load('cameras.yaml')
    clk = Clock()
    st = slc.LidarCameraStep(step_cfg(), cams, lambda: copy.deepcopy(cams),
                             lambda: list(mount or NOMINAL), clock=clk)
    return st, cams, clk


def true_mount():
    m = list(NOMINAL)
    m[3] += math.radians(TRUE_DELTA)
    return m


def inputs_for(target, clk, mount=None):
    scans = [dict(fake_scan([target], mount or true_mount(), seed=k), t=clk.t - 0.02 * (4 - k)) for k in range(5)]
    return {'scans': scans, 'now': clk.t}


def foot_uv(st, target):
    """Where the target's foot (floor, front face towards the car) appears in the front image."""
    g = st.geometry()
    d = np.array(target) - np.array(NOMINAL[:2])
    foot = np.array(target) - 0.03 * d / np.linalg.norm(d)
    uv, ok, _ = cm.project_ground(g['intr'], g['cam'], np.array([[foot[0], foot[1], 0.0]]))
    assert ok[0]
    return {'u': float(uv[0, 0]) / g['w'], 'v': float(uv[0, 1]) / g['h']}


def capture_all(st, clk):
    for tg in TARGETS:
        r = st.handle('CAPTURE', foot_uv(st, tg), inputs_for(tg, clk))
        assert r['ok'], r['message']


def test_config_is_complete():
    st, _, _ = make()
    assert st.scan_buffer_n == 5
    cfg = step_cfg()
    del cfg['procedure']['match_max_m']
    with pytest.raises(slc.ConfigError):
        slc.LidarCameraStep(cfg, {}, dict, list)
    assert cfg['pass']['max_bearing_error_deg'] == 2.0      # original key unchanged


def test_capture_compute_pass():
    st, _, clk = make()
    assert st.start(clk.t, {}) is not None                  # nothing captured: refused with a message
    capture_all(st, clk)
    assert len(st.captures) == 3
    assert st.start(clk.t, {'argument': ''}) is None
    res = st.tick(clk.t, {})
    assert res['passed'], res['summary']
    assert abs(res['yaw_offset_deg'] - TRUE_DELTA) < 0.5
    assert abs(res['base_to_laser'][3] - true_mount()[3]) < math.radians(0.5)
    assert res['base_to_laser'][:3] == pytest.approx(NOMINAL[:3])
    yaml.safe_dump(res)                                      # the result file must be plain YAML
    assert st.tick(clk.t, {}) is None


def test_measured_position_and_bad_values():
    st, _, clk = make()
    capture_all(st, clk)
    assert st.start(clk.t, {'argument': json.dumps({'lidar_x_m': '0.1', 'lidar_z_m': 0.2})}) is None
    res = st.tick(clk.t, {})
    assert res['base_to_laser'][0] == pytest.approx(0.1) and res['base_to_laser'][2] == pytest.approx(0.2)
    assert 'outside' in st.start(clk.t, {'argument': json.dumps({'lidar_x_m': 2.0})})
    assert 'number' in st.start(clk.t, {'argument': json.dumps({'lidar_z_m': 'abc'})})
    assert 'JSON' in st.start(clk.t, {'argument': 'x=1'})


def test_capture_refusals():
    st, _, clk = make()
    tg = TARGETS[1]
    assert not st.handle('CAPTURE', {}, inputs_for(tg, clk))['ok']
    assert not st.handle('CAPTURE', {'u': 1.5, 'v': 0.5}, inputs_for(tg, clk))['ok']
    assert 'horizon' in st.handle('CAPTURE', {'u': 0.5, 'v': 0.0}, inputs_for(tg, clk))['message']
    stale = inputs_for(tg, clk)
    stale['now'] += 5.0
    assert 'No recent /scan' in st.handle('CAPTURE', foot_uv(st, tg), stale)['message']
    far = st.handle('CAPTURE', foot_uv(st, (0.8, -0.3)), inputs_for((0.8, 0.3), clk))
    assert not far['ok'] and 'Not captured' in far['message']
    assert st.handle('BOGUS', {}, {})['ok'] is False
    assert st.handle('UNDO', {}, {})['ok'] is False
    capture_all(st, clk)
    assert st.handle('UNDO', {}, {})['ok'] and len(st.captures) == 2
    assert st.handle('CLEAR', {}, {})['ok'] and st.captures == []


def test_live_view():
    st, _, clk = make()
    lv = st.live(inputs_for(TARGETS[0], clk))
    assert lv['image_key'] == 'cam_front' and 'error' not in lv
    assert lv['points'] and all(0 <= p[0] <= 1 and 0 <= p[1] <= 1 for p in lv['points'])
    assert len([t for t in lv['targets'] if t['u'] is not None]) == 1
    capture_all(st, clk)
    lv = st.live(inputs_for(TARGETS[0], clk))
    assert len(lv['captures']) == 3 and abs(lv['estimate']['offset_deg'] - TRUE_DELTA) < 0.5
    json.dumps(lv)
    assert st.live({})['scan_age_s'] is None


def test_live_reports_setup_error():
    cams = load('cameras.yaml')
    del cams['mounts']['front']
    st, _, _ = make(cams=cams)
    assert 'error' in st.live({})


def test_save_and_keep(tmp_path):
    st, cams, clk = make()
    capture_all(st, clk)
    st.start(clk.t, {})
    res = st.tick(clk.t, {})
    s1 = str(tmp_path / 's1')
    os.makedirs(s1)
    files = st.save_data(s1, res)
    assert len(files) == 2 and all(os.path.isfile(f) for f in files)
    l2c = ct.load_yaml(os.path.join(s1, 'data', 'cameras.yaml'))['lidar_to_camera']
    assert l2c['calibrated'] is True and l2c['yaw_offset_deg'] == res['yaw_offset_deg']
    yaw = ct.overlay_value(s1, 'tunnel_wall_follower', 'lidar_angle_offset')
    tf = ct.overlay_value(s1, 'carbot_tf', 'base_to_laser')
    assert yaw == pytest.approx(tf[3]) and tf == pytest.approx(res['base_to_laser'])
    # the rest of cameras.yaml is carried over unchanged
    assert ct.load_yaml(os.path.join(s1, 'data', 'cameras.yaml'))['mounts'] == cams['mounts']

    s2 = str(tmp_path / 's2')
    os.makedirs(s2)
    st.keep_data(s1, s2)
    assert ct.overlay_value(s2, 'carbot_tf', 'base_to_laser') == pytest.approx(tf)
    assert ct.overlay_value(s2, 'tunnel_wall_follower', 'lidar_angle_offset') == pytest.approx(yaw)
    empty = str(tmp_path / 's3')
    os.makedirs(empty)
    with pytest.raises(StepRefused):
        st.keep_data(empty, str(tmp_path / 's2'))
