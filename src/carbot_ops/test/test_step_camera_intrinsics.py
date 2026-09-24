"""Step 3 (camera intrinsics) page logic with a fake calib_core (no OpenCV, no ROS)."""
import copy
import os
import types

import numpy as np
import pytest
import yaml

from carbot_ops import step_camera_intrinsics as sci
from helpers import OLD_SESSION_CAMERAS, REPO_CAMERAS, STEPS

STEP3 = next(s for s in STEPS['steps'] if s['id'] == 'camera_intrinsics')


class FakeCollector:
    def __init__(self, w, h, novelty):
        self.width, self.height = w, h
        self.views = []
        self.coverage = np.zeros((3, 3), int)

    def offer(self, c, cols, rows):
        self.views.append(c)
        self.coverage.flat[(len(self.views) - 1) % 9] += 1
        return True

    def coverage_cells(self):
        return int((self.coverage > 0).sum())


def fake_calib(rms=0.3):
    board = np.random.RandomState(0).rand(54, 2).astype(np.float32) * 100

    def detect(img, cols, rows, fast=False):
        return None if img.mean() == 0 else board + 0.0     # identical corners -> 'still'

    def calibrate(views, cols, rows, square, size):
        intr = types.SimpleNamespace(K=[[500.0, 0, 480.0], [0, 501.0, 272.0], [0, 0, 1]], model='plumb_bob',
                                     meta={}, hfov_deg=lambda: 88.0)
        fit = types.SimpleNamespace(intr=intr, rms_px=rms)
        return {'chosen': fit, 'pinhole': fit, 'fisheye': None, 'used_views': list(range(len(views)))}

    return types.SimpleNamespace(ViewCollector=FakeCollector, detect_chessboard=detect,
                                 calibrate_intrinsics=calibrate)


class Frames:
    def __init__(self, dark=False):
        self.seq, self.dark = 0, dark

    def __call__(self, sensor):
        self.seq += 1
        return self.seq, np.zeros((544, 960, 3), np.uint8) if self.dark else np.ones((544, 960, 3), np.uint8)


def run_until_done(step, frames, t0=0.0, dt=0.2, limit=4000):
    t = t0
    for _ in range(limit):
        t += dt
        res = step.tick(t, {'frame': frames})
        if res is not None:
            return res
        if step.run and step.run['thread'] is not None:
            step.run['thread'].join()
    raise AssertionError('step never finished')


def front_only():
    return copy.deepcopy(REPO_CAMERAS)          # the front Astra is the only sensor in the repo file


def test_repo_yaml_builds_with_the_astra_only():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib())
    assert st.enabled() == ['astra']
    rows = {r['name']: r for r in st.live({})['sensors']}
    assert list(rows) == ['astra'] and rows['astra']['state'] == 'todo'


def test_old_session_side_cameras_are_skipped_even_if_per_sensor_names_them():
    old_step = copy.deepcopy(STEP3)
    old_step['per_sensor'] = ['astra', 'ov5647', 'imx219']       # an older session's calibration_steps.yaml copy
    st = sci.CameraIntrinsicsStep(old_step, copy.deepcopy(OLD_SESSION_CAMERAS), calib=fake_calib())
    assert st.enabled() == ['astra'] and [r['name'] for r in st.live({})['sensors']] == ['astra']
    assert 'Unknown camera' in st.start(0.0, {'argument': 'imx219'})


def test_unknown_camera_run_is_refused():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib())
    assert 'Unknown camera' in st.start(0.0, {'argument': 'imx219'})
    assert 'Unknown camera' in st.start(0.0, {'argument': 'nope'})


def test_disabled_astra_is_refused_and_the_step_cannot_start():
    cams = front_only()
    cams['sensors']['astra']['enabled'] = False
    with pytest.raises(sci.ConfigError, match='no enabled camera'):
        sci.CameraIntrinsicsStep(STEP3, cams, calib=fake_calib())


def test_front_only_run_passes_whole_step():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib())
    assert st.start(0.0, {}) is None and st.running_sensor() == 'astra'
    res = run_until_done(st, Frames())
    assert res['passed'] is True
    assert res['sensors']['astra']['status'] == 'PASS' and res['sensors']['astra']['views'] == st.target_views
    assert res['disabled'] == [] and 'switched off' not in res['summary']
    assert [c['label'] for c in res['checks']] == ['Astra Pro (front)']


def test_live_capture_shows_progress_and_overlay():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib())
    st.start(0.0, {})
    fr = Frames()
    st.tick(0.2, {'frame': fr})
    st.tick(0.4, {'frame': fr})
    cap = st.live({})['capture']
    assert cap['sensor'] == 'astra' and cap['views'] == 1 and cap['state'] == 'new view captured'
    assert len(cap['corners']) <= STEP3['procedure']['max_live_corners'] and cap['preview'] == 'front'
    assert all(0 <= u <= 1.5 for u, _ in cap['corners'])


def test_high_reprojection_fails_with_reason():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib(rms=2.0))
    st.start(0.0, {})
    res = run_until_done(st, Frames())
    assert res['passed'] is False
    assert 'reprojection error' in res['sensors']['astra']['why'] and res['checks'][0]['fix']
    assert not st.fits


def test_no_frames_times_out_with_hint():
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib())
    st.start(0.0, {})
    res = run_until_done(st, lambda s: None, dt=10.0)
    assert res['passed'] is False and 'no frames' in res['sensors']['astra']['why']


def test_missing_procedure_key_is_loud():
    bad = copy.deepcopy(STEP3)
    del bad['procedure']['still_px']
    with pytest.raises(sci.ConfigError, match='still_px'):
        sci.CameraIntrinsicsStep(bad, front_only(), calib=fake_calib())


def test_save_and_keep_write_intrinsics_and_cameras_yaml(tmp_path):
    st = sci.CameraIntrinsicsStep(STEP3, front_only(), calib=fake_calib(),
                                  save_intrinsics=lambda p, intr, name: (os.makedirs(os.path.dirname(p), exist_ok=True),
                                                                         open(p, 'w').write(f'camera_name: {name}\n')))
    st.start(0.0, {})
    res = run_until_done(st, Frames())
    sess = tmp_path / 'calibration' / 's1'
    sess.mkdir(parents=True)
    paths = st.save_data(str(sess), res)
    ifile = sess / 'intrinsics' / 'astra.yaml'
    assert str(ifile) in paths and ifile.is_file()
    cams = yaml.safe_load(open(sess / 'data' / 'cameras.yaml', encoding='utf-8'))
    assert cams['sensors']['astra']['intrinsics_file'] == os.path.abspath(ifile)
    assert list(cams['sensors']) == ['astra']

    new = tmp_path / 'calibration' / 's2'
    new.mkdir()
    kept = st.keep_data(str(sess), str(new))
    cams2 = yaml.safe_load(open(new / 'data' / 'cameras.yaml', encoding='utf-8'))
    assert cams2['sensors']['astra']['intrinsics_file'] == os.path.abspath(new / 'intrinsics' / 'astra.yaml')
    assert (new / 'intrinsics' / 'astra.yaml').is_file() and len(kept) == 2


def test_keep_previous_refuses_intrinsics_for_another_image_size(tmp_path):
    from carbot_ops.wizard_core import StepRefused
    cams = front_only()
    w, h = int(cams['sensors']['astra']['width']), int(cams['sensors']['astra']['height'])
    st = sci.CameraIntrinsicsStep(STEP3, cams, calib=fake_calib())
    old = tmp_path / 'calibration' / 's1'
    (old / 'intrinsics').mkdir(parents=True)
    (old / 'intrinsics' / 'astra.yaml').write_text(yaml.safe_dump({'image_width': w // 2, 'image_height': h // 2}),
                                                   encoding='utf-8')
    new = tmp_path / 'calibration' / 's2'
    new.mkdir()
    with pytest.raises(StepRefused, match=f'{w // 2}x{h // 2}.*{w}x{h}'):
        st.keep_data(str(old), str(new))
    (old / 'intrinsics' / 'astra.yaml').write_text(yaml.safe_dump({'image_width': w, 'image_height': h}),
                                                   encoding='utf-8')
    assert len(st.keep_data(str(old), str(new))) == 2                       # same size: kept as before
