"""Step 4 gives usable failures before touching calibration data."""
import copy
import os

import pytest
import yaml

from carbot_common import calib_tools as ct
from carbot_ops.step_extrinsics_ipm import ExtrinsicsIpmStep
from carbot_ops.wizard_core import StepRefused


def fixture_data():
    root = os.path.join(os.path.dirname(__file__), '..', '..', 'carbot_bringup', 'config')
    with open(os.path.join(root, 'data', 'calibration_steps.yaml'), encoding='utf-8') as fh:
        cfg = next(s for s in yaml.safe_load(fh)['steps'] if s['id'] == 'extrinsics_ipm')
    with open(os.path.join(root, 'data', 'cameras.yaml'), encoding='utf-8') as fh:
        cameras = yaml.safe_load(fh)
    return cfg, cameras, root


def test_missing_intrinsics_reports_next_action(tmp_path):
    cfg, cameras, root = fixture_data()
    step = ExtrinsicsIpmStep(cfg, cameras, lambda: cameras, root, str(tmp_path))
    message = step.start(0.0, {})
    assert 'Finish and Save step 3' in message
    assert step.run is None


def test_silent_camera_names_missing_role(tmp_path):
    cfg, cameras, root = fixture_data()
    cameras = copy.deepcopy(cameras)
    file = tmp_path / 'intrinsics.yaml'
    file.write_text('valid placeholder', encoding='utf-8')
    for role, sensor in cameras['roles'].items():
        if cameras['sensors'][sensor].get('enabled', True):
            cameras['sensors'][sensor]['intrinsics_file'] = str(file)
    step = ExtrinsicsIpmStep(cfg, cameras, lambda: cameras, root, str(tmp_path))
    assert step.start(0.0, {}) is None
    result = step.tick(step.timeout + 1, {'frame': lambda sensor: None})
    assert result['passed'] is False
    assert 'front' in result['problems'][0]
    assert 'Check its camera in step 1' in result['problems'][0]


def test_run_waits_for_picture_after_start(tmp_path):
    cfg, cameras, root = fixture_data()
    cameras = copy.deepcopy(cameras)
    file = tmp_path / 'intrinsics.yaml'
    file.write_text('valid placeholder', encoding='utf-8')
    for role, sensor in cameras['roles'].items():
        if cameras['sensors'][sensor].get('enabled', True):
            cameras['sensors'][sensor]['intrinsics_file'] = str(file)
    step = ExtrinsicsIpmStep(cfg, cameras, lambda: cameras, root, str(tmp_path))
    cached = {sensor: (7, object()) for _, sensor in step.roles}
    assert step.start(0.0, {'frame': cached.get}) is None
    result = step.tick(step.timeout + 1, {'frame': cached.get})
    assert result['passed'] is False
    assert 'No fresh picture' in result['problems'][0]


def test_save_preserves_earlier_camera_data(tmp_path):
    cfg, cameras, root = fixture_data()
    step = ExtrinsicsIpmStep(cfg, cameras, lambda: cameras, root, str(tmp_path))
    session = str(tmp_path / 'session')
    ct.merge_data(session, 'cameras.yaml', cameras, {'roles_confirmed': True})
    mounts = {role: cameras['mounts'][role] for role, _ in step.roles}
    written = step.save_data(session, {'mounts': mounts})
    doc = ct.load_yaml(written[0])
    assert doc['roles_confirmed'] is True
    assert doc['extrinsics_calibrated'] is True
    with pytest.raises(StepRefused, match='missing'):
        step.save_data(session, {'mounts': {}})


def test_intrinsics_from_another_image_size_are_refused(tmp_path):
    cfg, cameras, root = fixture_data()
    cameras = copy.deepcopy(cameras)
    want = (int(cameras['sensors']['astra']['width']), int(cameras['sensors']['astra']['height']))
    old = tmp_path / 'astra_old.yaml'
    old.write_text(yaml.safe_dump({'image_width': 320, 'image_height': 240}), encoding='utf-8')
    cameras['sensors']['astra']['intrinsics_file'] = str(old)
    step = ExtrinsicsIpmStep(cfg, cameras, lambda: cameras, root, str(tmp_path))
    message = step.start(0.0, {})
    assert '320x240' in message and f'{want[0]}x{want[1]}' in message and 'Redo and Save step 3' in message
    assert step.run is None
    same = tmp_path / 'astra_now.yaml'
    same.write_text(yaml.safe_dump({'image_width': want[0], 'image_height': want[1]}), encoding='utf-8')
    cameras['sensors']['astra']['intrinsics_file'] = str(same)
    assert step.start(0.0, {}) is None
