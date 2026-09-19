"""Offline checks (no ROS needed): parameter files are loadable by rcl, the
launch builder resolves sessions/overlays correctly, cameras.yaml follows
Camera_Setup.md."""
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(os.path.dirname(PKG), 'carbot_common'))

from carbot_bringup import stack  # noqa: E402
from carbot_common import calibration_store as cs  # noqa: E402

PARAMS = os.path.join(PKG, 'config', 'params')
DATA = os.path.join(PKG, 'config', 'data')


def _check_leaf(path, v):
    """rcl_yaml_param_parser: scalars, or NON-EMPTY lists of one scalar type."""
    if isinstance(v, dict):
        for k, vv in v.items():
            _check_leaf(f'{path}.{k}', vv)
        return
    if isinstance(v, list):
        assert v, f'{path}: empty lists are rejected by rcl (type cannot be inferred)'
        kinds = {type(x) for x in v}
        assert not (kinds & {dict, list}), f'{path}: nested structures not allowed in ROS params'
        if kinds <= {int, float} and len(kinds) == 2:
            pytest.fail(f'{path}: mixed int/float list, write every value as float')
        assert len(kinds) == 1, f'{path}: mixed types {kinds}'
        return
    assert v is not None, f'{path}: null value'


@pytest.mark.parametrize('name', stack.PARAM_FILES)
def test_param_file_rcl_compatible(name):
    doc = yaml.safe_load(open(os.path.join(PARAMS, f'{name}.yaml')))
    assert isinstance(doc, dict) and doc
    for node, body in doc.items():
        assert 'ros__parameters' in body, f'{name}.yaml: {node} lacks ros__parameters'
        _check_leaf(f'{name}:{node}', body['ros__parameters'])


def test_every_carbot_node_has_params():
    names = set()
    for n in stack.PARAM_FILES:
        names |= set(yaml.safe_load(open(os.path.join(PARAMS, f'{n}.yaml'))))
    for group in stack.CARBOT_NODES.values():
        for _, exe in group:
            assert exe in names, f'{exe} has no section in config/params'


def test_data_files_parse():
    from carbot_common.data import DATA_KEYS
    for k in DATA_KEYS:
        assert isinstance(yaml.safe_load(open(os.path.join(DATA, f'{k}.yaml'))), dict)


def test_mipi_sensors_follow_camera_setup():
    cams = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml')))
    mipi = {s['name']: s for s in stack.mipi_sensors(cams)}
    assert mipi['ov5647']['channel'] == 2 and mipi['ov5647']['namespace'] == '/cam_ov5647'
    assert mipi['imx219']['channel'] == 0 and mipi['imx219']['namespace'] == '/cam_imx219'
    for s in mipi.values():
        assert (s['image_width'], s['image_height']) == (960, 544)
    assert cams['roles']['front'] == 'astra'
    assert cams['roles_confirmed'] is False     # only the wizard may set this


def test_camera_tfs():
    cams = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml')))
    tfs = {t['child']: t for t in stack.camera_static_tfs(cams)}
    assert set(tfs) == {'cam_front', 'cam_left_rear', 'cam_right_rear'}
    assert tfs['cam_front']['pitch'] > 0


def _make_session(root, name, overlay=None, cameras=None, active=True):
    d = os.path.join(cs.calibration_dir(root), name)
    os.makedirs(os.path.join(d, 'data'))
    if overlay is not None:
        yaml.safe_dump(overlay, open(os.path.join(d, 'params_overlay.yaml'), 'w'))
    if cameras is not None:
        yaml.safe_dump(cameras, open(os.path.join(d, 'data', 'cameras.yaml'), 'w'))
    if active:
        open(os.path.join(cs.calibration_dir(root), 'ACTIVE'), 'w').write(name)
    return d


def test_session_resolution_and_overrides(tmp_path):
    root = str(tmp_path)
    assert stack.resolve_session(root) is None
    old = _make_session(root, '20260924_170200', active=False)
    new = _make_session(root, '20260925_081500',
                        overlay={'carbot_tf': {'ros__parameters': {
                            'base_to_laser': [0.09, 0.0, 0.22, 0.0, 0.0, 0.0]}}},
                        cameras={'roles': {'front': 'astra'}})
    assert stack.resolve_session(root) == new
    assert stack.resolve_session(root, '20260924_170200') == old      # rollback
    with pytest.raises(FileNotFoundError):
        stack.resolve_session(root, 'nope')

    dp = stack.data_paths(PKG, new)
    assert dp['data.cameras'].startswith(new)
    assert dp['data.track_map'] == os.path.join(PKG, 'config', 'data', 'track_map.yaml')

    files = stack.param_file_list(PKG, new)
    assert files[-1].endswith('params_overlay.yaml') and files[0].endswith('common.yaml')

    cfg = stack.launch_cfg(PKG, new)
    assert cfg['carbot_tf']['base_to_laser'][2] == 0.22
    assert 'mipi_cam' in cfg['kill_patterns']
    assert stack.launch_cfg(PKG, old)['carbot_tf']['base_to_laser'][2] == 0.12


def test_root_helper_fallback(tmp_path):
    cfg = {'root_helper_dir': str(tmp_path)}
    cmd = stack.root_helper_cmd(cfg, PKG, 'kill_stale.sh', ['a'], is_root=False)
    assert cmd[0] == 'bash' and cmd[1].endswith('scripts/kill_stale.sh')
    (tmp_path / 'kill_stale.sh').write_text('#!/bin/sh\n')
    cmd = stack.root_helper_cmd(cfg, PKG, 'kill_stale.sh', ['a'], is_root=False)
    assert cmd[:2] == ['sudo', '-n'] and cmd[-1] == 'a'
    cmd = stack.root_helper_cmd(cfg, PKG, 'kill_stale.sh', ['a'], is_root=True)
    assert cmd[0].endswith('kill_stale.sh')
