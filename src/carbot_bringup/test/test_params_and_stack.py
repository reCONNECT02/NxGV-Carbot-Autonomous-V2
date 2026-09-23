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


def test_cameras_yaml_is_front_camera_only():
    cams = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml')))
    assert list(cams['sensors']) == ['astra'] and 'enabled' in cams['sensors']['astra']
    assert cams['roles'] == {'front': 'astra'} and list(cams['mounts']) == ['front']
    assert cams['roles_confirmed'] is False     # only the wizard may set this
    assert not hasattr(stack, 'mipi_sensors')   # the MIPI launch branch is gone


def test_camera_tfs():
    cams = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml')))
    tfs = {t['child']: t for t in stack.camera_static_tfs(cams)}
    assert set(tfs) == {'cam_front', 'cam_front_optical'}
    assert tfs['cam_front']['pitch'] > 0 and tfs['cam_front']['parent'] == 'base_link'
    assert tfs['cam_front_optical']['parent'] == 'cam_front'


def test_camera_tfs_ignore_old_session_side_mounts():
    cams = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml')))
    front = cams['mounts']['front']
    cams['mounts'].update({'left_rear': dict(front, yaw_deg=95.0), 'right_rear': dict(front, yaw_deg=-95.0)})
    tfs = {t['child'] for t in stack.camera_static_tfs(cams)}
    assert tfs == {'cam_front', 'cam_front_optical'}


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
    assert 'mipi_cam' not in cfg['kill_patterns'] and 'astra_camera_container' in cfg['kill_patterns']
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


# --------------------------------------------------------------------------- calibrate_profile:=lite
def _doc(name):
    with open(os.path.join(PARAMS, f'{name}.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_profile_values():
    assert stack.check_profile('', 'calibrate') == 'full'
    assert stack.check_profile('LITE', 'calibrate') == 'lite'
    with pytest.raises(ValueError, match='must be one of'):
        stack.check_profile('tiny', 'calibrate')
    with pytest.raises(ValueError, match='calibrate.launch.py only'):
        stack.check_profile('lite', 'race')


def test_lite_keep_is_read_from_yaml_and_validated():
    cfg = _doc('drivers')['carbot_launch']['ros__parameters']
    keep = stack.lite_keep(cfg)
    assert 'servo_controller' in keep and 'command_owner' in keep       # the car must still drive
    with pytest.raises(KeyError, match='missing YAML key'):
        stack.lite_keep({})
    with pytest.raises(KeyError, match='unknown nodes'):
        stack.lite_keep({'calibrate_lite_keep': ['no_such_node']})


def test_select_nodes_filters_and_keeps_order():
    keep = stack.lite_keep(_doc('drivers')['carbot_launch']['ros__parameters'])
    common = stack.select_nodes(stack.CARBOT_NODES['common'], keep)
    assert [e for _, e in common] == [e for _, e in stack.CARBOT_NODES['common'] if e in keep]
    assert 'bpu_detector' not in [e for _, e in common] and 'mission_logic' not in [e for _, e in common]
    assert stack.select_nodes(stack.BASE_NODES, None) == list(stack.BASE_NODES)      # full profile: everything
    assert [e for _, e in stack.select_nodes(stack.BASE_NODES, keep)] == ['servo_controller']


def test_lite_overlay_only_changes_existing_keys():
    lite = _doc('calibrate_lite')
    for node, body in lite.items():
        base_all = {}
        for n in stack.PARAM_FILES:
            base_all.update({k: v for k, v in _doc(n).items() if k == node})
        assert base_all, f'{node}: not in any normal params file'
        base_flat = stack_flat(base_all[node]['ros__parameters'])
        for k in stack_flat(body['ros__parameters']):
            assert k in base_flat, f'{node}.{k}: lite may not add a key'
    ov = lite['system_monitor']['ros__parameters']
    assert len(ov['watch_topics']) == len(ov['watch_expected_hz'])
    ops = _doc('ops')['system_monitor']['ros__parameters']
    for t, hz in zip(ov['watch_topics'], ov['watch_expected_hz']):
        assert (t, hz) in list(zip(ops['watch_topics'], ops['watch_expected_hz'])), f'{t}: differs from ops.yaml'


def stack_flat(d, prefix=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(stack_flat(v, f'{prefix}{k}.'))
        else:
            out[f'{prefix}{k}'] = v
    return out


def test_lite_overlay_goes_after_normal_files_before_session(tmp_path):
    share = tmp_path
    (share / 'config' / 'params').mkdir(parents=True)
    with pytest.raises(FileNotFoundError, match='calibrate_lite'):
        stack.with_lite_overlay(['x'] * len(stack.PARAM_FILES), str(share))
    (share / 'config' / 'params' / 'calibrate_lite.yaml').write_text('a: 1\n')
    files = [f'f{i}' for i in range(len(stack.PARAM_FILES))] + ['SESSION_OVERLAY']
    out = stack.with_lite_overlay(files, str(share))
    assert out[-1] == 'SESSION_OVERLAY' and out[len(stack.PARAM_FILES)].endswith('calibrate_lite.yaml')


def test_lite_yaml_is_rcl_compatible():
    for node, body in _doc('calibrate_lite').items():
        _check_leaf(node, body)


# --------------------------------------------------------------------------- Astra colour-only launch
def _load_launch(name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, os.path.join(PKG, 'launch', f'{name}.launch.py'))
    m = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(m)
    except ImportError as e:                          # launch / launch_ros only exist inside ROS
        pytest.skip(f'no ROS here: {e}')
    return m


def test_astra_rgb_params_are_colour_only():
    p = _load_launch('astra_rgb').astra_params(320, 240, 15)
    assert (p['color_width'], p['color_height'], p['color_fps']) == (320, 240, 15) and p['enable_color'] is True
    assert not (p['enable_depth'] or p['enable_ir'] or p['depth_align'] or p['publish_tf'])
    with pytest.raises(ValueError, match='positive'):
        _load_launch('astra_rgb').astra_params(320, 0, 15)


def test_cameras_yaml_astra_launch_is_consistent():
    from carbot_common.data import astra_launch
    astra = yaml.safe_load(open(os.path.join(DATA, 'cameras.yaml'), encoding='utf-8'))['sensors']['astra']
    pkg, f, args = astra_launch(astra)
    assert (pkg, f) == ('carbot_bringup', 'astra_rgb.launch.py')
    assert args == ['width:=320', 'height:=240', 'fps:=15']
    assert os.path.isfile(os.path.join(PKG, 'launch', f))
    assert astra['expected_hz'] == astra['fps'] == 15.0 or astra['expected_hz'] == float(astra['fps'])
    ops = yaml.safe_load(open(os.path.join(PARAMS, 'ops.yaml'), encoding='utf-8'))['system_monitor']['ros__parameters']
    assert ops['watch_expected_hz'][ops['watch_topics'].index(astra['image_topic'])] == astra['expected_hz']
    lite = _doc('calibrate_lite')['system_monitor']['ros__parameters']
    assert lite['watch_expected_hz'][lite['watch_topics'].index(astra['image_topic'])] == astra['expected_hz']


def test_astra_launch_defaults_and_missing_key():
    from carbot_common.data import astra_launch
    assert astra_launch({}) == ('astra_camera', 'astra_mini.launch.py', [])       # old behaviour
    with pytest.raises(KeyError, match='fps'):
        astra_launch({'launch_args': ['width', 'fps'], 'width': 320})


def test_safety_status_heartbeat_is_faster_than_every_consumers_stale_limit():
    ctl = _doc('control')
    hz = ctl['safety_monitor']['ros__parameters']['status_publish_hz']
    limits = {'command_owner': ctl['command_owner']['ros__parameters']['safety_max_age_s']}
    for n in ('planning',):
        for node, body in _doc(n).items():
            v = (body.get('ros__parameters') or {}).get('safety_max_age_s')
            if v is not None:
                limits[node] = v
    assert limits, 'no safety_max_age_s found'
    for node, lim in limits.items():
        assert 1.0 / hz <= 0.5 * lim, f'{node}: safety_max_age_s {lim} s needs status_publish_hz >= {2 / lim:g}'
