"""Every key a Carbot node REQUIREs is in the YAML the launch gives it (no ROS).

REQUIRED lists are read with ast (the node modules import rclpy). A node gets
common.yaml '/**' + its own section of config/params/*.yaml + the launch extras
(data.* paths, data_root, mode)."""
import ast
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(SRC, 'carbot_common'))

from carbot_bringup import stack  # noqa: E402
from carbot_common.data import DATA_KEYS  # noqa: E402

PARAMS = os.path.join(PKG, 'config', 'params')

NODES = {  # node name -> module file (phase-5 nodes + the phase-4 ones they touch)
    'parking_planner': 'carbot_planning/carbot_planning/parking_planner.py',
    'recovery_planner': 'carbot_planning/carbot_planning/recovery_planner.py',
    'mission_logic': 'carbot_planning/carbot_planning/mission_logic.py',
    'path_tracker': 'carbot_planning/carbot_planning/path_tracker.py',
    'safety_monitor': 'carbot_control/carbot_control/safety_monitor.py',
    'command_owner': 'carbot_control/carbot_control/command_owner.py',
    'tunnel_bridge': 'carbot_control/carbot_control/tunnel_bridge.py',
    'bpu_detector': 'carbot_detectors/carbot_detectors/bpu_detector.py',   # phase 6
}


def _flat(d, prefix=''):
    out = set()
    for k, v in d.items():
        key = f'{prefix}{k}'
        out.add(key)
        if isinstance(v, dict):
            out |= _flat(v, key + '.')
    return out


def _params_for(node):
    keys = set()
    for n in stack.PARAM_FILES:
        doc = yaml.safe_load(open(os.path.join(PARAMS, f'{n}.yaml')))
        for sect in ('/**', node):
            if sect in doc:
                keys |= _flat(doc[sect]['ros__parameters'])
    keys |= {f'data.{k}' for k in DATA_KEYS} | {'data_root', 'mode'}
    return keys


def _required(path):
    tree = ast.parse(open(os.path.join(SRC, path)).read())
    for n in ast.walk(tree):
        if isinstance(n, ast.Assign) and any(getattr(t, 'id', '') == 'REQUIRED' for t in n.targets):
            return ast.literal_eval(n.value)
    raise AssertionError(f'{path}: no REQUIRED list')


@pytest.mark.parametrize('node', sorted(NODES))
def test_required_keys_present(node):
    have = _params_for(node)
    missing = [k for k in _required(NODES[node]) if k not in have]
    assert not missing, f'{node}: missing YAML keys {missing}'


def test_tunnel_bridge_mirrors_owner_steering():
    doc = yaml.safe_load(open(os.path.join(PARAMS, 'control.yaml')))
    own = doc['command_owner']['ros__parameters']
    br = doc['tunnel_bridge']['ros__parameters']
    assert br['command_owner_steering'] == own['steering']
    assert br['command_owner_feedforward'] == own['feedforward']


def test_laser_mount_matches_base_tunnel_offset():
    drv = yaml.safe_load(open(os.path.join(PARAMS, 'drivers.yaml')))
    base = yaml.safe_load(open(os.path.join(PARAMS, 'base_nodes.yaml')))
    yaw = drv['carbot_tf']['ros__parameters']['base_to_laser'][3]
    assert abs(yaw - base['tunnel_wall_follower']['ros__parameters']['lidar_angle_offset']) < 1e-3
