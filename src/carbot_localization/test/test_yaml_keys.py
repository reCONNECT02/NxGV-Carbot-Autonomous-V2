"""Every REQUIRED key of the phase-3 nodes exists in the YAML the launch passes
(a missing key is a CONFIG_ERROR on the car). Reads REQUIRED with ast: no ROS."""
import ast
import os

import pytest
import yaml

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAMS = os.path.join(SRC, 'carbot_bringup', 'config', 'params')
NODES = {'local_pose': 'carbot_localization/carbot_localization/local_pose.py',
         'global_pose': 'carbot_localization/carbot_localization/global_pose.py',
         'uwb_ranges': 'uwb_localization/uwb_localization/uwb_ranges.py'}


def _flat(d, pre=''):
    out = set()
    for k, v in d.items():
        key = f'{pre}{k}'
        if isinstance(v, dict):
            out |= _flat(v, key + '.')
        else:
            out.add(key)
    return out


def _required(path):
    tree = ast.parse(open(os.path.join(SRC, path)).read())
    for n in tree.body:
        if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'REQUIRED':
            return ast.literal_eval(n.value)
    raise AssertionError('no REQUIRED in ' + path)


@pytest.mark.parametrize('node', sorted(NODES))
def test_required_keys_present(node):
    common = yaml.safe_load(open(os.path.join(PARAMS, 'common.yaml')))['/**']['ros__parameters']
    own = yaml.safe_load(open(os.path.join(PARAMS, 'localization.yaml')))[node]['ros__parameters']
    have = _flat(common) | _flat(own)
    missing = [k for k in _required(NODES[node]) if not k.startswith('data.') and k not in have]
    assert not missing, f'{node}: {missing}'
