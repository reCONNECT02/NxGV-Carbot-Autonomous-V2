"""calibration_wizard REQUIRED lists every camera_restart key (no ROS)."""
import ast
import os

from carbot_ops.camera_restart import CFG_KEYS

HERE = os.path.dirname(os.path.abspath(__file__))


def test_restart_keys_in_required():
    tree = ast.parse(open(os.path.join(HERE, '..', 'carbot_ops', 'calibration_wizard.py')).read())
    req = next(ast.literal_eval(n.value) for n in ast.walk(tree)
               if isinstance(n, ast.Assign) and any(getattr(t, 'id', '') == 'REQUIRED' for t in n.targets))
    assert {f'restart_cameras.{k}' for k in CFG_KEYS} <= set(req)
