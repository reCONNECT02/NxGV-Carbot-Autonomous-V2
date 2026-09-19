"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['local_pose', 'global_pose']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_localization.' + m)
        assert callable(getattr(mod, 'main'))
