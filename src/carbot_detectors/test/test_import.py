"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['bpu_detector']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_detectors.' + m)
        assert callable(getattr(mod, 'main'))
