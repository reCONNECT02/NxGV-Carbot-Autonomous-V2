"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['road_perception', 'local_memory', 'camera_preview']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_perception.' + m)
        assert callable(getattr(mod, 'main'))
