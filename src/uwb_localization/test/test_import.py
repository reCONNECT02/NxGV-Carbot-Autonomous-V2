"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['uwb_ranges']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('uwb_localization.' + m)
        assert callable(getattr(mod, 'main'))
