"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['calibration_wizard', 'race_supervisor', 'run_recorder', 'system_monitor', 'scoreboard']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_ops.' + m)
        assert callable(getattr(mod, 'main'))
