"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['calibration_wizard', 'race_supervisor', 'run_recorder', 'system_monitor', 'scoreboard']
PURE = ['monitor_core', 'sensor_checks', 'wizard_core', 'step_sensor_health', 'camera_restart',
        'wizard_uwb', 'step_uwb_survey']   # phase 8, no rclpy


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_ops.' + m)
        assert callable(getattr(mod, 'main'))


def test_pure_modules_import_without_ros():
    for m in PURE:
        importlib.import_module('carbot_ops.' + m)
