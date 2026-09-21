"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['safety_monitor', 'command_owner', 'tunnel_bridge', 'calib_steering', 'calib_speed']


def test_cores_import_without_ros():
    for m in ('owner_core', 'safety_core', 'calib_core'):
        importlib.import_module('carbot_control.' + m)


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('carbot_control.' + m)
        assert callable(getattr(mod, 'main'))
