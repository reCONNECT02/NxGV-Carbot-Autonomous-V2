"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['local_pose', 'global_pose', 'calib_odometry', 'calib_map_uwb']


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('carbot_localization.' + m)
        assert callable(getattr(mod, 'main'))
