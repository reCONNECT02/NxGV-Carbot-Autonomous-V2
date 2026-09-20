"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['road_perception', 'local_memory', 'camera_preview', 'calib_intrinsics',
           'calib_extrinsics']


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('carbot_perception.' + m)
        assert callable(getattr(mod, 'main'))
