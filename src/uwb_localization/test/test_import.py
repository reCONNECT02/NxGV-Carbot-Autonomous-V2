"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['uwb_ranges', 'calib_uwb']


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('uwb_localization.' + m)
        assert callable(getattr(mod, 'main'))
