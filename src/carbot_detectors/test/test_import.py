"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['bpu_detector']


def test_core_imports_without_ros():
    importlib.import_module('carbot_detectors.detector_core')


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('carbot_detectors.' + m)
        assert callable(getattr(mod, 'main'))
