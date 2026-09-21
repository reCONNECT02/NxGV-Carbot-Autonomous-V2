"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

import pytest

MODULES = ['track_map_server', 'global_planner', 'mission_logic', 'corridor', 'local_planner', 'parking_planner', 'recovery_planner', 'path_tracker']


def test_modules_import():
    pytest.importorskip('rclpy')   # needs ROS; runs under colcon test on the RDK
    for m in MODULES:
        mod = importlib.import_module('carbot_planning.' + m)
        assert callable(getattr(mod, 'main'))
