"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['track_map_server', 'global_planner', 'mission_logic', 'corridor', 'local_planner', 'parking_planner', 'recovery_planner', 'path_tracker']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_planning.' + m)
        assert callable(getattr(mod, 'main'))
