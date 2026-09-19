"""Every node module must import and expose main() (no ROS graph needed)."""
import importlib

MODULES = ['safety_monitor', 'command_owner', 'tunnel_bridge']


def test_modules_import():
    for m in MODULES:
        mod = importlib.import_module('carbot_control.' + m)
        assert callable(getattr(mod, 'main'))
