"""servo_controller extension: the arm rule (no ROS)."""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
EXT = os.path.join(HERE, '..', '..', 'control_servo', 'control_servo', 'carbot_extension.py')


def _load():
    import sys
    import types
    pkg = types.ModuleType('control_servo')
    pkg.__path__ = [os.path.dirname(EXT)]
    sys.modules.setdefault('control_servo', pkg)
    spec = importlib.util.spec_from_file_location('control_servo.carbot_extension', EXT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_arm_rule():
    d = _load().arm_decision
    assert d(True, 'IDLE', True, 0.02, 0.4) == (False, 'to_auto')
    assert d(False, 'IDLE', True, 0.02, 0.4) == (False, 'none')
    assert d(True, 'PLAYBACK', True, 0.02, 0.4) == (True, 'ignored_playback')
    assert d(True, 'IDLE', True, 1.0, 0.4) == (True, 'ignored_stale')        # no fresh /cmd_vel_auto
    assert d(False, 'IDLE', False, 0.02, 0.4) == (True, 'to_manual')
    assert d(True, 'IDLE', False, 0.02, 0.4) == (True, 'none')


def test_topic_names_match_carbot_common():
    from carbot_common import topics as T
    ns = {}
    exec(open(os.path.join(os.path.dirname(EXT), 'topics.py')).read(), ns)
    assert ns['VEHICLE_BATTERY_TOPIC'] == T.VEHICLE_BATTERY and ns['VEHICLE_ARM_TOPIC'] == T.VEHICLE_ARM
    assert ns['AUTO_CMD_VEL_TOPIC'] == T.CMD_VEL_AUTO
