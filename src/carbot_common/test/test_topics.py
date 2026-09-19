"""Base-repo topic names must never drift from the base packages."""
import importlib.util
import os

import pytest

from carbot_common import topics as T

HERE = os.path.dirname(__file__)
BASE_TOPICS = os.path.join(HERE, '..', '..', 'risabot_automode', 'risabot_automode', 'topics.py')


def _load(path):
    spec = importlib.util.spec_from_file_location('base_topics', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.skipif(not os.path.exists(BASE_TOPICS), reason='base package not in tree')
def test_base_names_unchanged():
    b = _load(BASE_TOPICS)
    assert T.CMD_VEL_AUTO == b.AUTO_CMD_VEL_TOPIC
    assert T.CMD_VEL == b.CMD_VEL_TOPIC
    assert T.AUTO_MODE == b.AUTO_MODE_TOPIC
    assert T.E_STOP == b.E_STOP_TOPIC
    assert T.ODOM == b.ODOM_TOPIC
    assert T.IMU_RPY == b.IMU_DATA_TOPIC
    assert T.IMU_PITCH == b.IMU_PITCH_TOPIC
    assert T.IMU_CALIBRATE == b.IMU_CALIBRATE_TOPIC
    assert T.TUNNEL_DETECTED == b.TUNNEL_DETECTED_TOPIC
    assert T.TUNNEL_CMD == b.TUNNEL_CMD_TOPIC
    assert T.TRAFFIC_LIGHT_STATE == b.TRAFFIC_LIGHT_TOPIC
    assert T.BOOM_GATE_OPEN == b.BOOM_GATE_TOPIC
    assert T.ASTRA_COLOR_IMAGE == b.CAMERA_IMAGE_TOPIC
    assert T.JOY == b.JOY_TOPIC


def test_request_topics():
    assert [T.request_topic(s) for s in T.REQUEST_SOURCES] == [
        '/carbot/request/road', '/carbot/request/tunnel',
        '/carbot/request/parking', '/carbot/request/recovery']
