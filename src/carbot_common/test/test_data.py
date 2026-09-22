"""camera_role_topics honours cameras.yaml sensors.<name>.enabled."""
import pytest

from carbot_common.data import camera_role_topics, sensor_enabled

CAMS = {'sensors': {'astra': {'enabled': True, 'image_topic': '/camera/color/image_raw'},
                    'ov5647': {'enabled': False, 'image_topic': '/cam_ov5647/image_raw'},
                    'imx219': {'enabled': True, 'image_topic': '/cam_imx219/image_raw'}},
        'roles': {'front': 'astra', 'left_rear': 'imx219', 'right_rear': 'ov5647'}}


def test_disabled_role_is_dropped():
    assert camera_role_topics(CAMS) == {'front': '/camera/color/image_raw',
                                        'left_rear': '/cam_imx219/image_raw'}


def test_missing_enabled_is_loud():
    cams = {'sensors': {'astra': {'image_topic': '/x'}}, 'roles': {'front': 'astra'}}
    with pytest.raises(KeyError, match='enabled'):
        sensor_enabled(cams, 'astra')
    with pytest.raises(KeyError):
        camera_role_topics(cams)
