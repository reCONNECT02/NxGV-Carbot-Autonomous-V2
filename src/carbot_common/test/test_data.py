"""camera_role_topics honours cameras.yaml sensors.<name>.enabled; front-only roles; merge_data."""
import pytest

from carbot_common import calib_tools as ct
from carbot_common.data import camera_role_topics, sensor_enabled, unconfirmed_roles
from carbot_common.topics import CAMERA_ROLES

CAMS = {'sensors': {'astra': {'enabled': True, 'image_topic': '/camera/color/image_raw'}},
        'roles': {'front': 'astra'}}
# an OLDER session's cameras.yaml still lists the two removed side cameras: they must be ignored
OLD_SESSION_CAMS = {'sensors': {'astra': {'enabled': True, 'image_topic': '/camera/color/image_raw'},
                                'ov5647': {'enabled': True, 'image_topic': '/cam_ov5647/image_raw'},
                                'imx219': {'enabled': True, 'image_topic': '/cam_imx219/image_raw'}},
                    'roles': {'front': 'astra', 'left_rear': 'imx219', 'right_rear': 'ov5647'}}


def test_front_only_topics():
    assert camera_role_topics(CAMS) == {'front': '/camera/color/image_raw'}
    assert CAMERA_ROLES == ('front',)


def test_disabled_front_role_is_dropped():
    cams = {'sensors': {'astra': {'enabled': False, 'image_topic': '/camera/color/image_raw'}},
            'roles': {'front': 'astra'}}
    assert camera_role_topics(cams) == {}


def test_old_session_side_cameras_are_ignored():
    assert camera_role_topics(OLD_SESSION_CAMS) == {'front': '/camera/color/image_raw'}
    cams = dict(OLD_SESSION_CAMS, roles_confirmed=True, roles_confirmed_for=['front'])
    assert unconfirmed_roles(cams) == []                              # side roles never need confirming


def test_missing_enabled_is_loud():
    cams = {'sensors': {'astra': {'image_topic': '/x'}}, 'roles': {'front': 'astra'}}
    with pytest.raises(KeyError, match='enabled'):
        sensor_enabled(cams, 'astra')
    with pytest.raises(KeyError):
        camera_role_topics(cams)


def test_unconfirmed_roles_front_only():
    cams = dict(CAMS, roles_confirmed=False, roles_confirmed_for=[])
    assert unconfirmed_roles(cams) == ['front']
    cams.update(roles_confirmed=True, roles_confirmed_for=['front'])
    assert unconfirmed_roles(cams) == []
    cams['roles_confirmed'] = False                                   # the flag still gates everything
    assert unconfirmed_roles(cams) == ['front']
    with pytest.raises(KeyError, match='roles_confirmed_for'):
        unconfirmed_roles({k: v for k, v in cams.items() if k != 'roles_confirmed_for'})


def test_merge_data_starts_from_session_copy(tmp_path):
    base = {'a': 1, 'roles': {'front': 'astra'}, 'keep': [1, 2]}
    p = ct.merge_data(str(tmp_path), 'cameras.yaml', base, {'roles': {'front': 'other'}, 'flag': True})
    want = {'a': 1, 'roles': {'front': 'other'}, 'keep': [1, 2], 'flag': True}
    assert ct.load_yaml(p) == want
    assert base['roles']['front'] == 'astra'                          # base not mutated
    ct.merge_data(str(tmp_path), 'cameras.yaml', {'a': 99}, {'b': 2})  # the session copy wins over base
    assert ct.load_yaml(p) == dict(want, b=2)
