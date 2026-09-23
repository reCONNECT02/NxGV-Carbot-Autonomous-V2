"""static_tf.find_static: the base_link -> laser_frame mount without a tf2 listener (pure, no ROS)."""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from carbot_common.static_tf import find_static, yaw_of  # noqa: E402


def tf(parent, child, x=0.0, y=0.0, yaw=0.0):
    q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=parent), child_frame_id=child,
                                 transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=x, y=y, z=0.1),
                                                                 rotation=q))


def test_finds_the_laser_mount_among_other_static_frames():
    msgs = [tf('base_link', 'cam_front', 0.5), tf('base_link', 'laser_frame', 0.0, 0.0, math.pi),
            tf('base_link', 'cam_left_rear', 0.1)]
    x, y, yaw = find_static(msgs, 'base_link', 'laser_frame')
    assert (x, y) == (0.0, 0.0) and abs(abs(yaw) - math.pi) < 1e-9


def test_leading_slash_and_missing():
    assert find_static([tf('/base_link', '/laser_frame', 0.09)], 'base_link', 'laser_frame')[0] == 0.09
    assert find_static([tf('odom', 'base_link')], 'base_link', 'laser_frame') is None
    assert find_static([], 'base_link', 'laser_frame') is None


def test_yaw_of_matches_tf2():
    for a in (-2.5, -0.3, 0.0, 1.2, 3.0):
        q = tf('a', 'b', yaw=a).transform.rotation
        assert abs(yaw_of(q) - a) < 1e-9
