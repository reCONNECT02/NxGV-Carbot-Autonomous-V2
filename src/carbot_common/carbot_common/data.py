"""Loaders for the YAML *data* files (map, mission, cameras, uwb, challenges,
calibration steps). Tunable scalars live in ROS parameter files instead.

Launch passes every node the resolved path of each data file as parameters
data.track_map, data.mission, data.cameras, data.uwb, data.challenges,
data.track_features, data.mission_rules (phase 4),
data.calibration_steps (a calibration session may override the repo default).
"""
import os
from typing import Any, Dict, List

import yaml

DATA_KEYS = ('track_map', 'mission', 'cameras', 'uwb', 'challenges', 'calibration_steps',
             'track_features', 'mission_rules')   # phase 4: + track_features, mission_rules (v2 companions)


def load_yaml(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    return data


def load_data(node, key: str) -> Dict[str, Any]:
    """Load data file `key` using the node's data.<key> parameter."""
    if key not in DATA_KEYS:
        raise KeyError(key)
    return load_yaml(node.p(f'data.{key}'))


def sensor_enabled(cameras: Dict[str, Any], name: str) -> bool:
    """cameras.yaml sensors.<name>.enabled (a missing key is an error)."""
    s = cameras['sensors'][name]
    if 'enabled' not in s:
        raise KeyError(f'cameras.yaml sensors.{name}.enabled missing')
    return bool(s['enabled'])


def unconfirmed_roles(cameras: Dict[str, Any]) -> List[str]:
    """Enabled camera roles that calibration step 2 has NOT confirmed ([] = all good).

    roles_confirmed: true only counts for the roles listed in roles_confirmed_for,
    so a side camera switched on after a front-only confirmation is unconfirmed
    again. Disabled roles never need a confirmation. Missing keys are an error."""
    for k in ('roles', 'roles_confirmed', 'roles_confirmed_for'):
        if k not in cameras:
            raise KeyError(f'cameras.yaml {k} missing')
    done = set(cameras['roles_confirmed_for'] or []) if cameras['roles_confirmed'] else set()
    return [role for role, sensor in cameras['roles'].items()
            if sensor_enabled(cameras, sensor) and role not in done]


def camera_role_topics(cameras: Dict[str, Any]) -> Dict[str, str]:
    """{'front': '/camera/color/image_raw', 'left_rear': ..., 'right_rear': ...}

    Roles whose sensor has enabled: false are left out."""
    sensors = cameras['sensors']
    return {role: sensors[sensor]['image_topic'] for role, sensor in cameras['roles'].items()
            if sensor_enabled(cameras, sensor)}


def subscribe_cameras(node, callback=None, qos=None) -> Dict[str, str]:
    """Subscribe `node` (a CarbotNode) to every role's image topic."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    topics = camera_role_topics(load_data(node, 'cameras'))
    for role, topic in topics.items():
        cb = (lambda m, r=role: callback(r, m)) if callback else None
        node.sub(Image, topic, cb, qos or qos_profile_sensor_data)
    return topics
