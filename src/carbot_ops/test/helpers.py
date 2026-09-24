"""Shared fixtures: the repo's real YAML and a healthy snapshot."""
import copy
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'carbot_bringup', 'config', 'data'))


def load(name):
    with open(os.path.join(DATA, name), encoding='utf-8') as f:
        return yaml.safe_load(f)


STEPS = load('calibration_steps.yaml')
REPO_CAMERAS = load('cameras.yaml')             # as committed: the front Astra is the only camera
CAMERAS = copy.deepcopy(REPO_CAMERAS)
for _s in CAMERAS['sensors'].values():
    _s['enabled'] = True
# an OLDER session's copy of cameras.yaml still lists the two removed MIPI side cameras
OLD_SESSION_CAMERAS = copy.deepcopy(CAMERAS)
OLD_SESSION_CAMERAS['sensors'].update({
    'ov5647': {'enabled': True, 'driver': 'mipi_cam', 'namespace': '/cam_ov5647', 'channel': 2, 'image_width': 960,
               'image_height': 544, 'image_topic': '/cam_ov5647/image_raw', 'expected_hz': 30.0},
    'imx219': {'enabled': True, 'driver': 'mipi_cam', 'namespace': '/cam_imx219', 'channel': 0, 'image_width': 960,
               'image_height': 544, 'image_topic': '/cam_imx219/image_raw', 'expected_hz': 30.0}})
OLD_SESSION_CAMERAS['roles'].update({'left_rear': 'imx219', 'right_rear': 'ov5647'})
OLD_SESSION_CAMERAS['mounts'].update({
    'left_rear': dict(CAMERAS['mounts']['front'], y_m=0.0654, yaw_deg=95.0),
    'right_rear': dict(CAMERAS['mounts']['front'], y_m=-0.0654, yaw_deg=-95.0)})
UWB = load('uwb.yaml')
STEP1 = next(s for s in STEPS['steps'] if s['id'] == 'sensor_health')

GOOD = {
    'health_age_s': 0.2,
    'topics': {'/camera/color/image_raw': {'hz': 14.9, 'age': 0.03, 'latency': 40},
               '/scan': {'hz': 10.1, 'age': 0.08, 'latency': 20},
               '/odom': {'hz': 20.0, 'age': 0.05, 'latency': 5},
               '/imu/rpy': {'hz': 19.9, 'age': 0.05, 'latency': -1},
               '/uwb3/input_json': {'hz': 9.8, 'age': 0.1, 'latency': -1}},
    'procs': [('astra_camera /', 100)],
    'agent': True,
    'battery_v': 11.62,
    'uwb': {'link': True, 'hz': 9.8, 'unknown': '',
            'anchors': {'1782': {'seen': True, 'age': 0.1}, '1786': {'seen': True, 'age': 0.1},
                        '1783': {'seen': True, 'age': 0.2}}},
    'env': {'domain_id': '1', 'localhost_only': '0', 'ok': True, 'problems': []},
}


def good():
    return copy.deepcopy(GOOD)
