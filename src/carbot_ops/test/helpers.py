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
CAMERAS = load('cameras.yaml')
UWB = load('uwb.yaml')
STEP1 = next(s for s in STEPS['steps'] if s['id'] == 'sensor_health')

GOOD = {
    'health_age_s': 0.2,
    'topics': {'/camera/color/image_raw': {'hz': 29.8, 'age': 0.03, 'latency': 40},
               '/cam_ov5647/image_raw': {'hz': 29.6, 'age': 0.03, 'latency': 35},
               '/cam_imx219/image_raw': {'hz': 29.4, 'age': 0.03, 'latency': 35},
               '/scan': {'hz': 10.1, 'age': 0.08, 'latency': 20},
               '/odom': {'hz': 20.0, 'age': 0.05, 'latency': 5},
               '/imu/rpy': {'hz': 19.9, 'age': 0.05, 'latency': -1},
               '/uwb3/input_json': {'hz': 9.8, 'age': 0.1, 'latency': -1}},
    'procs': [('astra_camera /', 100), ('mipi_cam /cam_imx219', 102), ('mipi_cam /cam_ov5647', 101)],
    'agent': True,
    'battery_v': 11.62,
    'uwb': {'link': True, 'hz': 9.8, 'unknown': '',
            'anchors': {'1782': {'seen': True, 'age': 0.1}, '1786': {'seen': True, 'age': 0.1},
                        '1783': {'seen': True, 'age': 0.2}}},
    'env': {'domain_id': '1', 'localhost_only': '0', 'ok': True, 'problems': []},
}


def good():
    return copy.deepcopy(GOOD)
