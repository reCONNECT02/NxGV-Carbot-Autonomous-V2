"""Shared plumbing for calib_intrinsics / calib_extrinsics (and the phase-8 wizard).

Where things are read from:
  cameras.yaml    <session>/data/cameras.yaml if this session already has one,
                  else the ACTIVE session's copy, else the repo default.
  steps           carbot_bringup/config/data/calibration_steps.yaml
Where results go (calibration_store layout):
  <session>/intrinsics/<sensor>.yaml       ROS camera_info format
  <session>/data/cameras.yaml              full copy with the new values
  <session>/03_camera_intrinsics.yaml      per-sensor results
  <session>/04_extrinsics_ipm.yaml         per-camera results
  <session>/captures/...                   the images used (for Redo / offline)
  <session>/summary.yaml                   step status
"""
import argparse
import copy
import os
import shutil
import time
from typing import Dict, Optional

import cv2
import yaml
from carbot_common import calibration_store as cs


def bringup_config_dir(override: str = '') -> str:
    if override:
        return override
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('carbot_bringup'), 'config')
    except Exception:  # noqa: BLE001  running from a source checkout without ROS
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, '..', '..', 'carbot_bringup', 'config'))


def common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument('--session', default='',
                   help='calibration session folder name (default: a new timestamped one). '
                        'Use the same name for steps 3 and 4.')
    p.add_argument('--data-root', default='', help='default $CARBOT_DATA or /home/sunrise/carbot_data')
    p.add_argument('--config-dir', default='', help='carbot_bringup/config (auto)')
    p.add_argument('--activate', action='store_true',
                   help='make this session ACTIVE so the launch files load it '
                        '(race mode still refuses to arm until every required step passes)')


def load_yaml(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def effective_cameras(session: str, config_dir: str, root: str) -> Dict:
    own = os.path.join(session, 'data', 'cameras.yaml')
    if os.path.isfile(own):
        return load_yaml(own)
    active = cs.data_override(cs.active_session(root), 'cameras.yaml')
    src = active or os.path.join(config_dir, 'data', 'cameras.yaml')
    print(f'[calib] starting from {src}')
    return copy.deepcopy(load_yaml(src))


def save_cameras(session: str, cameras: Dict) -> str:
    path = os.path.join(session, 'data', 'cameras.yaml')
    if os.path.isfile(path):
        shutil.copy2(path, path + '.bak')
    cs.write_yaml(path, cameras)
    return path


def step_cfg(config_dir: str, step_id: str) -> Dict:
    steps = load_yaml(os.path.join(config_dir, 'data', 'calibration_steps.yaml'))
    for s in steps.get('steps', []):
        if s['id'] == step_id:
            return s
    raise KeyError(f'calibration step {step_id} not in calibration_steps.yaml')


def merge_result(session: str, fname: str, key: str, value: Dict) -> Dict:
    path = os.path.join(session, fname)
    doc = load_yaml(path) if os.path.isfile(path) else {}
    doc.setdefault(key, {}).update(value)
    cs.write_yaml(path, doc)
    return doc


class FrameGrabber:
    """Minimal rclpy subscriber keeping the newest frame of each topic."""

    def __init__(self, topics: Dict[str, str]):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from .ros_image import image_to_bgr
        self._rclpy = rclpy
        self._conv = image_to_bgr
        rclpy.init()
        self.node = rclpy.create_node('carbot_calib_grabber')
        self.latest: Dict[str, Optional[object]] = {k: None for k in topics}
        self.count: Dict[str, int] = {k: 0 for k in topics}
        for key, topic in topics.items():
            self.node.create_subscription(
                Image, topic, lambda m, k=key: self._store(k, m), qos_profile_sensor_data)

    def _store(self, key, msg):
        self.latest[key] = msg
        self.count[key] += 1

    def next(self, key: str, timeout: float = 2.0):
        """Wait for a NEW frame of `key` -> BGR image, or None on timeout."""
        start = self.count[key]
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.count[key] > start and self.latest[key] is not None:
                return self._conv(self.latest[key]).copy()
        return None

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def draw_corners(img, corners, cols, rows, found=True):
    out = img.copy()
    if corners is not None:
        cv2.drawChessboardCorners(out, (cols, rows), corners.reshape(-1, 1, 2), found)
    return out
