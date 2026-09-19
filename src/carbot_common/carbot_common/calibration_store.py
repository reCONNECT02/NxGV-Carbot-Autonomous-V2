"""Calibration folder layout (shared by launch, wizard and race supervisor).

<data_root>/                       default /home/sunrise/carbot_data (env CARBOT_DATA)
  calibration/
    ACTIVE                         text file: name of the session race mode loads
    20260925_081500/               one wizard session (never deleted -> rollback)
      summary.yaml                 pass/fail per step (schema below)
      01_sensor_health.yaml ...    one result file per step
      params_overlay.yaml          ROS parameter overrides (node-namespaced)
      data/cameras.yaml            data-file overrides (full copies)
      data/uwb.yaml
      intrinsics/<sensor>.yaml     ROS camera_info format
  bags/                            rosbag of every run
  routes/                          global planner cache

summary.yaml:
  session: 20260925_081500
  created: 2026-09-25T08:15:00
  based_on: 20260924_170200        # session the "keep previous" values came from
  steps:
    sensor_health: {status: PASS, file: 01_sensor_health.yaml, time: ...}
    camera_identity: {status: KEPT_PREVIOUS, from_session: 20260924_170200}
  all_required_passed: true
"""
import os
from typing import Dict, List, Optional

import yaml

DEFAULT_DATA_ROOT = '/home/sunrise/carbot_data'
PASSING = ('PASS', 'KEPT_PREVIOUS')


def data_root(override: str = '') -> str:
    return os.path.expanduser(override or os.environ.get('CARBOT_DATA', DEFAULT_DATA_ROOT))


def calibration_dir(root: str) -> str:
    return os.path.join(root, 'calibration')


def bags_dir(root: str) -> str:
    return os.path.join(root, 'bags')


def active_session(root: str) -> Optional[str]:
    """Absolute path of the ACTIVE session, or None."""
    marker = os.path.join(calibration_dir(root), 'ACTIVE')
    if not os.path.isfile(marker):
        return None
    with open(marker, 'r', encoding='utf-8') as f:
        name = f.read().strip()
    path = os.path.join(calibration_dir(root), name)
    return path if name and os.path.isdir(path) else None


def list_sessions(root: str) -> List[str]:
    d = calibration_dir(root)
    if not os.path.isdir(d):
        return []
    return sorted(n for n in os.listdir(d) if os.path.isdir(os.path.join(d, n)))


def load_summary(session: Optional[str]) -> Dict:
    if not session:
        return {}
    path = os.path.join(session, 'summary.yaml')
    if not os.path.isfile(path):
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def missing_required(summary: Dict, steps_cfg: Dict) -> List[str]:
    """Required step ids whose status is not PASS / KEPT_PREVIOUS."""
    done = (summary or {}).get('steps', {}) or {}
    out = []
    for step in steps_cfg.get('steps', []):
        if step.get('required', True) and (done.get(step['id'], {}) or {}).get('status') not in PASSING:
            out.append(step['id'])
    return out


def overlay_params(session: Optional[str]) -> Optional[str]:
    if not session:
        return None
    path = os.path.join(session, 'params_overlay.yaml')
    return path if os.path.isfile(path) else None


def data_override(session: Optional[str], filename: str) -> Optional[str]:
    if not session:
        return None
    path = os.path.join(session, 'data', filename)
    return path if os.path.isfile(path) else None
