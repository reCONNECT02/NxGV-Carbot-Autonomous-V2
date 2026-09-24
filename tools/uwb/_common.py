"""Shared by tools/uwb/*.py: load anchors (uwb.yaml) and Haffiz's positioning config
(common.yaml uwb_positioning) exactly as the car does. Needs the workspace sourced
(source install/setup.bash) so uwb_localization / carbot_common import."""
import os

from carbot_common import calibration_store as cs
from carbot_common.calib_tools import bringup_config_dir
from carbot_common.data import load_yaml
from uwb_localization.positioning import cfg_from_common_yaml
from uwb_localization.uwb_core import AnchorSet


def uwb_yaml_path(override: str = '') -> str:
    """--uwb-yaml, else the ACTIVE calibration session's data/uwb.yaml (step 10), else the installed one."""
    return override or cs.data_override(cs.active_session(cs.data_root()), 'uwb.yaml') or \
        os.path.join(bringup_config_dir(), 'data', 'uwb.yaml')


def load(override: str = ''):
    path = uwb_yaml_path(override)
    doc = load_yaml(path)
    return doc, AnchorSet.from_yaml(doc), cfg_from_common_yaml(), path
