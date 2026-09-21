"""Shared fixtures: data folders, vehicle geometry, planned routes (no ROS)."""
import os

import yaml
from carbot_common.course import file_fingerprints, load_course
from carbot_common.geometry import geometry
from carbot_common.mission import load_mission
from carbot_planning.route_core import PlanCfg, plan_mission

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO = os.path.dirname(SRC)
CONFIG = os.path.join(SRC, 'carbot_bringup', 'config')
DATA = os.path.join(CONFIG, 'data')                       # team map + mission (v2)
V4 = os.path.join(DATA, 'v4_reference')                  # V4 simulator (v1)
HARNESS = os.path.join(REPO, 'tools', 'v4_harness')


def common():
    return yaml.safe_load(open(os.path.join(CONFIG, 'params', 'common.yaml')))['/**']['ros__parameters']


def geom():
    return geometry(common()['vehicle'])


def challenges():
    return yaml.safe_load(open(os.path.join(DATA, 'challenges.yaml')))['challenges']


def load(folder):
    c = load_course(os.path.join(folder, 'track_map.yaml'))
    m = load_mission(os.path.join(folder, 'mission.yaml'))
    return c, m


def route(folder, **kw):
    c, m = load(folder)
    r = plan_mission(c, m, geom(), PlanCfg(), file_fingerprints(os.path.join(folder, 'track_map.yaml')),
                     0.005, log=lambda s: None, **kw)
    return c, m, r
