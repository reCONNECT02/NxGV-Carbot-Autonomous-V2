"""track_map.yaml v2 (team map_builder.py) + mission loaders (phase 4)."""
import math
import os
import sys

import numpy as np
import pytest
from carbot_common import map_geometry as mg
from carbot_common.course import file_fingerprints, load_course
from carbot_common.mission import load_mission, named_poses

CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', 'carbot_bringup', 'config')
DATA = os.path.join(CONFIG, 'data')
TOOLS = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'map')


@pytest.fixture(scope='module')
def course():
    return load_course(os.path.join(DATA, 'track_map.yaml'))


def test_v2_basics(course):
    assert course.version == 2
    assert course.clearance(3.5, 4.75) == pytest.approx(0.15, abs=1e-6)     # top straight centreline
    assert course.clearance(3.5, 4.75 + 0.15) == pytest.approx(0.0, abs=3e-3)
    cx, cy, r = course.ring
    assert (cx, cy, r) == pytest.approx((2.4, 0.8, 0.6), abs=1e-9)
    assert set(course.exits) == {'west', 'north', 'east'}


def test_features_resolve_from_mission(course):
    assert course.start_pose() == pytest.approx((7.1212, 1.30, math.pi), abs=1e-6)       # P0
    assert course.pose('light_goal_pose') == pytest.approx((6.75, 2.47, -math.pi / 2), abs=1e-6)  # P1
    assert course.in_tunnel(0.47, 0.97) and not course.in_tunnel(5.0, 4.75)
    assert course.surface(3.0, 4.75) == pytest.approx(0.155)


def test_crossable_paint(course):
    x0, x1, y0, y1 = course.areas['lane_change']
    mid = ((x0 + x1) / 2, (y0 + y1) / 2)
    assert course.crossable(np.array([mid[0]]), np.array([mid[1]]))[0]          # dashed divider
    assert not course.crossable(np.array([3.5]), np.array([4.75]))[0]


def test_body_check_matches_team_tool_on_its_routes(course):
    import yaml
    from carbot_common.geometry import geometry
    v = yaml.safe_load(open(os.path.join(CONFIG, 'params', 'common.yaml')))['/**']['ros__parameters']['vehicle']
    g = geometry(v)
    m = load_mission(os.path.join(DATA, 'mission.yaml'))
    road = [p for p in m.pieces if p.kind == 'road']
    for p in road:
        P = p.points
        assert course.body_clear_many(P[:, 0], P[:, 1], P[:, 2], g, -0.005).all()


def test_fingerprints_accept_crlf(tmp_path):
    p = tmp_path / 'a.yaml'
    p.write_bytes(b'a: 1\nb: 2\n')
    lf = file_fingerprints(str(p))
    p.write_bytes(b'a: 1\r\nb: 2\r\n')
    crlf = file_fingerprints(str(p))
    assert lf['lf'] == crlf['lf'] and lf['crlf'] == crlf['raw']


def test_mission_v2_and_v1():
    m = load_mission(os.path.join(DATA, 'mission.yaml'))
    assert m.version == 2 and m.planned
    assert [p.kind for p in m.pieces] == ['road', 'road', 'manoeuvre', 'manoeuvre', 'road', 'manoeuvre']
    assert [v['direction'] for v in m.visits()] == ['west', 'north']
    m1 = load_mission(os.path.join(DATA, 'v4_reference', 'mission.yaml'))
    assert m1.version == 1 and [v['direction'] for v in m1.visits()] == ['west', 'north']
    assert [p.kind for p in m1.pieces] == ['road', 'road', 'manoeuvre']


def test_named_poses_v2():
    p = named_poses(CONFIG)
    assert 'start_pose' in p and 'P3' in p


def test_geometry_port_equals_map_builder():
    pytest.importorskip('cv2')
    sys.path.insert(0, os.path.abspath(TOOLS))
    import map_builder as mb
    import mission_planner as mp
    tpl, _ = mp.load_map(os.path.join(DATA, 'track_map.yaml'))
    course = load_course(os.path.join(DATA, 'track_map.yaml'))
    A, B = mb.centrelines(tpl, 0.025), mg.centrelines(course.tpl, 0.025)
    assert set(A) == set(B) and all(np.allclose(A[k], B[k]) for k in A)
    Aa, Ba = mb.areas(tpl), mg.areas(course.tpl)
    assert set(Aa) == set(Ba) and all(np.allclose(Aa[k]['poly'], Ba[k]['poly']) for k in Aa)
    f, x0, y0, res = mb.road_field(tpl)
    ny, nx = f.shape
    X, Y = np.meshgrid(x0 + np.arange(nx) * res, y0 + np.arange(ny) * res)
    mine = course.clearance(X, Y)
    near = (np.abs(f) < 0.2) & (mine > -5)
    flip = near & ((mine >= 0) != (f >= 0))
    assert np.abs(mine[flip]).max() < 0.01 and np.abs(f[flip]).max() <= 0.0101   # only at the edge
