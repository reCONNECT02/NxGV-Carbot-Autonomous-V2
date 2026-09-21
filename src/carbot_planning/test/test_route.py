"""Block 07: v2 route check, v1 planning, roundabout exits, failure reasons."""
import copy
import os
import shutil

import numpy as np
import pytest
import yaml
from carbot_common.mission import load_mission
from carbot_planning.route_core import resample, roundabout_visits, validate

from plan_fixtures import DATA, V4, geom, load, route


def test_team_v2_route_is_accepted():
    c, m, r = route(DATA)
    assert r.ok, r.reason
    assert [p['kind'] for p in r.pieces] == ['road', 'road', 'manoeuvre', 'manoeuvre', 'road', 'manoeuvre']
    assert [(v['visit'], v['exit']) for v in r.visits] == [(1, 'west'), (2, 'north')]
    for p in r.pieces:                                        # 1 cm resampled
        d = np.hypot(*np.diff(p['points'][:, :2], axis=0).T)
        assert d.max() < 0.0105


def test_v1_reference_route_plans_with_exits():
    c, m, r = route(V4)
    assert r.ok, r.reason
    assert [len(p['points']) for p in r.pieces if p['kind'] == 'road'] == [1777, 1017]   # V4 buildMission
    assert [v['exit'] for v in r.visits] == ['west', 'north']
    assert r.pieces[-1]['kind'] == 'manoeuvre' and r.pieces[-1]['bay'] == 'parallel_bay'


def _copy(tmp_path):
    for f in ('track_map.yaml', 'mission.yaml', 'mission_rules.yaml', 'track_features.yaml'):
        shutil.copy(os.path.join(DATA, f), tmp_path / f)
    return str(tmp_path)


def test_map_changed_after_planning_fails(tmp_path):
    d = _copy(tmp_path)
    with open(os.path.join(d, 'track_map.yaml'), 'a') as f:
        f.write('# edited\n')
    c, m, r = route(d)
    assert not r.ok and 'different track_map.yaml' in r.reason


def test_crlf_copy_of_the_map_still_matches(tmp_path):
    d = _copy(tmp_path)
    p = os.path.join(d, 'track_map.yaml')
    raw = open(p, 'rb').read().replace(b'\n', b'\r\n')
    open(p, 'wb').write(raw)
    c, m, r = route(d)
    assert r.ok, r.reason


def test_wrong_planned_exit_fails(tmp_path):
    d = _copy(tmp_path)
    p = os.path.join(d, 'mission_rules.yaml')
    rules = yaml.safe_load(open(p))
    rules['roundabout_visits'][0]['exit'] = 'north'
    yaml.safe_dump(rules, open(p, 'w'))
    c, m, r = route(d)
    assert not r.ok and 'roundabout visit 1' in r.reason


def test_resample_keeps_cusps():
    P = np.array([[0, 0, 0, 1], [0.1, 0, 0, 1], [0.05, 0, 0, -1], [0.0, 0, 0, -1]], float)
    R = resample(P, 0.01)
    assert set(R[:, 3]) == {1.0, -1.0}
    k = np.flatnonzero(np.diff(R[:, 3]))[0]
    assert R[k, 0] == pytest.approx(0.1) and R[k + 1, 0] == pytest.approx(0.1)


def test_off_road_piece_is_reported():
    c, m, r = route(DATA)
    pts = r.pieces[0]['points'].copy()
    pts[500, 1] += 0.3
    ok, bad, _ = validate(c, geom(), pts, 0.005)
    assert not ok and bad == 500
