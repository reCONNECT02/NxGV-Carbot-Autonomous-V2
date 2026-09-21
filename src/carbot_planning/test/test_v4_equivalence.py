"""Ports against the V4 simulator itself (docs/reference/v4_simulator run in Node).
Skipped when node is not installed."""
import json
import math
import os
import shutil
import subprocess

import numpy as np
import pytest
from carbot_planning.corridor_core import CorridorCfg, CorridorFinder
from carbot_planning.evidence import Evidence, EvidenceCfg
from carbot_planning.local_core import LocalCfg, LocalPlanner
from carbot_planning.sim_core import render_grid

from plan_fixtures import HARNESS, V4, geom, route

pytestmark = pytest.mark.skipif(shutil.which('node') is None, reason='node not installed')


def _js(script, *args):
    return json.loads(subprocess.check_output(['node', os.path.join(HARNESS, script), *args], timeout=120))


@pytest.fixture(scope='module')
def v1():
    return route(V4)


def test_hybrid_plan_equals_v4_build_mission(v1):
    c, m, r = v1
    js = _js('route_v4.js')
    for a, p in zip(js['routes'], [p for p in r.pieces if p['kind'] == 'road']):
        a = np.array(a)
        assert len(a) == len(p['points'])
        assert np.abs(a[:, :2] - p['points'][:, :2]).max() < 1e-4        # after the 1 cm resampling


SITUATIONS = [(0, 300, 0.02, 0.05), (0, 700, -0.015, -0.08), (0, 1200, 0.01, 0.0), (1, 400, 0.0, 0.03)]


@pytest.mark.parametrize('leg,index,dy,da', SITUATIONS)
def test_corridor_and_local_planner_equal_v4(v1, leg, index, dy, da):
    c, m, r = v1
    path = [p for p in r.pieces if p['kind'] == 'road'][leg]['points']
    q = path[index]
    pose = (q[0] - dy * math.sin(q[2]), q[1] + dy * math.cos(q[2]), q[2] + da)
    js = _js('planning_v4.js', json.dumps({'pose': {'x': pose[0], 'y': pose[1], 'a': pose[2]},
                                           'index': index, 'route': leg, 'speed': 0.1, 'steer': 0.0}))
    ev = Evidence(EvidenceCfg())
    ev.now = 10.0
    ev.set_live(render_grid(c, pose, 10.0), pose)
    cf = CorridorFinder(CorridorCfg())
    cf.index = index
    cr = cf.corridor(path, pose, ev, c)
    jc = js['corridor']
    # probes land exactly on grid-cell edges (9 mm steps, 1.8 cm cells): 1e-7 route
    # differences can move one edge by one step, so compare within that tie.
    assert cr.mode == jc['mode']
    assert abs(cr.observed - jc['observed']) <= 1
    assert abs(cr.offset - jc['offset']) <= 0.0025
    # local planner on the SAME guide: must be exact
    guide = np.array(jc['guide'])
    guide = np.c_[guide, np.ones(len(guide))]
    cands = LocalPlanner(LocalCfg(line_tolerance=0.05), geom()).candidates(pose, guide, 0.0, 0.1, ev, c, None)
    jk = {q['id']: q for q in js['candidates']}
    assert [q['id'] for q in js['candidates']] == [q.id for q in cands]
    for q in cands:
        assert q.valid == jk[q.id]['valid']
        assert q.cost == pytest.approx(jk[q.id]['cost'], rel=1e-6, abs=1e-9)
        assert q.command_steer == pytest.approx(jk[q.id]['steer'], abs=1e-9)
