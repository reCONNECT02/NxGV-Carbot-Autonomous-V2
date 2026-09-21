"""Block 12: V4 recovery rules + a closed-loop recovery on the team map."""
import math

import numpy as np
import pytest
from carbot_planning.evidence import Evidence, EvidenceCfg
from carbot_planning.recovery_core import (RecInputs, Recovery, RecoveryCfg, obstacles_clear_many,
                                           road_clear_many, search)
from carbot_planning.sim_core import render_grid, simulate
from plan_fixtures import DATA, challenges, common, geom, route


@pytest.fixture(scope='module')
def team():
    return route(DATA)


def _stuck_pose(r):
    P = r.pieces[0]['points']
    x, y, a = P[300, :3]
    off, ang = 0.02, 0.6
    return (x + off * math.sin(a), y - off * math.cos(a), a - ang)


def test_closed_loop_recovery_reverses_and_rejoins(team):
    c, m, r = team
    log = simulate(c, r, m.rules, challenges(), geom(), common()['limits'], t_max=30,
                   start_pose=_stuck_pose(r), stop_at_complete=False)
    names = [e[1] for e in log.events]
    for n in ('RECOVERY BRAKE', 'RECOVERY PATH SELECTED', 'RECOVERY GEAR CHANGE', 'RECOVERY COMPLETE',
              'RECOVERY DONE'):
        assert n in names, names
    assert min(log.margin) > -0.01
    assert any(m.startswith('RECOVERY') for m in log.mode)


def test_candidates_follow_v4_rules(team):
    c, _, r = team
    g = geom()
    st = _stuck_pose(r)
    ev = Evidence(EvidenceCfg())
    ev.now = 1.0
    ev.set_live(render_grid(c, st, 1.0), st)
    cfg = RecoveryCfg()
    ev_list, win = search(st, r.pieces[0]['points'], 290, c, g, cfg, ev, None)
    assert win is not None and win.valid
    assert win.points[0, 3] == -1 and win.points[-1, 3] == 1
    assert cfg.reverse_min <= win.reverse <= cfg.reverse_max
    assert all(not q.valid for q in ev_list if q.reverse > cfg.reverse_max)
    valid = [q for q in ev_list if q.valid]
    assert [q.cost for q in valid] == sorted(q.cost for q in valid) and ev_list[0] is win
    # a LiDAR return inside the swept footprint rejects the winner
    mid = win.points[len(win.points) // 3]
    hits = np.array([[mid[0], mid[1]]])
    _, win2 = search(st, r.pieces[0]['points'], 290, c, g, cfg, ev, hits)
    assert win2 is None or not np.allclose(win2.points[:, :2].mean(0), win.points[:, :2].mean(0))
    assert not obstacles_clear_many(g, win.points[:, :4], hits).all()
    assert road_clear_many(c, g, win.points, 0.05).all()


def test_holds_pause_and_no_problem_cancels(team):
    c, _, r = team
    g = geom()
    rec = Recovery(RecoveryCfg(), c, g)
    st = _stuck_pose(r)
    route_pts = r.pieces[0]['points']
    base = dict(pose=st, speed=0.0, route=route_pts, lidar_age=0.0, camera_age=0.0, ev=Evidence(EvidenceCfg()))
    assert rec.update(RecInputs(t=0.0, problem='x', **base)).request is None        # dwell 0.45 s
    assert rec.update(RecInputs(t=0.5, problem='x', **base)).request is not None    # active: BRAKE
    out = rec.update(RecInputs(t=0.6, problem='x', hard_hold='camera_fresh: stale', **base))
    assert out.request['speed'] == 0.0 and rec.reason.startswith('Recovery paused')
    out = rec.update(RecInputs(t=0.7, problem='', **base))                          # feasible again
    assert not rec.active and 'RECOVERY CANCELLED' in [e[0] for e in out.events]
    assert rec.update(RecInputs(t=1.0, problem='x', permitted=False, **base)).request is None
