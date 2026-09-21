"""Block 13: V4 Controller behaviour, gear sections."""
import math

import numpy as np
from carbot_planning.sim_core import Plant
from carbot_planning.tracker_core import GearSequencer, Tracker, TrackerCfg, split_gears

from plan_fixtures import DATA, geom, route


def test_split_gears_like_v4():
    P = np.array([[0, 0, 0, 1], [1, 0, 0, 1], [2, 0, 0, -1], [3, 0, 0, -1], [4, 0, 0, 1]], float)
    parts = split_gears(P)
    assert [len(p) for p in parts] == [2, 3, 2]
    assert parts[1][0][0] == 1 and parts[1][0][3] == -1        # cusp pose starts the next gear


def test_straight_arrival_and_speed():
    g = geom()
    tr = Tracker(TrackerCfg(), g)
    path = np.array([[x, 0.0, 0.0, 1.0] for x in np.arange(0, 1.0, 0.01)])
    pl = Plant(0.0, 0.01, 0.0)
    t, vmax = 0.0, 0.0
    for _ in range(3000):
        t += 1 / 30
        r = tr.update(path, (pl.x, pl.y, pl.a), pl.speed, t)
        vmax = max(vmax, r['speed'])
        pl.step(r['speed'], r['steer'], 1 / 30, g)
        if r['arrived']:
            break
    assert r['arrived'] and abs(pl.x - path[-1, 0]) < 0.03 and abs(pl.y) < 0.005
    assert vmax <= 0.14 + 1e-9                                 # 0.07 + 0.07 / (1 + 0) on a straight


def test_team_parking_preview_one_gear_at_a_time():
    c, m, r = route(DATA)
    g = geom()
    path = r.pieces[2]['points']                              # into the parallel bay
    pl = Plant(*path[0, :3])
    gs = GearSequencer(Tracker(TrackerCfg(), g), 0.4)
    gs.set_path(path, 'p')
    t = 0.0
    reasons = set()
    for _ in range(3000):
        t += 1 / 30
        q = gs.update((pl.x, pl.y, pl.a), pl.speed, t)
        reasons.add(q['reason'].split(' (')[0])
        pl.step(q['speed'], q['steer'], 1 / 30, g)
        if q['arrived']:
            break
    assert q['arrived']
    assert any(s.startswith('GEAR CHANGE') for s in reasons)
    assert math.hypot(pl.x - path[-1, 0], pl.y - path[-1, 1]) < 0.02
