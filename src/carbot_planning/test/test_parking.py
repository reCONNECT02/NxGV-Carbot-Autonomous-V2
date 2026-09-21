"""Block 11 on the team map: plans from the actual pose, bay observation, the
V4 transition() sequence (one gear section at a time, cusp / heading replans)."""
import math

import numpy as np
import pytest
from carbot_planning.parking_core import (Bay, ParkingCfg, ParkingSession, observe_bay, parked_check,
                                          parking_plan, reverse_time)
from carbot_planning.sim_core import Plant, paint_points
from carbot_planning.tracker_core import GearSequencer, Tracker, TrackerCfg
from plan_fixtures import DATA, geom, route


@pytest.fixture(scope='module')
def team():
    return route(DATA)


def _manoeuvres(r):
    return [p for p in r.pieces if p['kind'] == 'manoeuvre']


def test_every_manoeuvre_plans_from_perturbed_starts(team):
    c, m, r = team
    g = geom()
    rng = np.random.default_rng(1)
    for p in _manoeuvres(r):
        bay = Bay.from_course(c, p['bay'])
        goal = tuple(p['points'][-1, :3])
        dock = bay.contains(goal[0], goal[1])
        for k in range(3):
            start = tuple(np.asarray(p['points'][0, :3]) + (rng.normal(0, [0.01, 0.01, 0.03]) if k else 0))
            unpark = not dock and bay.contains(start[0], start[1])
            res = parking_plan(start, goal, c, g, ParkingCfg(), dock, bay.poly, unpark_from_bay=unpark)
            P = res.path
            assert len(P), (p['bay'], k, res.reason)
            assert np.hypot(*(P[0, :2] - start[:2])) < 1e-9 and np.hypot(*(P[-1, :2] - goal[:2])) < 1e-6
            assert c.body_clear_many(P[:, 0], P[:, 1], P[:, 2], g, 0.003).all()
            if dock:
                assert parked_check(P[-1, :3], goal, bay, g, ParkingCfg())[0]
            if unpark:
                assert res.stage.startswith('reversed parking search')


def test_reverse_time_flips_direction_keeps_poses():
    P = np.array([[0, 0, 0.1, -1], [0.1, 0, 0.1, -1], [0.1, 0, 0.1, 1], [0.2, 0.1, 0.3, 1]], float)
    R = reverse_time(P)
    assert np.allclose(R[:, :3], P[::-1, :3]) and list(R[:, 3]) == [-1, -1, 1, 1]


def test_bay_observation_follows_the_tape(team):
    c, _, _ = team
    bay = Bay.from_course(c, 'parallel_bay')
    goal = (2.015, 2.437, math.pi / 2)
    tape = paint_points(c, goal, radius=1.5)
    obs = observe_bay(bay, tape, goal, ParkingCfg())
    assert obs.valid and abs(obs.shift_along) < 0.012 and abs(obs.shift_lateral) < 0.012
    shifted = tape + 0.03 * bay.u + 0.02 * bay.n            # bay taped 3 cm further, 2 cm deeper
    obs = observe_bay(bay, shifted, goal, ParkingCfg())
    assert obs.valid
    assert abs(obs.goal[0] - (goal[0] + 0.03 * bay.u[0] + 0.02 * bay.n[0])) < 0.012
    assert abs(obs.goal[1] - (goal[1] + 0.03 * bay.u[1] + 0.02 * bay.n[1])) < 0.012
    far = tape + 0.2 * bay.u                                 # implausible: rejected
    assert not observe_bay(bay, far, goal, ParkingCfg()).valid
    assert not observe_bay(bay, np.zeros((0, 2)), goal, ParkingCfg()).valid


def _drive_session(c, g, piece, start, t_max=90.0):
    s = ParkingSession(ParkingCfg(), c, g)
    out = s.start('k', tuple(piece['points'][-1, :3]), piece['bay'], 0.0)
    assert out.publish is not None and len(out.publish) == 0          # previous path cleared at once
    pl = Plant(*start)
    gs = GearSequencer(Tracker(TrackerCfg(), g), 0.4)
    section, key, arrived_t, t, names = None, 0, -1e9, 0.0, []
    while t < t_max and s.state != 'DONE':
        t += 1 / 30
        pose = (pl.x, pl.y, pl.a)
        o = s.update(t, pose, pl.speed, arrived_t, lambda: paint_points(c, pose))
        names += [e[0] for e in o.events]
        if o.publish is not None:
            section, key = (o.publish if len(o.publish) else None), key + 1
        q = {'speed': 0.0, 'steer': 0.0, 'arrived': False}
        if section is not None:
            gs.set_path(section, key)
            q = gs.update(pose, pl.speed, t)
            if q['arrived']:
                arrived_t = t
        pl.step(q['speed'], q['steer'], 1 / 30, g)
    return s, pl, names


def test_session_parks_in_the_parallel_bay(team):
    c, _, r = team
    g = geom()
    p = _manoeuvres(r)[0]
    s, pl, names = _drive_session(c, g, p, tuple(p['points'][0, :3]))
    assert s.state == 'DONE' and s.parked_ok, names
    assert 'LIVE PARKING SEARCH' in names and 'PARKING PATH SELECTED' in names and 'GEAR CHANGE' in names
    assert parked_check((pl.x, pl.y, pl.a), s.goal, Bay.from_course(c, 'parallel_bay'), g, ParkingCfg())[0]


def test_session_unparks_and_parks_perpendicular(team):
    c, _, r = team
    g = geom()
    unpark, perp = _manoeuvres(r)[1], _manoeuvres(r)[2]
    s, pl, names = _drive_session(c, g, unpark, tuple(unpark['points'][0, :3]))
    assert s.state == 'DONE' and 'MANOEUVRE COMPLETE' in names
    end = unpark['points'][-1]
    assert math.hypot(pl.x - end[0], pl.y - end[1]) < 0.06 and abs(math.atan2(math.sin(pl.a - end[2]),
                                                                            math.cos(pl.a - end[2]))) < 0.06
    s, pl, names = _drive_session(c, g, perp, tuple(perp['points'][0, :3]))
    assert s.state == 'DONE' and s.parked_ok, names


def test_bay_not_seen_falls_back_to_the_map_after_timeout(team):
    c, _, r = team
    g = geom()
    p = _manoeuvres(r)[0]
    s = ParkingSession(ParkingCfg(fallback_after_s=1.0), c, g)
    s.start('k', tuple(p['points'][-1, :3]), p['bay'], 0.0)
    pose = tuple(p['points'][0, :3])
    o = s.update(0.5, pose, 0.0, -1e9, lambda: np.zeros((0, 2)))
    assert s.state == 'OBSERVING' and 'BAY OBSERVATION HOLD' in s.reason
    o = s.update(1.6, pose, 0.0, -1e9, lambda: np.zeros((0, 2)))
    assert 'BAY NOT OBSERVED' in [e[0] for e in o.events] and o.publish is not None and len(o.publish)
    assert s.goal == s.prior_goal
