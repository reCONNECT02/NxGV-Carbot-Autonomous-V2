"""Block 08 rules: observed-green light (no timer), gate hold, gate/route
mismatch = log only, e-stop = manual intervention, full closed-loop run."""
import numpy as np
import pytest
from carbot_planning.mission_core import Inputs, MissionCfg, MissionMachine
from carbot_planning.sim_core import pieces_from_route, simulate

from plan_fixtures import DATA, challenges, common, geom, route


@pytest.fixture(scope='module')
def team():
    return route(DATA)


def machine(team):
    c, m, r = team
    pcs, vs = pieces_from_route(r)
    return c, pcs, MissionMachine(c, pcs, vs, m.rules, challenges(), MissionCfg(), 0.18)


def test_idle_until_armed(team):
    c, pcs, mm = machine(team)
    out = mm.step(Inputs(t=1.0, pose=tuple(pcs[0].points[0, :3])))
    assert out.mode == 'IDLE' and out.source == 'HOLD'
    out = mm.step(Inputs(t=1.1, armed=True, pose=tuple(pcs[0].points[0, :3])))
    assert out.mode == 'ROAD' and out.source == 'ROAD' and out.path_changed


def _at_light(team):
    c, pcs, mm = machine(team)
    end = tuple(pcs[0].points[-1, :3])
    mm.step(Inputs(t=0.0, armed=True, pose=tuple(pcs[0].points[0, :3])))
    out = mm.step(Inputs(t=1.0, armed=True, pose=end, road_arrived=True, road_t=1.0))
    assert out.piece == 1                                       # leg 1 done -> leg 2
    return mm, end


def test_traffic_light_holds_forever_without_green(team):
    mm, end = _at_light(team)
    for k in range(600):                                       # 5 simulated minutes of RED
        t = 3.0 + k * 0.5
        out = mm.step(Inputs(t=t, armed=True, pose=end, light='RED', light_t=t))
        assert out.mode == 'HOLD' and out.hold_reason == 'TRAFFIC HOLD'


def test_traffic_light_needs_a_fresh_green(team):
    mm, end = _at_light(team)
    out = mm.step(Inputs(t=5.0, armed=True, pose=end, light='GREEN', light_t=1.0))    # stale
    assert out.hold_reason == 'TRAFFIC HOLD'
    out = mm.step(Inputs(t=5.1, armed=True, pose=end, light='GREEN', light_t=5.1))
    assert out.mode == 'ROAD' and any(e[0] == 'LIGHT GREEN' for e in out.events)


def test_gate_hold_until_open(team):
    c, pcs, mm = machine(team)
    gx, gy = c.point('boom_gates', 'challenge4')
    P = pcs[0].points
    k = int(np.argmin(np.hypot(P[:, 0] - gx, P[:, 1] - gy)))
    before = tuple(P[k - 30, :3])                              # 30 cm before the gate
    mm.step(Inputs(t=0.0, armed=True, pose=tuple(P[0, :3])))
    out = mm.step(Inputs(t=1.0, armed=True, pose=before, gate='CLOSED', gate_t=1.0, gate_conf=0.9))
    assert out.hold_reason == 'GATE HOLD'
    out = mm.step(Inputs(t=2.0, armed=True, pose=before, gate='OPEN', gate_t=2.0, gate_conf=0.9))
    assert out.mode == 'ROAD'


def test_gate_route_mismatch_logs_but_keeps_route(team):
    c, pcs, mm = machine(team)
    gx, gy = c.point('boom_gates', 'roundabout_exit1')
    mm.step(Inputs(t=0.0, armed=True, pose=tuple(pcs[0].points[0, :3])))
    P = pcs[0].points
    k = int(np.argmin(np.hypot(P[:, 0] - gx, P[:, 1] - gy)))
    route_before = [p.points.copy() for p in pcs]
    out = mm.step(Inputs(t=1.0, armed=True, pose=tuple(P[k - 40, :3]), gate='CLOSED', gate_t=1.0, gate_conf=0.9))
    assert out.mismatches and out.mismatches[0]['planned'] == 'west'
    assert out.mode == 'ROAD'                                   # no hold, no reroute
    assert all(np.array_equal(a, b.points) for a, b in zip(route_before, pcs))


def test_estop_is_reported_as_manual_intervention(team):
    c, pcs, mm = machine(team)
    mm.step(Inputs(t=0.0, armed=True, pose=tuple(pcs[0].points[0, :3])))
    out = mm.step(Inputs(t=1.0, armed=True, estop=True, pose=tuple(pcs[0].points[5, :3])))
    assert out.mode == 'SAFETY_STOP'
    assert any(e[0] == 'MANUAL INTERVENTION' for e in out.events)


def test_tunnel_needs_trigger_and_zone(team):
    c, pcs, mm = machine(team)
    P = pcs[0].points
    inside = [i for i in range(len(P)) if c.in_tunnel(P[i, 0], P[i, 1])]
    far = tuple(P[50, :3])
    mm.step(Inputs(t=0.0, armed=True, pose=tuple(P[0, :3])))
    out = mm.step(Inputs(t=1.0, armed=True, pose=far, tunnel=True, tunnel_t=1.0))
    assert out.mode == 'ROAD'                                   # trigger outside the zone ignored
    out = mm.step(Inputs(t=2.0, armed=True, pose=tuple(P[inside[len(inside) // 2], :3]), tunnel=True, tunnel_t=2.0))
    assert out.mode == 'TUNNEL' and out.source == 'TUNNEL'


def test_closed_loop_full_mission(team):
    c, m, r = team
    log = simulate(c, r, m.rules, challenges(), geom(), common()['limits'], t_max=600)
    names = [e[1] for e in log.events]
    assert 'MISSION COMPLETE' in names
    entered = sorted({e[3] for e in log.events if e[1] == 'CHALLENGE'})
    assert entered == list(range(1, 12))
    assert names.index('LIGHT GREEN') > names.index('TRAFFIC HOLD')
    road = [mg for mg, pc in zip(log.margin, log.piece) if pc in (0, 1)]
    assert min(road) > -0.01                                    # lane driving stays within 1 cm of the edge
    # phase 5: real block 11 (planned from the pose, one gear section at a time)
    assert names.count('PARKING PASSED') == 2 and 'MANOEUVRE COMPLETE' in names
    assert 'PARKING CHECK FAILED' not in names and 'HOLD' not in names
    manoeuvre = [mg for mg, pc in zip(log.margin, log.piece) if r.pieces[pc]['kind'] == 'manoeuvre']
    assert min(manoeuvre) > 0.0                                 # parking / un-parking stay on the map
