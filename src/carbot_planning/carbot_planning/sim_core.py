"""Closed-loop test drive of the phase-4 cores on a laptop (no ROS, no car).

The same core classes the ROS nodes use (route_core, corridor_core,
local_core, tracker_core, mission_core) drive a simple V4-style plant
(vehicle.js Plant: servo lag + rate limit, first-order speed) around the map.
Sensors are ideal: the road grid is rendered from the prior map like V4
scene.js (road where clearance >= 0, paint just outside), localization is the
true pose. Detectors are scripted (traffic light turns GREEN after
`light_red_s` of waiting, the boom gate is OPEN). Manoeuvre pieces are driven
along mission_planner's PREVIEW as a stand-in for block 11 (phase 5).

Used by tools/sandbox/run_planning.py and src/carbot_planning/test.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from .corridor_core import CorridorCfg, CorridorFinder
from .evidence import Evidence, EvidenceCfg, Grid
from .local_core import LocalCfg, LocalPlanner
from .mission_core import Inputs, MissionCfg, MissionMachine, RoutePiece, Visit
from .route_core import bicycle
from .tracker_core import GearSequencer, Tracker, TrackerCfg


def render_grid(course, pose, stamp, rows=100, res=0.018, x0=-0.65, y0=-0.9, paint_m=0.10) -> Grid:
    """V4-like camera road grid in base_link (road, paint border, other)."""
    lx = x0 + (np.arange(rows) + 0.5) * res
    ly = y0 + (np.arange(rows) + 0.5) * res
    X, Y = np.meshgrid(lx, ly, indexing='ij')
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = course.clearance(pose[0] + X * c - Y * s, pose[1] + X * s + Y * c)
    kind = np.where(d >= 0, 1, np.where(d >= -paint_m, 2, 3)).astype(np.uint8)
    return Grid(rows, rows, res, x0, y0, kind, (kind == 1).astype(np.uint8), stamp=stamp)


@dataclass
class Plant:
    x: float
    y: float
    a: float
    speed: float = 0.0
    steer: float = 0.0

    def step(self, speed_cmd, steer_cmd, dt, g, lag_steer=0.12, lag_speed=0.18, rate=math.pi):
        sd = max(-rate, min(rate, (steer_cmd - self.steer) / lag_steer))
        self.steer = max(-g.max_steer, min(g.max_steer, self.steer + sd * dt))
        self.speed += (speed_cmd - self.speed) * min(1.0, dt / lag_speed)
        if abs(speed_cmd) < 1e-8 and abs(self.speed) < 0.001:
            self.speed = 0.0
        self.x, self.y, self.a = bicycle(self.x, self.y, self.a, self.speed * dt, math.tan(self.steer) / g.wb)


@dataclass
class SimLog:
    t: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    mode: List[str] = field(default_factory=list)
    margin: List[float] = field(default_factory=list)
    events: List[tuple] = field(default_factory=list)
    candidates_valid: List[int] = field(default_factory=list)
    corridor_observed: List[int] = field(default_factory=list)
    reason: List[str] = field(default_factory=list)
    piece: List[int] = field(default_factory=list)
    track_error: List[float] = field(default_factory=list)


def anchor(points: np.ndarray, pose) -> np.ndarray:
    """Rigidly move a manoeuvre preview so it starts at the car's actual pose.
    Stand-in only: block 11 (phase 5) replans from the actual pose instead."""
    P = np.asarray(points, float).copy()
    da = pose[2] - P[0, 2]
    c, s = math.cos(da), math.sin(da)
    dx, dy = P[:, 0] - P[0, 0], P[:, 1] - P[0, 1]
    P[:, 0] = pose[0] + dx * c - dy * s
    P[:, 1] = pose[1] + dx * s + dy * c
    P[:, 2] = np.arctan2(np.sin(P[:, 2] + da), np.cos(P[:, 2] + da))
    return P


def pieces_from_route(route) -> (List[RoutePiece], List[Visit]):
    pcs = [RoutePiece(p['index'], p['leg'], p['leg_id'], p['kind'], p['bay'], p['points'],
                      p['end_behaviour'], p['sections']) for p in route.pieces]
    vs = [Visit(v['visit'], v['piece'], v['enter'], v['leave'], v['exit'], v['label']) for v in route.visits]
    return pcs, vs


def simulate(course, route, rules, challenges, g, limits: Dict, t_max=400.0, dt=1 / 30,
             light_red_s=2.0, gate_open=True, stop_at_complete=True, verbose=False) -> SimLog:
    pieces, visits = pieces_from_route(route)
    mm = MissionMachine(course, pieces, visits, rules, challenges, MissionCfg(),
                        float(limits['max_speed_mps']))
    tcfg = TrackerCfg(max_speed=float(limits['max_speed_mps']), parking_speed=float(limits['parking_speed_mps']))
    tracker = Tracker(tcfg, g)
    gears = GearSequencer(Tracker(tcfg, g), 0.4)
    cf = CorridorFinder(CorridorCfg())
    lp = LocalPlanner(LocalCfg(line_tolerance=float(limits['line_tolerance_m'])), g)
    ev = Evidence(EvidenceCfg())
    x, y, a = pieces[0].points[0, :3]
    plant = Plant(float(x), float(y), float(a))
    log = SimLog()
    t, last_cam, last_plan = 0.0, -1.0, -1.0
    local_path = None
    corr = None
    active = -1
    steer_est = 0.0
    hold_started = None
    road_req = {'arrived': False, 't': -1e9}
    park_req = {'arrived': False, 't': -1e9}
    cmd = (0.0, 0.0)
    out = None
    anchored = {}
    while t < t_max:
        t += dt
        pose = (plant.x, plant.y, plant.a)
        ev.now = t
        if t - last_cam >= 1 / 8:
            ev.set_live(render_grid(course, pose, t), pose)
            last_cam = t
        # scripted detectors
        in_hold = out is not None and out.mode == 'HOLD' and out.hold_reason == 'TRAFFIC HOLD'
        if in_hold and hold_started is None:
            hold_started = t
        light = 'GREEN' if hold_started is not None and t - hold_started > light_red_s else 'RED'
        inp = Inputs(t=t, armed=True, pose=pose, gpose=pose, light=light, light_t=t,
                     gate='OPEN' if gate_open else 'CLOSED', gate_conf=0.9, gate_t=t,
                     corridor_observed=corr.observed if corr else 0, corridor_t=t if corr else -1e9,
                     camera_t=last_cam, road_arrived=road_req['arrived'], road_t=road_req['t'],
                     parking_arrived=park_req['arrived'], parking_t=park_req['t'],
                     parking_end=tuple(anchored[mm.i][-1, :2]) if mm.i in anchored else None)
        inp.parking_arrived = park_req['arrived']
        out = mm.step(inp)
        for e in out.events:
            log.events.append((round(t, 2),) + e)
            if verbose:
                print(f'{t:7.2f} {e[0]}: {e[1]}')
        if mm.complete and stop_at_complete:
            break
        p = pieces[mm.i]
        if mm.i != active:
            active = mm.i
            tracker.reset()
            cf.reset()
            local_path = None
        parking = p.kind == 'manoeuvre'
        # blocks 09 + 10 at 5 Hz
        if not parking and t - last_plan >= 0.2:
            last_plan = t
            corr = cf.corridor(p.points, pose, ev, course)
            cands = lp.candidates(pose, corr.guide, steer_est, plant.speed, ev, course, None)
            win = next((q for q in cands if q.valid), None)
            log.candidates_valid.append(sum(q.valid for q in cands))
            log.corridor_observed.append(corr.observed)
            if win is not None:
                lpth = corr.guide.copy()
                lpth[:, 0] -= win.offset * np.sin(lpth[:, 2])
                lpth[:, 1] += win.offset * np.cos(lpth[:, 2])
                local_path = lpth
            else:
                local_path = np.zeros((0, 4))
        # block 13
        if parking:            # block 11 stand-in: follow the mission_planner preview, one gear at a time
            if gears.key != mm.i:
                anchored[mm.i] = anchor(p.points, pose)
            gears.set_path(anchored[mm.i], mm.i)
            req = gears.update(pose, plant.speed, t)
        else:
            req = tracker.update(p.points, pose, plant.speed, t, False)
        if not parking and not req['arrived'] and req['reason'] == 'TRACKING':
            if local_path is not None and not len(local_path):
                req = {'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'No feasible local candidate'}
            else:
                s = tracker.road_steer(local_path, pose)
                if s is not None:
                    req['steer'] = s
        if req['speed'] != 0.0 and out.zone_max > 0:
            req['speed'] = math.copysign(min(abs(req['speed']), out.zone_max), req['speed'])
        if parking:
            park_req = {'arrived': req['arrived'], 't': t}
        else:
            road_req = {'arrived': req['arrived'], 't': t}
        # block 15 stand-in: follow the active source, HOLD = stop
        cmd = (0.0, req['steer'] * 0.0) if out.source == 'HOLD' or out.mode == 'SAFETY_STOP' \
            else (req['speed'], req['steer'])
        steer_est += max(-math.pi * dt, min(math.pi * dt, (cmd[1] - steer_est) / 0.12 * dt))
        plant.step(cmd[0], cmd[1], dt, g)
        log.t.append(t)
        log.x.append(plant.x)
        log.y.append(plant.y)
        log.mode.append(out.mode + (':' + out.hold_reason if out.hold_reason else ''))
        log.margin.append(course.body_margin((plant.x, plant.y, plant.a), g))
        log.reason.append(req['reason'])
        log.piece.append(mm.i)
        log.track_error.append(gears.tracker.error if parking else tracker.error)
    return log
