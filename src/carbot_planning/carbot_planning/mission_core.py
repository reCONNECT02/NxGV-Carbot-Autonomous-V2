"""BLOCK 08 core (no ROS): choose what happens now (V4 app.js step / transition).

The route (block 07) is a list of PIECES: road pieces are driven in ROAD by the
lane planner + tracker, manoeuvre pieces in PARKING by block 11 + tracker.
This machine walks the pieces in order and each cycle decides:

  mode           IDLE | ROAD | TUNNEL | PARKING | RECOVERY | HOLD | SAFETY_STOP | COMPLETE
  active_source  which MotionRequest the command owner must follow
                 (ROAD | TUNNEL | PARKING | RECOVERY | HOLD)

Rules (rulebook + V4):
  * Traffic light: after the traffic_light_stop leg, while within 0.35 m of its
    end, HOLD until GREEN is OBSERVED (fresh detection). Never a timer.
  * Challenge 4 boom gate: V4 GATE HOLD within 0.53 m unless OPEN is observed;
    OPEN observed within 0.65 m with the gate ahead -> commit.
  * Roundabout exits are fixed by the route. The roundabout gate state is only
    compared with the planned exit: a disagreement is logged (GateRouteMismatch)
    and shown as a banner; the route never changes.
  * Tunnel: the base LiDAR trigger (/tunnel_detected) AND the route position
    in the tunnel zone -> TUNNEL (the base tunnel follower drives).
  * Speed zones: the lowest active cap wins (mission rules speed_zones).
  * Holds after transitions: 1.0 s after a leg, 0.5 s before parking, 0.4 s at
    gear/manoeuvre changes (V4 transition()).
  * Safety veto / e-stop -> SAFETY_STOP. The e-stop counts as a manual
    intervention (0 marks) and is reported as such.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .corridor_core import closest

ROAD, TUNNEL, PARKING, RECOVERY = 'ROAD', 'TUNNEL', 'PARKING', 'RECOVERY'
HOLD, IDLE, COMPLETE, SAFETY_STOP = 'HOLD', 'IDLE', 'COMPLETE', 'SAFETY_STOP'


@dataclass
class RoutePiece:
    """A route piece as block 07 publishes it (points dense, 1 cm)."""
    index: int
    leg: int
    leg_id: str
    kind: str                   # road | manoeuvre
    bay: str
    points: np.ndarray          # N x 4
    end_behaviour: str = ''     # set on the last piece of a leg
    sections: List[str] = field(default_factory=list)


@dataclass
class Visit:
    visit: int
    piece: int
    enter: int
    leave: int
    direction: str
    label: str


@dataclass
class MissionCfg:
    rate_hz: float = 20.0
    tunnel_requires_zone: bool = True
    tunnel_zone_margin: float = 0.30
    tunnel_exit_dwell_s: float = 0.5
    tunnel_trigger_max_age_s: float = 0.5
    detection_max_age_s: float = 0.5
    arrive_check_m: float = 0.05
    parking_arrive_m: float = 0.03
    request_max_age_s: float = 0.3
    safety_max_age_s: float = 0.3
    banner_hold_s: float = 4.0
    challenge_exit_dwell_s: float = 1.0
    reacquire_error_m: float = 0.30
    announce_banners: bool = True

    @classmethod
    def from_params(cls, p) -> 'MissionCfg':
        return cls(rate_hz=float(p('rate_hz')), tunnel_requires_zone=bool(p('tunnel_requires_route_zone')),
                   tunnel_zone_margin=float(p('tunnel_zone_margin_m')),
                   tunnel_exit_dwell_s=float(p('tunnel_exit_dwell_s')),
                   tunnel_trigger_max_age_s=float(p('tunnel_trigger_max_age_s')),
                   detection_max_age_s=float(p('detection_max_age_s')),
                   arrive_check_m=float(p('arrive_check_m')), parking_arrive_m=float(p('parking_arrive_m')),
                   request_max_age_s=float(p('request_max_age_s')), safety_max_age_s=float(p('safety_max_age_s')),
                   banner_hold_s=float(p('banner_hold_s')),
                   challenge_exit_dwell_s=float(p('challenge_exit_dwell_s')),
                   reacquire_error_m=float(p('reacquire_error_m')),
                   announce_banners=bool(p('announce_banners')))


@dataclass
class Inputs:
    t: float
    armed: bool = False
    estop: bool = False
    pose: Optional[Tuple[float, float, float]] = None        # local (block 05)
    gpose: Optional[Tuple[float, float, float]] = None       # coarse (block 06)
    light: str = 'UNKNOWN'                                   # detector (debounced)
    light_t: float = -1e9
    gate: str = 'UNKNOWN'
    gate_conf: float = 0.0
    gate_t: float = -1e9
    bump_sign_t: float = -1e9
    tunnel: bool = False
    tunnel_t: float = -1e9
    corridor_observed: int = 0
    corridor_t: float = -1e9
    camera_t: float = -1e9
    safety_ok: bool = True
    safety_reason: str = ''
    safety_t: float = -1e9
    road_arrived: bool = False
    road_t: float = -1e9
    parking_arrived: bool = False
    parking_t: float = -1e9
    parking_end: Optional[Tuple[float, float]] = None
    recovery_t: float = -1e9
    recovery_arrived: bool = False


@dataclass
class Output:
    mode: str = IDLE
    source: str = 'HOLD'
    challenge_id: int = 0
    challenge_name: str = ''
    leg: int = 0
    piece: int = -1
    hold_reason: str = ''
    zone: str = ''
    zone_max: float = 0.0
    set_max: float = 0.0
    next_exit: str = ''
    banner: str = ''
    banner_level: int = 0
    progress: int = 0
    events: List[Tuple[str, str, int]] = field(default_factory=list)       # (name, detail, challenge)
    mismatches: List[Dict] = field(default_factory=list)
    path_changed: bool = False


# --------------------------------------------------------------------------- challenge zones
class ChallengeTracker:
    """Which rubric challenge the car is in (challenges.yaml zones, visit counting)."""

    def __init__(self, challenges: List[Dict], course, dwell_s: float):
        self.ch = [c for c in challenges if (c.get('zone') or {}).get('type') != 'whole_run']
        self.course = course
        self.dwell = dwell_s
        self.count: Dict[str, int] = {}
        self.inside: Dict[str, bool] = {}
        self.left_at: Dict[str, float] = {}
        self.announced = set()

    @staticmethod
    def key(z: Dict) -> str:
        return repr(sorted((k, v) for k, v in z.items() if k != 'visit'))

    def _in(self, z: Dict, x: float, y: float) -> bool:
        t, c = z['type'], self.course
        if t == 'track_area':
            return z['area'] in c.area_polys and c.in_area(z['area'], x, y)
        if t == 'circle':
            return math.hypot(x - float(z['x']), y - float(z['y'])) < float(z['radius_m'])
        if t == 'roundabout':
            return math.hypot(x - c.ring[0], y - c.ring[1]) < float(z['radius_m'])
        if t == 'rect':
            return float(z['x0']) < x < float(z['x1']) and float(z['y0']) < y < float(z['y1'])
        if t == 'track_feature':
            if z['feature'] == 'tunnel':
                return c.in_tunnel(x, y)
            if z['feature'] == 'elevation':
                return c.in_elevation(x, y)
            return False
        if t == 'near_feature':
            parts = str(z['feature']).split('.')
            try:
                px, py = c.point(parts[0], parts[1] if len(parts) > 1 else None)
            except (KeyError, TypeError):
                return False
            return math.hypot(x - px, y - py) < float(z['radius_m'])
        if t == 'near_pose':
            if not c.has_feature(z['pose']):
                return False
            px, py, _ = c.pose(z['pose'])
            return math.hypot(x - px, y - py) < float(z['radius_m'])
        return False

    def update(self, x: float, y: float, t: float) -> Tuple[int, str, List[Tuple[int, str]]]:
        entered = []
        seen = set()
        for ch in self.ch:
            z = ch['zone']
            k = self.key(z)
            if k in seen:
                continue
            seen.add(k)
            now = self._in(z, x, y)
            was = self.inside.get(k, False)
            if now and not was:
                if t - self.left_at.get(k, -1e9) > self.dwell or k not in self.count:
                    self.count[k] = self.count.get(k, 0) + 1
                    entered.append(k)
            if was and not now:
                self.left_at[k] = t
            self.inside[k] = now
        cur, name, started = 0, '', []
        for ch in self.ch:
            z = ch['zone']
            k = self.key(z)
            if not self.inside.get(k):
                continue
            if 'visit' in z and self.count.get(k, 0) != int(z['visit']):
                continue
            if cur == 0:
                cur, name = int(ch['id']), str(ch['name'])
            if k in entered and int(ch['id']) not in self.announced:
                self.announced.add(int(ch['id']))
                started.append((int(ch['id']), str(ch['name'])))
        return cur, name, started


# --------------------------------------------------------------------------- the machine
class MissionMachine:

    def __init__(self, course, pieces: List[RoutePiece], visits: List[Visit], rules: Dict,
                 challenges: List[Dict], cfg: MissionCfg, max_speed: float):
        self.course = course
        self.pieces = pieces
        self.visits = visits
        self.rules = rules
        self.cfg = cfg
        self.max_speed = max_speed
        self.tracker = ChallengeTracker(challenges, course, cfg.challenge_exit_dwell_s)
        self.started = False
        self.complete = False
        self.i = 0                          # active piece
        self.piece_t = 0.0                  # when the active piece became active
        self.hold_until = -1.0
        self.hold_label = ''
        self.progress = 0
        self.light_commit = False
        self.gate_commit_piece = -1
        self.tunnel_on = False
        self.tunnel_off_since = None
        self.reported_visits = set()
        self.distance = 0.0
        self.last_pose = None
        self.bump_until = -1.0
        self.banner = ('', 0, -1e9)
        self.estop_reported = False
        self.recovering = False
        self.last_state = ''
        tl = rules['traffic_light']
        self.stop_distance = float(tl['stop_distance_m'])
        self.light_expiry = float(tl['observation_expiry_s'])
        light_legs = [p for p in pieces if p.end_behaviour == 'traffic_light_stop']
        self.light_piece = light_legs[0].index if light_legs else -1
        self.light_goal = tuple(light_legs[0].points[-1, :3]) if light_legs else None
        g4 = rules['challenge4_gate']
        self.g4 = g4
        self.gate4 = self._feature_point('boom_gates', str(g4['gate']))
        grc = rules['gate_route_check']
        self.grc = grc
        self.gate_rt = self._feature_point('boom_gates', str(grc['gate']))
        self.label_dir = {str(v['exit']): str(v.get('direction', v['exit']))
                          for v in rules.get('roundabout_visits', [])}
        self.gate4_near = [self._near_piece(p, self.gate4, float(g4['near_route_m'])) for p in pieces]
        # the roundabout gate belongs to the visit whose exit point is nearest to it
        self.gate_visit = None
        if self.gate_rt is not None and visits:
            def exit_dist(v):
                q = pieces[v.piece].points[v.leave]
                return math.hypot(q[0] - self.gate_rt[0], q[1] - self.gate_rt[1])
            self.gate_visit = min(visits, key=exit_dist)
        tr = rules['transitions']
        self.t_mission = float(tr['mission_hold_s'])
        self.t_parking = float(tr['parking_start_hold_s'])
        self.t_gear = float(tr['gear_change_hold_s'])

    # ------------------------------------------------------------------ helpers
    def _feature_point(self, key: str, sub: str):
        try:
            return self.course.point(key, sub)
        except (KeyError, TypeError):
            return None

    @staticmethod
    def _near_piece(p: RoutePiece, pt, dist: float) -> bool:
        if pt is None or not len(p.points):
            return False
        return float(np.min(np.hypot(p.points[:, 0] - pt[0], p.points[:, 1] - pt[1]))) < dist

    def direction_of(self, label: str) -> str:
        return self.label_dir.get(label, label)

    def piece(self) -> Optional[RoutePiece]:
        return self.pieces[self.i] if 0 <= self.i < len(self.pieces) else None

    def _hold(self, t: float, s: float, label: str) -> None:
        self.hold_until = max(self.hold_until, t + s)
        self.hold_label = label

    def _banner(self, t: float, text: str, level: int) -> None:
        if self.cfg.announce_banners:
            self.banner = (text, level, t)

    def upcoming_visit(self) -> Optional[Visit]:
        for v in self.visits:
            if v.piece > self.i or (v.piece == self.i and v.leave >= self.progress):
                return v
        return None

    # ------------------------------------------------------------------ zones
    def speed_zone(self, inp: Inputs, parking: bool) -> Tuple[str, float]:
        x, y, _ = inp.pose
        best, cap = '', self.max_speed
        p = self.piece()
        sec = p.sections[min(self.progress, len(p.sections) - 1)] if p is not None and p.sections else ''
        for z in self.rules['speed_zones']:
            r = z['region']
            t = r['type']
            on = False
            if t == 'track_feature':
                on = self.course.in_tunnel(x, y) if r['feature'] == 'tunnel' else \
                    self.course.in_elevation(x, y) if r['feature'] == 'elevation' else False
            elif t == 'roundabout':
                on = not parking and math.hypot(x - self.course.ring[0], y - self.course.ring[1]) < float(r['radius_m'])
            elif t == 'circle':
                on = not parking and math.hypot(x - float(r['x']), y - float(r['y'])) < float(r['radius_m'])
            elif t == 'rect':
                on = float(r['x0']) < x < float(r['x1']) and float(r['y0']) < y < float(r['y1'])
            elif t == 'section':
                on = sec in (r.get('sections') or [])
            elif t == 'detection':
                if r['detection'] == 'speed_bump_sign':
                    if inp.t - inp.bump_sign_t < self.cfg.detection_max_age_s:
                        self.bump_until = self.distance + float(r['hold_m'])
                    on = self.distance < self.bump_until
            elif t == 'condition':
                if r['condition'] == 'corridor_observed_lt_3':
                    obs = inp.corridor_observed if inp.t - inp.corridor_t < 1.0 else 0
                    on = not parking and obs < 3
                elif r['condition'] == 'camera_age_gt_0p25':
                    on = inp.t - inp.camera_t > 0.25
            if on and float(z['max_speed_mps']) < cap:
                best, cap = str(z['name']), float(z['max_speed_mps'])
        return best, cap

    # ------------------------------------------------------------------ step
    def step(self, inp: Inputs) -> Output:
        out = Output(set_max=self.max_speed)
        t = inp.t
        if inp.pose is not None:
            if self.last_pose is not None:
                self.distance += math.hypot(inp.pose[0] - self.last_pose[0], inp.pose[1] - self.last_pose[1])
            self.last_pose = inp.pose
        if not self.pieces:
            out.mode, out.source = IDLE, 'HOLD'
            out.banner, out.banner_level = 'No valid route: see Global map tab', 2
            return out
        if not self.started:
            if inp.armed and inp.pose is not None:
                self.started = True
                self.i = 0
                self.piece_t = t
                out.path_changed = True
                out.events.append(('RUN STARTED', 'Armed: fully autonomous from here. No further input.', 0))
                self._banner(t, 'RUN STARTED', 0)
            else:
                out.mode, out.source = IDLE, 'HOLD'
                out.hold_reason = 'Waiting for START' if inp.pose is not None else 'Waiting for local pose'
                self._fill_banner(out, t)
                return out
        if self.complete:
            out.mode, out.source, out.piece = COMPLETE, 'HOLD', self.i
            out.leg = self.pieces[-1].leg
            self._fill_banner(out, t)
            return out
        if inp.pose is None:
            out.mode, out.source, out.hold_reason = HOLD, 'HOLD', 'Local pose lost'
            return out

        p = self.piece()
        x, y, a = inp.pose
        # route progress on the active piece
        if len(p.points):
            k, err = closest(p.points, x, y, self.progress)
            if err > self.cfg.reacquire_error_m:
                d = np.hypot(p.points[:, 0] - x, p.points[:, 1] - y)
                k = int(np.argmin(d))
            self.progress = k
        parking = p.kind == 'manoeuvre'

        # ---------------------------------------------------- piece completion
        if t >= self.hold_until:
            done = False
            if p.kind == 'road' and inp.road_arrived and t - inp.road_t < self.cfg.request_max_age_s \
                    and inp.road_t > self.piece_t and len(p.points):
                done = math.hypot(x - p.points[-1, 0], y - p.points[-1, 1]) < self.cfg.arrive_check_m
            elif parking and inp.parking_arrived and t - inp.parking_t < self.cfg.request_max_age_s \
                    and inp.parking_t > self.piece_t and inp.parking_end is not None:
                done = math.hypot(x - inp.parking_end[0], y - inp.parking_end[1]) < self.cfg.parking_arrive_m
            if done:
                self._advance(t, out)
                if self.complete:
                    out.mode, out.source, out.piece = COMPLETE, 'HOLD', self.i
                    self._fill_banner(out, t)
                    return out
                p = self.piece()
                parking = p.kind == 'manoeuvre'
                self.progress = 0

        out.piece, out.leg, out.progress = self.i, p.leg, self.progress
        behaviour = PARKING if parking else ROAD

        # ---------------------------------------------------- tunnel (base LiDAR trigger + zone)
        if not parking:
            trig = inp.tunnel and t - inp.tunnel_t < self.cfg.tunnel_trigger_max_age_s
            zone = self.course.in_tunnel(x, y, self.cfg.tunnel_zone_margin)
            want = trig and (zone or not self.cfg.tunnel_requires_zone)
            if want:
                if not self.tunnel_on:
                    out.events.append(('TUNNEL', 'LiDAR tunnel trigger inside the tunnel zone: base '
                                       'tunnel follower drives', 3))
                self.tunnel_on, self.tunnel_off_since = True, None
            elif self.tunnel_on:
                if self.tunnel_off_since is None:
                    self.tunnel_off_since = t
                if t - self.tunnel_off_since > self.cfg.tunnel_exit_dwell_s or not zone:
                    self.tunnel_on = False
                    out.events.append(('TUNNEL EXIT', 'Back to lane following', 3))
            if self.tunnel_on:
                behaviour = TUNNEL
        else:
            self.tunnel_on = False

        # ---------------------------------------------------- recovery (block 12 asks)
        rec = t - inp.recovery_t < self.cfg.request_max_age_s and not inp.recovery_arrived
        if rec and not parking and behaviour == ROAD:
            if not self.recovering:
                out.events.append(('RECOVERY', 'Recovery planner took over: brake, reverse, rejoin', 0))
            self.recovering = True
            behaviour = RECOVERY
        elif self.recovering:
            self.recovering = False
            out.events.append(('RECOVERY DONE', 'Lane planner has control again', 0))

        # ---------------------------------------------------- holds
        hold = ''
        if t < self.hold_until:
            hold = self.hold_label or 'GEAR / MISSION HOLD'
        # traffic light: after the light leg, near its end, until OBSERVED green
        if self.light_goal is not None and self.i > self.light_piece and not self.light_commit:
            if math.hypot(x - self.light_goal[0], y - self.light_goal[1]) < self.stop_distance:
                if inp.light == 'GREEN' and t - inp.light_t < self.light_expiry:
                    self.light_commit = True
                    out.events.append(('LIGHT GREEN', 'Observed GREEN: proceeding', 7))
                elif not hold:
                    hold = 'TRAFFIC HOLD'
        # challenge 4 boom gate
        if self.gate4 is not None and self.gate4_near[self.i] and not parking and self.gate_commit_piece != self.i:
            gx, gy = self.gate4
            dist = math.hypot(x - gx, y - gy)
            ra = float(p.points[min(self.progress, len(p.points) - 1), 2]) if len(p.points) else a
            along = (gx - x) * math.cos(ra) + (gy - y) * math.sin(ra)       # > 0: gate ahead
            fresh = t - inp.gate_t < float(self.g4['observation_expiry_s'])
            is_open = fresh and inp.gate == 'OPEN'
            if dist < float(self.g4['commit_distance_m']) and along > 0 and is_open:
                self.gate_commit_piece = self.i
                out.events.append(('GATE OPEN', 'Observed OPEN boom gate: proceeding', 4))
            elif dist < float(self.g4['hold_distance_m']) and along > -float(self.g4['hold_x_margin_m']) \
                    and not hold:
                hold = 'GATE HOLD'
        if hold:
            if hold != self.last_state:
                detail = {'TRAFFIC HOLD': 'Waiting for observed GREEN',
                          'GATE HOLD': 'Gate closed or observation unavailable'}.get(hold, 'Vehicle stopped before transition')
                out.events.append((hold, detail, 7 if hold == 'TRAFFIC HOLD' else 4 if hold == 'GATE HOLD' else 0))
            mode, source = HOLD, 'HOLD'
        else:
            mode, source = behaviour, behaviour
        self.last_state = hold

        # ---------------------------------------------------- gate vs planned exit (log + warn only)
        self._gate_route_check(inp, out)

        # ---------------------------------------------------- safety / e-stop (display + owner veto)
        if inp.estop:
            if not self.estop_reported:
                self.estop_reported = True
                out.events.append(('MANUAL INTERVENTION', 'E-stop pressed: counts as manual intervention '
                                   '= 0 marks for the current challenge', 0))
                self._banner(t, 'E-STOP = manual intervention (0 marks)', 2)
            mode, out.hold_reason = SAFETY_STOP, 'E-stop'
        elif (not inp.safety_ok) and t - inp.safety_t < self.cfg.safety_max_age_s:
            mode, out.hold_reason = SAFETY_STOP, inp.safety_reason
        else:
            out.hold_reason = hold

        # ---------------------------------------------------- zones, challenge, exits
        out.zone, out.zone_max = self.speed_zone(inp, parking)
        cpose = inp.gpose or inp.pose
        cid, cname, started = self.tracker.update(cpose[0], cpose[1], t)
        for sid, sname in started:
            out.events.append(('CHALLENGE', f'Entered challenge {sid}: {sname}', sid))
        out.challenge_id, out.challenge_name = cid, cname
        v = self.upcoming_visit()
        out.next_exit = f'visit {v.visit}: {v.direction}' if v else ''
        out.mode, out.source = mode, source
        if mode == HOLD and hold == 'TRAFFIC HOLD':
            self._banner(t, 'TRAFFIC HOLD: waiting for observed GREEN', 1)
        elif mode == HOLD and hold == 'GATE HOLD':
            self._banner(t, 'GATE HOLD: boom gate closed or not seen', 1)
        self._fill_banner(out, t)
        return out

    def _fill_banner(self, out: Output, t: float) -> None:
        text, level, since = self.banner
        if text and t - since < self.cfg.banner_hold_s:
            out.banner, out.banner_level = text, level

    def _advance(self, t: float, out: Output) -> None:
        p = self.pieces[self.i]
        if self.i + 1 >= len(self.pieces):
            self.complete = True
            out.events.append(('MISSION COMPLETE', f'{p.leg_id} finished: all pieces done', 0))
            self._banner(t, 'MISSION COMPLETE', 0)
            return
        nxt = self.pieces[self.i + 1]
        if nxt.leg != p.leg:
            if p.end_behaviour == 'traffic_light_stop':
                out.events.append((f'{p.leg_id.upper()} COMPLETE', 'Stopped at the traffic-light approach. '
                                   'The next leg starts from this pose after an observed GREEN.', 7))
            else:
                out.events.append((f'{p.leg_id.upper()} COMPLETE', f'next: {nxt.leg_id}', 0))
            self._hold(t, self.t_mission, 'GEAR / MISSION HOLD')
        if nxt.kind == 'manoeuvre' and p.kind == 'road':
            out.events.append(('PARKING HANDOFF', f'Road route ends; block 11 plans into {nxt.bay}', 10))
            self._hold(t, self.t_parking, 'GEAR / MISSION HOLD')
        elif p.kind == 'manoeuvre':
            out.events.append(('MANOEUVRE DONE', f'{p.bay}: next {nxt.kind}', 0))
            self._hold(t, self.t_gear, 'GEAR / MISSION HOLD')
        self.i += 1
        self.piece_t = t
        out.path_changed = True

    def _gate_route_check(self, inp: Inputs, out: Output) -> None:
        if self.gate_rt is None:
            return
        x, y, _ = inp.pose
        if math.hypot(x - self.gate_rt[0], y - self.gate_rt[1]) > float(self.grc['observe_within_m']):
            return
        if inp.gate not in ('OPEN', 'CLOSED') or inp.t - inp.gate_t > self.cfg.detection_max_age_s:
            return
        if inp.gate_conf < float(self.grc['min_confidence']):
            return
        v = self.gate_visit
        if v is None or v.visit in self.reported_visits or v.piece != self.i:
            return
        self.reported_visits.add(v.visit)
        exp_label = str(self.grc['expected_exit_when_open' if inp.gate == 'OPEN' else 'expected_exit_when_closed'])
        expected = self.direction_of(exp_label)
        if expected == v.direction:
            out.events.append(('GATE AGREES', f'Gate {inp.gate} matches planned exit {v.direction} '
                               f'(visit {v.visit})', 0))
            return
        out.mismatches.append({'visit': v.visit, 'planned': v.label, 'gate': inp.gate,
                               'expected': exp_label, 'confidence': inp.gate_conf,
                               'note': 'Route NOT changed (global planner owns the exits)'})
        out.events.append(('GATE / ROUTE MISMATCH', f'Gate {inp.gate} suggests {expected}, planned exit is '
                           f'{v.direction} (visit {v.visit}); route unchanged', 0))
        self._banner(inp.t, f'Gate {inp.gate} disagrees with planned exit {v.direction}: route unchanged', 1)
