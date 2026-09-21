"""BLOCK 08 - Mission logic: choose what happens now (V4 app.js step, transition).

Walks the route pieces of block 07 (road -> ROAD, manoeuvre -> PARKING) and
decides, at rate_hz, the mode and the request source the command owner must
follow (MissionState.active_source). See mission_core.py for the rules:
observed-green traffic light (never a timer), Challenge 4 gate hold on the
detector, gate-vs-planned-exit check (log + banner only, route unchanged),
tunnel = base LiDAR trigger (/tunnel_detected) + route zone, speed zones,
transition holds, e-stop = manual intervention (0 marks).

Starts when /carbot/race/armed is true (race_supervisor, phase 8). Once
started, mode is never IDLE again, so the localization reset stays locked.

In : local/global pose, detections, /tunnel_detected, corridor, safety status,
     /carbot/race/armed, /e_stop, global route + route info, the road /
     parking / recovery requests (arrived flags), parking path, road grid (age),
     /carbot/parking/state (phase 5: block 11 DONE completes a manoeuvre piece)
Out: /carbot/mission/state (latched), /carbot/mission/events,
     /carbot/mission/gate_route_mismatch, /carbot/mission/active_path (latched:
     the active piece; for a manoeuvre, the mission_planner preview or empty)
"""
import json

import numpy as np
import rclpy
import yaml
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.mission import mission_from_params
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import (Corridor, DetectionArray, GateRouteMismatch, LocalGrid, MissionEvent,
                                   MissionState, MotionRequest, NodeStatus, SafetyStatus)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

from .mission_core import Inputs, MissionCfg, MissionMachine, RoutePiece, Visit
from .ros_util import path_from_msg, path_to_msg, pose_of

REQUIRED = ['rate_hz', 'tunnel_requires_route_zone', 'tunnel_zone_margin_m', 'tunnel_exit_dwell_s',
            'tunnel_trigger_max_age_s', 'detection_max_age_s', 'arrive_check_m', 'parking_arrive_m',
            'request_max_age_s', 'safety_max_age_s', 'banner_hold_s', 'challenge_exit_dwell_s',
            'reacquire_error_m', 'announce_banners', 'pose_max_age_s', 'parking_requires_planner_done',
            'data.mission',
            'data.mission_rules', 'data.track_map', 'data.track_features', 'data.challenges',
            'frames.track', 'limits.max_speed_mps']


class MissionLogic(CarbotNode):

    def __init__(self):
        super().__init__('mission_logic', '08', REQUIRED)
        self.cfg = MissionCfg.from_params(self.p)
        self.frame = str(self.p('frames.track'))
        self.course = course_from_params(self.p)
        self.mission = mission_from_params(self.p)
        with open(str(self.p('data.challenges')), 'r', encoding='utf-8') as f:
            self.challenges = (yaml.safe_load(f) or {}).get('challenges', [])
        self.inp = Inputs(t=0.0)
        self.pose_t = -1e9
        self.info = None
        self.route = None
        self.machine = None
        self.active_piece = None
        self.parking_path = None
        self.pub_state = self.create_publisher(MissionState, T.MISSION_STATE, LATCHED)
        self.pub_event = self.create_publisher(MissionEvent, T.MISSION_EVENTS, 10)
        self.pub_mm = self.create_publisher(GateRouteMismatch, T.GATE_ROUTE_MISMATCH, 10)
        self.pub_path = self.create_publisher(Path, T.ACTIVE_PATH, LATCHED)
        self.sub(Odometry, T.LOCAL_POSE, self._on_local, 10)
        self.sub(PoseWithCovarianceStamped, T.GLOBAL_POSE, self._on_global, 10)
        self.sub(DetectionArray, T.DETECTIONS, self._on_det, 10)
        self.sub(Bool, T.TUNNEL_DETECTED, self._on_tunnel, 10)
        self.sub(Corridor, T.CORRIDOR, self._on_corridor, 10)
        self.sub(SafetyStatus, T.SAFETY_STATUS, self._on_safety, 10)
        self.sub(Bool, T.RACE_ARMED, self._on_armed, LATCHED)
        self.sub(Bool, T.E_STOP, self._on_estop, 10)
        self.sub(Path, T.GLOBAL_ROUTE, self._on_route, LATCHED)
        self.sub(String, T.ROUTE_INFO_JSON, self._on_info, LATCHED)
        self.sub(MotionRequest, T.request_topic('ROAD'), self._on_road, 10)
        self.sub(MotionRequest, T.request_topic('PARKING'), self._on_park, 10)
        self.sub(MotionRequest, T.request_topic('RECOVERY'), self._on_rec, 10)
        self.sub(Path, T.PARKING_PATH, self._on_parking_path, LATCHED)
        self.sub(String, T.PARKING_STATE, self._on_parking_state, LATCHED)
        self.parking_done_keys = set()
        self.sub(LocalGrid, T.ROAD_GRID, self._on_grid, 10)
        self.create_timer(1.0 / max(self.cfg.rate_hz, 1.0), self._tick)
        self.set_status(NodeStatus.WARN, 'WAITING_ROUTE', 'waiting for the global route')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_local(self, m):
        self.inp.pose, self.pose_t = pose_of(m), self.now()

    def _on_global(self, m):
        self.inp.gpose = pose_of(m)

    def _on_det(self, m: DetectionArray):
        t = self.now()
        self.inp.light, self.inp.light_t = m.traffic_light_state or 'UNKNOWN', t
        self.inp.gate, self.inp.gate_t = m.boom_gate_state or 'UNKNOWN', t
        conf = [d.confidence for d in m.detections if d.class_name.startswith('boom_gate')]
        self.inp.gate_conf = float(max(conf)) if conf else 0.0
        if m.speed_bump_sign:
            self.inp.bump_sign_t = t

    def _on_tunnel(self, m):
        self.inp.tunnel, self.inp.tunnel_t = bool(m.data), self.now()

    def _on_corridor(self, m: Corridor):
        self.inp.corridor_observed, self.inp.corridor_t = int(m.observed), self.now()

    def _on_safety(self, m: SafetyStatus):
        self.inp.safety_ok, self.inp.safety_reason, self.inp.safety_t = bool(m.motion_allowed), \
            f'{m.veto_check}: {m.veto_reason}' if m.veto_check else m.veto_reason, self.now()

    def _on_armed(self, m):
        self.inp.armed = bool(m.data)

    def _on_estop(self, m):
        self.inp.estop = bool(m.data)

    def _on_road(self, m: MotionRequest):
        self.inp.road_arrived, self.inp.road_t = bool(m.arrived), self.now()

    def _on_park(self, m: MotionRequest):
        self.inp.parking_arrived, self.inp.parking_t = bool(m.arrived), self.now()

    def _on_rec(self, m: MotionRequest):
        self.inp.recovery_arrived, self.inp.recovery_t = bool(m.arrived), self.now()

    def _on_parking_path(self, m: Path):
        pts = path_from_msg(m)
        self.inp.parking_end = (float(pts[-1, 0]), float(pts[-1, 1])) if len(pts) else None

    def _on_parking_state(self, m: String):
        """Block 11 DONE, once per manoeuvre session (receive time -> Inputs)."""
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if d.get('done') and d.get('key') not in self.parking_done_keys:
            self.parking_done_keys.add(d.get('key'))
            self.inp.parking_done_t = self.now()

    def _on_grid(self, m):
        self.inp.camera_t = self.now()

    def _on_route(self, m: Path):
        self.route = path_from_msg(m)
        self._build()

    def _on_info(self, m: String):
        try:
            self.info = json.loads(m.data)
        except ValueError:
            self.info = None
        self._build()

    def _build(self) -> None:
        if self.info is None or self.route is None or self.machine is not None:
            return
        if not self.info.get('ok'):
            self.set_status(NodeStatus.ERROR, 'ROUTE_FAIL', self.info.get('reason', ''))
            self.machine = MissionMachine(self.course, [], [], self.mission.rules, self.challenges,
                                          self.cfg, float(self.p('limits.max_speed_mps')))
            return
        pieces = []
        for p in self.info['pieces']:
            pts = self.route[p['start']:p['end'] + 1] if p['start'] >= 0 else np.zeros((0, 4))
            labels = []
            for s in p['sections']:
                labels += [s['section']] * (s['end'] - s['start'] + 1)
            pieces.append(RoutePiece(p['index'], p['leg'], p['leg_id'], p['kind'], p['bay'], pts,
                                     p['end_behaviour'], labels))
        visits = [Visit(v['visit'], v['piece'], v['enter'], v['leave'], v['exit'], v['label'])
                  for v in self.info['visits']]
        self.machine = MissionMachine(self.course, pieces, visits, self.mission.rules, self.challenges,
                                      self.cfg, float(self.p('limits.max_speed_mps')))
        self.set_status(NodeStatus.OK, 'READY', f'{len(pieces)} pieces, waiting for START (armed)')

    # ------------------------------------------------------------------ cycle
    def _tick(self) -> None:
        t = self.now()
        if self.machine is None:
            m = MissionState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.mode, m.active_source, m.hold_reason = MissionState.MODE_IDLE, 'HOLD', 'Waiting for route'
            self.pub_state.publish(m)
            return
        self.inp.t = t
        pose_ok = t - self.pose_t < float(self.p('pose_max_age_s'))
        saved = self.inp.pose
        if not pose_ok:
            self.inp.pose = None
        out = self.machine.step(self.inp)
        self.inp.pose = saved
        stamp = self.get_clock().now().to_msg()
        if out.path_changed or (self.machine.started and self.active_piece != self.machine.i):
            self.active_piece = self.machine.i
            p = self.machine.piece()
            if p is not None:
                self.pub_path.publish(path_to_msg(p.points, self.frame, stamp))
        m = MissionState()
        m.header.stamp = stamp
        m.mode, m.active_source = out.mode, out.source
        m.challenge_id, m.challenge_name = int(out.challenge_id), out.challenge_name
        m.route_leg = int(out.leg)
        m.hold_reason, m.speed_zone = out.hold_reason, out.zone
        m.zone_max_speed_mps, m.set_max_speed_mps = float(out.zone_max), float(out.set_max)
        m.next_roundabout_exit = out.next_exit
        m.banner, m.banner_level = out.banner, int(out.banner_level)
        self.pub_state.publish(m)
        for name, detail, cid in out.events:
            e = MissionEvent()
            e.header.stamp = stamp
            e.name, e.detail, e.challenge_id = name, detail, int(cid)
            self.pub_event.publish(e)
            self.get_logger().info(f'[{name}] {detail}')
        for mm in out.mismatches:
            g = GateRouteMismatch()
            g.header.stamp = stamp
            g.roundabout_visit = int(mm['visit'])
            g.planned_exit, g.gate_state = str(mm['planned']), str(mm['gate'])
            g.expected_exit_for_gate_state = str(mm['expected'])
            g.gate_confidence, g.note = float(mm['confidence']), str(mm['note'])
            self.pub_mm.publish(g)
            self.get_logger().warn(f'gate/route mismatch: {mm}')
        if self.machine.started:
            self.set_status(NodeStatus.OK, out.mode, f'piece {out.piece} leg {out.leg} '
                            f'challenge {out.challenge_id} {out.hold_reason}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
