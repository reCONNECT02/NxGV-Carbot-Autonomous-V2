"""BLOCK 11 - Parking planner: compute the manoeuvre (V4 core.js parkingPlan,
reeds-shepp.js, app.js bayObservation / transition). See parking_core.py.

When block 08 makes a MANOEUVRE piece active (/carbot/mission/active_path =
that piece's mission_planner preview), this node:
  1. clears /carbot/parking/path at once (the tracker must never follow the
     previous piece's path);
  2. parking IN (goal inside the bay): observes the bay tape in local memory
     (start / end / far edges) and shifts the goal with it (map bay after
     bay_observation.fallback_after_s);
  3. plans from the ACTUAL estimated pose once the car has stopped (never a
     replay of the preview), un-parking included;
  4. publishes the plan ONE GEAR SECTION AT A TIME on /carbot/parking/path; the
     tracker (block 13) follows it and reports `arrived`; at each cusp it
     replans if the car stopped > 2.5 cm off, holds 0.4 s, publishes the next
     section; at the end it corrects a terminal heading error > 0.045 rad;
  5. publishes DONE on /carbot/parking/state -> block 08 completes the piece.

In : local pose, /odom, memory grid, road grid (EvidenceInputs), mission state,
     active path, global route + route info (bay of each piece), parking request
Out: /carbot/parking/path (latched), /carbot/parking/candidates,
     /carbot/parking/bay (observation JSON), /carbot/parking/state (latched JSON),
     /carbot/mission/events
"""
import json

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.geometry import geometry
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import Candidate, CandidateArray, MissionEvent, MissionState, MotionRequest, NodeStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from .parking_core import ParkingCfg, ParkingSession
from .ros_util import EvidenceInputs, evidence_cfg, memory_paint_xy, path_from_msg, path_key, path_to_msg
from .route_core import PlanCfg

REQUIRED = ['rate_hz', 'docking_tails_m', 'docking_step_m', 'handoff_extensions_m', 'clearance_pad_m',
            'rs_step_m', 'fallback_enabled', 'fallback_max_nodes', 'fallback_margin_m', 'fallback_time_budget_s',
            'fallback_bounds', 'bay_observation.edge_tolerance_m', 'bay_observation.min_samples',
            'bay_observation.max_age_s', 'bay_observation.max_shift_m', 'bay_observation.fallback_after_s',
            'cusp_replan_distance_m', 'max_cusp_replans', 'heading_tolerance_rad', 'max_corrections',
            'parked_heading_tolerance_rad', 'parked_margin_m', 'gear_hold_s', 'stopped_speed_mps',
            'section_arrive_m', 'arrived_after_publish_s', 'replan_retry_s', 'reverse_time_unpark',
            'request_max_age_s', 'pose_max_age_s', 'audit_point_stride', 'live_max_age_s',
            'grid_pose_max_gap_s', 'memory_evidence.uncertainty_base_m', 'memory_evidence.uncertainty_per_m',
            'memory_evidence.uncertainty_limit_m', 'data.track_map', 'data.track_features', 'data.mission',
            'frames.track', 'vehicle.wheelbase_m']


class ParkingPlanner(CarbotNode):

    def __init__(self):
        super().__init__('parking_planner', '11', REQUIRED)
        self.cfg = ParkingCfg.from_params(self.p)
        self.g = geometry(self.params_under('vehicle'))
        self.course = course_from_params(self.p)
        self.frame = str(self.p('frames.track'))
        self.session = ParkingSession(self.cfg, self.course, self.g, PlanCfg())
        self.inputs = EvidenceInputs(self, evidence_cfg(self.p), float(self.p('grid_pose_max_gap_s')))
        self.mission = None
        self.route, self.info = None, None
        self.active_key = None
        self.pending = None
        self.arrived_t = -1e9
        self.pub_path = self.create_publisher(Path, T.PARKING_PATH, LATCHED)
        self.pub_cand = self.create_publisher(CandidateArray, T.PARKING_CANDIDATES, 1)
        self.pub_bay = self.create_publisher(String, T.PARKING_BAY_JSON, 10)
        self.pub_state = self.create_publisher(String, T.PARKING_STATE, LATCHED)
        self.pub_event = self.create_publisher(MissionEvent, T.MISSION_EVENTS, 10)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mission', m), LATCHED)
        self.sub(Path, T.ACTIVE_PATH, self._on_active, LATCHED)
        self.sub(Path, T.GLOBAL_ROUTE, lambda m: setattr(self, 'route', path_from_msg(m)), LATCHED)
        self.sub(String, T.ROUTE_INFO_JSON, self._on_info, LATCHED)
        self.sub(MotionRequest, T.request_topic('PARKING'), self._on_request, 10)
        self.group = MutuallyExclusiveCallbackGroup()     # planning may take a while: own thread
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick, callback_group=self.group)
        self.create_timer(1.0, self._publish_state)
        self.set_status(NodeStatus.OK, 'IDLE', 'waiting for a manoeuvre piece')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, m: String):
        try:
            self.info = json.loads(m.data)
        except ValueError:
            self.info = None

    def _on_request(self, m: MotionRequest):
        if m.arrived:
            self.arrived_t = self.now()

    def _bay_of(self, pts: np.ndarray) -> str:
        """Which manoeuvre piece of the route this active path is (-> its bay)."""
        if self.info is None or self.route is None or not len(pts):
            return ''
        for p in self.info.get('pieces', []):
            if p.get('kind') != 'manoeuvre' or p.get('start', -1) < 0:
                continue
            q = self.route[p['start']:p['end'] + 1]
            if len(q) and np.hypot(*(q[-1, :2] - pts[-1, :2])) < 1e-3 and np.hypot(*(q[0, :2] - pts[0, :2])) < 1e-3:
                return str(p.get('bay', ''))
        return ''

    def _on_active(self, m: Path):
        key = path_key(m)
        if key == self.active_key:
            return
        self.active_key = key
        self.pending = (key, path_from_msg(m))     # handled in the planning thread (_tick)

    def _start_pending(self) -> None:
        key, pts = self.pending
        self.pending = None
        is_manoeuvre = len(pts) > 1 and (pts[:, 3] < 0).any()
        if not is_manoeuvre:
            if self.session.state != 'IDLE':
                self.session.state = 'IDLE'
                self.pub_path.publish(path_to_msg(np.zeros((0, 4)), self.frame, self.get_clock().now().to_msg()))
                self._publish_state()
            return
        bay = self._bay_of(pts)
        out = self.session.start(key, tuple(pts[-1, :3]), bay, self.now())
        self._emit(out)
        self.get_logger().info(f'manoeuvre piece active: bay "{bay}" docking={self.session.docking} '
                               f'goal {tuple(round(v, 3) for v in pts[-1, :3])}')

    def _tick(self) -> None:
        now = self.now()
        if self.pending is not None:
            self._start_pending()
        if self.session.state in ('IDLE', 'DONE'):
            return
        t, pose = self.inputs.pose()
        if pose is None or t is None or now - t > float(self.p('pose_max_age_s')):
            self.set_status(NodeStatus.WARN, self.session.state, 'local pose stale')
            return

        def paint():
            return memory_paint_xy(self.inputs.mem, self.inputs.pair, now, self.cfg.observation_max_age_s)

        out = self.session.update(now, pose, self.inputs.speed, self.arrived_t, paint)
        self._emit(out)
        lvl = NodeStatus.WARN if 'HOLD' in self.session.reason else NodeStatus.OK
        self.set_status(lvl, self.session.state, self.session.reason)

    def _emit(self, out) -> None:
        stamp = self.get_clock().now().to_msg()
        if out.publish is not None:
            self.pub_path.publish(path_to_msg(out.publish, self.frame, stamp))
        if out.audit is not None:
            self._publish_audit(out.audit, stamp)
        for name, detail in out.events:
            e = MissionEvent()
            e.header.stamp = stamp
            e.name, e.detail = name, detail
            e.challenge_id = 11 if self.session.bay and 'perp' in self.session.bay.name else 10
            self.pub_event.publish(e)
            self.get_logger().info(f'[{name}] {detail}')
        if out.publish is not None or out.events:
            self._publish_state()

    def _publish_audit(self, r, stamp) -> None:
        ca = CandidateArray()
        ca.header.stamp, ca.header.frame_id = stamp, self.frame
        ca.planner = 'parking'
        ca.selected_id = -1
        k = max(1, int(self.p('audit_point_stride')))
        for a in r.evaluated:
            c = Candidate()
            c.id = int(a.id)
            c.points = [Point(x=float(x), y=float(y), z=float(d)) for x, y, _, d in a.points[::k]]
            c.cost, c.valid, c.selected = float(a.cost), bool(a.valid), bool(a.selected)
            c.stage, c.reject = f'{a.stage} {a.types}'.strip(), a.reject
            if a.selected:
                ca.selected_id = int(a.id)
            ca.candidates.append(c)
        self.pub_cand.publish(ca)

    def _publish_state(self) -> None:
        d = self.session.state_dict()
        self.pub_state.publish(String(data=json.dumps(d)))
        if d.get('observation') is not None:
            self.pub_bay.publish(String(data=json.dumps(dict(d['observation'], bay=d['bay'], goal=d['goal'],
                                                             prior_goal=d['prior_goal']))))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPlanner()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
