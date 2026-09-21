"""BLOCK 12 - Recovery planner: make room, then rejoin (V4 recovery.js). See recovery_core.py.

Problem signal (V4 app.js step): block 10 found no feasible candidate or the
tracking error exceeds 10 cm (/carbot/plan/local_candidates selected_id == -1),
or the corridor holds for the recoverable branch reason. After 0.45 s, when
recovery is permitted (mission ROAD / RECOVERY, no TRAFFIC / GATE / GEAR hold),
this node searches reverse-then-forward Reeds-Shepp connections and drives
the winner at 3 cm/s.

It publishes /carbot/request/recovery ONLY while it is active (mission logic
switches to RECOVERY on a fresh request; the final request has arrived = true,
then it goes quiet). A safety veto or a mission hold pauses it (zero request):
it cannot bypass them. UWB is not an input.

In : local pose, /odom, road grid, memory (EvidenceInputs), active path, /scan
     (+ TF base_link -> laser_frame), safety status, local candidates, corridor,
     mission state
Out: /carbot/request/recovery, /carbot/recovery/candidates, /carbot/recovery/state,
     /carbot/mission/events
"""
import json

import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.geometry import geometry
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import (Candidate, CandidateArray, Corridor, MissionEvent, MissionState, MotionRequest,
                                   NodeStatus, SafetyStatus)
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .recovery_core import RecInputs, Recovery, RecoveryCfg
from .ros_util import EvidenceInputs, LaserMount, evidence_cfg, path_from_msg, path_key, scan_hits
from .tracker_core import TrackerCfg

REQUIRED = ['rate_hz', 'enabled', 'reverse_max_m', 'reverse_min_m', 'problem_dwell_s', 'max_attempts',
            'reset_attempts_after_m', 'rescan_period_s', 'align_s', 'max_goals', 'first_goal_m', 'goal_spacing_m',
            'goal_clearance_m', 'max_cost', 'cost_reverse_weight', 'cost_outside_weight', 'outside_step_m',
            'rear_evidence_min_ratio', 'rear_sample_every', 'rear_offset_m', 'rear_lateral_fraction',
            'evidence_max_age_s', 'lidar_max_age_s', 'camera_max_age_s', 'lookahead_points', 'stopped_speed_mps',
            'road_pad_m', 'road_step_m', 'obstacle_pad_m', 'rs_step_m', 'lidar_max_range_m', 'lidar_mount',
            'candidates_max_age_s', 'safety_max_age_s', 'pose_max_age_s', 'recoverable_branch_reason',
            'permitted_holds_block', 'audit_point_stride', 'live_max_age_s', 'grid_pose_max_gap_s',
            'memory_evidence.uncertainty_base_m', 'memory_evidence.uncertainty_per_m',
            'memory_evidence.uncertainty_limit_m', 'limits.recovery_speed_mps', 'limits.line_tolerance_m',
            'data.track_map', 'frames.track', 'frames.base', 'frames.laser', 'vehicle.lidar_x_m']


class RecoveryPlanner(CarbotNode):

    def __init__(self):
        super().__init__('recovery_planner', '12', REQUIRED)
        self.cfg = RecoveryCfg.from_params(self.p)
        self.g = geometry(self.params_under('vehicle'))
        self.course = course_from_params(self.p)
        self.frame = str(self.p('frames.track'))
        self.rec = Recovery(self.cfg, self.course, self.g, TrackerCfg())
        self.inputs = EvidenceInputs(self, evidence_cfg(self.p), float(self.p('grid_pose_max_gap_s')))
        self.laser = LaserMount(self, str(self.p('lidar_mount')) == 'tf', str(self.p('frames.base')),
                                str(self.p('frames.laser')), (float(self.p('vehicle.lidar_x_m')), 0.0, 0.0))
        self.mission = None
        self.route, self.route_key = None, None
        self.scan, self.scan_t = None, -1e9
        self.grid_t = -1e9
        self.safety, self.safety_t = None, -1e9
        self.cands_sel, self.cands_t = 0, -1e9
        self.corr = None
        self.pub_req = self.create_publisher(MotionRequest, T.request_topic('RECOVERY'), 10)
        self.pub_cand = self.create_publisher(CandidateArray, T.RECOVERY_CANDIDATES, 1)
        self.pub_state = self.create_publisher(String, T.RECOVERY_STATE, 10)
        self.pub_event = self.create_publisher(MissionEvent, T.MISSION_EVENTS, 10)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mission', m), LATCHED)
        self.sub(Path, T.ACTIVE_PATH, self._on_active, LATCHED)
        self.sub(LaserScan, T.SCAN, self._on_scan, SENSOR)
        self.sub(SafetyStatus, T.SAFETY_STATUS, self._on_safety, 10)
        self.sub(CandidateArray, T.LOCAL_CANDIDATES, self._on_cands, 1)
        self.sub(Corridor, T.CORRIDOR, lambda m: setattr(self, 'corr', m), 10)
        self.group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick, callback_group=self.group)
        self.create_timer(0.5, self._publish_state)
        self.set_status(NodeStatus.OK, 'IDLE', 'no problem')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_active(self, m: Path):
        if path_key(m) != self.route_key:
            self.route_key, self.route = path_key(m), path_from_msg(m)
            self.rec.route_index = 0

    def _on_scan(self, m):
        self.scan, self.scan_t = m, self.now()

    def _on_safety(self, m):
        self.safety, self.safety_t = m, self.now()

    def _on_cands(self, m: CandidateArray):
        self.cands_sel, self.cands_t = int(m.selected_id), self.now()

    def _inputs(self, now: float, pose) -> RecInputs:
        ms = self.mission
        mode = ms.mode if ms else ''
        hold = ms.hold_reason if ms else ''
        road_piece = self.route is not None and len(self.route) > 1 and not (self.route[:, 3] < 0).any()
        blocked = [s.strip() for s in str(self.p('permitted_holds_block')).split(',') if s.strip()]
        permitted = mode in (MissionState.MODE_ROAD, MissionState.MODE_RECOVERY) and road_piece and \
            not any(hold.startswith(b) for b in blocked)
        # problem (V4): no feasible candidate / tracking error > 10 cm / recoverable branch hold
        problem = ''
        if now - self.cands_t < float(self.p('candidates_max_age_s')) and self.cands_sel < 0:
            problem = 'No feasible local candidate or tracking error > 10 cm'
        if self.corr is not None and self.corr.branch_hold and \
                self.corr.branch_reason == str(self.p('recoverable_branch_reason')):
            problem = problem or self.corr.branch_reason
        # hard hold (V4: safety || (!permitted && active ? state : null))
        hard = ''
        if self.safety is None or now - self.safety_t > float(self.p('safety_max_age_s')):
            hard = 'safety status stale'
        elif not self.safety.motion_allowed:
            hard = f'{self.safety.veto_check}: {self.safety.veto_reason}'
        elif not permitted and self.rec.active:
            hard = hold or mode or 'not permitted'
        grid_age = now - self.inputs.grid.stamp if self.inputs.grid is not None else 1e9
        lidar_age = now - self.scan_t
        hits = None
        if lidar_age < self.cfg.lidar_max_age_s:
            hits = scan_hits(self.scan, pose, self.laser.get(), float(self.p('lidar_max_range_m')))
        return RecInputs(t=now, pose=pose, speed=self.inputs.speed, route=self.route if road_piece else None,
                         problem=problem, hard_hold=hard, permitted=permitted, lidar_age=lidar_age,
                         camera_age=grid_age, hits=hits, ev=self.inputs.evidence(now))

    def _tick(self) -> None:
        now = self.now()
        t, pose = self.inputs.pose()
        if pose is None or t is None or now - t > float(self.p('pose_max_age_s')):
            if self.rec.active:
                self._publish_req({'speed': 0.0, 'steer': 0.0, 'arrived': False, 'reason': 'local pose stale'})
            return
        out = self.rec.update(self._inputs(now, pose))
        stamp = self.get_clock().now().to_msg()
        if out.request is not None:
            self._publish_req(out.request)
        if out.evaluated is not None:
            self._publish_audit(out.evaluated, stamp)
        for name, detail in out.events:
            e = MissionEvent()
            e.header.stamp = stamp
            e.name, e.detail = name, detail
            self.pub_event.publish(e)
            self.get_logger().info(f'[{name}] {detail}')
        self.set_status(NodeStatus.WARN if self.rec.active else NodeStatus.OK, self.rec.state,
                        self.rec.reason if self.rec.active else f'completed {self.rec.completed}')

    def _publish_req(self, r) -> None:
        m = MotionRequest()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = 'RECOVERY'
        m.speed_mps, m.steer_rad = float(r['speed']), float(r['steer'])
        m.arrived, m.reason = bool(r['arrived']), str(r['reason'])
        self.pub_req.publish(m)

    def _publish_audit(self, evaluated, stamp) -> None:
        ca = CandidateArray()
        ca.header.stamp, ca.header.frame_id = stamp, self.frame
        ca.planner = 'recovery'
        ca.selected_id = -1
        k = max(1, int(self.p('audit_point_stride')))
        for q in evaluated:
            c = Candidate()
            c.id = int(q.id)
            c.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[3])) for p in q.points[::k]]
            c.cost, c.valid = float(q.cost), bool(q.valid)
            c.selected = self.rec.selected is not None and q is self.rec.selected
            c.stage = f'{q.types} reverse {q.reverse * 100:.0f} cm -> route idx {q.goal_index}'
            c.reject = q.reject
            if c.selected:
                ca.selected_id = int(q.id)
            ca.candidates.append(c)
        self.pub_cand.publish(ca)

    def _publish_state(self) -> None:
        r = self.rec
        self.pub_state.publish(String(data=json.dumps({
            'active': r.active, 'state': r.state, 'reason': r.reason, 'attempts': r.attempts,
            'completed': r.completed, 'enabled': self.cfg.enabled})))


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryPlanner()
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
