"""BLOCK 09 - Find the lane we should follow (V4 guidance.js corridor, branchCheck).

Combines the chosen route branch with the road actually seen: finds the
middle between paired road/paint edges (live camera, else remembered cells),
and applies a small, clamped common offset to the route guide. Branch check
holds when the intended opening lacks road evidence or the course location
conflicts (route identity); the safety monitor (block 14) turns that into a
hold and recovery (block 12) may resolve it. See corridor_core.py.

Frame: `track` (the local pose frame, as V4 builds the guide on estimate.pose).
Runs only on a road piece (not in PARKING / TUNNEL / IDLE).
Out: /carbot/plan/corridor  Corridor (guide Path z = direction, centres z = width,
     lane_locked = observed >= lane_locked_min_observed -> main-tab lane lines blue)
"""
import json

import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import Corridor, LocalizationStatus, MissionState, NodeStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import String

from .corridor_core import CorridorCfg, CorridorFinder, branch_check
from .ros_util import EvidenceInputs, evidence_cfg, path_from_msg, path_key, path_to_msg

REQUIRED = ['rate_hz', 'horizon_m', 'sample_every', 'edge_search_min_m', 'edge_search_max_m',
            'edge_search_step_m', 'inside_probe_m', 'edge_correction_m', 'lane_width_min_m',
            'lane_width_max_m', 'prior_search_max_m', 'prior_search_step_m', 'shift_clamp_m', 'shift_gain',
            'guide_points', 'live_max_age_s', 'evidence_max_age_s', 'camera_corridor_min_observed',
            'lane_locked_min_observed', 'branch.near_radius_m', 'branch.lane_change_margin_m',
            'branch.look_min_m', 'branch.look_max_m', 'branch.min_points', 'branch.min_seen_ratio',
            'branch.agree_min_m', 'branch.agree_sigma_mult', 'branch.visual_rank_min', 'pose_max_age_s',
            'grid_pose_max_gap_s', 'memory_evidence.uncertainty_base_m', 'memory_evidence.uncertainty_per_m',
            'memory_evidence.uncertainty_limit_m', 'data.track_map', 'data.track_features', 'data.mission',
            'frames.track']

ACTIVE_MODES = (MissionState.MODE_ROAD, MissionState.MODE_HOLD, MissionState.MODE_RECOVERY,
                MissionState.MODE_SAFETY_STOP)


class CorridorNode(CarbotNode):

    def __init__(self):
        super().__init__('corridor', '09', REQUIRED)
        self.cfg = CorridorCfg.from_params(self.p)
        self.course = course_from_params(self.p)
        self.frame = str(self.p('frames.track'))
        self.finder = CorridorFinder(self.cfg)
        self.inputs = EvidenceInputs(self, evidence_cfg(self.p), float(self.p('grid_pose_max_gap_s')))
        self.path = None
        self.key = None
        self.mode = ''
        self.exits = []
        self.loc = None
        self.pub = self.create_publisher(Corridor, T.CORRIDOR, 10)
        self.sub(Path, T.ACTIVE_PATH, self._on_path, LATCHED)
        self.sub(LocalizationStatus, T.LOCALIZATION_STATUS, lambda m: setattr(self, 'loc', m), 10)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mode', m.mode), LATCHED)
        self.sub(String, T.ROUTE_INFO_JSON, self._on_info, LATCHED)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 0.5), self._tick)
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for active path + local pose')

    def _on_path(self, m: Path) -> None:
        if path_key(m) != self.key:
            self.key = path_key(m)
            self.path = path_from_msg(m)
            self.finder.reset()
            if len(self.path) and (self.path[:, 3] < 0).any():
                self.path = None             # a manoeuvre preview: block 11's job, no lane corridor

    def _on_info(self, m: String) -> None:
        try:
            self.exits = sorted({v['exit'] for v in json.loads(m.data).get('visits', [])})
        except (ValueError, KeyError, TypeError):
            self.exits = []

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        t, pose = self.inputs.pose()
        if self.path is None or len(self.path) < 2 or pose is None or self.mode not in ACTIVE_MODES:
            return
        if now - t > float(self.p('pose_max_age_s')):
            self.set_status(NodeStatus.WARN, 'STALE_POSE', 'local pose stale')
            return
        ev = self.inputs.evidence(now)
        r = self.finder.corridor(self.path, pose, ev, self.course)
        if self.loc is not None:
            b = branch_check(pose, r.guide, ev, self.course, self.exits,
                             (self.loc.global_offset_x_m, self.loc.global_offset_y_m),
                             self.loc.global_sigma_m, self.loc.visual_rank, self.cfg)
        else:
            b = branch_check(pose, r.guide, ev, self.course, self.exits, cfg=self.cfg)
        m = Corridor()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.guide = path_to_msg(r.guide, self.frame, m.header.stamp)
        m.centres = [Point(x=float(x), y=float(y), z=float(w)) for x, y, _, w in r.centres]
        m.left_edge = [Point(x=float(x), y=float(y)) for x, y in r.left_edge]
        m.right_edge = [Point(x=float(x), y=float(y)) for x, y in r.right_edge]
        m.offset_m, m.observed, m.mode = float(r.offset), int(r.observed), r.mode
        m.lane_locked = r.observed >= self.cfg.lane_locked_min_observed
        m.branch_hold, m.branch_label, m.branch_reason = b.hold, b.label, b.reason
        self.pub.publish(m)
        self.set_status(NodeStatus.WARN if b.hold else NodeStatus.OK, r.mode,
                        f'observed {r.observed} offset {r.offset * 100:.1f} cm; {b.label}'
                        + (f' ({b.reason})' if b.hold else ''))


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
