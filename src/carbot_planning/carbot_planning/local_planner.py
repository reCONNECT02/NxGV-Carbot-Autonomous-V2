"""BLOCK 10 - Local planner: choose the next movement (V4 local-planner.js).

At 5 Hz, rolls out nine lateral offsets along the corridor guide with a
bicycle model + servo lag, checks swept body clearance, road/paint evidence and
LiDAR obstacles, and picks the lowest-cost valid candidate (grey = tried,
blue = selected). If all strict candidates fail, retries nine more with the
paint allowance (limits.line_tolerance_m). See local_core.py.

Out:
  /carbot/plan/local_candidates  CandidateArray (every candidate, cost, reject
                                 reason for the GUI hover text; selected_id -1 = none)
  /carbot/plan/local_path        Path, track frame: the corridor guide shifted by the
                                 selected offset = the line block 13 steers to.
                                 EMPTY = no feasible candidate or tracking error above
                                 problem_tracking_error_m (V4 'problem': block 13 stops,
                                 block 12 may recover).
Runs only on a road piece in ROAD / HOLD / RECOVERY / SAFETY_STOP.
"""
import math

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.geometry import geometry
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import (Candidate, CandidateArray, CommandOwnerState, Corridor, MissionState,
                                   NodeStatus)
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from .corridor_core import closest
from .local_core import LocalCfg, LocalPlanner
from .ros_util import EvidenceInputs, evidence_cfg, path_from_msg, path_key, path_to_msg, stamp_s

REQUIRED = ['rate_hz', 'offsets_m', 'rollout_steps', 'rollout_ds_m', 'lookahead_m', 'rollout_speed_min_mps',
            'rollout_speed_max_mps', 'check_every', 'body_pad_m', 'cost.tracking', 'cost.unseen_memory',
            'cost.unseen', 'cost.clearance', 'cost.clearance_floor_m', 'cost.low_margin_m',
            'cost.low_margin_weight', 'cost.offset', 'cost.outside', 'cost.paint', 'cost.unknown',
            'paint_block_steps', 'paint_block_cells', 'relaxed_retry', 'problem_tracking_error_m',
            'end_points', 'end_distance_m', 'footprint.inset_m', 'footprint.step_m',
            'footprint.crossable_pad_m', 'evidence_max_age_s', 'memory_cost_max_age_s',
            'obstacle_check_strict', 'obstacle_pad_m', 'lidar_max_range_m', 'lidar_max_age_s',
            'lidar_mount', 'corridor_max_age_s', 'pose_max_age_s', 'live_max_age_s', 'grid_pose_max_gap_s',
            'memory_evidence.uncertainty_base_m', 'memory_evidence.uncertainty_per_m',
            'memory_evidence.uncertainty_limit_m', 'limits.line_tolerance_m', 'vehicle.steering_lag_s',
            'vehicle.steering_rate_deg_s', 'vehicle.lidar_x_m', 'data.track_map', 'data.track_features',
            'data.mission', 'frames.track', 'frames.base', 'frames.laser']

ACTIVE_MODES = (MissionState.MODE_ROAD, MissionState.MODE_HOLD, MissionState.MODE_RECOVERY,
                MissionState.MODE_SAFETY_STOP)


class LocalPlannerNode(CarbotNode):

    def __init__(self):
        super().__init__('local_planner', '10', REQUIRED)
        self.cfg = LocalCfg.from_params(self.p)
        self.g = geometry(self.params_under('vehicle'))
        self.course = course_from_params(self.p)
        self.frame = str(self.p('frames.track'))
        self.lp = LocalPlanner(self.cfg, self.g)
        self.inputs = EvidenceInputs(self, evidence_cfg(self.p), float(self.p('grid_pose_max_gap_s')))
        self.corr = None
        self.corr_t = -1e9
        self.mode = ''
        self.active = None
        self.active_key = None
        self.track_index = 0
        self.scan = None
        self.scan_t = -1e9
        self.steer_est = 0.0
        self.owner_t = None
        self.mount = self._lidar_mount_fallback()
        self._tf = None
        self.pub_c = self.create_publisher(CandidateArray, T.LOCAL_CANDIDATES, 1)
        self.pub_p = self.create_publisher(Path, T.LOCAL_PATH, 10)
        self.sub(Corridor, T.CORRIDOR, self._on_corr, 10)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mode', m.mode), LATCHED)
        self.sub(Path, T.ACTIVE_PATH, self._on_active, LATCHED)
        self.sub(LaserScan, T.SCAN, self._on_scan, SENSOR)
        self.sub(CommandOwnerState, T.OWNER_STATE, self._on_owner, 10)
        if str(self.p('lidar_mount')) == 'tf':
            from tf2_ros import Buffer, TransformListener
            self._tf = Buffer()
            self._tfl = TransformListener(self._tf, self)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 0.5), self._tick)
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for corridor + local pose')

    # ------------------------------------------------------------------ inputs
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lidar_mount_fallback(self):
        return (float(self.p('vehicle.lidar_x_m')), 0.0, 0.0)

    def _lidar_mount(self):
        if self._tf is None:
            return self.mount
        try:
            tr = self._tf.lookup_transform(str(self.p('frames.base')), str(self.p('frames.laser')),
                                           Time())
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.mount = (tr.transform.translation.x, tr.transform.translation.y, yaw)
            self._tf = None                  # static: look it up once
        except Exception:  # noqa: BLE001  (not published yet: keep the fallback)
            pass
        return self.mount

    def _on_corr(self, m: Corridor):
        self.corr, self.corr_t = m, self.now()

    def _on_active(self, m: Path):
        if path_key(m) != self.active_key:
            self.active_key = path_key(m)
            self.active = path_from_msg(m)
            self.track_index = 0

    def _on_scan(self, m: LaserScan):
        self.scan, self.scan_t = m, self.now()

    def _on_owner(self, m: CommandOwnerState):
        """V4 steeringEstimate: follow the commanded steering with the servo lag / rate."""
        t = stamp_s(m.header.stamp)
        if self.owner_t is not None:
            dt = max(0.0, min(0.2, t - self.owner_t))
            rate = self.cfg.steer_rate
            self.steer_est += max(-rate, min(rate, (m.commanded_steer_rad - self.steer_est) / self.cfg.steer_lag)) * dt
            self.steer_est = max(-self.g.max_steer, min(self.g.max_steer, self.steer_est))
        self.owner_t = t

    def _hits(self, pose):
        """LiDAR returns (< lidar_max_range_m) in the track frame, V4 obstaclesClear origin."""
        if self.scan is None or self.now() - self.scan_t > float(self.p('lidar_max_age_s')):
            return None
        s = self.scan
        r = np.asarray(s.ranges, float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < float(self.p('lidar_max_range_m')))
        mx, my, myaw = self._lidar_mount()
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        ox, oy = pose[0] + mx * c - my * sn, pose[1] + mx * sn + my * c
        a = pose[2] + myaw + ang[ok]
        return np.column_stack([ox + r[ok] * np.cos(a), oy + r[ok] * np.sin(a)])

    # ------------------------------------------------------------------ cycle
    def _tick(self) -> None:
        now = self.now()
        t, pose = self.inputs.pose()
        if pose is None or self.corr is None or self.mode not in ACTIVE_MODES:
            return
        if now - t > float(self.p('pose_max_age_s')) or now - self.corr_t > float(self.p('corridor_max_age_s')):
            self._publish([], -1, now, 'corridor or pose stale')
            return
        guide = path_from_msg(self.corr.guide)
        if len(guide) < 2:
            return
        ev = self.inputs.evidence(now)
        cands = self.lp.candidates(pose, guide, self.steer_est, self.inputs.speed, ev, self.course, self._hits(pose))
        win = next((q for q in cands if q.valid), None)
        problem = '' if win is not None else 'No feasible local candidate'
        if self.active is not None and len(self.active) and not (self.active[:, 3] < 0).any():
            self.track_index, err = closest(self.active, pose[0], pose[1], self.track_index)
            if err > float(self.p('problem_tracking_error_m')):
                problem = f'Tracking error {err * 100:.1f} cm exceeds {float(self.p("problem_tracking_error_m")) * 100:.0f} cm'
        sel = win.id if win is not None and not problem else -1
        self._publish(cands, sel, now, problem, guide if sel >= 0 else None, win.offset if win else 0.0)

    def _publish(self, cands, sel, now, problem, guide=None, offset=0.0):
        stamp = self.get_clock().now().to_msg()
        ca = CandidateArray()
        ca.header.stamp, ca.header.frame_id = stamp, self.frame
        ca.planner, ca.selected_id = 'local', int(sel)
        for q in cands:
            c = Candidate()
            c.id, c.offset_m = int(q.id), float(q.offset)
            c.points = [Point(x=float(x), y=float(y), z=float(d)) for x, y, _, d in q.points[::2]]
            c.cost, c.valid, c.relaxed, c.selected = float(q.cost), bool(q.valid), bool(q.relaxed), q.id == sel
            c.min_clear_m, c.command_steer_rad = float(q.min_clear), float(q.command_steer)
            c.stage = 'relaxed (paint allowance)' if q.relaxed else 'strict'
            c.reject = q.reject or ('' if q.valid else 'rejected')
            ca.candidates.append(c)
        self.pub_c.publish(ca)
        if guide is not None:
            lp = guide.copy()
            lp[:, 0] -= offset * np.sin(lp[:, 2])
            lp[:, 1] += offset * np.cos(lp[:, 2])
            self.pub_p.publish(path_to_msg(lp, self.frame, stamp))
        else:
            self.pub_p.publish(path_to_msg(np.zeros((0, 4)), self.frame, stamp))
        n_valid = sum(q.valid for q in cands)
        self.set_status(NodeStatus.OK if sel >= 0 else NodeStatus.WARN, 'PLANNING' if sel >= 0 else 'PROBLEM',
                        f'{n_valid}/{len(cands)} feasible, selected {sel}' + (f'; {problem}' if problem else ''))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
