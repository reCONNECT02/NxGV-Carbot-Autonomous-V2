"""BLOCK 06 - Let UWB help in the background (V4 Estimator.globalPose).

Keeps a separate coarse course position = block-05 local pose + a slowly
corrected offset. One EKF update per fresh anchor range, each individually
innovation-gated and predicted at the local pose at that range's own
measurement time (UWB_Handoff section 11); a well-conditioned visual landmark
from block 05 also corrects it (V4). Used for route identity (block 08/09
branch check, preflight start check), NEVER for steering, and it never writes
the local estimate.

In : /carbot/localization/local_pose, /carbot/localization/local_status,
     UWB, chosen by uwb_input (localization.yaml):
       position (default) /carbot/uwb/position  Haffiz solver + CV Kalman filter; one
                          whole-fix update per fix, R = filter covariance (track frame)
                          + position_sigma_floor_m^2, gated at position_gate_chi2
       ranges             /carbot/uwb/ranges    one gated update per fresh anchor range
       raw_fix            /carbot/uwb/raw_fix   V4 whole-fix mode, R = fix_sigma_m^2
     /carbot/localization/reset, /carbot/mission/state
Out: /carbot/localization/global_pose  PoseWithCovarianceStamped (track)
     /carbot/localization/status       LocalizationStatus (block 05 + 06 fields)
     static TF track -> venue from uwb.yaml track_to_venue
"""
import math
import time

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import LocalizationStatus, MissionState, NodeStatus, UwbRanges
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster
from uwb_localization.uwb_core import AnchorSet

from .estimator_core import GlobalCfg, GlobalEstimator, PoseHistory, TrackToVenue

REQUIRED = ['rate_hz', 'initial_variance_m2', 'growth_per_s_m2', 'growth_per_m_m2',
            'range_sigma_m', 'range_gate_chi2', 'position_gate_chi2', 'fix_sigma_m',
            'uwb_input', 'position_sigma_floor_m', 'skip_repeated_sample_seq', 'latency_compensation',
            'visual_landmark_min_rank', 'visual_landmark_sigma_m', 'max_uwb_age_s',
            'reacquire.reject_streak', 'reacquire.inflate_variance_m2', 'max_variance_m2',
            'pose_history_s', 'require_alignment', 'publish_venue_tf',
            'data.uwb', 'data.track_map', 'frames.track', 'frames.venue']

IDLE_MODES = ('', MissionState.MODE_IDLE, MissionState.MODE_COMPLETE)


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GlobalPoseNode(CarbotNode):

    def __init__(self):
        super().__init__('global_pose', '06', REQUIRED)
        doc = load_data(self, 'uwb')
        self.anchors = AnchorSet.from_yaml(doc)
        self.t2v = TrackToVenue.from_yaml(doc)
        self.aligned = bool(doc['track_to_venue'].get('aligned', False))
        lever = (doc.get('tag') or {}).get('mount_xy_m', [0.0, 0.0])
        self.lever = (float(lever[0]), float(lever[1]))     # tag antenna in base_link
        self.track = str(self.p('frames.track'))
        self.venue = str(self.p('frames.venue'))
        self.est = GlobalEstimator(GlobalCfg.from_params(self.p))
        self.hist = PoseHistory(float(self.p('pose_history_s')))
        self.local = None               # (t, x, y, a, sigma)
        self.local_status = None
        self.last_visual_updates = None
        self.last_uwb_accept_wall = None
        self.mode = ''
        self.stale_skipped = 0

        self.pub = self.create_publisher(PoseWithCovarianceStamped, T.GLOBAL_POSE, 10)
        self.pub_status = self.create_publisher(LocalizationStatus, T.LOCALIZATION_STATUS, 10)
        self.sub(Odometry, T.LOCAL_POSE, self._on_local, 10)
        self.sub(LocalizationStatus, T.LOCAL_STATUS, self._on_local_status, 10)
        self.uwb_input = str(self.p('uwb_input'))
        if self.uwb_input == 'position':
            self.sub(Odometry, T.UWB_POSITION, self._on_position, 10)
        elif self.uwb_input == 'ranges':
            self.sub(UwbRanges, T.UWB_RANGES, self._on_ranges, 10)
        elif self.uwb_input == 'raw_fix':
            self.sub(PointStamped, T.UWB_RAW_FIX, self._on_fix, 10)
        else:
            self.get_logger().error(f'localization.yaml global_pose.uwb_input={self.uwb_input!r}: '
                                    'use position | ranges | raw_fix. UWB is OFF.')
        self.sub(PoseWithCovarianceStamped, T.LOCALIZATION_RESET, self._on_reset, 10)
        self.sub(MissionState, T.MISSION_STATE, self._on_mission, LATCHED)
        if bool(self.p('publish_venue_tf')):
            self._static = StaticTransformBroadcaster(self)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.track
            tf.child_frame_id = self.venue
            # track -> venue: p_track = R^T (p_venue - t)  => venue origin in track, yaw -yaw
            ox, oy = self.t2v.to_track(0.0, 0.0)
            tf.transform.translation.x, tf.transform.translation.y = float(ox), float(oy)
            tf.transform.rotation.z = math.sin(-self.t2v.yaw / 2.0)
            tf.transform.rotation.w = math.cos(-self.t2v.yaw / 2.0)
            self._static.sendTransform(tf)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._publish)
        if not self.aligned:
            self.get_logger().warn('uwb.yaml track_to_venue.aligned=false: UWB updates are '
                                   f'{"paused" if bool(self.p("require_alignment")) else "UNRELIABLE"} '
                                   'until calibration step 11 (map-to-UWB alignment) passes')
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for local pose')

    # ------------------------------------------------------------------ inputs
    def _on_mission(self, msg: MissionState) -> None:
        self.mode = msg.mode

    def _on_reset(self, msg: PoseWithCovarianceStamped) -> None:
        if self.mode not in IDLE_MODES:
            return
        self.est.reset()
        self.hist.clear()
        self.get_logger().info('reset: offset cleared, variance back to initial')

    def _on_local(self, msg: Odometry) -> None:
        t = stamp_s(msg.header.stamp)
        p = msg.pose.pose.position
        a = yaw_of(msg.pose.pose.orientation)
        if self.local is not None:
            dt = max(0.0, t - self.local[0])
            ds = math.hypot(p.x - self.local[1], p.y - self.local[2])
            self.est.predict(dt, ds)
        self.local = (t, p.x, p.y, a, math.sqrt(max(msg.pose.covariance[0], 0.0)))
        self.hist.add(t, p.x, p.y, a)

    def _on_local_status(self, msg: LocalizationStatus) -> None:
        self.local_status = msg
        if self.last_visual_updates is not None and msg.visual_updates > self.last_visual_updates:
            self.est.landmark_update(float(msg.visual_rank))
        self.last_visual_updates = msg.visual_updates

    def _tag_xy(self, loc):
        """Tag antenna position (track) for a base_link pose: UWB measures the tag,
        not the rear axle. The offset correction is the same for both points."""
        c, s = math.cos(loc[2]), math.sin(loc[2])
        return (loc[0] + self.lever[0] * c - self.lever[1] * s,
                loc[1] + self.lever[0] * s + self.lever[1] * c)

    def _uwb_allowed(self) -> bool:
        return self.local is not None and (self.aligned or not bool(self.p('require_alignment')))

    def _on_ranges(self, msg: UwbRanges) -> None:
        if not self._uwb_allowed():
            return
        use_repeats = not bool(self.p('skip_repeated_sample_seq'))
        max_age = float(self.p('max_uwb_age_s'))
        for r in msg.ranges:
            if r.anchor_id not in self.anchors.anchors or not (r.fresh or use_repeats):
                continue
            t = stamp_s(r.measured_stamp) if bool(self.p('latency_compensation')) \
                else stamp_s(msg.header.stamp)
            if self.local[0] - t > max_age:
                self.stale_skipped += 1
                continue
            loc = self.hist.at(t, max_age)
            if loc is None:
                self.stale_skipped += 1
                continue
            a = self.anchors.anchors[r.anchor_id]
            u = self.est.range_update(r.anchor_id, self._tag_xy(loc), (a.x, a.y),
                                      float(r.range_corrected_m), self.t2v)
            if u is not None and u.accepted:
                self.last_uwb_accept_wall = time.monotonic()

    def _on_fix(self, msg: PointStamped) -> None:
        if not self._uwb_allowed():
            return
        t = stamp_s(msg.header.stamp)
        loc = self.hist.at(t, float(self.p('max_uwb_age_s')))
        if loc is None:
            return
        fx, fy = self.t2v.to_track(msg.point.x, msg.point.y)
        if self.est.fix_update(self._tag_xy(loc), (fx, fy)):
            self.last_uwb_accept_wall = time.monotonic()

    def _on_position(self, msg: Odometry) -> None:
        """Haffiz filtered fix (venue) -> one whole-fix update at the local pose of its time."""
        if not self._uwb_allowed():
            return
        t = stamp_s(msg.header.stamp)
        loc = self.hist.at(t, float(self.p('max_uwb_age_s')))
        if loc is None:
            self.stale_skipped += 1
            return
        p = msg.pose.pose.position
        fx, fy = self.t2v.to_track(p.x, p.y)
        c = msg.pose.covariance
        Cv = np.array([[c[0], c[1]], [c[6], c[7]]], float)
        Rtv = self.t2v.R                                  # venue = Rtv track + t
        floor = float(self.p('position_sigma_floor_m')) ** 2
        R = Rtv.T @ Cv @ Rtv + np.eye(2) * floor
        if self.est.fix_update(self._tag_xy(loc), (fx, fy), R):
            self.last_uwb_accept_wall = time.monotonic()

    # ------------------------------------------------------------------ outputs
    def _publish(self) -> None:
        if self.local is None:
            return
        t, x, y, a, lsig = self.local
        gx, gy, ga = self.est.pose((x, y, a))
        m = PoseWithCovarianceStamped()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = min(int(round((t - int(t)) * 1e9)), 999_999_999)
        m.header.frame_id = self.track
        m.pose.pose.position.x, m.pose.pose.position.y = float(gx), float(gy)
        m.pose.pose.orientation.z = math.sin(ga / 2.0)
        m.pose.pose.orientation.w = math.cos(ga / 2.0)
        P = self.est.P + np.eye(2) * lsig * lsig
        cov = [0.0] * 36
        cov[0], cov[1], cov[6], cov[7] = float(P[0, 0]), float(P[0, 1]), float(P[1, 0]), float(P[1, 1])
        cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = 0.02 ** 2
        m.pose.covariance = cov
        self.pub.publish(m)

        s = LocalizationStatus()
        if self.local_status is not None:
            ls = self.local_status
            s.local_sigma_m, s.visual_matches, s.visual_rank = ls.local_sigma_m, ls.visual_matches, ls.visual_rank
            s.visual_enabled, s.distance_travelled_m = ls.visual_enabled, ls.distance_travelled_m
            s.visual_updates, s.imu_ok, s.heading_rad = ls.visual_updates, ls.imu_ok, ls.heading_rad
        s.header = m.header
        s.global_sigma_m = float(self.est.sigma)
        s.global_offset_x_m, s.global_offset_y_m = float(self.est.off[0]), float(self.est.off[1])
        s.uwb_residual_m = float(self.est.residual)
        s.uwb_gain = float(self.est.last_gain)
        s.uwb_last_accepted = bool(self.est.last_accepted)
        age = -1.0 if self.last_uwb_accept_wall is None else time.monotonic() - self.last_uwb_accept_wall
        s.uwb_age_s = float(age)
        s.uwb_accepted, s.uwb_rejected = int(self.est.accepted), int(self.est.rejected)
        s.uwb_reacquires = int(self.est.reacquires)

        off = math.hypot(*self.est.off)
        detail = (f'offset {off * 100:.1f} cm, sigma {self.est.sigma * 100:.1f} cm, '
                  f'ranges ok {self.est.accepted} gated {self.est.rejected} '
                  f'reacq {self.est.reacquires}, stale {self.stale_skipped}')
        if not self._uwb_allowed():
            s.state = 'NO_ALIGNMENT'
            self.set_status(NodeStatus.WARN, 'NO_ALIGNMENT',
                            'track_to_venue not aligned (step 11): coarse pose = local pose')
        elif age < 0 or age > 2.0:
            s.state = 'NO_UWB'
            self.set_status(NodeStatus.WARN, 'NO_UWB', 'no accepted UWB range for >2 s: ' + detail)
        else:
            s.state = 'RUNNING'
            self.set_status(NodeStatus.OK, 'RUNNING', detail)
        self.pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
