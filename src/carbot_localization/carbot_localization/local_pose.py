"""BLOCK 05 - Keep a smooth local position (V4 vehicle.js Estimator).

Predict with wheel distance (/odom) + IMU heading (/imu/rpy); correct the
translation by matching camera road/paint edges (block 03 road grid) to the
prior-map boundary in small bounded steps (<= 1.5 mm per frame). UWB NEVER
writes this estimate, so it never jumps. Matching pauses on steep pitch and in
parking. See estimator_core.py for the algorithm and the real-car differences.

In : /odom (servo_controller), /imu/rpy (JSON deg), /imu/pitch (deg),
     /carbot/perception/road_grid, /carbot/mission/state,
     /carbot/localization/reset (PoseWithCovarianceStamped, track; refused while
     a run is in progress)
Out: /carbot/localization/local_pose   nav_msgs/Odometry, frame track, child base_link
     /carbot/localization/local_status LocalizationStatus (block 05 fields)
     TF track -> base_link (servo_controller publishes no odom TF)
"""
import json
import math
import time

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import load_course
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import LocalGrid, LocalizationStatus, MissionState, NodeStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from .estimator_core import (LocalCfg, LocalEstimator, PoseHistory, grid_axes,
                             odom_increment, wrap)

REQUIRED = ['rate_hz', 'motion_timeout_s', 'heading_blend', 'sigma.initial_m',
            'sigma.idle_growth_m_per_s', 'sigma.per_distance', 'sigma.per_sqrt_s',
            'sigma.floor_m', 'sigma.visual_decay', 'visual.enabled', 'visual.max_radius_m',
            'visual.max_edge_distance_m', 'visual.gradient_step_m', 'visual.gradient_norm_min',
            'visual.gradient_norm_max', 'visual.min_rows', 'visual.prior_weight',
            'visual.weight_scale_m', 'visual.step_cap', 'visual.step_gain',
            'visual.sample_stride', 'visual.disable_pitch_rad', 'visual.disable_in_parking',
            'publish_tf', 'imu_timeout_s', 'pose_history_s', 'max_stamp_gap_s',
            'max_odom_step_m', 'heading_sigma_rad', 'status_publish_hz', 'init.source',
            'data.track_map', 'frames.track', 'frames.base']

IDLE_MODES = ('', MissionState.MODE_IDLE, MissionState.MODE_COMPLETE)


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat(a: float):
    return 0.0, 0.0, math.sin(a / 2.0), math.cos(a / 2.0)


class LocalPoseNode(CarbotNode):

    def __init__(self):
        super().__init__('local_pose', '05', REQUIRED)
        self.course = load_course(str(self.p('data.track_map')))
        self.track = str(self.p('frames.track'))
        self.base = str(self.p('frames.base'))
        self.est = LocalEstimator(LocalCfg.from_params(self.p), *self._initial_pose())
        self.hist = PoseHistory(float(self.p('pose_history_s')))     # V4 odom (track-aligned)
        self.prev_odom = None
        self.prev_t = None
        self.last_odom_wall = None
        self.imu_yaw = None
        self.imu_wall = None
        self.pitch_deg = 0.0
        self.mode = ''
        self.twist = None
        self.visual_skipped = ''
        self.odom_jumps = 0
        self.last_vis_ms = 0.0
        self.tf = TransformBroadcaster(self) if bool(self.p('publish_tf')) else None

        self.pub = self.create_publisher(Odometry, T.LOCAL_POSE, 10)
        self.pub_status = self.create_publisher(LocalizationStatus, T.LOCAL_STATUS, 10)
        self.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        self.sub(String, T.IMU_RPY, self._on_imu, SENSOR)
        self.sub(Float32, T.IMU_PITCH, self._on_pitch, SENSOR)
        self.sub(LocalGrid, T.ROAD_GRID, self._on_grid, 10)
        self.sub(MissionState, T.MISSION_STATE, self._on_mission, LATCHED)
        self.sub(PoseWithCovarianceStamped, T.LOCALIZATION_RESET, self._on_reset, 10)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick)
        self.create_timer(1.0 / max(float(self.p('status_publish_hz')), 0.5), self._publish_status)
        self._last_tick = time.monotonic()
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for /odom')

    # ------------------------------------------------------------------ inputs
    def _initial_pose(self):
        src = str(self.p('init.source'))
        if src == 'start_pose':
            return self.course.start_pose()
        if src == 'origin':
            return 0.0, 0.0, 0.0
        raise ValueError(f'local_pose.init.source must be start_pose or origin, not {src}')

    def _imu_fresh(self) -> bool:
        return self.imu_wall is not None and \
            time.monotonic() - self.imu_wall < float(self.p('imu_timeout_s'))

    def _on_imu(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self.imu_yaw = math.radians(float(d['yaw']))
            self.imu_wall = time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass

    def _on_pitch(self, msg: Float32) -> None:
        self.pitch_deg = float(msg.data)

    def _on_mission(self, msg: MissionState) -> None:
        self.mode = msg.mode

    def _on_reset(self, msg: PoseWithCovarianceStamped) -> None:
        if self.mode not in IDLE_MODES:
            self.get_logger().warn(f'reset refused: mission mode {self.mode} (run in progress)')
            return
        if msg.header.frame_id and msg.header.frame_id != self.track:
            self.get_logger().warn(f'reset refused: frame {msg.header.frame_id} != {self.track}')
            return
        p = msg.pose.pose
        self.est.reset(p.position.x, p.position.y, yaw_of(p.orientation))
        self.hist.clear()
        self.get_logger().info(f'reset to ({p.position.x:.3f}, {p.position.y:.3f}, '
                               f'{math.degrees(yaw_of(p.orientation)):.1f} deg); IMU offset re-taken')

    def _on_odom(self, msg: Odometry) -> None:
        t = stamp_s(msg.header.stamp)
        q = msg.pose.pose
        cur = (q.position.x, q.position.y, yaw_of(q.orientation))
        ds, dyaw = odom_increment(self.prev_odom, cur)
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_odom, self.prev_t = cur, t
        if abs(ds) > float(self.p('max_odom_step_m')):
            self.odom_jumps += 1                         # servo_controller restart / odom reset
            self.get_logger().warn(f'/odom jumped {ds:.3f} m in one step: ignored')
            ds, dyaw = 0.0, 0.0
        imu = self.imu_yaw if self._imu_fresh() else None
        self.est.predict(ds, dt, imu, dyaw)
        self.hist.add(t, *self.est.odom)
        self.last_odom_wall = time.monotonic()
        self.twist = msg.twist.twist
        self._publish_pose(msg.header.stamp)

    def _on_grid(self, g: LocalGrid) -> None:
        if not bool(self.p('visual.enabled')):
            self.visual_skipped = 'disabled'
            return
        if abs(math.radians(self.pitch_deg)) > float(self.p('visual.disable_pitch_rad')):
            self.visual_skipped = f'pitch {self.pitch_deg:.1f} deg'
            return
        if bool(self.p('visual.disable_in_parking')) and self.mode == MissionState.MODE_PARKING:
            self.visual_skipped = 'parking'
            return
        od = self.hist.at(stamp_s(g.header.stamp), float(self.p('max_stamp_gap_s')))
        if od is None:
            self.visual_skipped = 'no odom at grid stamp'
            return
        rows, cols = int(g.rows), int(g.cols)
        kind = np.frombuffer(bytes(g.kind), np.uint8).reshape(rows, cols)
        grown = np.frombuffer(bytes(g.grown), np.uint8).reshape(rows, cols)
        lx, ly = grid_axes(rows, cols, g.resolution_m, g.origin_x_m, g.origin_y_m)
        pose_at = (od[0] + self.est.tx, od[1] + self.est.ty, od[2])
        t0 = time.perf_counter()
        r = self.est.visual_update(kind, grown, lx, ly, pose_at, self.course)
        self.last_vis_ms = (time.perf_counter() - t0) * 1000.0
        self.visual_skipped = '' if r.applied else f'only {r.matches} edge matches'

    # ------------------------------------------------------------------ outputs
    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        if self.last_odom_wall is None or now - self.last_odom_wall > float(self.p('motion_timeout_s')):
            self.est.idle(dt)

    def _publish_pose(self, stamp) -> None:
        x, y, a = self.est.pose
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = self.track
        m.child_frame_id = self.base
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = quat(a)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        cov = [0.0] * 36
        cov[0] = cov[7] = float(self.est.sigma ** 2)
        cov[35] = float(self.p('heading_sigma_rad')) ** 2
        cov[14] = cov[21] = cov[28] = 1e6          # z, roll, pitch not estimated
        m.pose.covariance = cov
        if self.twist is not None:
            m.twist.twist = self.twist
        self.pub.publish(m)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = self.base
            tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = qx, qy, qz, qw
            self.tf.sendTransform(tf)

    def _publish_status(self) -> None:
        s = LocalizationStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.track
        s.local_sigma_m = float(self.est.sigma)
        s.visual_matches = int(self.est.visual_matches)
        s.visual_rank = float(self.est.visual_rank)
        s.visual_enabled = bool(self.p('visual.enabled')) and not self.visual_skipped.startswith(
            ('pitch', 'parking', 'disabled'))
        s.distance_travelled_m = float(self.est.distance)
        s.visual_updates = int(self.est.visual_updates)
        s.imu_ok = self._imu_fresh()
        s.heading_rad = float(wrap(self.est.pose[2]))
        waiting = self.last_odom_wall is None
        stale = not waiting and time.monotonic() - self.last_odom_wall > float(self.p('motion_timeout_s'))
        s.state = 'WAITING_INPUT' if waiting else ('NO_ODOM' if stale else 'RUNNING')
        self.pub_status.publish(s)

        detail = (f'sigma {self.est.sigma * 100:.1f} cm, vis {self.est.visual_matches} rows '
                  f'rank {self.est.visual_rank:.2f} ({self.last_vis_ms:.1f} ms) updates '
                  f'{self.est.visual_updates}, travelled {self.est.distance:.2f} m')
        if self.visual_skipped:
            detail += f', vis skipped: {self.visual_skipped}'
        if waiting:
            self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for /odom')
        elif stale:
            self.set_status(NodeStatus.ERROR, 'NO_ODOM', '/odom stale: ' + detail)
        elif not s.imu_ok:
            self.set_status(NodeStatus.WARN, 'NO_IMU', 'heading from /odom yaw: ' + detail)
        elif self.odom_jumps:
            self.set_status(NodeStatus.WARN, 'RUNNING', f'{self.odom_jumps} odom jumps ignored, ' + detail)
        else:
            self.set_status(NodeStatus.OK, 'RUNNING', detail)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
