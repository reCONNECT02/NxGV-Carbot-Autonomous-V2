"""BLOCK 14 - Safety: decide whether motion is allowed (V4 app.js safety checks).

See safety_core.py for the checks. Publishes SafetyStatus at rate_hz with
every check (value, limit, detail) and the first failing one as the veto.
The command owner (block 15) enforces the veto; mission logic only shows it.
UWB is NOT an input (a WiFi dropout must never stop the car).

Tunnel: the check runs while mission mode is TUNNEL or the local pose is in
the map's tunnel zone (V4 uses the zone). LiDAR angles are taken into
base_link with the static TF base_link -> laser_frame (the scan is mounted
reversed: see drivers.yaml carbot_tf.base_to_laser).
"""
import math

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import (Corridor, LocalGrid, LocalizationStatus, MissionState, NodeStatus, SafetyCheck,
                                   SafetyStatus)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .safety_core import SafetyCfg, SafetyCore, SafetyInputs

REQUIRED = ['rate_hz', 'camera_max_age_s', 'motion_max_age_s', 'local_sigma_max_m', 'road_min_connected_cells',
            'road_low_dwell_s', 'tunnel_front_half_angle_rad', 'tunnel_min_clearance_m', 'lidar_max_age_s',
            'hold_on_branch_conflict', 'recoverable_branch_reason', 'localization_max_age_s',
            'tunnel_zone_margin_m', 'corridor_max_age_s', 'lidar_mount', 'frames.base', 'frames.laser',
            'data.track_map']


class SafetyMonitor(CarbotNode):

    def __init__(self):
        super().__init__('safety_monitor', '14', REQUIRED)
        self.core = SafetyCore(SafetyCfg.from_params(self.p))
        self.course = course_from_params(self.p)
        self.i = SafetyInputs(t=0.0)
        self.pose = None
        self.mission = None
        self.corr_t = -1e9
        self.scan = None
        self.mount_yaw = math.pi
        self._tf = None
        if str(self.p('lidar_mount')) == 'tf':
            from tf2_ros import Buffer, TransformListener
            self._tf = Buffer()
            self._tfl = TransformListener(self._tf, self)
        self.pub = self.create_publisher(SafetyStatus, T.SAFETY_STATUS, 10)
        self.sub(Bool, T.E_STOP, self._on_estop, 10)
        self.sub(LocalGrid, T.ROAD_GRID, self._on_grid, 10)
        self.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        self.sub(Odometry, T.LOCAL_POSE, self._on_pose, 10)
        self.sub(LocalizationStatus, T.LOCAL_STATUS, self._on_status, 10)
        self.sub(LaserScan, T.SCAN, self._on_scan, SENSOR)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mission', m), LATCHED)
        self.sub(Corridor, T.CORRIDOR, self._on_corr, 10)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick)
        self.set_status(NodeStatus.OK, 'RUNNING', '')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_estop(self, m):
        # race: latched, an e-stop is final (manual intervention = 0 marks). calibrate: follows the button.
        if m.data or str(self.p('mode', 'race')) == 'calibrate':
            self.i.estop = bool(m.data) or (self.i.estop and str(self.p('mode', 'race')) != 'calibrate')

    def _on_grid(self, m: LocalGrid):
        self.i.grid_t, self.i.connected = self.now(), int(m.connected)

    def _on_odom(self, m):
        self.i.odom_t = self.now()

    def _on_pose(self, m: Odometry):
        p = m.pose.pose
        q = p.orientation
        self.pose = (p.position.x, p.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _on_status(self, m: LocalizationStatus):
        self.i.local_sigma, self.i.local_sigma_t = float(m.local_sigma_m), self.now()

    def _on_scan(self, m: LaserScan):
        self.scan, self.i.scan_t = m, self.now()

    def _on_corr(self, m: Corridor):
        self.i.branch_hold, self.i.branch_reason, self.corr_t = bool(m.branch_hold), m.branch_reason, self.now()

    def _mount_yaw(self) -> float:
        if self._tf is not None:
            try:
                from rclpy.time import Time
                tr = self._tf.lookup_transform(str(self.p('frames.base')), str(self.p('frames.laser')), Time())
                q = tr.transform.rotation
                self.mount_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self._tf = None
            except Exception:  # noqa: BLE001  not yet published: keep pi (reversed mount)
                pass
        return self.mount_yaw

    def _tick(self) -> None:
        t = self.now()
        i = self.i
        i.t = t
        ms = self.mission
        i.parking = ms is not None and ms.mode == MissionState.MODE_PARKING
        if t - self.corr_t > float(self.p('corridor_max_age_s')):
            i.branch_hold, i.branch_reason = False, ''
        in_zone = self.pose is not None and self.course.in_tunnel(self.pose[0], self.pose[1],
                                                                   float(self.p('tunnel_zone_margin_m')))
        i.in_tunnel = (ms is not None and ms.mode == MissionState.MODE_TUNNEL) or in_zone
        if self.scan is not None:
            s = self.scan
            i.scan_ranges = np.asarray(s.ranges, float)
            i.scan_angles = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment + self._mount_yaw()
        r = self.core.evaluate(i)
        m = SafetyStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.motion_allowed, m.veto_check, m.veto_reason = r.motion_allowed, r.veto_check, r.veto_reason
        for c in r.checks:
            m.checks.append(SafetyCheck(name=c.name, ok=c.ok, value=float(c.value), limit=float(c.limit),
                                        detail=c.detail))
        self.pub.publish(m)
        self.set_status(NodeStatus.OK if r.motion_allowed else NodeStatus.WARN,
                        'ALLOWED' if r.motion_allowed else 'VETO', f'{r.veto_check}: {r.veto_reason}'
                        if r.veto_check else 'all checks pass')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
