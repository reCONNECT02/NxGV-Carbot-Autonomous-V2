"""Race supervisor: calibration gate, PREFLIGHT, READY, the one START (phase 8).

* Loads the ACTIVE calibration session; refuses to arm while any required step is missing or
  failed and lists which.
* Preflight (not a recalibration), every preflight_period_s: front camera + LiDAR rate/age, UWB link
  and every anchor, UWB calibration flags, battery, start pose vs mission.yaml P0, required nodes
  alive and not stubs, safety_monitor motion_allowed, e-stop / manual takeover clear.
  All green for ready_hold_s -> STATE_READY. The GUI enables START only then.
* START (std_srvs/Trigger /carbot/race/start) -> /carbot/race/armed latched True, exactly once;
  after that nothing un-arms it (safety_monitor / e-stop / command_owner own the veto).
"""
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import load_data, unconfirmed_roles
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import (MissionState, NodeStatus, PreflightCheck, PreflightReport,
                                   SafetyStatus, SystemHealth, UwbStatus)
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from . import preflight_core as pc
from .monitor_core import RateMeter

REQUIRED = ['preflight_period_s', 'camera_min_rate_ratio', 'camera_max_age_s', 'lidar_min_hz',
            'lidar_max_age_s', 'uwb_anchor_max_age_s', 'battery_min_v', 'start_position_tolerance_m',
            'start_heading_tolerance_deg', 'start_camera_uwb_agreement_m', 'auto_record',
            'ready_hold_s', 'status_max_age_s', 'pose_max_age_s', 'startup_grace_s',
            'required_nodes', 'warn_only_nodes', 'data_root', 'data.calibration_steps',
            'data.cameras', 'data.uwb', 'data.mission']


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RaceSupervisor(CarbotNode):

    def __init__(self):
        super().__init__('race_supervisor', '', REQUIRED)
        self.pf = pc.Cfg(
            camera_min_rate_ratio=float(self.p('camera_min_rate_ratio')),
            camera_max_age_s=float(self.p('camera_max_age_s')),
            lidar_min_hz=float(self.p('lidar_min_hz')), lidar_max_age_s=float(self.p('lidar_max_age_s')),
            uwb_anchor_max_age_s=float(self.p('uwb_anchor_max_age_s')),
            battery_min_v=float(self.p('battery_min_v')),
            start_position_tolerance_m=float(self.p('start_position_tolerance_m')),
            start_heading_tolerance_deg=float(self.p('start_heading_tolerance_deg')),
            ready_hold_s=float(self.p('ready_hold_s')), status_max_age_s=float(self.p('status_max_age_s')),
            pose_max_age_s=float(self.p('pose_max_age_s')), startup_grace_s=float(self.p('startup_grace_s')),
            required_nodes=tuple(self.p('required_nodes')), warn_only_nodes=tuple(self.p('warn_only_nodes')))

        root = cs.data_root(self.p('data_root'))
        self.session = cs.active_session(root)
        self.missing = cs.missing_required(cs.load_summary(self.session), load_data(self, 'calibration_steps'))
        cameras = load_data(self, 'cameras')
        self.unconfirmed = unconfirmed_roles(cameras)
        self.cam_topic, self.cam_name = self._front_camera(cameras)
        p0 = load_data(self, 'mission')['poses'][0]
        self.start_pose = (float(p0['x']), float(p0['y']), float(p0['yaw_deg']))
        self.root = root

        self.machine = pc.Machine(self.pf, time.monotonic())
        self.node_levels: Dict[str, Tuple[int, float]] = {}
        self.scan_meter = RateMeter(2.0)
        self.camera: Optional[Tuple[float, float, float]] = None
        self.camera_t = -1.0
        self.uwb: Optional[Tuple[float, UwbStatus]] = None
        self.battery: Optional[Tuple[float, float]] = None
        self.pose: Optional[Tuple[float, Tuple[float, float, float]]] = None
        self.safety: Optional[Tuple[float, SafetyStatus]] = None
        self.takeover = False
        self.estop = False

        self.sub(NodeStatus, T.STATUS, self._on_status, 50)
        self.sub(LaserScan, T.SCAN, lambda m: self.scan_meter.add(time.monotonic()), SENSOR)
        self.sub(UwbStatus, T.UWB_STATUS, lambda m: setattr(self, 'uwb', (time.monotonic(), m)), 10)
        self.sub(Float32, T.VEHICLE_BATTERY, lambda m: setattr(self, 'battery', (time.monotonic(), float(m.data))), 10)
        self.sub(PoseWithCovarianceStamped, T.GLOBAL_POSE, self._on_pose, 10)
        self.sub(SystemHealth, T.SYSTEM_HEALTH, self._on_health, 10)
        self.sub(SafetyStatus, T.SAFETY_STATUS, lambda m: setattr(self, 'safety', (time.monotonic(), m)), 10)
        self.sub(Bool, T.MANUAL_TAKEOVER, lambda m: setattr(self, 'takeover', bool(m.data)), LATCHED)
        self.sub(Bool, T.E_STOP, self._on_estop, 10)
        self.sub(MissionState, T.MISSION_STATE, self._on_mission, LATCHED)

        self.pub_report = self.create_publisher(PreflightReport, T.RACE_PREFLIGHT, 10)
        self.pub_armed = self.create_publisher(Bool, T.RACE_ARMED, LATCHED)
        self.pub_armed.publish(Bool(data=False))
        self.create_service(Trigger, T.RACE_START_SRV, self._on_start)
        self.create_timer(float(self.p('preflight_period_s')), self._tick)
        self.set_status(NodeStatus.OK, 'PREFLIGHT', 'checking')
        self.get_logger().info(
            f'race_supervisor: session {self.session or "NONE"}, start pose P0 {self.start_pose}, '
            f'{len(self.missing)} calibration step(s) missing')

    @staticmethod
    def _front_camera(cameras) -> Tuple[str, str]:
        sensor = cameras['roles'][T.CAMERA_ROLES[0]]
        return str(cameras['sensors'][sensor]['image_topic']), str(sensor)

    def _on_status(self, m: NodeStatus):
        self.node_levels[m.node] = (int(m.level), time.monotonic())

    def _on_pose(self, m: PoseWithCovarianceStamped):
        p = m.pose.pose
        self.pose = (time.monotonic(), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def _on_health(self, m: SystemHealth):
        for t in m.topics:
            if t.topic == self.cam_topic:
                self.camera = (float(t.rate_hz), float(t.expected_hz), float(t.age_s))
                self.camera_t = time.monotonic()

    def _on_estop(self, m: Bool):
        self.estop = bool(m.data)
        if self.estop:
            self.machine.estop()

    def _on_mission(self, m: MissionState):
        if m.mode == MissionState.MODE_COMPLETE:
            self.machine.mission_complete()

    def _snapshot(self, now: float) -> pc.Snapshot:
        s = pc.Snapshot(now=now, calibration_missing=self.missing, unconfirmed_roles=self.unconfirmed,
                        camera_name=self.cam_name, camera=self.camera,
                        camera_report_age_s=(now - self.camera_t) if self.camera_t >= 0 else -1.0,
                        lidar_rate_hz=self.scan_meter.rate_hz(now), lidar_age_s=self.scan_meter.age_s(now),
                        start_pose=self.start_pose, takeover=self.takeover, estop=self.estop,
                        node_levels={n: (lv, now - t) for n, (lv, t) in self.node_levels.items()})
        if self.uwb:
            t, u = self.uwb
            s.uwb_link_ok, s.uwb_report_age_s = bool(u.agent_link_ok), now - t
            s.uwb_anchor_ids = list(u.anchor_ids)
            s.uwb_anchor_seen = list(u.anchor_seen)
            s.uwb_anchor_age_s = [float(a) for a in u.anchor_age_s]
            s.uwb_anchors_surveyed, s.uwb_offsets_calibrated = bool(u.anchors_surveyed), bool(u.offsets_calibrated)
        if self.battery:
            s.battery_age_s, s.battery_v = now - self.battery[0], self.battery[1]
        if self.pose:
            s.pose_age_s, s.pose = now - self.pose[0], self.pose[1]
        if self.safety:
            t, m = self.safety
            s.safety_age_s, s.safety_allowed, s.safety_veto = now - t, bool(m.motion_allowed), m.veto_check
        return s

    def _evaluate(self):
        now = time.monotonic()
        checks = pc.evaluate(self.pf, self._snapshot(now))
        return self.machine.step(now, self.missing, checks), checks

    def _tick(self):
        state, checks = self._evaluate()
        r = PreflightReport()
        r.header.stamp = self.get_clock().now().to_msg()
        r.state = state
        r.calibration_session = self.session or ''
        r.missing_calibrations = list(self.missing)
        bad = [c for c in checks if not c.ok]
        if self.missing:
            r.summary = ('REFUSING TO ARM - missing/failed calibration: ' + ', '.join(self.missing)
                         + ('' if self.session else ' (no ACTIVE session in ' + self.root + ')'))
        elif state == pc.RUNNING:
            r.summary = 'RUNNING: race started'
        elif state == pc.FINISHED:
            r.summary = 'FINISHED: mission complete'
        elif state == pc.ESTOPPED:
            r.summary = 'ESTOPPED: e-stop after START'
        elif state == pc.READY:
            r.summary = 'READY: press START'
        elif bad:
            r.summary = f'NOT READY: {len(bad)} check(s) failing: ' + ', '.join(c.name for c in bad)
        else:
            r.summary = 'all checks green: holding before READY'
        r.checks.append(PreflightCheck(name='calibration', ok=not self.missing, value=self.session or 'none',
                                       expected='all required PASS', detail=', '.join(self.missing)))
        for c in checks:
            r.checks.append(PreflightCheck(name=c.name, ok=c.ok, value=c.value, expected=c.expected,
                                           detail=c.detail))
        self.pub_report.publish(r)
        self.set_status(NodeStatus.OK if state in (pc.READY, pc.RUNNING, pc.FINISHED) else NodeStatus.WARN,
                        'STATE_%d' % state, r.summary)

    def _on_start(self, req, resp):
        if self.missing:
            resp.success, resp.message = False, 'START refused: calibration missing: ' + ', '.join(self.missing)
            return resp
        self._evaluate()
        ok, msg = self.machine.start()
        resp.success, resp.message = ok, msg
        if ok:
            self.pub_armed.publish(Bool(data=True))
            self.get_logger().warn('START accepted: /carbot/race/armed = True')
            self._tick()
        else:
            self.get_logger().warn(msg)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = RaceSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
