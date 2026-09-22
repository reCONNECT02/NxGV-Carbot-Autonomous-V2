"""BLOCK 15 - Command owner: one final writer (V4 vehicle.js arbitrate) + BLOCK 16 interface.

Every block ASKS (MotionRequest on /carbot/request/<source>); only this node
COMMANDS the car. Priority (owner_core.arbitrate):
  DISARMED -> SAFETY STOP (e-stop, stale / vetoing SafetyStatus, mission SAFETY_STOP)
  -> WATCHDOG (mission state stale, request older than request_expiry_s)
  -> HOLD (mission active_source HOLD) -> the source mission logic marked active.
Converts the winning speed (m/s; feedforward + PID on /odom) and steering
(rad, REP-103) into the base servo_controller's /cmd_vel_auto units, and
publishes them at rate_hz ALWAYS (zeros when not driving), so the base
auto_cmd_timeout never trips while this node lives.

Arming (block 16, servo_controller extension): /carbot/vehicle/arm True at
arm_publish_hz while the race is armed (base AUTO mode), False once on
disarm / e-stop (base MANUAL + stop). In calibrate mode the calibration steps
7/8 drive through /carbot/calibration/request (CALIBRATION / CALIBRATION_RAW);
the owner arms the base while those requests are fresh and disarms once
they stop, so the joystick works again in between.

E-stop: race mode latches it (manual intervention = 0 marks); calibrate mode
follows the button. The base auto_driver and cmd_safety_controller are NOT
launched (they would be competing writers).
"""
import time

import rclpy
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import CommandOwnerState, MissionState, MotionRequest, NodeStatus, SafetyStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from .owner_core import OwnerCfg, Req, SpeedCfg, SpeedController, SteeringMap, arbitrate

REQUIRED = ['rate_hz', 'request_expiry_s', 'safety_max_age_s', 'mission_max_age_s',
            'speed_pid.kp', 'speed_pid.ki', 'speed_pid.kd', 'speed_pid.integral_limit',
            'feedforward.duty_per_mps', 'feedforward.static_duty', 'duty_max',
            'duty_reverse_max', 'duty_slew_per_s', 'stop_speed_mps', 'measured_max_age_s',
            'steering.steer_sign', 'steering.left_max_rad', 'steering.right_max_rad', 'steering.trim_rad',
            'steering.angular_limit', 'output_topic', 'arm_publish_hz', 'estop_latch_in_race',
            'calibration.max_speed_mps', 'calibration.duty_max']


class CommandOwner(CarbotNode):

    def __init__(self):
        super().__init__('command_owner', '15', REQUIRED)
        self.mode = str(self.p('mode', 'race'))
        self.cfg = OwnerCfg(float(self.p('request_expiry_s')), float(self.p('safety_max_age_s')),
                            float(self.p('mission_max_age_s')), self.mode == 'calibrate',
                            float(self.p('calibration.max_speed_mps')), float(self.p('calibration.duty_max')))
        self.steering = SteeringMap.from_params(self.p)
        self.speed_cfg = SpeedCfg.from_params(self.p)
        self.speed = SpeedController(self.speed_cfg)
        self.requests = {}
        self.calib = None
        self.safety = None
        self.safety_msg = None
        self.mission = None
        self.race_armed = False
        self.estop = False
        self.manual = False
        self.meas, self.meas_t = 0.0, -1e9
        self.base_armed = None             # what we last told servo_controller
        self.last_arm_pub = 0.0
        self.last_t = None
        self.pub_cmd = self.create_publisher(Twist, str(self.p('output_topic')), 10)
        self.pub_state = self.create_publisher(CommandOwnerState, T.OWNER_STATE, 10)
        self.pub_arm = self.create_publisher(Bool, T.VEHICLE_ARM, 10)
        for s in T.REQUEST_SOURCES:
            self.sub(MotionRequest, T.request_topic(s), self._on_request, 10)
        self.sub(MotionRequest, T.CALIBRATION_REQUEST, self._on_calib, 10)
        self.sub(SafetyStatus, T.SAFETY_STATUS, self._on_safety, 10)
        self.sub(MissionState, T.MISSION_STATE, self._on_mission, LATCHED)
        self.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        self.sub(Bool, T.RACE_ARMED, self._on_armed, LATCHED)
        self.sub(Bool, T.E_STOP, self._on_estop, 10)
        self.sub(Bool, T.MANUAL_TAKEOVER, self._on_manual, LATCHED)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick)
        self.create_timer(1.0, self._refresh)
        self.set_status(NodeStatus.OK, 'DISARMED', f'mode {self.mode}')

    def _refresh(self) -> None:
        """Calibrate mode: pick up live parameter changes (steps 7/8, tuning tab).
        Never while the race is armed (race: tuning is disabled)."""
        if self.mode != 'calibrate' or self.race_armed:
            return
        self.steering = SteeringMap.from_params(self.p)
        cfg = SpeedCfg.from_params(self.p)
        if cfg != self.speed_cfg:
            self.speed_cfg = cfg
            self.speed.cfg = cfg

    def now(self) -> float:
        return time.monotonic()

    # ------------------------------------------------------------------ inputs
    def _req(self, m: MotionRequest) -> Req:
        return Req(m.source, float(m.speed_mps), float(m.steer_rad), bool(m.arrived), m.reason, self.now())

    def _on_request(self, m: MotionRequest):
        if m.source in T.REQUEST_SOURCES:
            self.requests[m.source] = self._req(m)

    def _on_calib(self, m: MotionRequest):
        if m.source in ('CALIBRATION', 'CALIBRATION_RAW'):
            self.calib = self._req(m)

    def _on_safety(self, m: SafetyStatus):
        self.safety_msg = m
        self.safety = (bool(m.motion_allowed), f'{m.veto_check}: {m.veto_reason}' if m.veto_check else
                       m.veto_reason, self.now())

    def _on_mission(self, m: MissionState):
        self.mission = (m.mode, m.active_source, self.now())

    def _on_odom(self, m: Odometry):
        self.meas, self.meas_t = float(m.twist.twist.linear.x), self.now()

    def _on_armed(self, m: Bool):
        self.race_armed = bool(m.data)

    def _on_estop(self, m: Bool):
        if m.data:
            self.estop = True
        elif self.mode == 'calibrate' or not bool(self.p('estop_latch_in_race')):
            self.estop = False

    def _on_manual(self, m: Bool):
        on = bool(m.data)
        if on != self.manual:
            self.get_logger().warn('MANUAL control ' + ('ON: base AUTO released, joystick drives'
                                                        + (' (race: counts as manual intervention)'
                                                           if self.mode == 'race' and self.race_armed else '')
                                                        if on else 'OFF: handed back'))
        self.manual = on

    # ------------------------------------------------------------------ cycle
    def _arm(self, want: bool, t: float) -> None:
        period = 1.0 / max(float(self.p('arm_publish_hz')), 0.1)
        if want:
            if self.base_armed is not True or t - self.last_arm_pub > period:
                self.pub_arm.publish(Bool(data=True))
                self.base_armed, self.last_arm_pub = True, t
        elif self.base_armed is True:            # disarm ONCE (the joystick may take over in calibrate)
            self.pub_arm.publish(Bool(data=False))
            self.base_armed = False

    def _tick(self) -> None:
        t = self.now()
        dt = 0.02 if self.last_t is None else t - self.last_t
        self.last_t = t
        armed = self.race_armed and not self.estop
        d = arbitrate(t, armed, self.estop, self.safety, self.mission, self.requests, self.calib, self.cfg,
                      manual=self.manual)
        calib_active = d.winner in ('CALIBRATION', 'CALIBRATION_RAW')
        self._arm((armed or calib_active) and not self.estop and not self.manual, t)
        meas = self.meas if t - self.meas_t <= self.speed_cfg.measured_max_age_s else None
        if d.drive and d.raw:
            self.speed.reset()
            lin, ang = float(d.speed), max(-self.steering.angular_limit, min(self.steering.angular_limit, d.steer))
            cmd_v, cmd_s = 0.0, self.steering.to_steer(ang)
        elif d.drive:
            lin = self.speed.update(d.speed, meas, dt)
            ang = self.steering.to_angular(d.steer)
            cmd_v, cmd_s = d.speed, d.steer
        else:
            self.speed.reset()
            lin, ang, cmd_v, cmd_s = 0.0, 0.0, 0.0, 0.0
        tw = Twist()
        tw.linear.x, tw.angular.z = float(lin), float(ang)
        self.pub_cmd.publish(tw)
        st = CommandOwnerState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.active_source = self.mission[1] if self.mission else ''
        st.winner, st.reason, st.armed = d.winner, d.reason, bool(armed or calib_active)
        if d.request is not None:
            r = d.request
            st.request.source, st.request.speed_mps, st.request.steer_rad = r.source, r.speed, r.steer
            st.request.arrived, st.request.reason = r.arrived, r.reason
        st.request_age_s = float(d.request_age)
        st.commanded_speed_mps, st.commanded_steer_rad = float(cmd_v), float(cmd_s)
        st.measured_speed_mps = float(meas if meas is not None else float('nan'))
        st.out_linear_x, st.out_angular_z = float(lin), float(ang)
        st.watchdog_ok = d.winner != 'WATCHDOG'
        self.pub_state.publish(st)
        lvl = NodeStatus.OK if d.winner not in ('WATCHDOG', 'SAFETY_STOP') else NodeStatus.WARN
        self.set_status(lvl, d.winner, d.reason)


def main(args=None):
    rclpy.init(args=args)
    node = CommandOwner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
            node.pub_arm.publish(Bool(data=False))
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
