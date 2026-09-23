"""Drive + measure plumbing for the wizard steps that move the car (step 7 servo_steering,
step 8 speed_pid). Same path and conventions as carbot_control.calib_drive (the terminal
tools), rebuilt for the wizard's single-threaded, tick-driven node.

The wizard NEVER writes the final motion topic. It publishes MotionRequest on
/carbot/calibration/request (T.CALIBRATION_REQUEST); the command owner (the only
final writer, safety veto, e-stop always wins) accepts it only in calibrate mode with
the race not armed:
  source CALIBRATION_RAW : speed_mps = base duty, steer_rad = base angular.z
  source CALIBRATION     : m/s + rad through the owner's speed PID and steering map
The owner rejects a request older than command_owner.request_expiry_s, so the
request is re-published at drive.publish_hz, and only while the step keeps refreshing
it (DriveCommand.set every tick): a step that stops ticking (crash, cancel) stops the
car after drive.command_hold_s. /e_stop True clears the command at once.

Pure pieces (no rclpy, unit tested, used by the steps and the mock server):
  DriveCommand(hold_s)           .set(now, source, speed, steer, reason) / .stop() / .current(now)
  DriveMeter(history_n)          .on_odom(now, x, y, vx) / .on_yaw_deg(now, deg) / .on_estop(now, pressed)
                                 .on_owner(now, winner, reason) / .snapshot(now, max_age_s) / .history(t_from)
  Segment(source, speed, steer, reason, until, timeout_s, settle_s, accept_s)
                                 one "drive until" leg: .start(now, inputs) / .tick(now, inputs, cmd) -> result | None
  DriveKit(cmd, servo, owner, accept_s)   what a driving step's constructor takes
  DictParams(values)             in-memory stand-in for ParamLink (tests, mock server)
  SimCar(meter, cmd, servo, ...) synthetic car that obeys the DriveCommand (tests, mock server)
ROS glue (rclpy imported lazily):
  ParamLink(node, target, timeout_s)   non-blocking get(names, done) / set(values, done) on another node
  WizardDrive(node, cfg)               publisher + subscriptions + ParamLinks; .cmd .meter .servo .owner .kit
                                       .inputs() -> the keys below, merged into the wizard's inputs dict

inputs keys (calibration_wizard.inputs()):
  'drive'          DriveMeter.snapshot(): {distance_m, yaw_rad, speed_mps, odom_ok, imu_ok, odom_age_s,
                   imu_age_s, estop, estop_count, owner_winner, owner_reason, owner_age_s}
                   distance_m = signed path length integrated from /odom (x, y) (sign of twist.linear.x);
                   yaw_rad = unwrapped /imu/rpy yaw, + = LEFT (counter-clockwise), both since launch:
                   steps use differences.
  'drive_history'  callable(t_from) -> [{t, dist, yaw, v}] at /odom rate (for speed averages).
Steps get a DriveKit through their constructor (WizardDrive.kit: .cmd, .servo = servo_controller,
.owner = command_owner, .accept_s), run Segments in tick() and call kit.cmd.stop() in cancel().

Config: ops.yaml calibration_wizard.drive.{publish_hz, command_hold_s, input_max_age_s,
param_timeout_s, owner_accept_s, history_n} (DRIVE_KEYS, all required).
"""
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

RAW = 'CALIBRATION_RAW'
CLOSED = 'CALIBRATION'
DRIVE_KEYS = ('publish_hz', 'command_hold_s', 'input_max_age_s', 'param_timeout_s', 'owner_accept_s',
              'history_n')
SERVO = 'servo_controller'
OWNER = 'command_owner'


class DriveCommand:
    """What the wizard asks the command owner for. Expires hold_s after the last set()."""

    def __init__(self, hold_s: float):
        self.hold_s = float(hold_s)
        self._cmd: Optional[Tuple[str, float, float, str]] = None
        self._t = -1e9

    def set(self, now: float, source: str, speed: float, steer: float, reason: str) -> None:
        if source not in (RAW, CLOSED):
            raise ValueError(f'drive source must be {RAW} or {CLOSED}, not {source!r}')
        self._cmd, self._t = (source, float(speed), float(steer), str(reason)), float(now)

    def stop(self) -> None:
        self._cmd = None

    def current(self, now: float) -> Optional[Tuple[str, float, float, str]]:
        if self._cmd is not None and now - self._t > self.hold_s:
            self._cmd = None
        return self._cmd


class DriveMeter:
    """Distance from /odom, unwrapped yaw from /imu/rpy, e-stop and command-owner state."""

    def __init__(self, history_n: int = 3000):
        self.dist = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._prev_yaw: Optional[float] = None
        self.t_odom = self.t_imu = self.t_owner = None
        self.estop = False
        self.estop_count = 0
        self.owner = ('', '')
        self._hist: deque = deque(maxlen=max(int(history_n), 10))

    def on_odom(self, now: float, x: float, y: float, vx: float) -> None:
        if self._prev_xy is not None:
            ds = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            self.dist += ds if vx >= 0 else -ds
        self._prev_xy = (float(x), float(y))
        self.speed, self.t_odom = float(vx), float(now)
        self._hist.append({'t': float(now), 'dist': self.dist, 'yaw': self.yaw, 'v': self.speed})

    def on_yaw_deg(self, now: float, yaw_deg: float) -> None:
        y = math.radians(float(yaw_deg))
        if self._prev_yaw is not None:
            self.yaw += math.atan2(math.sin(y - self._prev_yaw), math.cos(y - self._prev_yaw))
        self._prev_yaw, self.t_imu = y, float(now)

    def on_imu_json(self, now: float, data: str) -> bool:
        """/imu/rpy String JSON {"roll","pitch","yaw"} (deg). False if unreadable."""
        try:
            self.on_yaw_deg(now, float(json.loads(data)['yaw']))
        except (ValueError, KeyError, TypeError):
            return False
        return True

    def on_estop(self, now: float, pressed: bool) -> None:
        if pressed:
            self.estop_count += 1
        self.estop = bool(pressed)

    def on_owner(self, now: float, winner: str, reason: str) -> None:
        self.owner, self.t_owner = (str(winner), str(reason)), float(now)

    def history(self, t_from: float) -> List[Dict]:
        return [dict(r) for r in self._hist if r['t'] >= t_from]

    def snapshot(self, now: float, max_age_s: float) -> Dict:
        age = lambda t: None if t is None else round(now - t, 3)  # noqa: E731
        oa, ia, wa = age(self.t_odom), age(self.t_imu), age(self.t_owner)
        return {'distance_m': self.dist, 'yaw_rad': self.yaw, 'speed_mps': self.speed,
                'odom_ok': oa is not None and oa <= max_age_s, 'imu_ok': ia is not None and ia <= max_age_s,
                'odom_age_s': oa, 'imu_age_s': ia, 'estop': self.estop, 'estop_count': self.estop_count,
                'owner_winner': self.owner[0] if wa is not None and wa <= max_age_s else '',
                'owner_reason': self.owner[1], 'owner_age_s': wa}


def inputs_problem(snap: Optional[Dict], need_imu: bool = True) -> str:
    """'' when the drive inputs are usable, else why not (for start() refusals)."""
    if not snap:
        return 'The wizard has no odometry / IMU feed (drive plumbing not running).'
    if snap.get('estop'):
        return 'STOP MOTORS is pressed: release it first (header button).'
    if not snap.get('odom_ok'):
        return ('No fresh /odom from servo_controller: is the base running and the battery on? '
                '(step 1 checks it)')
    if need_imu and not snap.get('imu_ok'):
        return 'No fresh /imu/rpy from servo_controller: step 6 (IMU) must work first.'
    return ''


class Segment:
    """Drive with a fixed command until until(distance, yaw, elapsed) is true, then hold
    zero speed for settle_s and report the change in distance and yaw (calib_drive.Driver.run).

    Aborts (result['aborted'] != ''): e-stop pressed (stops at once, no settle), odom / IMU
    stale, the command owner not showing this source as winner within accept_s, timeout_s."""

    def __init__(self, source: str, speed: float, steer: float, reason: str,
                 until: Callable[[float, float, float], bool], timeout_s: float, settle_s: float,
                 accept_s: float, need_imu: bool = True):
        self.source, self.speed, self.steer, self.reason = source, float(speed), float(steer), reason
        self.until, self.timeout_s, self.settle_s = until, float(timeout_s), float(settle_s)
        self.accept_s, self.need_imu = float(accept_s), need_imu
        self.state = 'idle'            # idle | drive | settle | done
        self.t0 = self.t_settle = 0.0
        self.d0 = self.y0 = 0.0
        self.d = self.y = self.elapsed = 0.0
        self.aborted = ''
        self.estops = 0
        self.accepted = False
        self.rows: List[Dict] = []

    def start(self, now: float, inputs: Dict) -> None:
        snap = inputs.get('drive') or {}
        self.t0, self.state = now, 'drive'
        self.d0, self.y0 = float(snap.get('distance_m', 0.0)), float(snap.get('yaw_rad', 0.0))
        self.estops = int(snap.get('estop_count', 0))
        self.d = self.y = self.elapsed = 0.0
        self.aborted, self.accepted, self.rows = '', False, []

    def _measure(self, now: float, snap: Dict) -> None:
        self.d = float(snap.get('distance_m', 0.0)) - self.d0
        self.y = float(snap.get('yaw_rad', 0.0)) - self.y0
        self.elapsed = now - self.t0

    def tick(self, now: float, inputs: Dict, cmd: DriveCommand) -> Optional[Dict]:
        snap = inputs.get('drive') or {}
        if self.state in ('idle', 'done'):
            return None
        self._measure(now, snap)
        if snap.get('estop') or int(snap.get('estop_count', 0)) != self.estops:
            cmd.stop()
            self.aborted = 'STOP MOTORS pressed'
            return self._finish(now, inputs)
        if self.state == 'drive':
            self.rows.append({'t': round(self.elapsed, 3), 'dist': self.d, 'yaw': self.y,
                              'v': float(snap.get('speed_mps', 0.0))})
            if snap.get('owner_winner') == self.source:
                self.accepted = True
            why = ''
            if not snap.get('odom_ok'):
                why = '/odom stopped while driving'
            elif self.need_imu and not snap.get('imu_ok'):
                why = '/imu/rpy stopped while driving'
            elif not self.accepted and self.elapsed > self.accept_s:
                w = snap.get('owner_winner') or 'no state'
                why = (f'command_owner did not take the calibration request (winner {w}'
                       f'{": " + snap["owner_reason"] if snap.get("owner_reason") else ""})')
            elif self.until(self.d, self.y, self.elapsed):
                why = None
            elif self.elapsed > self.timeout_s:
                why = f'timeout {self.timeout_s:g} s'
            if why is None or why:
                self.aborted = why or ''
                self.state, self.t_settle = 'settle', now
            else:
                cmd.set(now, self.source, self.speed, self.steer, self.reason)
                return None
        if self.state == 'settle':
            if now - self.t_settle < self.settle_s:
                cmd.set(now, self.source, 0.0, self.steer if self.source == RAW else 0.0, 'stop')
                return None
            cmd.stop()
            return self._finish(now, inputs)
        return None

    def _finish(self, now: float, inputs: Dict) -> Dict:
        self.state = 'done'
        hist = inputs.get('drive_history')
        rows = self.rows
        if callable(hist):
            try:
                rows = [dict(r, t=round(r['t'] - self.t0, 3), dist=r['dist'] - self.d0, yaw=r['yaw'] - self.y0)
                        for r in hist(self.t0)] or self.rows
            except Exception:  # noqa: BLE001  per-tick rows are good enough
                rows = self.rows
        return {'distance': self.d, 'yaw': self.y, 'elapsed': round(self.elapsed, 2), 'aborted': self.aborted,
                'rows': rows}

    def progress(self) -> Dict:
        return {'state': self.state, 'distance_m': round(self.d, 3), 'yaw_deg': round(math.degrees(self.y), 1),
                'elapsed_s': round(self.elapsed, 1), 'accepted': self.accepted}


@dataclass
class DriveKit:
    """What a driving step gets in its constructor (WizardDrive.kit; tests build one by hand)."""
    cmd: DriveCommand
    servo: Any          # ParamLink('servo_controller') | DictParams
    owner: Any          # ParamLink('command_owner') | DictParams
    accept_s: float     # drive.owner_accept_s (Segment accept_s)


class DictParams:
    """In-memory parameter link (tests, mock server): same get/set(done) interface as ParamLink."""

    def __init__(self, values: Optional[Dict[str, Any]] = None, fail: bool = False):
        self.values = dict(values or {})
        self.fail = fail
        self.sets: List[Dict[str, Any]] = []

    def get(self, names: List[str], done: Callable[[Optional[Dict[str, Any]]], None]) -> None:
        done(None if self.fail else {n: self.values.get(n) for n in names})

    def set(self, values: Dict[str, Any], done: Callable[[bool], None]) -> None:
        if self.fail:
            done(False)
            return
        self.values.update(values)
        self.sets.append(dict(values))
        done(True)


class SimCar:
    """Tiny kinematic car for the tests and the GUI mock (no physics claims): obeys the
    DriveCommand like the command owner would (CALIBRATION_RAW only), feeds a DriveMeter.

    speed = duty * mps_per_duty. Base angular.z of +-1 is a full lock; a physical LEFT lock is
    z == true_sign (the base default steer_sign -1). At z = 0 the car curves by
    (true_center - servo.values['servo_center']) * k_per_unit (+ = left, base convention:
    a larger servo angle turns right)."""

    def __init__(self, meter: DriveMeter, cmd: DriveCommand, servo: DictParams, true_sign: float = -1.0,
                 radius_left: float = 0.38, radius_right: float = 0.41, true_center: int = 93,
                 k_per_unit: float = 0.02, mps_per_duty: float = 2.0):
        self.meter, self.cmd, self.servo = meter, cmd, servo
        self.true_sign, self.rl, self.rr = float(true_sign), float(radius_left), float(radius_right)
        self.true_center, self.ku, self.gain = int(true_center), float(k_per_unit), float(mps_per_duty)
        self.x = self.y = self.th = 0.0
        self.blocked = False          # True = the owner refuses (e.g. not calibrate mode)

    def curvature(self, z: float) -> float:
        if abs(z) < 1e-6:
            return (self.true_center - int(self.servo.values.get('servo_center', self.true_center))) * self.ku
        left = (z > 0) == (self.true_sign > 0)
        return abs(z) / self.rl if left else -abs(z) / self.rr

    def step(self, now: float, dt: float) -> None:
        c = None if self.meter.estop or self.blocked else self.cmd.current(now)
        v = float(c[1]) * self.gain if c and c[0] == RAW else 0.0
        k = self.curvature(float(c[2])) if c else 0.0
        self.th += v * k * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.meter.on_odom(now, self.x, self.y, v)
        self.meter.on_yaw_deg(now, math.degrees(math.atan2(math.sin(self.th), math.cos(self.th))))
        self.meter.on_owner(now, 'DISARMED' if self.blocked else (c[0] if c else 'DISARMED'),
                            'mode race' if self.blocked else '')


# --------------------------------------------------------------------------- ROS glue
class ParamLink:
    """Non-blocking get/set of another node's parameters. The wizard runs a single-threaded
    executor, so it can never wait for a reply inside a callback (calib_tools.RemoteParams
    spins its own node and would deadlock here): the request is sent and done(...) is called
    from the executor when the reply arrives, or done(None / False) after timeout_s."""

    def __init__(self, node, target: str, timeout_s: float):
        from rcl_interfaces.srv import GetParameters, SetParameters
        self.node, self.target, self.timeout_s = node, target.lstrip('/'), float(timeout_s)
        self._get = node.create_client(GetParameters, f'/{self.target}/get_parameters')
        self._set = node.create_client(SetParameters, f'/{self.target}/set_parameters')
        self._pending: List[Dict] = []
        node.create_timer(0.2, self._expire)

    def _call(self, cli, req, on_reply: Callable[[Any], None], on_fail: Callable[[], None]) -> None:
        if not cli.service_is_ready():
            on_fail()
            return
        fut = cli.call_async(req)
        entry = {'fut': fut, 'until': time.monotonic() + self.timeout_s, 'fail': on_fail}
        self._pending.append(entry)

        def _done(f):
            if entry not in self._pending:
                return                               # already timed out
            self._pending.remove(entry)
            try:
                on_reply(f.result())
            except Exception:  # noqa: BLE001
                on_fail()
        fut.add_done_callback(_done)

    def _expire(self) -> None:
        now = time.monotonic()
        for e in [e for e in self._pending if now > e['until']]:
            self._pending.remove(e)
            e['fut'].cancel()
            e['fail']()

    def get(self, names: List[str], done: Callable[[Optional[Dict[str, Any]]], None]) -> None:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        names = list(names)
        self._call(self._get, GetParameters.Request(names=names),
                   lambda r: done({n: parameter_value_to_python(v) for n, v in zip(names, r.values)}),
                   lambda: done(None))

    def set(self, values: Dict[str, Any], done: Callable[[bool], None]) -> None:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req = SetParameters.Request(parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
        self._call(self._set, req, lambda r: done(all(x.successful for x in r.results)), lambda: done(False))


class WizardDrive:
    """ROS side: MotionRequest publisher (re-published at publish_hz while DriveCommand is
    fresh), /odom + /imu/rpy + /e_stop + owner state subscriptions into a DriveMeter, and
    ParamLinks to servo_controller and command_owner."""

    def __init__(self, node, cfg: Dict):
        miss = [k for k in DRIVE_KEYS if k not in cfg]
        if miss:
            raise KeyError('ops.yaml calibration_wizard.drive: missing ' + ', '.join(miss))
        from carbot_common import topics as T
        from carbot_interfaces.msg import CommandOwnerState, MotionRequest
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool, String
        self.node, self.cfg, self.MotionRequest = node, cfg, MotionRequest
        self.cmd = DriveCommand(float(cfg['command_hold_s']))
        self.meter = DriveMeter(int(cfg['history_n']))
        self.max_age = float(cfg['input_max_age_s'])
        self.servo = ParamLink(node, SERVO, float(cfg['param_timeout_s']))
        self.owner = ParamLink(node, OWNER, float(cfg['param_timeout_s']))
        self.kit = DriveKit(self.cmd, self.servo, self.owner, float(cfg['owner_accept_s']))
        self.pub = node.create_publisher(MotionRequest, T.CALIBRATION_REQUEST, 10)
        mono = time.monotonic
        node.create_subscription(Odometry, T.ODOM, lambda m: self.meter.on_odom(
            mono(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x), qos_profile_sensor_data)
        node.create_subscription(String, T.IMU_RPY, lambda m: self.meter.on_imu_json(mono(), m.data),
                                 qos_profile_sensor_data)
        node.create_subscription(CommandOwnerState, T.OWNER_STATE,
                                 lambda m: self.meter.on_owner(mono(), m.winner, m.reason), 10)
        node.create_subscription(Bool, T.E_STOP, self._estop, 10)
        node.create_timer(1.0 / max(float(cfg['publish_hz']), 5.0), self._publish)

    def _estop(self, m) -> None:
        if m.data:
            self.cmd.stop()
        self.meter.on_estop(time.monotonic(), bool(m.data))

    def _publish(self) -> None:
        c = self.cmd.current(time.monotonic())
        if c is None:
            return
        r = self.MotionRequest()
        r.header.stamp = self.node.get_clock().now().to_msg()
        r.source, r.speed_mps, r.steer_rad, r.reason = c[0], float(c[1]), float(c[2]), c[3]
        self.pub.publish(r)

    def inputs(self) -> Dict:
        return {'drive': self.meter.snapshot(time.monotonic(), self.max_age), 'drive_history': self.meter.history}

    def stop(self) -> None:
        self.cmd.stop()
