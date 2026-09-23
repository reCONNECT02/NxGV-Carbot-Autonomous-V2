"""Guided calibration wizard backend (calibrate.launch.py only, phase 8).

Replaces the phase-1 stub; same node name, topics, service and message types.

  /carbot/calibration/state   CalibrationState (latched): every step, status, can_advance
  /carbot/calibration/live    String JSON (latched, live_rate_hz): the OPEN step's live view,
                              result, instructions, sessions (for rollback), running task
  /carbot/calibration/action  CalibrationAction: SELECT | RUN | REDO | SAVE | KEEP_PREVIOUS |
                              CANCEL | ROLLBACK | RESTART_CAMERAS

Logic lives in wizard_core (order, sessions, save/keep/rollback) and one StepImpl
per built step page (step 1: step_sensor_health, step 2: step_camera_identity).
Steps without a page yet are
placeholders: they show their instructions and terminal tool.

Never crashes on user input or broken YAML: a configuration problem is reported
as NodeStatus CONFIG_ERROR, in every service reply and in the live JSON, so the
GUI can show the exact message. STOP MOTORS (/e_stop) cancels a running step.
"""
import json
import os
import time
import traceback

import rclpy
from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import CalibrationState, CalibrationStepState, NodeStatus, SystemHealth, UwbStatus
from carbot_interfaces.srv import CalibrationAction
from std_msgs.msg import Bool, Float32, String

from . import monitor_core as mc
from .camera_restart import CFG_KEYS as RESTART_KEYS
from .camera_restart import CameraRestart
from .frame_tap import FrameTap
from .step_camera_identity import CameraIdentityStep
from .step_camera_intrinsics import CameraIntrinsicsStep
from .step_mission_planner import MissionPlannerStep
from .step_sensor_health import SensorHealthStep
from .wizard_core import StepImpl, Wizard

REQUIRED = ['session_format', 'allow_keep_previous', 'data_root', 'data.calibration_steps', 'data.cameras',
            'data.uwb',
            # phase 8
            'resume_max_age_h', 'live_rate_hz', 'state_rate_hz', 'tick_hz', 'refresh_period_s',
            'input_timeout_s',
            # literal (test_required_keys reads it with ast); = camera_restart.CFG_KEYS
            'restart_cameras.enabled', 'restart_cameras.kill_patterns', 'restart_cameras.grace_s',
            'restart_cameras.root_helper_dir', 'restart_cameras.delay_mipi_second_s',
            'restart_cameras.settle_s', 'restart_cameras.timeout_s', 'restart_cameras.log_dir']


class BrokenStep(StepImpl):
    """A built step whose YAML is wrong: explains why instead of crashing."""
    can_keep_previous = False

    def __init__(self, cfg, error):
        super().__init__(cfg)
        self.error = error

    def start(self, now, inputs):
        return self.error

    def live(self, inputs):
        return {'error': self.error}


class CalibrationWizard(CarbotNode):

    def __init__(self):
        super().__init__('calibration_wizard', '', REQUIRED)
        self.error = ''
        self.wiz = None
        self.restart = None
        self.health = self.uwb_status = self.battery = None
        self.health_seq = 0
        self.tap = None
        self.pub_state = self.create_publisher(CalibrationState, T.CALIBRATION_STATE, LATCHED)
        self.pub_live = self.create_publisher(String, T.CALIBRATION_LIVE, LATCHED)
        self.create_service(CalibrationAction, T.CALIBRATION_ACTION_SRV, self._srv)
        try:
            self._setup()
        except Exception as e:  # noqa: BLE001  report, stay up, answer every request with the reason
            self.error = f'{type(e).__name__}: {e}'
            self.get_logger().error('calibration wizard cannot start: ' + self.error + '\n' + traceback.format_exc())
            self.set_status(NodeStatus.ERROR, 'CONFIG_ERROR', self.error)
            self.create_timer(1.0, self._publish_error)
            self._publish_error()

    # ------------------------------------------------------------------ setup
    def _setup(self):
        missing = [k for k in REQUIRED if not self.has_parameter(k)]
        if missing:
            raise KeyError('missing YAML keys ' + ', '.join(missing) + ' (ops.yaml calibration_wizard)')
        steps_doc = load_data(self, 'calibration_steps')
        cameras, uwb = load_data(self, 'cameras'), load_data(self, 'uwb')
        self.domain = int((uwb.get('agent') or {}).get('domain_id', 1))
        # one line per built step page (step id -> StepImpl); every other step is a placeholder
        factories = {
            'sensor_health': lambda s: SensorHealthStep(s, cameras, uwb),
            'camera_identity': lambda s: CameraIdentityStep(s, cameras),
            'camera_intrinsics': lambda s: CameraIntrinsicsStep(s, cameras),
            'mission_planner': lambda s: MissionPlannerStep(s, ct.bringup_config_dir(),
                                                            lambda: self.wiz.session if self.wiz else None),
        }
        impls = {}
        for s in steps_doc.get('steps', []):
            make = factories.get(s.get('id'))
            if make is None:
                continue
            try:
                impls[s['id']] = make(s)
            except Exception as e:  # noqa: BLE001
                why = f'step {s.get("index")} configuration: {e}'
                impls[s['id']] = BrokenStep(s, why)
                self.get_logger().error(why)
        self.tap = FrameTap(self, {n: x['image_topic'] for n, x in cameras['sensors'].items() if 'image_topic' in x})
        root = cs.data_root(str(self.p('data_root')))
        self.wiz = Wizard(steps_doc, root, {k: self.p(k) for k in ('session_format', 'allow_keep_previous',
                                                                   'resume_max_age_h')}, impls)
        if self.wiz.notice:
            self.get_logger().info(self.wiz.notice)
        rc = {k: self.p(f'restart_cameras.{k}') for k in RESTART_KEYS}
        self.restart = CameraRestart(rc, cameras, self._scripts_dir())
        self.timeout = float(self.p('input_timeout_s'))
        self.sub(SystemHealth, T.SYSTEM_HEALTH, self._on_health, 5)
        self.sub(UwbStatus, T.UWB_STATUS, lambda m: setattr(self, 'uwb_status', (time.monotonic(), m)), 5)
        self.sub(Float32, T.VEHICLE_BATTERY, lambda m: setattr(self, 'battery', (time.monotonic(), m.data)), 5)
        self.create_subscription(Bool, T.E_STOP, self._on_estop, 10)
        self.create_timer(1.0 / max(float(self.p('tick_hz')), 1.0), self._tick)
        self.create_timer(1.0 / max(float(self.p('live_rate_hz')), 0.2), self._publish_live)
        self.create_timer(1.0 / max(float(self.p('state_rate_hz')), 0.2), self._publish_state)
        self.create_timer(float(self.p('refresh_period_s')), self._refresh)
        where = self.wiz.session_name() or 'created on the first Save'
        self.get_logger().info(f'calibration wizard ready: {len(self.wiz.slots)} steps, data {root}, session {where}')
        self._publish_state()
        self._publish_live()
        self._status()

    @staticmethod
    def _scripts_dir():
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('carbot_bringup'), 'scripts')
        except Exception:  # noqa: BLE001
            return ''

    # ------------------------------------------------------------------ inputs
    def _on_health(self, m):
        self.health = (time.monotonic(), m)
        self.health_seq += 1

    def _on_estop(self, m):
        if m.data and self.wiz is not None:
            r = self.wiz.cancel_running('STOP MOTORS pressed')
            if r:
                self.get_logger().warn(r['message'])
                self._publish_all()

    def inputs(self):
        now = time.monotonic()
        snap = {'health_age_s': None, 'topics': {}, 'procs': [], 'agent': None, 'battery_v': None,
                'uwb': None, 'env': mc.env_network(self.domain)}
        if self.health is not None and now - self.health[0] <= self.timeout:
            t0, h = self.health
            dt = now - t0
            snap['health_age_s'] = dt
            # age as measured by system_monitor at report time; a silent monitor is caught by
            # input_timeout_s (health_age_s None -> 'wait'), not by adding dt to every sensor
            snap['topics'] = {t.topic: {'hz': t.rate_hz, 'age': t.age_s if t.age_s >= 0 else -1.0,
                                        'latency': t.latency_ms} for t in h.topics}
            snap['procs'] = list(zip(h.camera_process_names, h.camera_process_pids))
            snap['agent'] = bool(h.uwb_agent_running)
        if self.battery is not None and now - self.battery[0] <= self.timeout:
            snap['battery_v'] = float(self.battery[1])
        if self.uwb_status is not None and now - self.uwb_status[0] <= self.timeout:
            u = self.uwb_status[1]
            snap['uwb'] = {'link': bool(u.agent_link_ok), 'hz': float(u.rate_hz), 'unknown': u.last_unknown_id,
                           'anchors': {a: {'seen': bool(u.anchor_seen[i]) if i < len(u.anchor_seen) else False,
                                           'age': float(u.anchor_age_s[i]) if i < len(u.anchor_age_s) else -1.0}
                                       for i, a in enumerate(u.anchor_ids)}}
        return {'snap': snap, 'health_seq': self.health_seq, 'frame': self.tap.frame if self.tap else None}

    # ------------------------------------------------------------------ loop
    def _tick(self):
        try:
            # camera images only while step 3 is capturing (raw frames cost CPU)
            impl = self.wiz.impls.get('camera_intrinsics')
            self.tap.want([impl.running_sensor()] if isinstance(impl, CameraIntrinsicsStep) and impl.running_sensor()
                          else [])
            done = self.wiz.tick(self.inputs())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'wizard tick failed: {e!r}\n{traceback.format_exc()}')
            return
        if done is not None:
            txt = f'step {done.index} {done.title}: {done.status} - {(done.result or {}).get("summary", "")}'
            (self.get_logger().info if done.status == 'PASS' else self.get_logger().warn)(txt)
            self._publish_all()

    def _refresh(self):
        try:
            if self.wiz.refresh():
                self.get_logger().info('picked up results written by a terminal calibration tool')
                self._publish_state()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'refresh failed: {e!r}')

    def _status(self):
        s = self.wiz.slot(self.wiz.current)
        self.set_status(NodeStatus.OK, 'RUNNING' if self.wiz.running else 'WAITING',
                        f'step {s.index} {s.title}: {s.status}' + (f' · session {self.wiz.session_name()}'
                                                                     if self.wiz.session else ''))

    def _publish_state(self):
        if self.wiz is None:
            return
        st = CalibrationState()
        st.header.stamp = self.get_clock().now().to_msg()
        d = self.wiz.state()
        st.session = d['session'] or '(not started)'
        st.current_index = int(d['current'])
        for x in d['steps']:
            e = CalibrationStepState()
            e.index, e.id, e.title, e.status = int(x['index']), x['id'], x['title'], x['status']
            e.required, e.can_advance = bool(x['required']), bool(x['can_advance'])
            e.result_summary, e.result_file, e.previous_session = x['summary'][:200], x['file'], x['previous']
            st.steps.append(e)
        self.pub_state.publish(st)
        self._status()

    def _publish_live(self):
        if self.wiz is None:
            return
        try:
            doc = self.wiz.live(self.inputs(), self.restart.snapshot())
            self.pub_live.publish(String(data=json.dumps(doc, default=str, separators=(',', ':'))))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'live view failed: {e!r}\n{traceback.format_exc()}')
            self.pub_live.publish(String(data=json.dumps({'error': f'live view failed: {e!r}'})))

    def _publish_error(self):
        self.pub_live.publish(String(data=json.dumps({'error': 'calibration_wizard configuration error: ' + self.error})))

    def _publish_all(self):
        self._publish_state()
        self._publish_live()

    # ------------------------------------------------------------------ service
    def _srv(self, req, resp):
        resp.ok, resp.passed, resp.result_yaml = False, False, ''
        try:
            if self.wiz is None:
                resp.message = 'calibration_wizard configuration error: ' + (self.error or 'not ready')
                return resp
            action = (req.action or '').upper()
            if action == 'RESTART_CAMERAS':
                if self.wiz.running is not None:
                    resp.message = 'A step is measuring: wait for it or cancel it before restarting cameras'
                    return resp
                err = self.restart.start()
                resp.ok = err is None
                resp.message = err or 'Restarting camera drivers (about 10 s): stale ones are killed first'
                if resp.ok:
                    self.get_logger().warn('GUI: restarting camera drivers')
            else:
                r = self.wiz.action(req.step_id, action, req.argument, self.inputs())
                resp.ok, resp.message, resp.passed, resp.result_yaml = r['ok'], r['message'], r['passed'], r['result_yaml']
                if action != 'SELECT':
                    (self.get_logger().info if r['ok'] else self.get_logger().warn)(
                        f'{action} {req.step_id}: {r["message"]}')
            self._publish_all()
        except Exception as e:  # noqa: BLE001  never let one request kill the node
            self.get_logger().error(f'action {req.action} {req.step_id} failed: {e!r}\n{traceback.format_exc()}')
            resp.ok = False
            resp.message = f'Internal error in calibration_wizard: {e!r} (details in the launch terminal)'
        return resp

    def destroy_node(self):
        if self.restart is not None:
            self.restart.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationWizard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
