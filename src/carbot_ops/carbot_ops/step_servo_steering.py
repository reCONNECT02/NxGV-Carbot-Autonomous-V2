"""Calibration step 7 -- servo centre + steering limits (pure, unit tested).

Same drives, analysis (carbot_control.calib_core) and outputs as the terminal tool
(carbot_control calib_steering), run from the wizard page. The car DRIVES ITSELF
through the command owner (CALIBRATION_RAW on /carbot/calibration/request; the owner
accepts it in calibrate mode only, never while armed, and the e-stop always wins).

RUN starts a supervised sequence. Before EVERY segment the step waits in phase
'ready' until the user presses Go (page op `go`, action STEP; allowed while RUNNING):

  left, right   full lock at procedure.raw_duty until the IMU turned circle_yaw_deg
                (or circle_max_distance_m) -> radius each side. Both locks turning the
                wrong way flips command_owner steering.steer_sign.
  straight 1..N angular.z 0 for straight_run_m: residual curvature -> integer
                servo_controller.servo_center correction (set LIVE, next run verifies),
                until the drift per metre passes, the correction is below one unit, or
                max_runs.

Each segment ends by commanding speed 0 for stop_settle_s (like calib_drive.Driver),
then the request stream stops (the owner's watchdog then holds zero). A segment
is aborted (step FAILS, car stopped) on timeout_s, or when /odom is older than
odom_timeout_s while driving. Cancel / STOP MOTORS stop the requests at once.

IMU yaw is measured with imu_yaw_scale set to +-1 (sign kept) and multiplied by
|scale|: servo_controller re-wraps yaw after scaling (BACKLOG #31), which would
corrupt a 180 deg circle crossing +-180. The scale is restored when the run ends.

Save writes, like the terminal tool: servo_controller.servo_center,
command_owner.steering.{steer_sign,left_max_rad,right_max_rad,trim_rad,angular_limit},
the tunnel_bridge.command_owner_steering.* mirror and '/**' vehicle.min_turning_radius_m.
The steering values apply from the next launch of the session (servo_center is live now).
"""
import math
import os
import time
from typing import Callable, Dict, List, Optional

from carbot_common import calib_tools as ct
from carbot_control import calib_core as cc

from .sensor_checks import ConfigError
from .servo_link import SERVO
from .step_imu_odometry import MotionRecorder
from .wizard_core import StepImpl, StepRefused

OWNER, BRIDGE, COMMON = 'command_owner', 'tunnel_bridge', '/**'
PROC_KEYS = ('raw_duty', 'circle_yaw_deg', 'circle_max_distance_m', 'straight_run_m', 'max_runs', 'timeout_s',
             'stop_settle_s', 'imu_settle_s', 'odom_timeout_s',
             'stall_grace_s', 'stall_window_s', 'stall_min_progress_m', 'stall_boost_step', 'stall_boost_max_duty')
PASS_KEYS = ('straight_drift_m_per_m', 'min_radius_m_max')
SERVO_PARAMS = ('servo_center', 'servo_range_left', 'servo_range_right', 'imu_yaw_scale')
OWNER_PARAMS = ('mode', 'steering.steer_sign')
STEER_KEYS = ('steer_sign', 'left_max_rad', 'right_max_rad', 'trim_rad', 'angular_limit')
READ_RETRY_S = 2.0
SOURCE = 'CALIBRATION_RAW'
LABEL = {'left': 'LEFT circle (full lock)', 'right': 'RIGHT circle (full lock)', 'straight': 'straight run'}


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


class ServoSteeringStep(StepImpl):
    can_keep_previous = True
    ops_while_running = True           # Go between segments

    def __init__(self, cfg: Dict, recorder: MotionRecorder, servo, owner, drive, wheelbase_m: float,
                 clock: Callable[[], float] = time.monotonic):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml servo_steering.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml servo_steering.pass')
        self.rec, self.servo, self.owner, self.drive, self.clock = recorder, servo, owner, drive, clock
        self.wb = float(wheelbase_m)
        if self.wb <= 0:
            raise ConfigError('common.yaml vehicle.wheelbase_m must be > 0')
        p = self.proc
        self.duty, self.max_runs = float(p['raw_duty']), int(p['max_runs'])
        self.target = math.radians(float(p['circle_yaw_deg']))
        self.circle_max, self.straight_m = float(p['circle_max_distance_m']), float(p['straight_run_m'])
        self.timeout, self.stop_s = float(p['timeout_s']), float(p['stop_settle_s'])
        self.imu_settle, self.odom_to = float(p['imu_settle_s']), float(p['odom_timeout_s'])
        # stall boost: while the odometry shows (almost) no progress the duty is raised a little, capped
        self.stall_grace, self.stall_window = float(p['stall_grace_s']), float(p['stall_window_s'])
        self.stall_min, self.boost_step = float(p['stall_min_progress_m']), float(p['stall_boost_step'])
        self.boost_max = float(p['stall_boost_max_duty'])
        if self.stall_window <= 0 or self.boost_step < 0 or self.boost_max < self.duty:
            raise ConfigError('calibration_steps.yaml servo_steering.procedure: stall_window_s must be > 0, '
                              'stall_boost_step >= 0 and stall_boost_max_duty >= raw_duty')
        self.drift_lim, self.radius_lim = float(self.pass_cfg['straight_drift_m_per_m']), float(self.pass_cfg['min_radius_m_max'])
        self.values: Optional[Dict] = None       # servo_controller + command_owner values
        self.busy, self.link_error, self.read_at = '', '', -math.inf
        self.run: Optional[Dict] = None

    # ------------------------------------------------------------------ parameter links
    def _read(self) -> None:
        self.busy, self.read_at = 'reading servo_controller / command_owner parameters', self.clock()
        got: Dict = {}

        def both():
            if 'servo' not in got or 'owner' not in got:
                return
            self.busy = ''
            s, o = got['servo'], got['owner']
            miss = ([f'{SERVO}.{k}' for k in SERVO_PARAMS if not s or s.get(k) is None] +
                    [f'{OWNER}.{k}' for k in OWNER_PARAMS if not o or o.get(k) is None])
            if miss:
                self.link_error = f'Cannot read {", ".join(miss)}: is calibrate.launch.py running?'
                return
            self.link_error = ''
            self.values = {'servo_center': int(s['servo_center']), 'servo_range_left': int(s['servo_range_left']),
                           'servo_range_right': int(s['servo_range_right']), 'imu_yaw_scale': float(s['imu_yaw_scale']),
                           'mode': str(o['mode']), 'steer_sign': float(o['steering.steer_sign'])}
        self.servo.get(list(SERVO_PARAMS), lambda v: (got.__setitem__('servo', v), both()))
        self.owner.get(list(OWNER_PARAMS), lambda v: (got.__setitem__('owner', v), both()))

    def _set(self, values: Dict, why: str, then: Optional[Callable[[bool], None]] = None) -> None:
        self.busy = f'setting {", ".join(f"{k}={v}" for k, v in values.items())} on {SERVO}'

        def done(ok):
            self.busy = ''
            if ok:
                self.link_error = ''
                if self.values is not None:
                    self.values.update(values)
            else:
                self.link_error = f'{SERVO} did not accept {", ".join(values)} ({why}).'
            if then is not None:
                then(ok)
        self.servo.set(dict(values), done)

    # ------------------------------------------------------------------ run
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        if self.busy:
            return f'Wait: {self.busy}.'
        if self.values is None:
            self._read()
            return (self.link_error or 'Reading the current values from servo_controller and command_owner') + \
                ' Try again in a moment.'
        if self.values['mode'] != 'calibrate':
            return (f'command_owner runs in {self.values["mode"]!r} mode: the car only drives for calibration '
                    'under calibrate.launch.py.')
        s = self.values['imu_yaw_scale']
        self.run = {'seg': 0, 'plan': ['left', 'right', 'straight'], 'phase': 'ready', 'cap': {
            'servo_center': self.values['servo_center'], 'steer_sign': self.values['steer_sign'], 'straight': []},
            'scale': s, 'unit': -1.0 if s < 0 else 1.0, 'abort': '', 'kl': None, 'kr': None, 'result': None,
            'note': ''}
        return None

    def _seg(self) -> str:
        return self.run['plan'][min(self.run['seg'], len(self.run['plan']) - 1)]

    def _steer(self, seg: str) -> float:
        sign = self.run['cap']['steer_sign']
        # base angular.z for a physical LEFT lock = steer_sign (default -1: negative = left)
        return sign if seg == 'left' else -sign if seg == 'right' else 0.0

    def handle(self, op: str, args: Dict, inputs: Dict) -> Dict:
        r = self.run
        if op == 'reread':
            if r is not None:
                return {'ok': False, 'message': 'Not while the calibration drive runs.'}
            if self.busy:
                return {'ok': False, 'message': f'Wait: {self.busy}.'}
            self._read()
            return {'ok': True, 'message': 'Reading the current values.'}
        if op != 'go':
            return {'ok': False, 'message': f'Unknown operation {op!r}'}
        if r is None:
            return {'ok': False, 'message': 'Press Run first.'}
        if r['phase'] != 'ready':
            return {'ok': False, 'message': 'Wait: the car is still driving or stopping.'}
        if self.busy:
            return {'ok': False, 'message': f'Wait: {self.busy}.'}
        seg = self._seg()
        r['phase'] = 'arming'

        def armed(ok):
            if self.run is not r:
                return
            if not ok:
                r['abort'] = 'could not set imu_yaw_scale for the measurement'
                return
            r['phase'], r['t'] = 'settling', self.clock()
            self.rec.zero(r['t'] + self.imu_settle)
        if self.values['imu_yaw_scale'] != r['unit']:
            self._set({'imu_yaw_scale': r['unit']}, 'unit scale for the measurement', armed)
        else:
            armed(True)
        return {'ok': True, 'message': f'Driving the {LABEL[seg]}: keep the e-stop ready.'}

    def _yaw_rad(self) -> float:
        return math.radians(self.rec.yaw * abs(self.run['scale']))

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        if r['abort']:
            return self._finish()
        seg = self._seg()
        if r['phase'] == 'settling' and now >= self.rec.ignore_until:
            r['phase'], r['t'] = 'driving', now
            r['duty'], r['boosts'] = self.duty, 0
            r['prog_t'], r['prog_d'] = now, abs(self.rec.dist)
            self.drive.command(SOURCE, r['duty'], self._steer(seg), f'step 7 {seg}')
        elif r['phase'] == 'driving':
            d, y, el = self.rec.dist, self._yaw_rad(), now - r['t']
            if self.rec.odom_t is None or now - self.rec.odom_t > self.odom_to:
                r['abort'] = f'/odom stopped while driving the {LABEL[seg]}'
                return self._finish()
            if el > self.timeout:
                r['abort'] = f'{LABEL[seg]} not finished within {self.timeout:g} s (stuck, or the floor too small?)'
                return self._finish()
            done = (abs(d) >= self.straight_m if seg == 'straight'
                    else abs(y) >= self.target or abs(d) >= self.circle_max)
            if not done:
                self._stall_boost(r, seg, now, abs(d), el)
            if done:
                r['phase'], r['t'] = 'stopping', now
                self.drive.command(SOURCE, 0.0, self._steer(seg), 'stop')
        elif r['phase'] == 'stopping' and now - r['t'] >= self.stop_s:
            self.drive.stop()
            return self._segment_done(seg, self.rec.dist, self._yaw_rad())
        return None

    def _stall_boost(self, r: Dict, seg: str, now: float, dist: float, elapsed: float) -> None:
        """The car may need more than raw_duty to get going (full lock drags, static friction, a weak battery).
        Every stall_window_s in which the odometry moved less than stall_min_progress_m, after stall_grace_s of
        the segment, raise the duty by stall_boost_step, never above stall_boost_max_duty (the command owner
        caps calibration duty at calibration.duty_max anyway). Starts again from raw_duty on every segment."""
        if dist - r['prog_d'] >= self.stall_min:
            r['prog_t'], r['prog_d'] = now, dist                 # moving: restart the window
            return
        if elapsed < self.stall_grace or now - r['prog_t'] < self.stall_window:
            return
        r['prog_t'] = now                                        # next check after another window
        if r['duty'] + 1e-9 >= self.boost_max or self.boost_step <= 0:
            return
        r['duty'] = min(self.boost_max, round(r['duty'] + self.boost_step, 4))
        r['boosts'] += 1
        self.drive.command(SOURCE, r['duty'], self._steer(seg), f'step 7 {seg} (stall boost)')

    def _segment_done(self, seg: str, d: float, y: float) -> Optional[Dict]:
        r, cap = self.run, self.run['cap']
        if seg in ('left', 'right'):
            cap[seg] = {'distance': d, 'yaw': y}
            r['seg'] += 1
            if seg == 'right':
                self._after_circles()
            r['phase'] = 'ready'
            return None
        centre = cap['servo_center']
        cap['straight'].append({'distance': d, 'yaw': y, 'servo_center': centre})
        s = cc.straight_result(d, y)
        if s.drift_m_per_m <= self.drift_lim or len(cap['straight']) >= self.max_runs:
            return self._finish()
        du = cc.centre_correction(s.curvature, r['kl'], r['kr'])
        if du == 0:
            r['note'] = 'drift below one servo unit: cannot correct further'
            return self._finish()
        cap['servo_center'] = centre + du
        r['phase'] = 'correcting'

        def done(ok):
            if self.run is not r:
                return
            if ok:
                r['phase'] = 'ready'
            else:
                r['abort'] = f'{SERVO} did not accept servo_center {centre + du}'
        self._set({'servo_center': centre + du}, 'servo centre correction', done)
        return None

    def _after_circles(self) -> None:
        r, cap = self.run, self.run['cap']
        L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], self.wb)
        R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], self.wb)
        if not L.turned_correct_way and not R.turned_correct_way:
            cap['steer_sign'] = -cap['steer_sign']
            for s in ('left', 'right'):
                cap[s]['yaw'] = -cap[s]['yaw']
            r['note'] = f'both locks turned the wrong way: steer_sign -> {cap["steer_sign"]:+g}'
            L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], self.wb)
            R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], self.wb)
        r['kl'] = (1.0 / L.radius_m) / max(self.values['servo_range_left'], 1)
        r['kr'] = (1.0 / R.radius_m) / max(self.values['servo_range_right'], 1)

    def _restore_scale(self) -> None:
        r = self.run
        if r is not None and self.values is not None and self.values['imu_yaw_scale'] != r['scale']:
            self._set({'imu_yaw_scale': r['scale']}, 'restore after the calibration drive')

    def cancel(self) -> None:
        self.drive.stop()
        self._restore_scale()
        self.run = None

    def _finish(self) -> Dict:
        self.drive.stop()
        self._restore_scale()
        r, self.run = self.run, None
        return self._result(r)

    # ------------------------------------------------------------------ result
    def _result(self, r: Dict) -> Dict:
        cap = r['cap']
        checks: List[Dict] = []
        doc: Dict = {'wheelbase_m': self.wb, 'servo_center': cap['servo_center'], 'note': r['note']}
        if r['abort']:
            checks.append({'key': 'drive', 'label': 'Calibration drive completed', 'measured': 'aborted',
                           'limit': 'all segments', 'passed': False, 'why': r['abort'],
                           'fix': 'Clear the floor, check the car can move freely and /odom is live, then Redo.'})
        if 'left' in cap and 'right' in cap:
            L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], self.wb)
            R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], self.wb)
            runs = [cc.straight_result(x['distance'], x['yaw']) for x in cap['straight']]
            summ = cc.steering_summary(L, R, runs[-1] if runs else None, self.pass_cfg)
            steering = {'steer_sign': cap['steer_sign'], 'left_max_rad': round(summ['left_max_rad'], 4),
                        'right_max_rad': round(summ['right_max_rad'], 4), 'trim_rad': 0.0, 'angular_limit': 1.0}
            doc.update(left=L.as_dict(), right=R.as_dict(), straight_runs=[x.as_dict() for x in runs],
                       min_turning_radius_m=round(summ['min_turning_radius_m'], 4), steering=steering)
            ch = summ['checks']
            checks.append({'key': 'direction', 'label': 'Both locks turn the right way', 'passed': ch['direction'],
                           'measured': f'left {math.degrees(L.yaw_change_rad):+.0f} deg, right {math.degrees(R.yaw_change_rad):+.0f} deg',
                           'limit': 'left +, right -',
                           'why': '' if ch['direction'] else 'only one lock turned the expected way',
                           'fix': '' if ch['direction'] else 'Check the servo linkage and the IMU sign (step 6), then Redo.'})
            checks.append({'key': 'min_radius', 'label': 'Turning radius (larger side)', 'passed': ch['min_radius'],
                           'measured': f'L {L.radius_m:.3f} m, R {R.radius_m:.3f} m', 'limit': f'<= {self.radius_lim:g} m',
                           'why': '' if ch['min_radius'] else 'the car turns wider than the track needs',
                           'fix': '' if ch['min_radius'] else ('Raise servo_range_left / right in the Tuning tab (stop if the '
                                                               'servo strains or buzzes at full lock), then Redo.')})
            last = runs[-1] if runs else None
            checks.append({'key': 'straight', 'label': 'Straight: lateral drift per metre', 'passed': ch['straight'],
                           'measured': f'{last.drift_m_per_m * 100:.2f} cm/m ({len(runs)} run{"s" if len(runs) != 1 else ""})' if last else 'not run',
                           'limit': f'<= {self.drift_lim * 100:g} cm/m',
                           'why': '' if ch['straight'] else (r['note'] or 'still drifting after the last run'),
                           'fix': '' if ch['straight'] else 'Check the wheels and the floor are straight and level, then Redo.'})
        passed = not r['abort'] and 'steering' in doc and all(c['passed'] for c in checks)
        if passed:
            summary = (f'servo_center {cap["servo_center"]}, radius L {doc["left"]["radius_m"]:.3f} / '
                       f'R {doc["right"]["radius_m"]:.3f} m, drift {doc["straight_runs"][-1]["drift_m_per_m"] * 100:.2f} cm/m')
        else:
            summary = 'failed: ' + (r['abort'] or ', '.join(c['label'].lower() for c in checks if not c['passed']))
        return cc.plain(dict(doc, passed=passed, summary=summary, checks=checks))

    # ------------------------------------------------------------------ save / keep
    @staticmethod
    def _overlays(servo_center: int, steering: Dict, radius: float) -> Dict[str, Dict]:
        return {SERVO: {'servo_center': int(servo_center)},
                OWNER: {f'steering.{k}': steering[k] for k in STEER_KEYS},
                BRIDGE: {f'command_owner_steering.{k}': steering[k] for k in STEER_KEYS},
                COMMON: {'vehicle.min_turning_radius_m': float(radius)}}

    def _write(self, session: str, ov: Dict[str, Dict]) -> List[str]:
        path = ''
        for node, params in ov.items():
            path = ct.merge_overlay(session, node, params)
        return [path]

    def save_data(self, session: str, res: Dict) -> List[str]:
        if not res.get('steering') or res.get('servo_center') is None:
            raise StepRefused('The result has no steering values: run the step again.')
        if self.values is not None and self.values['servo_center'] != res['servo_center']:
            raise StepRefused(f'servo_controller.servo_center is {self.values["servo_center"]} now, not the verified '
                              f'{res["servo_center"]}: run the step again.')
        return self._write(session, self._overlays(res['servo_center'], res['steering'], res['min_turning_radius_m']))

    def keep_data(self, src_session: str, session: str) -> List[str]:
        centre = ct.overlay_value(src_session, SERVO, 'servo_center')
        steering = {k: ct.overlay_value(src_session, OWNER, f'steering.{k}') for k in STEER_KEYS}
        radius = ct.overlay_value(src_session, COMMON, 'vehicle.min_turning_radius_m')
        miss = ([f'{SERVO}.servo_center'] if centre is None else []) + \
               [f'{OWNER}.steering.{k}' for k, v in steering.items() if v is None] + \
               (['vehicle.min_turning_radius_m'] if radius is None else [])
        if miss:
            raise StepRefused(f'{os.path.basename(src_session)}/params_overlay.yaml has no {", ".join(miss)}: '
                              'run this step again.')
        paths = self._write(session, self._overlays(int(centre), steering, float(radius)))
        self._set({'servo_center': int(centre)}, 'keep previous value')
        return paths

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        now = self.clock()
        if self.values is None and not self.busy and self.run is None and now - self.read_at >= READ_RETRY_S:
            self._read()
        r = self.run
        run = None
        if r is not None:
            seg = self._seg()
            cap = r['cap']
            run = {'phase': r['phase'], 'segment': seg, 'label': LABEL[seg],
                   'straight_n': len(cap['straight']) + (1 if seg == 'straight' else 0), 'max_runs': self.max_runs,
                   'distance_m': round(self.rec.dist, 3), 'yaw_deg': round(math.degrees(self._yaw_rad()), 1),
                   'elapsed_s': round(now - r['t'], 1) if 't' in r else 0.0, 'note': r['note'],
                   'duty': r.get('duty', self.duty), 'boosts': r.get('boosts', 0), 'base_duty': self.duty,
                   'servo_center': cap['servo_center'], 'steer_sign': cap['steer_sign'],
                   'circles': {s: {'distance_m': round(cap[s]['distance'], 3),
                                   'yaw_deg': round(math.degrees(cap[s]['yaw']), 1),
                                   'radius_m': round(abs(cap[s]['distance']) / max(abs(cap[s]['yaw']), 1e-6), 3)}
                               for s in ('left', 'right') if s in cap},
                   'straights': [{'distance_m': round(x['distance'], 3), 'servo_center': x['servo_center'],
                                  'drift_cm_per_m': round(cc.straight_result(x['distance'], x['yaw']).drift_m_per_m * 100, 2)}
                                 for x in cap['straight']]}
        return {'values': self.values, 'busy': self.busy, 'link_error': self.link_error, 'run': run,
                'ages': self.rec.ages(now), 'wheelbase_m': self.wb,
                'procedure': {'raw_duty': self.duty, 'circle_yaw_deg': math.degrees(self.target),
                              'straight_run_m': self.straight_m, 'max_runs': self.max_runs, 'timeout_s': self.timeout},
                'limits': {'straight_drift_cm_per_m': self.drift_lim * 100, 'min_radius_m_max': self.radius_lim}}
