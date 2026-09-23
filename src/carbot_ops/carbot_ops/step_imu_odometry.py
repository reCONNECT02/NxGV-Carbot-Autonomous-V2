"""Calibration step 6 -- IMU + wheel odometry (pure, unit tested).

Same three tests and the same corrections as the terminal tool
(carbot_localization calib_odometry), driven from the wizard page. The car is
pushed / turned BY HAND: nothing here commands the motors.

  distance  page ops distance_start / distance_stop (action STEP). Rear axle on
            the first tape mark -> Start, push straight to the second -> Stop.
            Negative distance toggles odom_reverse_polarity, an error above
            pass.max_distance_error_pct rescales ticks_per_meter; both are set
            LIVE on servo_controller and the run does not pass: the next run
            with the new value is the verify run.
  spin      page ops spin_start / spin_stop. One full turn to the LEFT by hand.
            Measured with imu_yaw_scale set to +-1 (its sign kept): servo_controller
            re-normalises yaw AFTER scaling, so with |scale| != 1 the published yaw
            jumps by 360 * (1 - |scale|) at +-180 deg and a full turn would always
            read 360. The yaw at the current scale is |scale| * measured; a wrong
            sign or an error above pass.max_spin_error_pct sets scale = sign * 360 /
            measured (verify spin needed). The final scale is set back after the spin.
  drift     RUN (= the wizard's measurement, e-stop cancels it): hands off for
            procedure.drift_test_s, IMU yaw drift below pass.max_heading_drift_deg_per_min.
            Refused until distance and spin (if procedure.spin_test) have passed, and it
            produces the step result: pass = the LATEST distance run, spin and this drift
            all passed. A passing run was measured with the values it saves.

Save writes <session>/params_overlay.yaml servo_controller.{ticks_per_meter,
odom_reverse_polarity, imu_yaw_scale} (calib_tools.merge_overlay, the same file the
terminal tool writes). Keep previous copies those three from the older session's
overlay and sets them live, so step 7 in the same launch uses them.

The node feeds MotionRecorder from /odom and /imu/rpy, and gives a ServoLink
(servo_link.py; get/set with callbacks, never blocking).
"""
import math
import os
import time
from typing import Callable, Dict, List, Optional

from carbot_common import calib_tools as ct

from .sensor_checks import ConfigError
from .servo_link import SERVO
from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('straight_run_m', 'spin_test', 'drift_test_s', 'max_runs', 'min_rate_hz',
             'distance_min_fraction', 'spin_min_fraction', 'settle_s')
PASS_KEYS = ('max_distance_error_pct', 'max_heading_drift_deg_per_min', 'max_spin_error_pct')
PARAMS = ('ticks_per_meter', 'odom_reverse_polarity', 'imu_yaw_scale')
READ_RETRY_S = 2.0


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class MotionRecorder:
    """Signed wheel distance from /odom and unwrapped IMU yaw (deg) from /imu/rpy.
    t = the wizard's monotonic clock. Samples before ignore_until only set the baseline
    (lets a just-changed imu_yaw_scale settle without counting its jump)."""

    def __init__(self):
        self.prev = None
        self.dist = 0.0
        self.odom_n = 0
        self.odom_t: Optional[float] = None
        self.yaw_prev: Optional[float] = None
        self.yaw = 0.0
        self.imu_n = 0
        self.imu_t: Optional[float] = None
        self.ignore_until = -math.inf

    def on_odom(self, t: float, x: float, y: float, yaw_rad: float) -> None:
        # = carbot_localization.estimator_core.odom_increment: the base integrates
        # x += v cos(yaw) dt, so the step projected on the heading is the signed distance
        if self.prev is not None and t >= self.ignore_until:
            self.dist += (x - self.prev[0]) * math.cos(yaw_rad) + (y - self.prev[1]) * math.sin(yaw_rad)
            self.odom_n += 1
        self.prev, self.odom_t = (x, y), t

    def on_imu(self, t: float, yaw_deg: float) -> None:
        if self.yaw_prev is not None and t >= self.ignore_until:
            self.yaw += wrap_deg(yaw_deg - self.yaw_prev)
            self.imu_n += 1
        self.yaw_prev, self.imu_t = yaw_deg, t

    def zero(self, ignore_until: float = -math.inf) -> None:
        self.dist = self.yaw = 0.0
        self.odom_n = self.imu_n = 0
        self.ignore_until = ignore_until

    def ages(self, now: float) -> Dict[str, Optional[float]]:
        return {'odom': None if self.odom_t is None else round(now - self.odom_t, 2),
                'imu': None if self.imu_t is None else round(now - self.imu_t, 2)}


class ImuOdometryStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, recorder: MotionRecorder, link, clock: Callable[[], float] = time.monotonic):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml imu_odometry.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml imu_odometry.pass')
        self.rec, self.link, self.clock = recorder, link, clock
        self.true_m = float(self.proc['straight_run_m'])
        self.drift_s = float(self.proc['drift_test_s'])
        self.spin_test = bool(self.proc['spin_test'])
        self.max_runs = int(self.proc['max_runs'])
        self.min_rate = float(self.proc['min_rate_hz'])
        self.settle_s = float(self.proc['settle_s'])
        self.dist_lim = float(self.pass_cfg['max_distance_error_pct'])
        self.drift_lim = float(self.pass_cfg['max_heading_drift_deg_per_min'])
        self.spin_lim = float(self.pass_cfg['max_spin_error_pct'])
        if self.true_m <= 0 or self.drift_s <= 0:
            raise ConfigError('calibration_steps.yaml imu_odometry.procedure: straight_run_m and drift_test_s must be > 0')
        self.values: Optional[Dict] = None      # servo_controller's live values (None = not read yet)
        self.busy = ''                          # a parameter request in flight
        self.link_error = ''
        self.read_at = -math.inf
        self.active: Optional[Dict] = None      # distance / spin test in progress
        self.runs: Dict[str, List[Dict]] = {'distance': [], 'spin': []}
        self.t0: Optional[float] = None         # drift (RUN)

    # ------------------------------------------------------------------ servo_controller link
    def _read(self) -> None:
        self.busy, self.read_at = 'reading servo_controller parameters', self.clock()

        def done(v):
            self.busy = ''
            if v is None or any(v.get(k) is None for k in PARAMS):
                self.link_error = (f'Cannot read {", ".join(PARAMS)} from {SERVO}: is calibrate.launch.py '
                                   'running (servo_controller up)?')
                return
            self.link_error = ''
            self.values = {'ticks_per_meter': float(v['ticks_per_meter']),
                           'odom_reverse_polarity': bool(v['odom_reverse_polarity']),
                           'imu_yaw_scale': float(v['imu_yaw_scale'])}
        self.link.get(list(PARAMS), done)

    def _set(self, values: Dict, why: str, then: Optional[Callable[[bool], None]] = None) -> None:
        self.busy = f'setting {", ".join(f"{k}={v}" for k, v in values.items())} on {SERVO}'

        def done(ok):
            self.busy = ''
            if ok:
                self.link_error = ''
                if self.values is not None:
                    self.values.update(values)
            else:
                self.link_error = (f'{SERVO} did not accept {", ".join(values)} ({why}): the live value is '
                                   'unchanged. Check calibrate.launch.py is running, then press Re-read.')
            if then is not None:
                then(ok)
        self.link.set(dict(values), done)

    def _ready(self) -> Optional[str]:
        if self.busy:
            return f'Wait: {self.busy}.'
        if self.values is None:
            if not self.busy:
                self._read()
            return (self.link_error or f'Reading the current values from {SERVO}') + ' Try again in a moment.'
        return None

    # ------------------------------------------------------------------ page operations (action STEP)
    def handle(self, op: str, args: Dict, inputs: Dict) -> Dict:
        now = self.clock()
        if op == 'reread':
            if self.busy:
                return {'ok': False, 'message': f'Wait: {self.busy}.'}
            self._read()
            return {'ok': True, 'message': f'Reading the current values from {SERVO}.'}
        if op == 'abort':
            return self._abort()
        if op in ('distance_start', 'spin_start'):
            if self.active is not None:
                return {'ok': False, 'message': f'The {self.active["test"]} test is in progress: press Stop or Abort first.'}
            err = self._ready()
            if err:
                return {'ok': False, 'message': err}
            if op == 'distance_start':
                self.rec.zero(now)
                self.active = {'test': 'distance', 't0': now, 'phase': 'measuring'}
                return {'ok': True, 'message': f'Distance run started: push the car straight {self.true_m:.2f} m '
                                               'to the second mark, then press Stop.'}
            return self._spin_start(now)
        if op in ('distance_stop', 'spin_stop'):
            test = op.split('_')[0]
            if self.active is None or self.active['test'] != test:
                return {'ok': False, 'message': f'No {test} test is in progress: press Start first.'}
            if self.active['phase'] != 'measuring':
                return {'ok': False, 'message': 'Wait: the IMU scale is still being set for the spin.'}
            return self._distance_stop(now) if test == 'distance' else self._spin_stop(now)
        return {'ok': False, 'message': f'Unknown operation {op!r}'}

    def _abort(self) -> Dict:
        a, self.active = self.active, None
        if a is None:
            return {'ok': True, 'message': 'Nothing in progress.'}
        if a['test'] == 'spin' and a.get('restore') is not None:
            self._set({'imu_yaw_scale': a['restore']}, 'restore after an aborted spin')
        return {'ok': True, 'message': f'{a["test"].capitalize()} test aborted; nothing recorded.'}

    def _rate_ok(self, n: int, dur: float) -> bool:
        return n >= max(2, int(self.min_rate * dur))

    def _distance_stop(self, now: float) -> Dict:
        a, self.active = self.active, None
        d, n, dur = self.rec.dist, self.rec.odom_n, now - a['t0']
        tpm, pol = self.values['ticks_per_meter'], self.values['odom_reverse_polarity']
        run = {'odom_m': round(d, 4), 'true_m': self.true_m, 'seconds': round(dur, 1), 'odom_msgs': n,
               'ticks_per_meter': tpm, 'reverse_polarity': pol}
        err = (d - self.true_m) / self.true_m * 100.0
        if not self._rate_ok(n, dur):
            run.update(status='NO_DATA', why=f'only {n} /odom messages in {dur:.1f} s',
                       fix='Check the /odom row in step 1 (servo_controller running, encoder cable), then redo the run.')
        elif d < 0:
            run.update(error_pct=round(err, 2), status='CORRECTED', corrected={'odom_reverse_polarity': not pol},
                       why='the distance counted BACKWARDS while you pushed forward',
                       fix='odom_reverse_polarity was toggled: do the run again to verify.')
            self._set({'odom_reverse_polarity': not pol}, 'encoder direction')
        elif abs(err) <= self.dist_lim:
            run.update(error_pct=round(err, 2), status='PASS')
        elif d < float(self.proc['distance_min_fraction']) * self.true_m:
            run.update(error_pct=round(err, 2), status='FAIL',
                       why=f'odom counted only {d:.2f} m of {self.true_m:.2f} m',
                       fix='Is the drive wheel turning on the floor (car not lifted) and drive_motor_index right? '
                           'Nothing was changed; redo the run.')
        else:
            new = round(tpm * d / self.true_m, 2)
            run.update(error_pct=round(err, 2), status='CORRECTED', corrected={'ticks_per_meter': new},
                       why=f'odom {d:.3f} m vs tape {self.true_m:.2f} m ({err:+.1f} %)',
                       fix=f'ticks_per_meter {tpm:g} -> {new:g}: do the run again to verify.')
            self._set({'ticks_per_meter': new}, 'distance correction')
        self.runs['distance'].append(run)
        return {'ok': True, 'message': self._run_message('Distance', run)}

    def _spin_start(self, now: float) -> Dict:
        s = self.values['imu_yaw_scale']
        u = -1.0 if s < 0 else 1.0
        self.active = {'test': 'spin', 't0': now, 'phase': 'measuring', 'scale': s, 'unit': u,
                       'restore': None if s == u else s}
        if s == u:
            self.rec.zero(now + self.settle_s)
            return {'ok': True, 'message': 'Spin started: turn the car ONE full turn to the LEFT, back onto the mark, then press Stop.'}
        self.active['phase'] = 'arming'
        a = self.active

        def then(ok):
            if self.active is not a:
                return
            if not ok:
                self.active = None
                return
            a['phase'], a['t0'] = 'measuring', self.clock()
            self.rec.zero(a['t0'] + self.settle_s)
        self._set({'imu_yaw_scale': u}, 'unit scale for the spin measurement', then)
        return {'ok': True, 'message': f'Setting imu_yaw_scale to {u:+g} for the measurement; start turning once the page says so.'}

    def _spin_stop(self, now: float) -> Dict:
        a, self.active = self.active, None
        m, n, dur = self.rec.yaw, self.rec.imu_n, now - a['t0']
        s, u = a['scale'], a['unit']
        final = s
        run = {'yaw_change_deg': round(m, 2), 'measured_at_scale': u, 'imu_yaw_scale': s,
               'seconds': round(dur, 1), 'imu_msgs': n}
        eff = abs(s) * m
        err = (abs(eff) - 360.0) / 360.0 * 100.0
        if not self._rate_ok(n, dur):
            run.update(status='NO_DATA', why=f'only {n} /imu/rpy messages in {dur:.1f} s',
                       fix='Check the IMU row in step 1 (servo_controller running), then redo the spin.')
        elif abs(m) < float(self.proc['spin_min_fraction']) * 360.0:
            run.update(status='FAIL', why=f'the IMU turned only {m:+.0f} deg',
                       fix='Turn the car a FULL turn back onto the tape mark (or the IMU is not updating). '
                           'Nothing was changed; redo the spin.')
        elif eff > 0 and abs(err) <= self.spin_lim:
            run.update(status='PASS', yaw_at_scale_deg=round(eff, 2), error_pct=round(err, 2))
        else:
            final = round(u * 360.0 / m, 4)
            why = ('yaw DECREASED turning left (sign flipped)' if eff < 0
                   else f'the IMU read {eff:.1f} deg for 360 ({err:+.1f} %)')
            run.update(status='CORRECTED', yaw_at_scale_deg=round(eff, 2), error_pct=round(err, 2),
                       corrected={'imu_yaw_scale': final}, why=why,
                       fix=f'imu_yaw_scale {s:+g} -> {final:+g}: do the spin again to verify.')
        if final != u:
            self._set({'imu_yaw_scale': final}, 'spin correction' if final != s else 'restore after the spin')
        self.runs['spin'].append(run)
        return {'ok': True, 'message': self._run_message('Spin', run)}

    @staticmethod
    def _run_message(name: str, run: Dict) -> str:
        if run['status'] == 'PASS':
            return f'{name} test passed.'
        return f'{name} run: {run.get("why", run["status"])}. {run.get("fix", "")}'.strip()

    # ------------------------------------------------------------------ test state
    def _latest(self, test: str) -> Optional[Dict]:
        return self.runs[test][-1] if self.runs[test] else None

    def _test_passed(self, test: str) -> bool:
        r = self._latest(test)
        return bool(r and r['status'] == 'PASS')

    def _needed(self) -> List[str]:
        return ['distance'] + (['spin'] if self.spin_test else [])

    # ------------------------------------------------------------------ drift = RUN
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        if self.active is not None:
            return f'The {self.active["test"]} test is in progress: press Stop or Abort first.'
        err = self._ready()
        if err:
            return err
        todo = [t for t in self._needed() if not self._test_passed(t)]
        if todo:
            return (f'Pass the {" and ".join(todo)} test{"s" if len(todo) > 1 else ""} first: the drift test '
                    'finishes this step and needs them.')
        self.rec.zero(now + self.settle_s)       # a scale restored right after a spin must not count
        self.t0 = now
        return None

    def cancel(self) -> None:
        self.t0 = None

    def progress(self, now: float) -> Dict:
        if self.t0 is None:
            return {}
        el = now - self.t0
        return {'elapsed_s': round(el, 1), 'remaining_s': round(max(0.0, self.drift_s - el), 1),
                'fraction': round(min(1.0, el / self.drift_s), 2), 'samples': self.rec.imu_n,
                'drift_deg': round(self.rec.yaw, 2)}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        if self.t0 is None or now - self.t0 < self.drift_s:
            return None
        dur, n, yaw = now - self.t0 - self.settle_s, self.rec.imu_n, self.rec.yaw
        self.t0 = None
        return self._result(dur, n, yaw)

    def _result(self, dur: float, n: int, yaw: float) -> Dict:
        rate_ok = self._rate_ok(n, dur)
        drift = yaw / dur * 60.0
        drift_ok = rate_ok and abs(drift) <= self.drift_lim
        d, sp = self._latest('distance'), self._latest('spin')
        v = dict(self.values or {})
        checks = [{
            'key': 'distance', 'label': f'Distance ({self.true_m:.2f} m tape)',
            'measured': f'{d["odom_m"]:.3f} m ({d.get("error_pct", 0):+.1f} %)' if d else 'not run',
            'limit': f'<= {self.dist_lim:g} %', 'passed': self._test_passed('distance'),
            'why': '' if self._test_passed('distance') else 'the last distance run did not pass',
            'fix': 'Do a distance run that passes, then run the drift test again.'}]
        if self.spin_test:
            checks.append({
                'key': 'spin', 'label': 'Spin (one full turn left)',
                'measured': f'{sp.get("yaw_at_scale_deg", sp["yaw_change_deg"]):+.1f} deg' if sp else 'not run',
                'limit': f'+360 deg, <= {self.spin_lim:g} %', 'passed': self._test_passed('spin'),
                'why': '' if self._test_passed('spin') else 'the last spin did not pass',
                'fix': 'Do a spin that passes, then run the drift test again.'})
        checks.append({
            'key': 'imu_rate', 'label': 'IMU publishing during the drift test',
            'measured': f'{n} messages in {dur:.0f} s', 'limit': f'>= {self.min_rate:g} Hz', 'passed': rate_ok,
            'why': '' if rate_ok else '/imu/rpy stopped or slowed down',
            'fix': '' if rate_ok else 'Check the IMU row in step 1, then Redo.'})
        checks.append({
            'key': 'drift', 'label': 'Heading drift, car still',
            'measured': f'{drift:+.2f} deg/min' if rate_ok else 'no IMU data', 'limit': f'<= {self.drift_lim:g} deg/min',
            'passed': drift_ok,
            'why': '' if drift_ok else 'the IMU heading moved while the car stood still',
            'fix': '' if drift_ok else ('Nobody touches the car or the table; keep it away from motors, magnets and '
                                        'speakers. The gyro bias is taken at power-on: switch the car off and on '
                                        'standing still, wait 10 s, then Redo.')})
        passed = all(c['passed'] for c in checks)
        warnings = []
        if abs(abs(v.get('imu_yaw_scale', 1.0)) - 1.0) > 1e-9:
            warnings.append(f'imu_yaw_scale is {v["imu_yaw_scale"]:+g}: servo_controller re-normalises yaw after '
                            'scaling, so /imu/rpy jumps by 360 x (1 - |scale|) deg at +-180 deg (BACKLOG #31).')
        summary = (f'ticks_per_meter {v.get("ticks_per_meter", 0):g}, reverse {v.get("odom_reverse_polarity")}, '
                   f'imu_yaw_scale {v.get("imu_yaw_scale", 0):+g}, drift {drift:+.2f} deg/min' if passed else
                   'failed: ' + ', '.join(c['label'].lower() for c in checks if not c['passed']))
        drift_doc = {'status': 'PASS' if drift_ok else 'FAIL', 'seconds': round(dur, 1),
                     'drift_deg_per_min': round(drift, 3), 'limit': self.drift_lim, 'imu_msgs': n}
        tests = {'distance': {'status': d['status'] if d else 'NOT_RUN', 'runs': list(self.runs['distance'])},
                 'drift': drift_doc}
        if self.spin_test:
            tests['spin'] = {'status': sp['status'] if sp else 'NOT_RUN', 'runs': list(self.runs['spin'])}
        return {'passed': passed, 'summary': summary, 'checks': checks, 'warnings': warnings,
                'values': {k: v.get(k) for k in PARAMS}, 'tests': tests,
                'overlay': {SERVO: {k: v.get(k) for k in PARAMS}} if passed else {},
                'measure_s': round(dur, 1), 'samples': n}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        vals = res.get('values') or {}
        if any(vals.get(k) is None for k in PARAMS):
            raise StepRefused('The result has no servo_controller values: run the tests again.')
        if self.values is not None and any(self.values[k] != vals[k] for k in PARAMS):
            raise StepRefused('servo_controller values changed after this result (another run corrected them): '
                              'run the drift test again so the saved values are the verified ones.')
        return [ct.merge_overlay(session, SERVO, {k: vals[k] for k in PARAMS})]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        vals = {k: ct.overlay_value(src_session, SERVO, k) for k in PARAMS}
        miss = [k for k, x in vals.items() if x is None]
        if miss:
            raise StepRefused(f'{os.path.basename(src_session)}/params_overlay.yaml has no {SERVO} '
                              f'{", ".join(miss)}: run this step again.')
        vals = {'ticks_per_meter': float(vals['ticks_per_meter']),
                'odom_reverse_polarity': bool(vals['odom_reverse_polarity']),
                'imu_yaw_scale': float(vals['imu_yaw_scale'])}
        path = ct.merge_overlay(session, SERVO, vals)
        self._set(vals, 'keep previous value')          # live too: step 7 runs in this launch
        return [path]

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        now = self.clock()
        if self.values is None and not self.busy and now - self.read_at >= READ_RETRY_S:
            self._read()
        a = self.active
        act = None
        if a is not None:
            act = {'test': a['test'], 'phase': a['phase'], 'elapsed_s': round(now - a['t0'], 1),
                   'distance_m': round(self.rec.dist, 3), 'yaw_deg': round(self.rec.yaw, 1),
                   'odom_msgs': self.rec.odom_n, 'imu_msgs': self.rec.imu_n,
                   'settling': a['phase'] == 'measuring' and now < self.rec.ignore_until}
        tests = {}
        for t in ('distance', 'spin'):
            runs = self.runs[t]
            tests[t] = {'status': runs[-1]['status'] if runs else 'NOT_RUN', 'runs': runs[-self.max_runs:],
                        'n_runs': len(runs), 'failed_runs': sum(1 for r in runs if r['status'] != 'PASS')}
        return {'values': self.values, 'busy': self.busy, 'link_error': self.link_error, 'active': act,
                'tests': tests, 'spin_test': self.spin_test, 'straight_run_m': self.true_m,
                'drift_test_s': self.drift_s, 'max_runs': self.max_runs, 'ages': self.rec.ages(now),
                'limits': {'distance_pct': self.dist_lim, 'spin_pct': self.spin_lim, 'drift_deg_per_min': self.drift_lim},
                'drift_ready': all(self._test_passed(t) for t in self._needed())}
