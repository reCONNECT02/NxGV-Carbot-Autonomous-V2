"""Calibration step 7 -- servo centre + steering limits (pure, unit tested).

Same procedure and analysis as the terminal tool (carbot_control.calib_steering +
calib_core); the wizard drives the car itself through the command owner
(CALIBRATION_RAW via carbot_ops.wizard_drive, calibrate mode only; never the motion
topic). STOP MOTORS (/e_stop) stops the car at once and cancels the run.

RUN argument (no argument = circles first, then straight):
  "circles"   full-LEFT lock, then full-RIGHT lock, each until the IMU turned
              procedure.circle_yaw_deg (or circle_max_distance_m) -> radius per side,
              left/right_max_rad = atan(wheelbase / R), min_turning_radius_m = max(R).
              Both locks turning the wrong way flips steer_sign (as the CLI does).
              Starts the step over (straight runs cleared).
  "straight"  ONE straight run (angular.z 0) of straight_run_m: drift per metre. Above
              pass.straight_drift_m_per_m the integer servo_controller.servo_center is
              corrected LIVE (calib_core.centre_correction); line the car up and run again.
              At most procedure.max_runs straight runs per circles run.
The step PASSES when both circles and the last straight run pass (calib_core.steering_summary).
Every Run returns the combined result so far; the page says what to do next.

Parameters read at the start of each Run (non-blocking, DriveKit.servo / .owner):
command_owner mode (must be calibrate) + steering.steer_sign; servo_controller
servo_center, servo_range_left, servo_range_right (no defaults: a missing one fails the run).

Result doc = the CLI's keys (left, right, straight_runs, passed, checks {direction,
min_radius, straight}, min_turning_radius_m, left_max_rad, right_max_rad, step,
wheelbase_m, servo_center, steering) + wizard keys: summary, check_rows (GUI table),
capture (what captures/steering.json holds, replayable with calib_steering --replay),
last_run, note, next.
Save writes params_overlay.yaml (calib_steering.write_overlay: the keys in `writes`) and
captures/steering.json. Keep previous copies those overlay keys (BACKLOG #21).
"""
import copy
import json
import math
import os
from typing import Dict, List, Optional

from carbot_common import calib_tools as ct
from carbot_control import calib_core as cc
from carbot_control import calib_steering as cst

from .wizard_core import StepImpl, StepRefused
from .wizard_drive import RAW, DriveKit, Segment, inputs_problem

PROC_KEYS = ('raw_duty', 'circle_yaw_deg', 'circle_max_distance_m', 'straight_run_m', 'max_runs', 'timeout_s',
             'settle_s')
PASS_KEYS = ('straight_drift_m_per_m', 'min_radius_m_max')
OWNER_PARAMS = ['mode', 'steering.steer_sign']
SERVO_PARAMS = ['servo_center', 'servo_range_left', 'servo_range_right']
RUNS = ('circles', 'straight')
PARAM_WAIT_S = 10.0          # guard only: ParamLink answers or times out (drive.param_timeout_s) first


class ConfigError(ValueError):
    pass


class ServoSteeringStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, wheelbase_m: float, kit: DriveKit):
        super().__init__(cfg)
        self.proc, self.pas = cfg.get('procedure') or {}, cfg.get('pass') or {}
        for where, d, keys in (('procedure', self.proc, PROC_KEYS), ('pass', self.pas, PASS_KEYS)):
            miss = [k for k in keys if k not in d]
            if miss:
                raise ConfigError(f'calibration_steps.yaml servo_steering.{where}: missing {", ".join(miss)}')
        self.wb = float(wheelbase_m)
        if self.wb <= 0:
            raise ConfigError(f'common.yaml vehicle.wheelbase_m must be > 0 (got {wheelbase_m})')
        self.kit = kit
        self.duty = float(self.proc['raw_duty'])
        self.target_yaw = math.radians(float(self.proc['circle_yaw_deg']))
        self.max_circle_d = float(self.proc['circle_max_distance_m'])
        self.run_m = float(self.proc['straight_run_m'])
        self.max_runs = int(self.proc['max_runs'])
        self.timeout = float(self.proc['timeout_s'])
        self.settle = float(self.proc['settle_s'])
        self.drift_max = float(self.pas['straight_drift_m_per_m'])
        self.cap: Optional[Dict] = None            # committed capture (left, right, straight runs)
        self.k_units = (0.0, 0.0)                  # curvature per servo unit, left / right lock
        self.ranges = (0.0, 0.0)                   # servo_range_left / right read at the last Run
        self.run_id = 0
        self._clear_run()

    def _clear_run(self) -> None:
        self.mode: Optional[str] = None            # 'circles' | 'straight' while running
        self.phase: Optional[str] = None           # params | left | right | straight | apply
        self.seg: Optional[Segment] = None
        self.new: Optional[Dict] = None            # capture being built by this run
        self.replies: Dict[str, object] = {}
        self.run_id = getattr(self, 'run_id', 0) + 1   # late parameter replies of an older run are ignored
        self.t0 = self.t_phase = 0.0
        self.centre_used = 0
        self.correction = 0

    # ------------------------------------------------------------------ helpers
    def _cap_complete(self) -> bool:
        return bool(self.cap and 'left' in self.cap and 'right' in self.cap)

    def _pick(self, arg: str) -> str:
        arg = (arg or '').strip().lower()
        if arg:
            return arg
        return 'straight' if self._cap_complete() else 'circles'

    def _analyse(self, cap: Dict) -> Dict:
        return cst.analyse(cap, self.cfg, self.wb)

    # ------------------------------------------------------------------ run
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        mode = self._pick(inputs.get('argument', ''))
        if mode not in RUNS:
            return f'Unknown run {mode!r}: use circles or straight.'
        prob = inputs_problem(inputs.get('drive'))
        if prob:
            return prob
        if mode == 'straight':
            if not self._cap_complete():
                return 'Run circles first: the straight-run correction needs the full-lock radius of each side.'
            if len(self.cap['straight']) >= self.max_runs:
                return (f'{self.max_runs} straight runs done (procedure.max_runs). Check the servo horn and '
                        'steering linkage for play, then press Run circles to start over.')
        self._clear_run()
        self.mode, self.phase, self.t0, self.t_phase = mode, 'params', now, now
        self.kit.owner.get(OWNER_PARAMS, self._reply('owner'))
        self.kit.servo.get(SERVO_PARAMS, self._reply('servo'))
        return None

    def _reply(self, key: str):
        run = self.run_id

        def done(v):
            if run == self.run_id:
                self.replies[key] = v if v is not None else False
        return done

    def cancel(self) -> None:
        self.kit.cmd.stop()
        self._clear_run()

    def _segment(self, now: float, inputs: Dict, steer: float, reason: str, until) -> Segment:
        seg = Segment(RAW, self.duty, steer, reason, until, self.timeout, self.settle, self.kit.accept_s)
        seg.start(now, inputs)
        return seg

    def _start_circle(self, now: float, inputs: Dict, side: str) -> None:
        sign = float(self.new['steer_sign'])
        z = sign if side == 'left' else -sign       # base angular.z for a physical LEFT lock = steer_sign
        target, dmax = self.target_yaw, self.max_circle_d
        self.seg = self._segment(now, inputs, z, f'step 7 {side} lock',
                                 lambda d, y, el: abs(y) >= target or abs(d) >= dmax)
        self.phase, self.t_phase = side, now

    def _params(self, now: float, inputs: Dict) -> Optional[Dict]:
        if 'owner' not in self.replies or 'servo' not in self.replies:
            if now - self.t_phase > PARAM_WAIT_S:
                return self._abort('no parameter reply from command_owner / servo_controller')
            return None
        own, srv = self.replies['owner'], self.replies['servo']
        if not own:
            return self._abort('command_owner does not answer parameter requests: is calibrate.launch.py running?')
        if own.get('mode') != 'calibrate':
            return self._abort(f'command_owner is in mode {own.get("mode")!r}, not calibrate: start '
                               'ros2 launch carbot_bringup calibrate.launch.py')
        if own.get('steering.steer_sign') is None:
            return self._abort('command_owner has no steering.steer_sign parameter (control.yaml)')
        if not srv or any(srv.get(k) is None for k in SERVO_PARAMS):
            miss = [k for k in SERVO_PARAMS if not srv or srv.get(k) is None]
            return self._abort('servo_controller does not report ' + ', '.join(miss) +
                               ': is the base servo_controller running?')
        centre = int(srv['servo_center'])
        self.ranges = (float(srv['servo_range_left']), float(srv['servo_range_right']))
        if self.mode == 'circles':
            self.new = {'servo_center': centre, 'steer_sign': float(own['steering.steer_sign']),
                        'servo_range_left': self.ranges[0], 'servo_range_right': self.ranges[1], 'straight': []}
            self._start_circle(now, inputs, 'left')
        else:
            self.new = copy.deepcopy(self.cap)
            self.new['servo_center'] = centre                # live value (Tuning tab may have changed it)
            self.centre_used = centre
            run_m = self.run_m
            self.seg = self._segment(now, inputs, 0.0, 'step 7 straight', lambda d, y, el: abs(d) >= run_m)
            self.phase, self.t_phase = 'straight', now
        return None

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        if self.phase is None:
            return None
        if self.phase == 'params':
            return self._params(now, inputs)
        if self.phase == 'apply':
            if 'set' not in self.replies:
                if now - self.t_phase > PARAM_WAIT_S:
                    self.replies['set'] = False
                else:
                    return None
            new_c = self.centre_used + self.correction
            if self.replies['set']:
                self.new['servo_center'] = new_c
                note = (f'drift above the limit: servo_center {self.centre_used} -> {new_c} set LIVE. '
                        'Line the car up again and press Straight run.')
            else:
                note = (f'drift above the limit, but setting servo_center {new_c} on servo_controller failed: '
                        f'set it in the Tuning tab, then press Straight run.')
            return self._finish(note)
        r = self.seg.tick(now, inputs, self.kit.cmd)
        if r is None:
            return None
        if r['aborted']:
            return self._abort(f'{self.phase} run aborted: {r["aborted"]}', r)
        if self.phase in ('left', 'right'):
            self.new[self.phase] = {'distance': r['distance'], 'yaw': r['yaw']}
            if self.phase == 'left':
                self._start_circle(now, inputs, 'right')
                return None
            return self._circles_done()
        return self._straight_done(now, r)

    def _circles_done(self) -> Dict:
        cap = self.new
        L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], self.wb)
        R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], self.wb)
        note = ''
        if not L.turned_correct_way and not R.turned_correct_way:
            cap['steer_sign'] = -float(cap['steer_sign'])
            for s in ('left', 'right'):
                cap[s]['yaw'] = -cap[s]['yaw']
            note = f'both locks turned the wrong way: steer_sign -> {cap["steer_sign"]:g} (saved with this step). '
            L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], self.wb)
            R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], self.wb)
        self.k_units = ((1.0 / L.radius_m) / max(self.ranges[0], 1.0), (1.0 / R.radius_m) / max(self.ranges[1], 1.0))
        cap['k_per_unit'] = list(self.k_units)
        self.cap = cap
        return self._finish(note + f'circles: R left {L.radius_m:.3f} m, right {R.radius_m:.3f} m. '
                                   'Next: line the car up and press Straight run.')

    def _straight_done(self, now: float, r: Dict) -> Optional[Dict]:
        self.new['straight'].append({'distance': r['distance'], 'yaw': r['yaw'], 'servo_center': self.centre_used})
        s = cc.straight_result(r['distance'], r['yaw'])
        if s.drift_m_per_m <= self.drift_max:
            self.cap = self.new
            return self._finish(f'straight run {len(self.new["straight"])}: drift {s.drift_m_per_m * 100:.2f} cm/m '
                                'passes.')
        kl, kr = self.cap.get('k_per_unit') or self.k_units
        du = cc.centre_correction(s.curvature, kl, kr)
        if du == 0:
            self.cap = self.new
            return self._finish('drift above the limit but below one servo unit: cannot correct further. Check '
                                'the wheels / tyres for rubbing, then Redo circles.')
        self.cap = self.new
        self.correction = du
        self.phase, self.t_phase = 'apply', now
        self.kit.servo.set({'servo_center': int(self.centre_used + du)}, self._reply('set'))
        return None

    def _abort(self, why: str, seg_result: Optional[Dict] = None) -> Dict:
        self.kit.cmd.stop()
        res = self._finish(why, aborted=True)
        if seg_result is not None:
            res['aborted_at'] = {'distance_m': round(seg_result['distance'], 3),
                                 'yaw_deg': round(math.degrees(seg_result['yaw']), 1)}
        return res

    # ------------------------------------------------------------------ result
    def _finish(self, note: str, aborted: bool = False) -> Dict:
        mode = self.mode
        self.kit.cmd.stop()
        self._clear_run()
        return self.result(mode, note, aborted)

    def result(self, last_run: str, note: str, aborted: bool = False) -> Dict:
        if not self._cap_complete():
            return {'passed': False, 'summary': note if aborted else 'circles not run yet', 'checks': {},
                    'check_rows': [self._row('circles', 'Full-lock circles', 'not run', 'left + right', False,
                                             note, 'Clear the floor, release STOP MOTORS and press Run circles.')],
                    'last_run': last_run, 'note': note, 'next': 'circles', 'aborted': aborted}
        cap = self.cap
        res = self._analyse(cap)
        steering = cst.steering_params(res, cap['steer_sign'])
        runs = res['straight_runs']
        passed = bool(res['passed']) and not aborted
        nxt = 'save' if passed else ('circles' if not res['checks']['direction'] or not res['checks']['min_radius']
                                     or len(runs) >= self.max_runs else 'straight')
        if passed:
            summary = (f'R {res["left"]["radius_m"]:.3f} / {res["right"]["radius_m"]:.3f} m, drift '
                       f'{runs[-1]["drift_m_per_m"] * 100:.2f} cm/m, servo_center {cap["servo_center"]}')
        elif aborted:
            summary = note
        elif not runs:
            summary = f'circles done (R {res["min_turning_radius_m"]:.3f} m); still to do: straight run'
        else:
            bad = [k for k, v in res['checks'].items() if not v]
            summary = 'failed: ' + ', '.join(bad) + ('' if nxt == 'circles' else '; press Straight run again')
        doc = dict(res, step=cst.STEP_ID, wheelbase_m=self.wb, servo_center=int(cap['servo_center']),
                   steering=steering)
        doc.update(passed=passed, summary=summary, check_rows=self._rows(res, cap),
                   capture=cc.plain(copy.deepcopy(cap)), last_run=last_run, note=note, next=nxt, aborted=aborted)
        return cc.plain(doc)

    @staticmethod
    def _row(key, label, measured, limit, passed, why='', fix='') -> Dict:
        return {'key': key, 'label': label, 'measured': measured, 'limit': limit, 'passed': bool(passed),
                'why': '' if passed else why, 'fix': '' if passed else fix}

    def _rows(self, res: Dict, cap: Dict) -> List[Dict]:
        L, R, ch = res['left'], res['right'], res['checks']
        way = lambda c: 'ok' if c['turned_correct_way'] else 'WRONG way'  # noqa: E731
        rows = [
            self._row('direction', 'Each full lock turns its own way', f'left {way(L)}, right {way(R)}',
                      'both ok', ch['direction'],
                      'Only one lock turned the expected way: the servo does not reach one side, or /imu/rpy yaw '
                      'has the wrong sign (step 6).',
                      'Check the servo linkage and servo_range_left / right (Tuning tab), then Run circles again.'),
            self._row('min_radius', 'Tightest turn (larger of the two radii)',
                      f'L {L["radius_m"]:.3f} m, R {R["radius_m"]:.3f} m',
                      f'<= {float(self.pas["min_radius_m_max"]):.2f} m', ch['min_radius'],
                      'The car cannot turn tightly enough on one side for the track (the planners assume one '
                      'radius both ways).',
                      'If the servo does not buzz at full lock, raise servo_range_left / right in the Tuning tab, '
                      'then Run circles again.')]
        runs = res['straight_runs']
        if runs:
            last = runs[-1]
            n = len(runs)
            rows.append(self._row('straight', f'Straight-run drift (run {n} of max {self.max_runs})',
                                  f'{last["drift_m_per_m"] * 100:.2f} cm/m, servo_center '
                                  f'{cap["straight"][-1]["servo_center"]}',
                                  f'<= {self.drift_max * 100:.1f} cm/m', ch['straight'],
                                  'The car still curves with the steering at centre.',
                                  'servo_center was corrected live: line the car up and press Straight run again.'
                                  if n < self.max_runs else
                                  'max_runs reached: check the servo horn / linkage for play, then Run circles again.'))
        else:
            rows.append(self._row('straight', 'Straight-run drift', 'not run', f'<= {self.drift_max * 100:.1f} cm/m',
                                  False, 'No straight run since the circles.',
                                  f'Line the car up with {self.run_m:g} m free ahead and press Straight run.'))
        return rows

    # ------------------------------------------------------------------ live view
    def progress(self, now: float) -> Dict:
        if self.phase is None:
            return {}
        el = now - self.t0
        seg = self.seg.progress() if self.seg is not None and self.phase in ('left', 'right', 'straight') else {}
        if self.phase in ('left', 'right'):
            frac = min(1.0, abs(math.radians(seg.get('yaw_deg', 0.0))) / self.target_yaw)
            frac = (0.5 if self.phase == 'right' else 0.0) + frac / 2
        elif self.phase == 'straight':
            frac = min(1.0, abs(seg.get('distance_m', 0.0)) / self.run_m)
        else:
            frac = 0.0
        left = self.timeout * (2 if self.mode == 'circles' and self.phase in ('params', 'left') else 1)
        return {'phase': self.phase, 'mode': self.mode, 'elapsed_s': round(el, 1),
                'remaining_s': round(max(0.0, left - (now - self.t_phase)), 1), 'fraction': round(frac, 2),
                'segment': seg}

    def live(self, inputs: Dict) -> Dict:
        snap = inputs.get('drive') or {}
        seg = None
        if self.seg is not None and self.phase in ('left', 'right', 'straight'):
            seg = self.seg.progress()
            if self.phase != 'straight':
                y = abs(math.radians(seg['yaw_deg']))
                seg['radius_m'] = round(abs(seg['distance_m']) / y, 3) if y > 0.2 else None
            else:
                k = cc.straight_result(seg['distance_m'], math.radians(seg['yaw_deg'])) if abs(seg['distance_m']) > 0.2 \
                    else None
                seg['drift_m_per_m'] = round(k.drift_m_per_m, 4) if k else None
        cap = self.cap if self.phase is None or self.new is None else self.new
        circles = {}
        for side in ('left', 'right'):
            c = (cap or {}).get(side)
            if c:
                r = cc.circle_result(side, c['distance'], c['yaw'], self.wb)
                circles[side] = {'distance_m': round(r.distance_m, 3), 'yaw_deg': round(math.degrees(r.yaw_change_rad), 1),
                                 'radius_m': round(r.radius_m, 3), 'ok': r.turned_correct_way}
        runs = []
        for s in (cap or {}).get('straight', []):
            r = cc.straight_result(s['distance'], s['yaw'])
            runs.append({'distance_m': round(r.distance_m, 3), 'yaw_deg': round(math.degrees(r.yaw_change_rad), 2),
                         'drift_m_per_m': round(r.drift_m_per_m, 4), 'servo_center': s['servo_center'],
                         'ok': r.drift_m_per_m <= self.drift_max})
        return {'phase': self.phase, 'mode': self.mode, 'segment': seg,
                'input_problem': inputs_problem(snap) if self.phase is None else '',
                'drive': {k: snap.get(k) for k in ('odom_ok', 'imu_ok', 'estop', 'owner_winner', 'owner_reason',
                                                    'speed_mps')},
                'circles': circles, 'straight_runs': runs, 'max_runs': self.max_runs,
                'servo_center': (cap or {}).get('servo_center'), 'steer_sign': (cap or {}).get('steer_sign'),
                'next': 'straight' if self._cap_complete() and len(self.cap['straight']) < self.max_runs else 'circles',
                'limits': {'circle_yaw_deg': float(self.proc['circle_yaw_deg']), 'circle_max_distance_m': self.max_circle_d,
                           'straight_run_m': self.run_m, 'straight_drift_m_per_m': self.drift_max,
                           'min_radius_m_max': float(self.pas['min_radius_m_max']), 'raw_duty': self.duty}}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        cap = res.get('capture') or {}
        if not res.get('passed') or 'left' not in cap or not cap.get('straight'):
            raise StepRefused('Nothing complete to save: run circles and a passing straight run first.')
        written = [cst.write_overlay(session, res, res['steering'], int(res['servo_center']))]
        path = ct.capture_path(session, 'steering.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(cap, f, indent=1)
        return written + [path]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        name = os.path.basename(src_session)
        vals = {}
        for node, key in cst.OVERLAY_KEYS:
            v = ct.overlay_value(src_session, node, key)
            if v is None:
                raise StepRefused(f'{name}/params_overlay.yaml has no {node} {key}: run this step again.')
            vals.setdefault(node, {})[key] = v
        path = ''
        for node, params in vals.items():
            path = ct.merge_overlay(session, node, params)
        out = [path]
        src_cap = os.path.join(src_session, 'captures', 'steering.json')
        if os.path.isfile(src_cap):
            dst = ct.capture_path(session, 'steering.json')
            with open(src_cap, 'rb') as a, open(dst, 'wb') as b:
                b.write(a.read())
            out.append(dst)
        return out
