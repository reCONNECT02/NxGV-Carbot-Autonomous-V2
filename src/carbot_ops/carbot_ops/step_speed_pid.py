"""Calibration step 8 -- speed feedforward + PID (pure, unit tested).

Same drives, analysis (carbot_control.calib_core, calib_speed.analyse) and outputs as the
terminal tool (carbot_control calib_speed), run from the wizard page. The car DRIVES ITSELF
straight, forward and back alternately, through the command owner (requests on
/carbot/calibration/request; the owner accepts them in calibrate mode only, never while
armed, and the e-stop always wins).

RUN starts a supervised sequence. Before EVERY segment the step waits in phase 'ready'
until the user presses Go (page op `go`, action STEP; allowed while RUNNING):

  sweep 1..N    CALIBRATION_RAW at procedure.sweep_duties[i] (odd steps backwards) for
                hold_s; steady speed = mean /odom twist speed over the last steady_s.
                After the last one: least-squares feedforward duty = static + per_mps * |v|,
                set LIVE on command_owner (with the PID gains of the first verify run).
  verify r.i    CALIBRATION (m/s through the owner's feedforward + PID) at
                procedure.step_targets_mps[i] (odd steps backwards) for hold_s: steady
                error and overshoot. When a run of all targets fails, kp / ki are retuned
                like the CLI (calib_core.retune), set live, and the targets run again, up
                to max_runs. A failure no retune can fix (creep, fit) ends the run at once.

Each segment ends by commanding speed 0 for stop_settle_s (like calib_drive.Driver), then
the request stream stops (the owner's watchdog then holds zero). /odom older than
odom_timeout_s while driving aborts (step FAILS, car stopped). Cancel / STOP MOTORS stop
the requests at once. A run that does not pass puts the command_owner values it started
with back (live); a PASS leaves the verified ones live.

Save writes, like the terminal tool: command_owner.feedforward.{duty_per_mps,static_duty},
command_owner.speed_pid.{kp,ki,kd,integral_limit} and the tunnel_bridge
command_owner_feedforward.* mirror (calib_speed.write_overlay), plus
captures/speed.json that `calib_speed --replay` reads.
"""
import json
import math
import os
import time
from typing import Callable, Dict, List, Optional

from carbot_common import calib_tools as ct
from carbot_control import calib_core as cc
from carbot_control import calib_speed as csp

from .sensor_checks import ConfigError
from .wizard_core import StepImpl, StepRefused

OWNER = csp.OWNER
PROC_KEYS = ('sweep_duties', 'step_targets_mps', 'hold_s', 'steady_s', 'max_runs', 'stop_settle_s',
             'odom_timeout_s')
PASS_KEYS = ('max_steady_error_mps', 'max_overshoot_pct', 'min_creep_speed_mps')
PID_PARAMS = tuple(f'speed_pid.{k}' for k in csp.PID_KEYS)
FF_PARAMS = tuple(f'feedforward.{k}' for k in csp.FF_KEYS)
OWNER_PARAMS = ('mode',) + PID_PARAMS + FF_PARAMS
READ_RETRY_S = 2.0
RAW, CLOSED = 'CALIBRATION_RAW', 'CALIBRATION'
TRACE_POINTS = 80
CHECK_LABEL = {'feedforward_fit': 'Feedforward fit (duty sweep)', 'steady_error': 'Steady speed error (worst step)',
               'overshoot': 'Overshoot (worst step)', 'creep': 'Slowest moving sweep speed (creep)'}


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def _num_list(v, where: str) -> List[float]:
    if not isinstance(v, (list, tuple)) or not v:
        raise ConfigError(f'{where} must be a non-empty list')
    out = [float(x) for x in v]
    if any(x <= 0 for x in out):
        raise ConfigError(f'{where}: every value must be > 0 (the direction alternates by itself)')
    return out


class SpeedPidStep(StepImpl):
    can_keep_previous = True
    ops_while_running = True           # Go between segments

    def __init__(self, cfg: Dict, recorder, owner, drive, clock: Callable[[], float] = time.monotonic):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml speed_pid.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml speed_pid.pass')
        self.rec, self.owner, self.drive, self.clock = recorder, owner, drive, clock
        p = self.proc
        self.duties = _num_list(p['sweep_duties'], 'calibration_steps.yaml speed_pid.procedure.sweep_duties')
        self.targets = _num_list(p['step_targets_mps'], 'calibration_steps.yaml speed_pid.procedure.step_targets_mps')
        self.hold, self.steady = float(p['hold_s']), float(p['steady_s'])
        self.max_runs, self.stop_s, self.odom_to = int(p['max_runs']), float(p['stop_settle_s']), float(p['odom_timeout_s'])
        if not 0 < self.steady <= self.hold:
            raise ConfigError('calibration_steps.yaml speed_pid.procedure: need 0 < steady_s <= hold_s')
        if self.max_runs < 1:
            raise ConfigError('calibration_steps.yaml speed_pid.procedure.max_runs must be >= 1')
        self.values: Optional[Dict] = None       # command_owner: mode, pid {kp..}, ff {duty_per_mps, static_duty}
        self.busy, self.link_error, self.read_at = '', '', -math.inf
        self.run: Optional[Dict] = None

    # ------------------------------------------------------------------ command_owner link
    def _read(self) -> None:
        self.busy, self.read_at = 'reading command_owner parameters', self.clock()

        def done(o):
            self.busy = ''
            miss = [f'{OWNER}.{k}' for k in OWNER_PARAMS if not o or o.get(k) is None]
            if miss:
                self.link_error = f'Cannot read {", ".join(miss)}: is calibrate.launch.py running?'
                return
            self.link_error = ''
            self.values = {'mode': str(o['mode']),
                           'pid': {k: float(o[f'speed_pid.{k}']) for k in csp.PID_KEYS},
                           'ff': {k: float(o[f'feedforward.{k}']) for k in csp.FF_KEYS}}
        self.owner.get(list(OWNER_PARAMS), done)

    @staticmethod
    def _params(ff: Optional[Dict], pid: Optional[Dict]) -> Dict:
        out = {}
        if ff is not None:
            out.update({f'feedforward.{k}': float(ff[k]) for k in csp.FF_KEYS})
        if pid is not None:
            out.update({f'speed_pid.{k}': float(pid[k]) for k in csp.PID_KEYS})
        return out

    def _set(self, ff: Optional[Dict], pid: Optional[Dict], why: str,
             then: Optional[Callable[[bool], None]] = None) -> None:
        values = self._params(ff, pid)
        self.busy = f'setting {", ".join(values)} on {OWNER}'

        def done(ok):
            self.busy = ''
            if ok:
                self.link_error = ''
                if self.values is not None:
                    if ff is not None:
                        self.values['ff'] = {k: float(ff[k]) for k in csp.FF_KEYS}
                    if pid is not None:
                        self.values['pid'] = {k: float(pid[k]) for k in csp.PID_KEYS}
            else:
                self.link_error = f'{OWNER} did not accept {", ".join(values)} ({why}).'
            if then is not None:
                then(ok)
        self.owner.set(values, done)

    # ------------------------------------------------------------------ run
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        if self.busy:
            return f'Wait: {self.busy}.'
        if self.values is None:
            self._read()
            return (self.link_error or 'Reading the current values from command_owner') + ' Try again in a moment.'
        if self.values['mode'] != 'calibrate':
            return (f'command_owner runs in {self.values["mode"]!r} mode: the car only drives for calibration '
                    'under calibrate.launch.py.')
        pid = dict(self.values['pid'])
        self.run = {'kind': 'sweep', 'i': 0, 'phase': 'ready', 'pid': pid, 'ff': None, 'metrics': [],
                    'cap': {'sweep': [], 'verify': [], 'pid_start': dict(pid)},
                    'start': {'pid': dict(pid), 'ff': dict(self.values['ff'])},
                    'abort': '', 'note': '', 'retunes': [], 'rows': [], 'seg': None}
        return None

    def _segment(self, r: Dict) -> Dict:
        """The segment at (kind, i): source, signed command, label."""
        i = r['i']
        back = i % 2 == 1
        if r['kind'] == 'sweep':
            d = self.duties[i] * (-1 if back else 1)
            return {'source': RAW, 'command': d, 'direction': 'backward' if back else 'forward',
                    'label': f'duty sweep {i + 1} of {len(self.duties)}: duty {d:+.2f}'}
        tg = self.targets[i] * (-1 if back else 1)
        return {'source': CLOSED, 'command': tg, 'direction': 'backward' if back else 'forward',
                'label': f'speed step {i + 1} of {len(self.targets)} (run {len(r["cap"]["verify"]) + 1}): '
                         f'{tg:+.3f} m/s'}

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
            return {'ok': False, 'message': 'Wait: the car is still driving, stopping or setting parameters.'}
        if self.busy:
            return {'ok': False, 'message': f'Wait: {self.busy}.'}
        seg = self._segment(r)
        now = self.clock()
        r.update(phase='driving', t=now, seg=seg, rows=[])
        self.rec.zero()
        self.rec.trace = r['rows']
        self.drive.command(seg['source'], seg['command'], 0.0, f'step 8 {r["kind"]} {seg["command"]:+.3f}')
        return {'ok': True, 'message': f'Driving the {seg["label"]} ({seg["direction"]}): keep the e-stop ready.'}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        if r['abort']:
            return self._finish()
        if r['phase'] == 'driving':
            if self.rec.odom_t is None or now - self.rec.odom_t > self.odom_to:
                r['abort'] = f'/odom stopped while driving the {r["seg"]["label"]}'
                return self._finish()
            if now - r['t'] >= self.hold:
                r['phase'], r['t_stop'] = 'stopping', now
                self.rec.trace = None
                self.drive.command(r['seg']['source'], 0.0, 0.0, 'stop')
        elif r['phase'] == 'stopping' and now - r['t_stop'] >= self.stop_s:
            self.drive.stop()
            return self._segment_done()
        return None

    def _tv(self, r: Dict):
        rows = r['rows']
        tt = [t - r['t'] for t, _ in rows]
        vv = [v for _, v in rows]
        return tt, vv, (tt[-1] if tt else 0.0)

    def _segment_done(self) -> Optional[Dict]:
        r, cap = self.run, self.run['cap']
        seg = r['seg']
        tt, vv, t_end = self._tv(r)
        if r['kind'] == 'sweep':
            v = cc.steady_speed(tt, vv, t_end, self.steady) if tt else 0.0
            cap['sweep'].append([seg['command'], v])
            r['i'] += 1
            if r['i'] < len(self.duties):
                r['phase'] = 'ready'
                return None
            ff = cc.fit_feedforward([tuple(s) for s in cap['sweep']])
            if not ff.ok:
                r['note'] = f'duty sweep: {ff.reason}'
                return self._finish()
            r['ff'] = ff
            r['kind'], r['i'] = 'verify', 0
            self._apply(r, {'duty_per_mps': ff.duty_per_mps, 'static_duty': ff.static_duty}, 'feedforward from the sweep')
            return None
        m = cc.step_metrics(tt, vv, seg['command'], t_end, self.steady)
        r['metrics'].append(m)
        r['i'] += 1
        if r['i'] < len(self.targets):
            r['phase'] = 'ready'
            return None
        cap['verify'].append({'pid': dict(r['pid']), 'metrics': [x.as_dict() for x in r['metrics']]})
        verdict = cc.pid_verdict(r['metrics'], r['ff'], self.pass_cfg)
        if verdict['passed'] or len(cap['verify']) >= self.max_runs:
            return self._finish()
        kp, ki, why = cc.retune(r['pid']['kp'], r['pid']['ki'], verdict)
        if why == 'no change':
            r['note'] = 'failing checks a kp / ki retune cannot fix: ' + \
                ', '.join(k for k, ok in verdict['checks'].items() if not ok)
            return self._finish()
        r['pid'].update(kp=kp, ki=ki)
        r['retunes'].append(f'run {len(cap["verify"])}: {why}')
        r['metrics'], r['i'] = [], 0
        self._apply(r, None, 'retuned PID gains')
        return None

    def _apply(self, r: Dict, ff: Optional[Dict], why: str) -> None:
        r['phase'] = 'applying'

        def done(ok):
            if self.run is not r:
                return
            if ok:
                r['phase'] = 'ready'
            else:
                r['abort'] = f'{OWNER} did not accept the {why}'
        self._set(ff, r['pid'], why, done)

    def _restore(self, r: Optional[Dict]) -> None:
        if r is None or self.values is None:
            return
        s = r['start']
        if self.values['ff'] != s['ff'] or self.values['pid'] != s['pid']:
            self._set(s['ff'], s['pid'], 'restore the values the run started with')

    def cancel(self) -> None:
        self.drive.stop()
        self.rec.trace = None
        r, self.run = self.run, None
        self._restore(r)

    def _finish(self) -> Dict:
        self.drive.stop()
        self.rec.trace = None
        r, self.run = self.run, None
        res = self._result(r)
        if not res['passed']:
            self._restore(r)
        return res

    # ------------------------------------------------------------------ result
    def _checks(self, res: Dict, r: Dict) -> List[Dict]:
        pc, ff, flags = self.pass_cfg, res['feedforward'], res['checks']
        n_verify = len(res['verify_runs'])
        err, over = res['max_steady_error_mps'], res['max_overshoot_pct']
        out: List[Dict] = []
        if r['abort']:
            out.append({'key': 'drive', 'label': 'Calibration drive completed', 'measured': 'aborted',
                        'limit': 'all segments', 'passed': False, 'why': r['abort'],
                        'fix': 'Clear the floor, check the car can move freely and /odom is live, then Redo.'})
        fit_txt = (f'{ff["static_duty"]:.3f} + {ff["duty_per_mps"]:.3f} x |v| ({len(ff["points"])} moving steps)'
                   if ff['duty_per_mps'] > 0 else (ff['reason'] or 'no fit'))
        rows = [
            ('feedforward_fit', fit_txt, 'static >= 0, per_mps > 0, 3+ moving steps',
             ff['reason'] or 'the car barely moved in the sweep',
             'Check the car moves at the sweep duties (command_owner winner CALIBRATION_RAW, battery, wheels free) '
             'and /odom speed has the right sign (step 6), then Redo.'),
            ('steady_error', f'{err * 100:.1f} cm/s' if n_verify else 'not run', f'<= {pc["max_steady_error_mps"] * 100:g} cm/s',
             'the speed settles away from the target', 'Redo (ki is raised on each failing run); check the floor is level.'),
            ('overshoot', f'{over:.0f} %' if n_verify else 'not run', f'<= {pc["max_overshoot_pct"]:g} %',
             'the speed swings past the target', 'Redo (kp / ki are lowered on each failing run).'),
            ('creep', f'{res["min_moving_speed_mps"] * 100:.1f} cm/s' if ff['duty_per_mps'] > 0 else 'no fit',
             f'<= {res["creep_limit_mps"] * 100:.1f} cm/s',
             'the lowest sweep duty that moves the car is already too fast for creeping',
             'Add a lower duty to procedure.sweep_duties (calibration_steps.yaml) or check the motor / gearing.'),
        ]
        for key, measured, limit, why, fix in rows:
            ok = bool(flags.get(key))
            out.append({'key': key, 'label': CHECK_LABEL[key], 'measured': measured, 'limit': limit, 'passed': ok,
                        'why': '' if ok else why, 'fix': '' if ok else fix})
        return out

    def _result(self, r: Dict) -> Dict:
        cap = r['cap']
        res = csp.analyse(cap, self.cfg)                   # the CLI's keys
        flags = dict(res['checks'])
        checks = self._checks(res, r)
        passed = bool(res['passed']) and not r['abort']
        ff, pid = res['feedforward'], res['pid']
        if passed:
            summary = (f'feedforward {ff["static_duty"]:.3f} + {ff["duty_per_mps"]:.3f} x |v|, kp {pid["kp"]:.3g} '
                       f'ki {pid["ki"]:.3g}, error {res["max_steady_error_mps"] * 100:.1f} cm/s, '
                       f'overshoot {res["max_overshoot_pct"]:.0f} %')
        else:
            summary = 'failed: ' + (r['abort'] or ', '.join(c['label'].lower() for c in checks if not c['passed']))
        doc = dict(res, step=csp.STEP_ID, passed=passed, summary=summary, checks=checks, check_flags=flags,
                   note=r['note'], retunes=list(r['retunes']), capture=cap)
        return cc.plain(doc)

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        ff, pid, cap = res.get('feedforward') or {}, res.get('pid') or {}, res.get('capture')
        if float(ff.get('duty_per_mps', 0.0)) <= 0 or any(k not in pid for k in csp.PID_KEYS) or not cap:
            raise StepRefused('The result has no feedforward / PID values: run the step again.')
        if self.values is not None:
            live_ff, live_pid = self.values['ff'], self.values['pid']
            if (any(abs(live_ff[k] - float(ff[k])) > 1e-6 for k in csp.FF_KEYS)
                    or any(abs(live_pid[k] - float(pid[k])) > 1e-9 for k in csp.PID_KEYS)):
                raise StepRefused(f'{OWNER} runs other feedforward / PID values now than the verified ones: '
                                  'run the step again.')
        path = csp.write_overlay(session, ff, pid)
        cpath = ct.capture_path(session, 'speed.json')
        with open(cpath, 'w', encoding='utf-8') as f:
            json.dump(cap, f, indent=1)
        return [path, cpath]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        got = {k: ct.overlay_value(src_session, OWNER, k) for k in csp.OVERLAY_KEYS[OWNER]}
        miss = [f'{OWNER}.{k}' for k, v in got.items() if v is None]
        if miss:
            raise StepRefused(f'{os.path.basename(src_session)}/params_overlay.yaml has no {", ".join(miss)}: '
                              'run this step again.')
        ff = {k: float(got[f'feedforward.{k}']) for k in csp.FF_KEYS}
        pid = {k: float(got[f'speed_pid.{k}']) for k in csp.PID_KEYS}
        path = csp.write_overlay(session, ff, pid)
        self._set(ff, pid, 'keep previous values')
        return [path]

    # ------------------------------------------------------------------ live view
    def _trace(self, r: Dict) -> List[List[float]]:
        rows = r['rows']
        if not rows:
            return []
        step = max(1, len(rows) // TRACE_POINTS)
        return [[round(t - r['t'], 2), round(v, 4)] for t, v in rows[::step]]

    def live(self, inputs: Dict) -> Dict:
        now = self.clock()
        if self.values is None and not self.busy and self.run is None and now - self.read_at >= READ_RETRY_S:
            self._read()
        r = self.run
        run = None
        if r is not None:
            cap = r['cap']
            seg = r['seg'] if r['phase'] in ('driving', 'stopping') else (
                self._segment(r) if r['phase'] == 'ready' else None)
            ff = r['ff']
            verdicts = []
            for v in cap['verify']:
                ms = [cc.StepMetrics(**m) for m in v['metrics']]
                verdicts.append({'pid': v['pid'], 'metrics': v['metrics'],
                                 'checks': cc.pid_verdict(ms, ff, self.pass_cfg)['checks'] if ff else {}})
            run = {'phase': r['phase'], 'kind': r['kind'], 'segment': seg,
                   'index': r['i'], 'n': len(self.duties) if r['kind'] == 'sweep' else len(self.targets),
                   'verify_run': len(cap['verify']) + 1, 'max_runs': self.max_runs,
                   'elapsed_s': round(now - r['t'], 1) if r['phase'] == 'driving' else 0.0, 'hold_s': self.hold,
                   'speed_mps': round(self.rec.v, 4), 'distance_m': round(self.rec.dist, 3),
                   'trace': self._trace(r) if r['phase'] in ('driving', 'stopping') else [],
                   'sweep': [{'duty': d, 'speed_mps': round(v, 4)} for d, v in cap['sweep']],
                   'fit': None if ff is None else {'static_duty': round(ff.static_duty, 4),
                                                   'duty_per_mps': round(ff.duty_per_mps, 4),
                                                   'residual_mps': round(ff.residual_mps, 4)},
                   'pid': dict(r['pid']), 'metrics': [m.as_dict() for m in r['metrics']], 'runs': verdicts,
                   'retunes': list(r['retunes']), 'note': r['note']}
        return cc.plain({'values': self.values, 'busy': self.busy, 'link_error': self.link_error, 'run': run,
                         'ages': self.rec.ages(now), 'speed_mps': round(self.rec.v, 4),
                         'procedure': {'sweep_duties': self.duties, 'step_targets_mps': self.targets,
                                       'hold_s': self.hold, 'steady_s': self.steady, 'max_runs': self.max_runs},
                         'limits': {'max_steady_error_cm_s': self.pass_cfg['max_steady_error_mps'] * 100,
                                    'max_overshoot_pct': self.pass_cfg['max_overshoot_pct'],
                                    'creep_limit_cm_s': 1.5 * self.pass_cfg['min_creep_speed_mps'] * 100}})
