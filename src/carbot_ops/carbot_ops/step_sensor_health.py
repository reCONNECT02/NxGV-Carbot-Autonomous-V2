"""Calibration step 1 -- sensor health check (pure, unit tested).

Live: every SystemHealth snapshot is evaluated (sensor_checks.evaluate) and
shown as a table, green/red per row, with the reason and the fix.
Run: collects the snapshots that arrive during procedure.measure_s and passes
when each check was ok in >= procedure.min_ok_fraction of them.
Writes only its result file + summary (no calibration data), so keeping a
previous value is allowed.
"""
from typing import Dict, Optional

from . import sensor_checks as sc
from .wizard_core import StepImpl

PROC_KEYS = ('measure_s', 'min_samples', 'min_ok_fraction')


class SensorHealthStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, cameras: Dict, uwb: Dict):
        super().__init__(cfg)
        proc = cfg.get('procedure') or {}
        miss = [k for k in PROC_KEYS if k not in proc]
        if miss:
            raise sc.ConfigError('calibration_steps.yaml sensor_health.procedure: missing ' + ', '.join(miss))
        self.pass_cfg = cfg.get('pass') or {}
        self.checks = sc.build_checks(cameras, uwb, self.pass_cfg)
        self.max_age = float(self.pass_cfg['max_age_s'])
        self.measure_s = float(proc['measure_s'])
        self.min_samples = int(proc['min_samples'])
        self.min_ok = float(proc['min_ok_fraction'])
        self.t0: Optional[float] = None
        self.samples = []
        self.last_seq = None

    def live(self, inputs: Dict) -> Dict:
        rows = sc.evaluate(self.checks, inputs.get('snap') or {}, self.max_age)
        return {'rows': rows, 'n_ok': sum(1 for r in rows if r['state'] == 'ok'), 'n': len(rows),
                'health_age_s': (inputs.get('snap') or {}).get('health_age_s')}

    def start(self, now: float, inputs: Dict) -> Optional[str]:
        self.t0, self.samples, self.last_seq = now, [], inputs.get('health_seq')
        return None

    def cancel(self) -> None:
        self.t0, self.samples = None, []

    def progress(self, now: float) -> Dict:
        if self.t0 is None:
            return {}
        el = now - self.t0
        return {'elapsed_s': round(el, 1), 'remaining_s': round(max(0.0, self.measure_s - el), 1),
                'fraction': round(min(1.0, el / self.measure_s), 2), 'samples': len(self.samples)}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        if self.t0 is None:
            return None
        seq = inputs.get('health_seq')
        if seq is not None and seq != self.last_seq:
            self.last_seq = seq
            self.samples.append(sc.evaluate(self.checks, inputs.get('snap') or {}, self.max_age))
        if now - self.t0 < self.measure_s:
            return None
        res = sc.aggregate(self.samples, self.min_ok)
        if len(self.samples) < self.min_samples:
            res['passed'] = False
            res['summary'] = (f'only {len(self.samples)} health reports in {self.measure_s:.0f} s '
                              f'(need {self.min_samples}): system_monitor is not publishing')
        res.update({'measure_s': self.measure_s, 'min_ok_fraction': self.min_ok,
                    'limits': {k: self.pass_cfg[k] for k in self.pass_cfg}})
        self.t0 = None
        return res
