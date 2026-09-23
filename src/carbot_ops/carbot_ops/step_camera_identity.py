"""Calibration step 2 -- camera identity (pure, unit tested).

The car has ONE camera (the front Astra Pro; the two MIPI side cameras were removed
2026-09-24). The user looks at its live preview and confirms it shows what is straight
ahead of the front bumper. Roles that an older session's cameras.yaml still lists
(left_rear / right_rear) are ignored: only topics.CAMERA_ROLES = ('front',) count.

Confirm = RUN with argument JSON {"confirm": true}. ("swap": true is refused: there is
nothing to swap.) The run then checks for procedure.measure_s that the image topic is
live (system_monitor age <= pass.max_image_age_s in >= min_ok_fraction of the
reports): a confirmation of a frozen or missing picture means nothing.

Save writes <session>/data/cameras.yaml: roles, roles_confirmed: true,
roles_confirmed_for: [front]. Keep previous copies those three keys from the older
session, and refuses when that confirmation does not cover the front camera.
"""
import json
import os
from typing import Dict, List, Optional, Tuple

from carbot_common import calib_tools as ct
from carbot_common import topics as T
from carbot_common.data import sensor_enabled

from .sensor_checks import SENSOR_LABEL, ConfigError
from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('measure_s', 'min_samples', 'min_ok_fraction')
PASS_KEYS = ('user_confirmed', 'max_image_age_s')
ROLE_LABEL = {'front': 'Front'}


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def parse_argument(arg: str) -> Tuple[Optional[bool], str]:
    """RUN argument -> (swap, error). swap None = refused. swap=True is parsed (old pages send the key)
    but start() refuses it: there is no second camera to swap with."""
    try:
        a = json.loads(arg) if arg else {}
    except ValueError:
        a = None
    if not isinstance(a, dict) or a.get('confirm') is not True:
        return None, ('Look at the camera picture, then press Confirm: '
                      'Run needs your confirmation.')
    return bool(a.get('swap', False)), ''


class CameraIdentityStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, cameras: Dict):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml camera_identity.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml camera_identity.pass')
        _need(cameras, ('sensors', 'roles', 'roles_confirmed', 'roles_confirmed_for'), 'cameras.yaml')
        for role in T.CAMERA_ROLES:
            sensor = (cameras['roles'] or {}).get(role)
            if not sensor or sensor not in cameras['sensors']:
                raise ConfigError(f'cameras.yaml roles.{role} = {sensor!r} is not a sensor')
            sensor_enabled(cameras, sensor)                  # KeyError if `enabled` is missing
            _need(cameras['sensors'][sensor], ('image_topic',), f'cameras.yaml sensors.{sensor}')
        if not self.pass_cfg['user_confirmed']:
            raise ConfigError('calibration_steps.yaml camera_identity.pass.user_confirmed must be true '
                              '(nobody else can tell which picture is which)')
        self.cameras = cameras
        self.measure_s = float(self.proc['measure_s'])
        self.min_samples = int(self.proc['min_samples'])
        self.min_ok = float(self.proc['min_ok_fraction'])
        self.max_age = float(self.pass_cfg['max_image_age_s'])
        self.t0: Optional[float] = None
        self.swap = False
        self.samples: List[Dict[str, bool]] = []
        self.last_seq = None

    # ------------------------------------------------------------------ roles
    def roles(self, swap: bool = False) -> Dict[str, str]:
        r = dict(self.cameras['roles'])
        return {role: r[role] for role in T.CAMERA_ROLES}

    def enabled_roles(self, roles: Dict[str, str]) -> List[str]:
        return [role for role in T.CAMERA_ROLES if sensor_enabled(self.cameras, roles[role])]

    def _image_state(self, sensor: str, snap: Dict) -> Tuple[str, Optional[float], Optional[float]]:
        """('live' | 'stale' | 'none' | 'wait', hz, age) for a sensor's raw image topic."""
        if snap.get('health_age_s') is None:
            return 'wait', None, None
        t = (snap.get('topics') or {}).get(self.cameras['sensors'][sensor]['image_topic'])
        if not t or t.get('age') is None or t['age'] < 0:
            return 'none', None, None
        return ('live' if t['age'] <= self.max_age else 'stale'), t.get('hz'), t['age']

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        snap = inputs.get('snap') or {}
        # as loaded (never swapped): the preview topics follow the launch's roles until relaunch
        roles = self.roles(False)
        cams = []
        for role in T.CAMERA_ROLES:
            sensor = roles[role]
            s = self.cameras['sensors'][sensor]
            on = sensor_enabled(self.cameras, sensor)
            state, hz, age = self._image_state(sensor, snap) if on else ('off', None, None)
            cams.append({'role': role, 'label': ROLE_LABEL[role], 'sensor': sensor,
                         'sensor_label': SENSOR_LABEL.get(sensor, sensor), 'enabled': on,
                         'topic': s['image_topic'], 'preview': 'cam_' + role, 'state': state,
                         'hz': None if hz is None else round(float(hz), 1),
                         'age': None if age is None else round(float(age), 2)})
        return {'cameras': cams,
                'roles_confirmed': bool(self.cameras['roles_confirmed']),
                'roles_confirmed_for': list(self.cameras['roles_confirmed_for'] or []),
                'max_image_age_s': self.max_age, 'health_age_s': snap.get('health_age_s')}

    # ------------------------------------------------------------------ run
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        swap, err = parse_argument(inputs.get('argument', ''))
        if swap is None:
            return err
        if swap:
            return 'There is only the front camera (the side cameras were removed): nothing to swap. Press Confirm.'
        if not self.enabled_roles(self.roles()):
            return 'The front camera is switched off in cameras.yaml: switch it on first.'
        self.t0, self.swap, self.samples, self.last_seq = now, swap, [], inputs.get('health_seq')
        return None

    def cancel(self) -> None:
        self.t0, self.samples, self.swap = None, [], False

    def progress(self, now: float) -> Dict:
        if self.t0 is None:
            return {}
        el = now - self.t0
        return {'elapsed_s': round(el, 1), 'remaining_s': round(max(0.0, self.measure_s - el), 1),
                'fraction': round(min(1.0, el / self.measure_s), 2), 'samples': len(self.samples)}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        if self.t0 is None:
            return None
        roles = self.roles(self.swap)
        seq = inputs.get('health_seq')
        if seq is not None and seq != self.last_seq:
            self.last_seq = seq
            snap = inputs.get('snap') or {}
            self.samples.append({role: self._image_state(roles[role], snap)[0] == 'live'
                                 for role in self.enabled_roles(roles)})
        if now - self.t0 < self.measure_s:
            return None
        res = self._result(roles)
        self.t0, self.swap = None, False
        return res

    def _result(self, roles: Dict[str, str]) -> Dict:
        enabled = self.enabled_roles(roles)
        n = len(self.samples)
        checks = []
        for role in enabled:
            sensor = roles[role]
            ok_n = sum(1 for smp in self.samples if smp.get(role))
            frac = ok_n / n if n else 0.0
            passed = n >= self.min_samples and frac >= self.min_ok
            topic = self.cameras['sensors'][sensor]['image_topic']
            checks.append({
                'key': f'stream_{role}', 'label': f'{ROLE_LABEL[role]} picture ({SENSOR_LABEL.get(sensor, sensor)}) streaming',
                'measured': f'{ok_n} of {n} reports live', 'limit': f'age <= {self.max_age:g} s in {self.min_ok:.0%}',
                'passed': passed, 'ok_fraction': round(frac, 2),
                'why': '' if passed else f'{topic} was not live while you confirmed, so the picture you looked at may be frozen or old.',
                'fix': '' if passed else 'Go back to step 1 and press Restart camera drivers, wait for the picture to move, then Confirm again.'})
        checks.append({'key': 'user_confirmed', 'label': 'You confirmed the pictures',
                       'measured': 'as shown',
                       'limit': 'confirmed', 'passed': True, 'why': '', 'fix': ''})
        skipped = {role: roles[role] for role in T.CAMERA_ROLES if role not in enabled}
        passed = n >= self.min_samples and all(c['passed'] for c in checks)
        names = ', '.join(ROLE_LABEL[r].lower() for r in enabled)
        if n < self.min_samples:
            summary = (f'only {n} health reports in {self.measure_s:g} s (need {self.min_samples}): '
                       'system_monitor is not publishing')
        elif passed:
            summary = f'confirmed {names}' + \
                      (f'; skipped {", ".join(ROLE_LABEL[r].lower() for r in skipped)} (switched off)' if skipped else '')
        else:
            summary = 'a confirmed camera was not streaming'
        return {'passed': passed, 'summary': summary, 'checks': checks,
                'roles': roles, 'swapped': self.swap, 'confirmed_roles': enabled,
                'skipped_roles': skipped, 'measure_s': self.measure_s, 'samples': n}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        if not res.get('confirmed_roles'):
            raise StepRefused('Nothing was confirmed: press Confirm first.')
        return [ct.merge_data(session, 'cameras.yaml', self.cameras,
                              {'roles': dict(res['roles']), 'roles_confirmed': True,
                               'roles_confirmed_for': list(res['confirmed_roles'])})]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        path = os.path.join(src_session, 'data', 'cameras.yaml')
        if not os.path.isfile(path):
            raise StepRefused(f'{os.path.basename(src_session)} has no data/cameras.yaml with the camera '
                              'roles: run this step again.')
        old = ct.load_yaml(path)
        roles = old.get('roles') or {}
        if not old.get('roles_confirmed') or any(r not in roles or roles[r] not in self.cameras['sensors']
                                                 for r in T.CAMERA_ROLES):
            raise StepRefused(f'The camera roles in {os.path.basename(src_session)} are not confirmed: '
                              'run this step again.')
        done = set(old.get('roles_confirmed_for') or [])
        now_on = [r for r in T.CAMERA_ROLES if sensor_enabled(self.cameras, roles[r])]
        missing = [r for r in now_on if r not in done]
        if missing:
            raise StepRefused(f'{os.path.basename(src_session)} confirmed only '
                              f'{", ".join(sorted(done)) or "nothing"}, but '
                              f'{", ".join(ROLE_LABEL[r].lower() for r in missing)} is switched on now: '
                              'run this step again.')
        return [ct.merge_data(session, 'cameras.yaml', self.cameras,
                              {'roles': {r: roles[r] for r in T.CAMERA_ROLES}, 'roles_confirmed': True,
                               'roles_confirmed_for': sorted(done, key=T.CAMERA_ROLES.index)})]
