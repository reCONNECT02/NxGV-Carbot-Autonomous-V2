"""Wizard page for camera extrinsics (front camera only), using the existing calibration solver."""
import copy
import datetime
import os
import tempfile
import threading
from typing import Dict

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import sensor_enabled

from .wizard_core import StepImpl, StepRefused


class ExtrinsicsIpmStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, cameras: Dict, load_cameras, config_dir: str, work_root=''):
        super().__init__(cfg)
        self.base_cameras, self.load_cameras = cameras, load_cameras
        self.config_dir, self.work_root = config_dir, work_root
        self.timeout = float((cfg.get('procedure') or {}).get('capture_timeout_s', 20.0))
        self.roles = []
        for role, sensor in (cameras.get('roles') or {}).items():
            if role in T.CAMERA_ROLES and sensor_enabled(cameras, sensor):   # side roles of old sessions: ignored
                self.roles.append((role, sensor))
        if not self.roles:
            raise ValueError('Step 4: no enabled cameras in cameras.yaml')
        boards = (cfg.get('target') or {}).get('boards') or []
        for role, _ in self.roles:
            if not any(role in (b.get('roles') or []) for b in boards):
                raise ValueError(f'Step 4: no floor board configured for {role}')
        self.run = None
        self.last = None

    def running_sensors(self):
        return [sensor for _, sensor in self.roles] if self.run else []

    def start(self, now: float, inputs: Dict):
        cameras = self.load_cameras()
        for role, sensor in self.roles:
            path = ((cameras.get('sensors') or {}).get(sensor) or {}).get('intrinsics_file', '')
            if not path or not os.path.isfile(path):
                return f'{role}: calibrated intrinsics are missing. Finish and Save step 3, then retry step 4.'
        get = inputs.get('frame')
        baseline = {}
        if callable(get):
            for _, sensor in self.roles:
                current = get(sensor)
                if current is not None:
                    baseline[sensor] = current[0]
        self.run = {'started': now, 'frames': {}, 'seq': baseline, 'thread': None, 'result': None,
                    'error': '', 'cameras': copy.deepcopy(cameras)}
        return None

    def cancel(self):
        self.run = None

    def progress(self, now: float):
        if not self.run:
            return {}
        return {'fraction': len(self.run['frames']) / len(self.roles),
                'samples': len(self.run['frames']), 'target_views': len(self.roles),
                'remaining_s': max(0.0, self.timeout - (now - self.run['started'])),
                'state': 'solving camera mounts' if self.run['thread'] else 'waiting for camera pictures'}

    def tick(self, now: float, inputs: Dict):
        run = self.run
        if run is None:
            return None
        thread = run['thread']
        if thread:
            if thread.is_alive():
                return None
            self.run = None
            result = run['result'] or {'passed': False, 'summary': 'Step 4 solver failed', 'problems': [run['error']]}
            self.last = result
            return result
        get = inputs.get('frame')
        for role, sensor in self.roles:
            if role in run['frames'] or not callable(get):
                continue
            got = get(sensor)
            if got is not None and got[0] != run['seq'].get(sensor):
                run['seq'][sensor] = got[0]
                run['frames'][role] = got[1].copy()
        if len(run['frames']) == len(self.roles):
            run['thread'] = threading.Thread(target=self._solve, args=(run,), daemon=True)
            run['thread'].start()
        elif now - run['started'] >= self.timeout:
            missing = [role for role, _ in self.roles if role not in run['frames']]
            self.run = None
            return {'passed': False, 'summary': 'Camera pictures did not arrive',
                    'problems': [f'No fresh picture from {role}. Check its camera in step 1 and retry.' for role in missing]}
        return None

    def _solve(self, run):
        try:
            import cv2
            from carbot_perception import calib_extrinsics as solver
            if self.work_root:
                os.makedirs(self.work_root, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix='carbot_step4_', dir=self.work_root or None) as root:
                name = 'wizard'
                session = cs.open_session(root, name)
                cs.write_yaml(os.path.join(session, 'data', 'cameras.yaml'), run['cameras'])
                args = ['--data-root', root, '--config-dir', self.config_dir, '--session', name,
                        '--roles'] + [r for r, _ in self.roles] + ['--images']
                for role, image in run['frames'].items():
                    path = os.path.join(root, role + '.png')
                    if not cv2.imwrite(path, image):
                        raise OSError(f'Could not save the {role} picture for calibration')
                    args.append(f'{role}={path}')
                solver.main(args)
                doc = ct.load_yaml(os.path.join(session, solver.RESULT))
                fitted = ct.load_yaml(os.path.join(session, 'data', 'cameras.yaml'))
                problems = [str(x) for x in doc.get('problems') or []]
                rows = doc.get('cameras') or {}
                checks = []
                for role, _ in self.roles:
                    row = rows.get(role) or {}
                    checks.append({'label': role, 'measured': f"ground RMS {row.get('ground_rms_m', 'unknown')} m",
                                   'limit': f"<= {doc.get('limits', {}).get('max_ground_error_m', '?')} m",
                                   'passed': row.get('status') == 'PASS',
                                   'why': '; '.join(p for p in problems if p.startswith(role + ':')),
                                   'fix': 'Check that the printed board is flat, fully visible, and at its measured floor position.'})
                passed = doc.get('status') == 'PASS'
                run['result'] = {'passed': passed,
                                 'summary': 'Camera mounts solved' if passed else 'Camera mount check failed',
                                 'checks': checks, 'problems': problems, 'cameras': rows,
                                 'seam_error_m': doc.get('seam_error_m'),
                                 'mounts': {role: fitted['mounts'][role] for role, _ in self.roles
                                            if role in fitted.get('mounts', {})}}
        except Exception as exc:
            run['error'] = f'{type(exc).__name__}: {exc}. Check camera images and calibration_steps.yaml, then retry.'

    def live(self, inputs: Dict):
        return {'roles': [{'role': role, 'sensor': sensor, 'preview': 'cam_' + role,
                           'captured': bool(self.run and role in self.run['frames'])}
                          for role, sensor in self.roles],
                'state': self.progress(0).get('state', '') if self.run else '',
                'problems': (self.last or {}).get('problems', [])}

    def save_data(self, session: str, result: Dict):
        mounts = result.get('mounts') or {}
        if any(role not in mounts for role, _ in self.roles):
            raise StepRefused('A camera mount is missing from the result. Redo step 4 before saving.')
        path = ct.merge_data(session, 'cameras.yaml', self.load_cameras(),
                             {'mounts': mounts, 'extrinsics_calibrated': True,
                              'extrinsics_date': datetime.datetime.now().isoformat(timespec='seconds')})
        return [path]

    def keep_data(self, src_session: str, session: str):
        src = os.path.join(src_session, 'data', 'cameras.yaml')
        if not os.path.isfile(src):
            raise StepRefused('The earlier session has no cameras.yaml mounts. Run step 4 again.')
        doc = ct.load_yaml(src)
        mounts = doc.get('mounts') or {}
        if not doc.get('extrinsics_calibrated') or any(role not in mounts for role, _ in self.roles):
            raise StepRefused('The earlier session has no passing mounts for the enabled cameras. Run step 4 again.')
        return [ct.merge_data(session, 'cameras.yaml', self.load_cameras(),
                              {'mounts': {role: mounts[role] for role, _ in self.roles},
                               'extrinsics_calibrated': True,
                               'extrinsics_date': doc.get('extrinsics_date', '')})]
