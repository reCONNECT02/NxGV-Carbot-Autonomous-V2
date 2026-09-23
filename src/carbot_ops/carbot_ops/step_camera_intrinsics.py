"""Calibration step 3 -- camera intrinsics, one camera per Run (pure, unit tested).

Same math as the terminal tool (carbot_perception.calib_intrinsics): the
hand-held chessboard (step 3 target) is shown to ONE camera; a view is kept
automatically when the board is held still in a new position/size/tilt
(calib_core.ViewCollector); then both lens models are fitted and the better one
kept (calib_core.calibrate_intrinsics).

* Run's argument is the sensor name (cameras.yaml sensors key). Without one,
  the first enabled sensor that has not passed yet is taken.
* Sensors with cameras.yaml `enabled: false` cannot be run and do not count:
  the step passes when every ENABLED sensor in per_sensor has passed.
* Frames come from the node: inputs['frame'](sensor) -> (seq, bgr) | None.
  The node only subscribes while this step is running, and decodes lazily.
* Detection runs in tick (node tick_hz); the fit (a few seconds on the RDK)
  runs in a thread so the wizard keeps publishing.
* Save writes <session>/intrinsics/<sensor>.yaml and cameras.yaml
  sensors.<sensor>.intrinsics_file through the step-2 data hooks.
"""
import os
import shutil
import threading
import time
from typing import Callable, Dict, List, Optional

from carbot_common import calib_tools as ct
from carbot_common.data import sensor_enabled

from .wizard_core import StepImpl

PROC_KEYS = ('extra_views', 'capture_timeout_s', 'novelty', 'still_px', 'max_live_corners')
PASS_KEYS = ('min_views', 'max_reprojection_px', 'min_coverage_cells')
LABEL = {'astra': 'Astra Pro (front)', 'ov5647': 'OV5647 (MIPI ch 2)', 'imx219': 'IMX219 (MIPI ch 0)'}


class ConfigError(ValueError):
    pass


def _calib():
    """calib_core is imported lazily: OpenCV is heavy and the tests inject a fake."""
    from carbot_perception import calib_core
    return calib_core


class CameraIntrinsicsStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, cameras: Dict, calib=None, save_intrinsics: Optional[Callable] = None):
        super().__init__(cfg)
        proc, pas, tgt = cfg.get('procedure') or {}, cfg.get('pass') or {}, cfg.get('target') or {}
        for where, d, keys in (('procedure', proc, PROC_KEYS), ('pass', pas, PASS_KEYS),
                               ('target', tgt, ('inner_corners', 'square_m'))):
            miss = [k for k in keys if k not in d]
            if miss:
                raise ConfigError(f'calibration_steps.yaml camera_intrinsics.{where}: missing {", ".join(miss)}')
        self.cols, self.rows = (int(x) for x in tgt['inner_corners'])
        self.square = float(tgt['square_m'])
        self.proc, self.pas = proc, pas
        self.min_views = int(pas['min_views'])
        self.target_views = self.min_views + int(proc['extra_views'])
        self.cameras = cameras
        self.sensors: List[Dict] = []
        for name in cfg.get('per_sensor') or []:
            if name not in cameras['sensors']:
                raise ConfigError(f'calibration_steps.yaml camera_intrinsics.per_sensor: {name} is not in cameras.yaml')
            s = cameras['sensors'][name]
            self.sensors.append({'name': name, 'label': LABEL.get(name, name), 'topic': s.get('image_topic', ''),
                                 'enabled': sensor_enabled(cameras, name)})
        if not any(s['enabled'] for s in self.sensors):
            raise ConfigError('camera_intrinsics: no enabled camera in per_sensor (cameras.yaml enabled)')
        self._calib = calib
        self._save_intr = save_intrinsics
        self.results: Dict[str, Dict] = {}      # sensor -> result row (this wizard run)
        self.fits: Dict[str, object] = {}       # sensor -> Intrinsics waiting for Save
        self.run: Optional[Dict] = None

    # ------------------------------------------------------------------ helpers
    @property
    def calib(self):
        if self._calib is None:
            self._calib = _calib()
        return self._calib

    def enabled(self) -> List[str]:
        return [s['name'] for s in self.sensors if s['enabled']]

    def running_sensor(self) -> str:
        return self.run['sensor'] if self.run else ''

    def _pick(self, arg: str) -> (str, str):
        arg = (arg or '').strip()
        if arg:
            s = next((x for x in self.sensors if x['name'] == arg), None)
            if s is None:
                return '', f'Unknown camera {arg!r}. This step calibrates: {", ".join(self.enabled())}'
            if not s['enabled']:
                return '', (f'{s["label"]} is switched off (cameras.yaml sensors.{arg}.enabled: false): '
                            'camera not detected. Fix the hardware and set enabled: true to calibrate it.')
            return arg, ''
        todo = [n for n in self.enabled() if (self.results.get(n) or {}).get('status') != 'PASS']
        return (todo or self.enabled())[0], ''

    # ------------------------------------------------------------------ StepImpl
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        sensor, err = self._pick(inputs.get('argument', ''))
        if err:
            return err
        self.run = {'sensor': sensor, 't0': now, 'col': None, 'prev': None, 'seq': None, 'state': 'waiting for frames',
                    'corners': None, 'size': None, 'frames': 0, 'thread': None, 'out': None, 'error': ''}
        return None

    def cancel(self) -> None:
        self.run = None

    def progress(self, now: float) -> Dict:
        r = self.run
        if r is None:
            return {}
        el = now - r['t0']
        views = len(r['col'].views) if r['col'] else 0
        return {'sensor': r['sensor'], 'elapsed_s': round(el, 1),
                'remaining_s': round(max(0.0, float(self.proc['capture_timeout_s']) - el), 1),
                'fraction': round(min(1.0, views / self.target_views), 2), 'samples': views,
                'views': views, 'target_views': self.target_views, 'calibrating': r['thread'] is not None}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        if r['thread'] is not None:                       # fitting in the background
            if r['thread'].is_alive():
                return None
            return self._finish(r)
        get = inputs.get('frame')
        got = get(r['sensor']) if callable(get) else None
        if got is not None and got[0] != r['seq']:
            r['seq'], img = got
            r['frames'] += 1
            self._offer(r, img)
        col = r['col']
        views = len(col.views) if col else 0
        cells = col.coverage_cells() if col else 0
        timed_out = now - r['t0'] >= float(self.proc['capture_timeout_s'])
        if (views >= self.target_views and cells >= int(self.pas['min_coverage_cells'])) or timed_out:
            if views < 6:
                return self._fail_early(r, timed_out)
            r['state'] = 'calibrating'
            r['thread'] = threading.Thread(target=self._fit, args=(r,), daemon=True)
            r['thread'].start()
        return None

    def _offer(self, r: Dict, img) -> None:
        cc = self.calib
        h, w = img.shape[:2]
        if r['col'] is None:
            r['col'] = cc.ViewCollector(w, h, float(self.proc['novelty']))
        r['size'] = [w, h]
        c = cc.detect_chessboard(img, self.cols, self.rows, fast=True)
        still = c is not None and r['prev'] is not None and \
            float(((c - r['prev']) ** 2).sum(axis=1).mean() ** 0.5) < float(self.proc['still_px'])
        r['prev'] = c
        added = bool(still and r['col'].offer(c, self.cols, self.rows))
        r['corners'] = c
        now = time.monotonic()
        if added:
            r['added_t'] = now
        # hold "new view captured" (green) ~1 s: one green frame is too short to see on the page
        if c is not None and now - r.get('added_t', -1e9) < 1.0:
            r['state'] = 'new view captured'
            return
        r['state'] = ('hold the board still' if c is not None and not still
                      else 'board seen' if c is not None else 'no board in view')

    def _fit(self, r: Dict) -> None:
        try:
            col = r['col']
            r['out'] = self.calib.calibrate_intrinsics(col.views, self.cols, self.rows, self.square,
                                                       (col.width, col.height))
        except Exception as e:  # noqa: BLE001  shown as a failed result, never kills the wizard
            r['error'] = repr(e)

    def _row(self, sensor: str, status: str, **kw) -> Dict:
        return dict({'sensor': sensor, 'label': LABEL.get(sensor, sensor), 'status': status}, **kw)

    def _fail_early(self, r: Dict, timed_out: bool) -> Dict:
        s = r['sensor']
        views = len(r['col'].views) if r['col'] else 0
        why = (f'no frames from {self._topic(s)} in {self.proc["capture_timeout_s"]} s' if r['frames'] == 0 else
               f'only {views} usable views in {self.proc["capture_timeout_s"]} s (need at least 6 to fit)')
        fix = ('Check step 1 shows this camera publishing.' if r['frames'] == 0 else
               'Hold the whole board in view, still for a moment, and move it to new places, distances and tilts.')
        self.results[s] = self._row(s, 'FAIL', why=why, fix=fix, views=views,
                                    coverage_cells=r['col'].coverage_cells() if r['col'] else 0)
        self.fits.pop(s, None)
        self.run = None
        return self._result()

    def _finish(self, r: Dict) -> Dict:
        s, col = r['sensor'], r['col']
        self.run = None
        if r['error'] or r['out'] is None:
            self.results[s] = self._row(s, 'FAIL', why=f'fit failed: {r["error"] or "no result"}',
                                        fix='Press Redo and collect more varied views.', views=len(col.views))
            self.fits.pop(s, None)
            return self._result()
        out = r['out']
        fit = out['chosen']
        used, cells = len(out['used_views']), col.coverage_cells()
        mx = float(self.pas['max_reprojection_px'])
        checks = {'views': used >= self.min_views, 'reprojection': fit.rms_px <= mx,
                  'coverage': cells >= int(self.pas['min_coverage_cells'])}
        why = []
        if not checks['views']:
            why.append(f'{used} views kept after dropping bad ones, need {self.min_views}')
        if not checks['reprojection']:
            why.append(f'reprojection error {fit.rms_px:.3f} px is above {mx} px')
        if not checks['coverage']:
            why.append(f'board covered {cells} of 9 image areas, need {self.pas["min_coverage_cells"]}')
        K = fit.intr.K
        fish = out.get('fisheye')
        row = self._row(s, 'PASS' if all(checks.values()) else 'FAIL', checks=checks, model=fit.intr.model,
                        rms_px=round(float(fit.rms_px), 4), pinhole_rms_px=round(float(out['pinhole'].rms_px), 4),
                        fisheye_rms_px=None if fish is None else round(float(fish.rms_px), 4),
                        views=used, coverage_cells=cells, image_size=[col.width, col.height],
                        hfov_deg=round(float(fit.intr.hfov_deg()), 2),
                        fx=round(float(K[0][0]), 2), fy=round(float(K[1][1]), 2),
                        cx=round(float(K[0][2]), 2), cy=round(float(K[1][2]), 2),
                        why='; '.join(why),
                        fix='' if not why else ('Redo: bring the board close to every edge and corner of the image, '
                                                'tilt it, and hold it still each time.'))
        self.results[s] = row
        if row['status'] == 'PASS':
            fit.intr.meta.update({'sensor': s, 'coverage_cells': cells})
            self.fits[s] = fit.intr
        else:
            self.fits.pop(s, None)
        return self._result()

    def _topic(self, sensor: str) -> str:
        return next((x['topic'] for x in self.sensors if x['name'] == sensor), '')

    def _result(self) -> Dict:
        en = self.enabled()
        rows = [self.results.get(n) for n in en]
        done = [r for r in rows if r and r['status'] == 'PASS']
        todo = [n for n in en if not self.results.get(n)]
        failed = [r for r in rows if r and r['status'] != 'PASS']
        passed = len(done) == len(en)
        checks = []
        for sname in en:
            r = self.results.get(sname)
            lab = LABEL.get(sname, sname)
            if r is None:
                checks.append({'label': lab, 'measured': 'not run', 'limit': 'PASS', 'passed': False,
                               'why': 'not calibrated yet', 'fix': f'Choose {lab} and press Run.'})
            else:
                meas = (f'{r["rms_px"]:.3f} px · {r["views"]} views · {r.get("coverage_cells", 0)}/9 areas'
                        if 'rms_px' in r else f'{r.get("views", 0)} views')
                checks.append({'label': lab, 'measured': meas,
                               'limit': f'≤ {self.pas["max_reprojection_px"]} px · ≥ {self.min_views} views · '
                                        f'{self.pas["min_coverage_cells"]}/9',
                               'passed': r['status'] == 'PASS', 'why': r.get('why', ''), 'fix': r.get('fix', '')})
        off = [x['label'] for x in self.sensors if not x['enabled']]
        summary = (f'{len(done)} of {len(en)} cameras calibrated' +
                   (f' ({", ".join(off)} switched off)' if off else ''))
        if failed:
            summary += '; ' + ', '.join(r['label'] for r in failed) + ' failed'
        elif todo:
            summary += '; still to do: ' + ', '.join(LABEL.get(n, n) for n in todo)
        return {'passed': passed, 'summary': summary, 'checks': checks,
                'sensors': {k: v for k, v in self.results.items()},
                'disabled': [x['name'] for x in self.sensors if not x['enabled']]}

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        r = self.run
        rows = []
        for x in self.sensors:
            res = self.results.get(x['name'])
            st = ('off' if not x['enabled'] else 'capturing' if r and r['sensor'] == x['name']
                  else (res or {}).get('status', 'todo').lower())
            rows.append({'name': x['name'], 'label': x['label'], 'topic': x['topic'], 'enabled': x['enabled'],
                         'state': st, 'result': res,
                         'note': '' if x['enabled'] else 'Camera not detected: switched off in cameras.yaml'})
        cap = None
        if r is not None:
            col = r['col']
            corners = None
            if r['corners'] is not None and r['size']:
                w, h = r['size']
                step = max(1, len(r['corners']) // int(self.proc['max_live_corners']))
                corners = [[round(float(u) / w, 4), round(float(v) / h, 4)] for u, v in r['corners'][::step]]
            cap = {'sensor': r['sensor'], 'label': LABEL.get(r['sensor'], r['sensor']), 'state': r['state'],
                   'views': len(col.views) if col else 0, 'target_views': self.target_views,
                   'min_views': self.min_views, 'coverage': col.coverage.tolist() if col else [[0] * 3] * 3,
                   'coverage_cells': col.coverage_cells() if col else 0, 'frames': r['frames'],
                   'corners': corners, 'size': r['size'], 'calibrating': r['thread'] is not None,
                   'preview': x_role(self.cameras, r['sensor'])}
        return {'sensors': rows, 'capture': cap,
                'board': {'inner_corners': [self.cols, self.rows], 'square_mm': round(self.square * 1000, 1)}}

    # ------------------------------------------------------------------ save / keep (step-2 hooks)
    def save_data(self, session: str, result: Dict) -> List[str]:
        save = self._save_intr
        if save is None:
            from carbot_perception.camera_model import save_intrinsics as save
        paths, upd = [], {}
        for s, intr in self.fits.items():
            p = os.path.join(session, 'intrinsics', f'{s}.yaml')
            save(p, intr, s)
            paths.append(p)
            upd[s] = {'intrinsics_file': os.path.abspath(p)}
        if upd:
            paths.append(ct.merge_data(session, 'cameras.yaml', self.cameras, {'sensors': upd}))
        return paths

    def keep_data(self, src_session: str, session: str) -> List[str]:
        src = os.path.join(src_session, 'intrinsics')
        paths, upd = [], {}
        for s in self.enabled():
            f = os.path.join(src, f'{s}.yaml')
            if os.path.isfile(f):
                dst = os.path.join(session, 'intrinsics', f'{s}.yaml')
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(f, dst)
                paths.append(dst)
                upd[s] = {'intrinsics_file': os.path.abspath(dst)}
        if upd:
            paths.append(ct.merge_data(session, 'cameras.yaml', self.cameras, {'sensors': upd}))
        return paths


def x_role(cameras: Dict, sensor: str) -> str:
    """Role whose preview stream shows this sensor ('' if none)."""
    return next((r for r, s in (cameras.get('roles') or {}).items() if s == sensor), '')
