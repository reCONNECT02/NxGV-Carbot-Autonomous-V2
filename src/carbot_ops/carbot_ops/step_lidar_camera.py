"""Calibration step 5 -- LiDAR-camera alignment (pure, unit tested).

Live: the latest /scan drawn into the front camera image (current mount), the
target candidates the LiDAR sees ahead, the captures so far and the running
estimate.
STEP ops (GUI -> wizard, action STEP, argument JSON {"op": ...}):
  CAPTURE {u, v}   u, v = clicked foot of the target in the front image, 0..1
  UNDO             drop the last capture
  CLEAR            drop all captures
Run (argument JSON, optional {"lidar_x_m", "lidar_z_m"} measured with a tape):
evaluates the captures at once (lidar_align.evaluate).
Save writes <session>/data/cameras.yaml lidar_to_camera and the overlay
tunnel_wall_follower.lidar_angle_offset + carbot_tf.base_to_laser (the same
corrected yaw: they must agree, test_required_keys enforces it for the repo
files). The launch applies them on the NEXT start.
"""
import json
import math
import time
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from carbot_common import calib_tools as ct

from . import lidar_align as la
from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('roi_min_m', 'roi_max_m', 'cluster_gap_m', 'min_cluster_points', 'max_cluster_width_m',
             'match_max_m', 'scan_max_age_s', 'capture_scans', 'min_captures', 'min_spread_deg',
             'overlay_max_points', 'mount_x_range_m', 'mount_z_range_m', 'reload_period_s')
PASS_KEYS = ('max_bearing_error_deg', 'max_correction_deg', 'warn_range_diff_m')
ROLE = 'front'
IMAGE_KEY = 'cam_front'          # camera_preview JPEG key the page shows (layer cam, role front)


class ConfigError(Exception):
    pass


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing ' + ', '.join(miss))


def sensor_size(s: Dict, sensor: str):
    w = s.get('width', s.get('image_width'))
    h = s.get('height', s.get('image_height'))
    if not w or not h:
        raise ConfigError(f'cameras.yaml sensors.{sensor}: width/height (or image_width/image_height) missing')
    return int(w), int(h)


def _r(v, n=3):
    return round(float(v), n)


class LidarCameraStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, base_cameras: Dict, load_cameras: Callable[[], Dict],
                 load_mount: Callable[[], List[float]], clock: Callable[[], float] = time.monotonic):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml lidar_camera.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml lidar_camera.pass')
        self.base_cameras = base_cameras
        self.load_cameras, self.load_mount, self.clock = load_cameras, load_mount, clock
        self.scan_buffer_n = max(1, int(self.proc['capture_scans']))
        self.captures: List[Dict] = []
        self.pending: Optional[Dict] = None      # result computed by start(), returned by tick()
        self._geo = None
        self._geo_t = -1e9

    # ------------------------------------------------------------------ geometry
    def geometry(self, force: bool = False) -> Dict:
        """Front camera (intrinsics, mount, size) and LiDAR mount, reloaded every
        reload_period_s (steps 3/4 may have just saved new files)."""
        now = self.clock()
        if not force and self._geo is not None and now - self._geo_t < float(self.proc['reload_period_s']):
            return self._geo
        from carbot_perception import camera_model as cm
        cams = self.load_cameras()
        if ROLE not in (cams.get('roles') or {}):
            raise ConfigError('cameras.yaml roles.front missing')
        sensor, s, mount, hfov = cm.camera_setup(cams, ROLE)
        w, h = sensor_size(s, sensor)
        intr = cm.intrinsics_for(cams, ROLE, w, h)
        lm = [float(x) for x in self.load_mount()]
        if len(lm) != 6:
            raise ConfigError(f'carbot_tf.base_to_laser must have 6 numbers, got {lm}')
        calibrated = bool((s.get('intrinsics_file') or '').strip())
        self._geo = {'cm': cm, 'sensor': sensor, 'intr': intr, 'cam': mount, 'w': w, 'h': h,
                     # search area: the camera view widened by the largest correction we accept,
                     # else a target near the image edge would be lost by exactly the error we measure
                     'half': math.radians(min(89.0, intr.hfov_deg() / 2.0 + float(self.pass_cfg['max_correction_deg']))),
                     'laser': lm,
                     'intr_src': 'calibrated (step 3)' if calibrated else f'ideal pinhole, assumed hfov {hfov:.0f}°'}
        self._geo_t = now
        return self._geo

    def _scans(self, inputs: Dict) -> List[Dict]:
        """Recent scans, newest last, or [] when the newest is too old."""
        scans = list(inputs.get('scans') or [])
        if not scans:
            return []
        if inputs.get('now', self.clock()) - float(scans[-1]['t']) > float(self.proc['scan_max_age_s']):
            return []
        return scans[-self.scan_buffer_n:]

    def _project(self, g: Dict, base_xy: np.ndarray) -> np.ndarray:
        """base_link floor-plane points at LiDAR height -> normalised image uv (N,2), NaN if not visible."""
        if len(base_xy) == 0:
            return np.zeros((0, 2))
        pts = np.column_stack([base_xy, np.full(len(base_xy), g['laser'][2])])
        uv, ok, depth = g['cm'].project_ground(g['intr'], g['cam'], pts)
        uv = np.asarray(uv, float) / np.array([g['w'], g['h']])
        vis = np.asarray(ok, bool) & (np.asarray(depth) > 0) & (uv[:, 0] >= 0) & (uv[:, 0] <= 1) & \
            (uv[:, 1] >= 0) & (uv[:, 1] <= 1)
        uv[~vis] = np.nan
        return uv

    # ------------------------------------------------------------------ STEP ops
    def handle(self, op: str, args: Dict, inputs: Dict) -> Dict:
        op = (op or '').upper()
        if op == 'CLEAR':
            n, self.captures = len(self.captures), []
            return {'ok': True, 'message': f'{n} capture(s) cleared'}
        if op == 'UNDO':
            if not self.captures:
                return {'ok': False, 'message': 'No capture to undo'}
            self.captures.pop()
            return {'ok': True, 'message': f'Last capture removed ({len(self.captures)} left)'}
        if op != 'CAPTURE':
            return {'ok': False, 'message': f'Unknown step 5 operation {op!r} (CAPTURE, UNDO, CLEAR)'}
        try:
            u, v = float(args['u']), float(args['v'])
        except (KeyError, TypeError, ValueError):
            return {'ok': False, 'message': 'Click the foot of the target in the camera image first'}
        if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
            return {'ok': False, 'message': 'The clicked point is outside the image'}
        g = self.geometry()
        scans = self._scans(inputs)
        if not scans:
            return {'ok': False, 'message': 'No recent /scan: is the LiDAR running (step 1)?'}
        gr, ok = g['cm'].pixels_to_ground(g['intr'], g['cam'], np.array([[u * g['w'], v * g['h']]]))
        if not bool(ok[0]):
            return {'ok': False, 'message': 'That point is above the horizon: click where the target touches the floor'}
        cap, why = la.match_capture(scans, gr[0], g['laser'], self.proc, g['half'])
        if cap is None:
            return {'ok': False, 'message': f'Not captured: {why}. The target must be taller than the LiDAR, '
                                            'nothing else within ~20 cm of it, and the click on its foot.'}
        cap.update({'u': _r(u, 4), 'v': _r(v, 4)})
        self.captures.append(cap)
        err = la.capture_errors([cap], g['laser'])[0]
        return {'ok': True, 'message': f'Capture {len(self.captures)}: bearing {err["base_bearing_deg"]:+.1f}°, '
                                       f'LiDAR vs camera {math.degrees(err["error"]):+.2f}°'}

    # ------------------------------------------------------------------ run
    def _run_mount(self, arg: str, g: Dict) -> List[float]:
        m = list(g['laser'])
        if not (arg or '').strip():
            return m
        try:
            a = json.loads(arg)
        except ValueError:
            raise StepRefused(f'Run argument is not JSON: {arg!r}')
        for key, i, rng in (('lidar_x_m', 0, 'mount_x_range_m'), ('lidar_z_m', 2, 'mount_z_range_m')):
            val = a.get(key) if isinstance(a, dict) else None
            if val in (None, ''):
                continue
            try:
                val = float(val)
            except (TypeError, ValueError):
                raise StepRefused(f'{key} must be a number of metres, got {val!r}')
            lo, hi = (float(x) for x in self.proc[rng])
            if not lo <= val <= hi:
                raise StepRefused(f'{key} {val:.3f} m is outside {lo:.2f} .. {hi:.2f} m '
                                  f'(calibration_steps.yaml lidar_camera.procedure.{rng}): measure again')
            m[i] = val
        return m

    def start(self, now: float, inputs: Dict) -> Optional[str]:
        if not self.captures:
            return (f'Capture the target at {int(self.proc["min_captures"])} or more positions first '
                    '(click its foot in the camera image, then Capture).')
        g = self.geometry(force=True)
        try:
            mount = self._run_mount(inputs.get('argument', ''), g)
        except StepRefused as e:
            return str(e)
        res = la.evaluate(self.captures, mount, self.proc, self.pass_cfg)
        est = res.pop('estimate')
        after = list(mount)
        after[3] = est['new_yaw']
        res.update({
            'yaw_offset_deg': _r(math.degrees(est['offset'])),
            'max_residual_deg': _r(est['max_residual_deg']),
            'spread_deg': _r(est['spread_deg'], 1),
            'base_to_laser_before': [_r(x, 5) for x in g['laser']],
            'base_to_laser': [_r(x, 5) for x in after],
            'intrinsics': g['intr_src'],
            'captures': [dict(c, bearing_deg=_r(r['base_bearing_deg'], 2), error_deg=_r(math.degrees(r['error'])),
                              residual_deg=_r(math.degrees(r['residual'])), cam_range_m=_r(r['cam_range_m']),
                              lidar_range_m=_r(r['lidar_range_m']))
                         for c, r in zip(self.captures, est['rows'])],
            'limits': dict(self.pass_cfg),
        })
        for c in res['checks']:
            c['measured'] = str(c['measured'])
        self.pending = res
        return None

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        res, self.pending = self.pending, None
        return res

    def cancel(self) -> None:
        self.pending = None

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        m = [float(x) for x in res['base_to_laser']]
        cam = ct.merge_data(session, 'cameras.yaml', self.base_cameras, {'lidar_to_camera': {
            'calibrated': True, 'yaw_offset_deg': float(res['yaw_offset_deg']),
            'base_to_laser': m, 'max_residual_deg': float(res['max_residual_deg']),
            'n_captures': int(res['n_captures'])}})
        ov = ct.merge_overlay(session, 'tunnel_wall_follower', {'lidar_angle_offset': m[3]})
        ct.merge_overlay(session, 'carbot_tf', {'base_to_laser': m})
        self._geo = None
        return [cam, ov]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        import os
        src_cam = os.path.join(src_session, 'data', 'cameras.yaml')
        l2c = (ct.load_yaml(src_cam).get('lidar_to_camera') if os.path.isfile(src_cam) else None) or {}
        m = ct.overlay_value(src_session, 'carbot_tf', 'base_to_laser')
        off = ct.overlay_value(src_session, 'tunnel_wall_follower', 'lidar_angle_offset')
        if not l2c.get('calibrated') or m is None or off is None:
            raise StepRefused(f'{os.path.basename(src_session)} has no complete step 5 result '
                              '(cameras.yaml lidar_to_camera + overlay base_to_laser / lidar_angle_offset)')
        cam = ct.merge_data(session, 'cameras.yaml', self.base_cameras, {'lidar_to_camera': l2c})
        ov = ct.merge_overlay(session, 'tunnel_wall_follower', {'lidar_angle_offset': float(off)})
        ct.merge_overlay(session, 'carbot_tf', {'base_to_laser': [float(x) for x in m]})
        self._geo = None
        return [cam, ov]

    # ------------------------------------------------------------------ live
    def live(self, inputs: Dict) -> Dict:
        try:
            g = self.geometry()
        except Exception as e:  # noqa: BLE001  shown on the page with the reason
            return {'error': f'front camera / LiDAR setup: {e}', 'image_key': IMAGE_KEY}
        out = {'image_key': IMAGE_KEY, 'intrinsics': g['intr_src'], 'sensor': g['sensor'],
               'base_to_laser': [_r(x, 4) for x in g['laser']], 'min_captures': int(self.proc['min_captures']),
               'min_spread_deg': float(self.proc['min_spread_deg']), 'points': [], 'targets': [],
               'scan_age_s': None}
        scans = list(inputs.get('scans') or [])
        if scans:
            out['scan_age_s'] = _r(inputs.get('now', self.clock()) - float(scans[-1]['t']), 2)
        fresh = self._scans(inputs)
        if fresh:
            sc = fresh[-1]
            pts, _ = la.scan_points(sc['ranges'], sc['angle_min'], sc['angle_increment'],
                                    sc['range_min'], sc['range_max'])
            bxy = la.laser_to_base(pts, g['laser'])
            keep = la.in_roi(bxy, g['laser'], 0.05, float(self.proc['roi_max_m']) * 2.0, g['half'])
            bxy = bxy[keep]
            uv = self._project(g, bxy)
            rng = np.hypot(bxy[:, 0] - g['laser'][0], bxy[:, 1] - g['laser'][1]) if len(bxy) else np.zeros(0)
            vis = ~np.isnan(uv[:, 0]) if len(uv) else np.zeros(0, bool)
            idx = np.flatnonzero(vis)
            mx = int(self.proc['overlay_max_points'])
            if len(idx) > mx:
                idx = idx[np.linspace(0, len(idx) - 1, mx).astype(int)]
            out['points'] = [[_r(uv[i, 0], 4), _r(uv[i, 1], 4), _r(rng[i], 2)] for i in idx]
            tg = la.front_clusters(sc, g['laser'], self.proc, g['half'])
            if tg:
                tuv = self._project(g, np.array([t['base_xy'] for t in tg]))
                out['targets'] = [{'u': None if np.isnan(p[0]) else _r(p[0], 4), 'v': None if np.isnan(p[1]) else _r(p[1], 4),
                                   'bearing_deg': _r(t['bearing_deg'], 1), 'range_m': _r(t['range_m'], 2), 'n': t['n']}
                                  for t, p in zip(tg, tuv)]
        est = la.estimate(self.captures, g['laser'])
        caps = []
        for i, c in enumerate(self.captures):
            r = est['rows'][i]
            lp = self._project(g, la.laser_to_base(np.array([c['laser_xy']]), g['laser']))[0]
            caps.append({'u': c['u'], 'v': c['v'], 'lu': None if np.isnan(lp[0]) else _r(lp[0], 4),
                         'lv': None if np.isnan(lp[1]) else _r(lp[1], 4),
                         'bearing_deg': _r(r['base_bearing_deg'], 1), 'error_deg': _r(math.degrees(r['error']), 2),
                         'residual_deg': _r(math.degrees(r['residual']), 2),
                         'range_diff_m': _r(r['lidar_range_m'] - r['cam_range_m'], 3)})
        out['captures'] = caps
        if est:
            out['estimate'] = {'offset_deg': _r(math.degrees(est['offset'])), 'max_residual_deg': _r(est['max_residual_deg']),
                               'spread_deg': _r(est['spread_deg'], 1), 'n': len(self.captures)}
        return out
