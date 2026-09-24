"""Calibration step 11 -- map-to-UWB alignment (pure, unit tested).

Three modes. The first is new with the Haffiz UWB switch and is the default:

  {"mode": "uwb_lap"}             push / drive the car SLOWLY once around the track (any
                                  start point), then STEP {"op": "stop"}. Records Haffiz's
                                  FILTERED position (solver + CV Kalman filter, common.yaml
                                  uwb_positioning, = /carbot/uwb/position), moves each fix
                                  from the tag to the rear axle (heading from the filter's
                                  velocity; slower than uwb_lap_min_speed_mps = dropped) and
                                  fits the map onto it with the team's map builder
                                  (tools/map/map_builder.py fit_rigid, the same fit that makes
                                  track_map.yaml venue_transform). No odometry, no IMU, no
                                  exact start pose. The fit runs in a worker thread.
                                  track_map.yaml is NOT rewritten (mission.yaml stays valid);
                                  Save also stores the lap as captures/step11_uwb_lap.csv for
                                  `map_builder.py edit` if the road shape needs touching up.

The two older modes (terminal tool carbot_localization.calib_map_uwb as a wizard page) find
uwb.yaml track_to_venue (p_venue = R(yaw) p_track + [x, y]) with the SAME fit
(alignment.fit_track_to_venue: Huber loss on the ranges, inliers within inlier_m)
and the same range processing (calib_map_uwb.lap_samples / point_samples on the
raw tag JSON, this session's step-10 anchors + offsets). RUN/REDO argument JSON:

  {"mode": "lap"}                 car on the map start pose; push / drive it SLOWLY once
                                  around the track, then STEP {"op": "stop"} (Stop lap).
                                  procedure.lap_min_s and min_extent_m are enforced;
                                  the lap ends by itself after lap_max_s.
  {"mode": "points", "pose": N}   car parked EXACTLY on the named map pose N
                                  (start_pose, light_goal_pose, mission P0.. / checkpoints)
                                  for points_seconds. Points add up; the fit runs over all
                                  of them once min_points are recorded (extent >=
                                  points_min_extent_m). STEP {"op": "clear_points"} restarts.

Car pose during the lap (track frame): dead reckoning from the wizard's
MotionRecorder (step 6's /odom signed distance + /imu/rpy yaw, the same inputs
block 05 predicts with), anchored on the map start pose when the lap starts.
No second /odom subscription, and nothing is published: the local estimate is
never reset or touched from here. UWB is read-only (never reaches the servo).

Needs this session's data/uwb.yaml from step 10 (anchors_surveyed and
offsets_calibrated true); refuses otherwise.

Save merges ONLY track_to_venue {x_m, y_m, yaw_deg, aligned: true} into
<session>/data/uwb.yaml (calib_tools.merge_data): step 10's keys stay. It refuses
when step 10 was saved again after the fit (other anchors / offsets). Keep
previous copies track_to_venue and refuses when the older session's anchors,
offsets or tag mount differ from this session's (the alignment would be stale).

The result file (11_map_uwb_alignment.yaml) records the map the alignment was
checked on: result['map'] = {'file', 'sha1' (raw), 'sha1_lf', 'sha1_crlf'}
(carbot_common.course.file_fingerprints, the hash mission.yaml map.sha1 uses),
and result['basis'] = the step-10 anchors / offsets / tag the fit used.
"""
import importlib
import json
import math
import os
import sys
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from carbot_common import calib_tools as ct
from carbot_common.course import file_fingerprints
from carbot_localization.alignment import fit_track_to_venue, tag_position
from carbot_localization.calib_map_uwb import lap_samples, named_poses, point_samples
from uwb_localization.positioning import PositioningCfg, rear_axle_from_tag

from . import wizard_uwb as wu
from .sensor_checks import ConfigError
from .wizard_core import StepImpl, StepRefused

STEP_ID = 'map_uwb_alignment'
PROC_KEYS = ('modes', 'lap_min_s', 'min_extent_m', 'huber_m', 'inlier_m', 'points_seconds', 'min_points',
             # wizard page (phase 8 page 11)
             'points_min_extent_m', 'lap_max_s', 'max_input_age_s', 'live_fit_period_s', 'overlay_max_points',
             # Haffiz switch: uwb_lap mode
             'uwb_lap_min_speed_mps', 'uwb_lap_init', 'uwb_lap_min_points', 'map_tools_dir')
PASS_KEYS = ('max_rms_m', 'min_inlier_frac', 'uwb_lap_max_rms_m', 'uwb_lap_min_inlier_frac', 'uwb_lap_min_sections')
MODES = ('uwb_lap', 'lap', 'points')
UWB_LAP_INITS = ('none', 'map')
# uwb.yaml keys this step writes (= calibration_steps.yaml map_uwb_alignment.writes)
WRITES = ('track_to_venue',)
STEP10 = 'step 10 (UWB anchor survey + offsets)'


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def parse_argument(arg: str, poses: Dict) -> Tuple[Optional[Dict], str]:
    """RUN argument -> ({'mode', 'pose'?}, '') or (None, why refused)."""
    try:
        a = json.loads(arg) if arg else None
    except ValueError:
        a = None
    if not isinstance(a, dict) or a.get('mode') not in MODES:
        return None, ('Choose Start UWB lap, Start lap or Record point on this page '
                      '(argument {"mode": "uwb_lap" | "lap" | "points"}).')
    if a['mode'] == 'uwb_lap':
        return {'mode': 'uwb_lap'}, ''
    if a['mode'] == 'points':
        name = str(a.get('pose') or '').strip()
        if name not in poses:
            return None, (f'Points: pick a known map pose ({", ".join(sorted(poses)) or "none in the map"}), '
                          f'not {name!r}.')
        return {'mode': 'points', 'pose': name}, ''
    return {'mode': 'lap'}, ''


def basis(doc: Dict) -> Dict:
    """What the fit depends on from step 10: anchors (id, xyz, offset) + tag height / mount."""
    tag = doc.get('tag') or {}
    return {'anchors': sorted(([str(a['id']).upper(), [round(float(v), 4) for v in a['xyz_m']],
                                round(float(a.get('range_offset_m', 0.0)), 4)] for a in doc.get('anchors') or []),
                              key=lambda r: r[0]),
            'tag': {'z_m': round(float(tag.get('z_m', 0.0)), 4),
                    'mount_xy_m': [round(float(v), 4) for v in tag.get('mount_xy_m', [0.0, 0.0])]}}


def to_track(t2v: Dict, x: float, y: float) -> Tuple[float, float]:
    """venue -> track with track_to_venue {x_m, y_m, yaw_deg}."""
    yaw = math.radians(float(t2v['yaw_deg']))
    dx, dy = x - float(t2v['x_m']), y - float(t2v['y_m'])
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def load_map_builder(tools_dir: str, config_dir: str = ''):
    """Import tools/map/map_builder.py from the repo checkout (same search as step 12)."""
    from .step_mission_planner import find_planner_dir
    d = find_planner_dir(tools_dir, config_dir)
    if not d:
        raise ConfigError(f'calibration_steps.yaml map_uwb_alignment.procedure.map_tools_dir: {tools_dir} with '
                          'map_builder.py not found (looked in $CARBOT_REPO and the parents of the package and '
                          'working directory). Set CARBOT_REPO to the repo checkout.')
    d = os.path.abspath(d)
    if d not in sys.path:
        sys.path.insert(0, d)
    return importlib.import_module('map_builder')


def uwb_lap_points(fixes, lever: Tuple[float, float], min_speed: float) -> Tuple[np.ndarray, int]:
    """Haffiz fixes -> rear-axle lap points (venue). Fixes slower than min_speed (no heading
    from the filter velocity) are dropped; returns (Nx2, dropped)."""
    pts, dropped = [], 0
    for f in fixes:
        if f.vel is None or math.hypot(*f.vel) < min_speed:
            dropped += 1
            continue
        pts.append(rear_axle_from_tag(f.xy, math.atan2(f.vel[1], f.vel[0]), lever))
    return (np.array(pts, float).reshape(-1, 2), dropped)


def fit_uwb_lap(mb, tpl: Dict, lap: np.ndarray, init=None) -> Dict:
    """map_builder.fit_rigid + overall residuals. T = track -> venue (x, y, yaw rad)."""
    T, report = mb.fit_rigid(lap, tpl=tpl, init=init)
    lines = mb.centrelines(tpl, mb.FIT['sample_step_m'])
    _, dist = mb.LineIndex(lines).nearest(mb.apply(mb.invert(T), lap))
    inl = dist < mb.FIT['icp_reject_m']
    rms = float(np.sqrt(np.mean(dist[inl] ** 2))) if inl.any() else float('inf')
    fitted = [n for n, r in report.items() if r['fitted']]
    ext = float(np.hypot(*(lap.max(axis=0) - lap.min(axis=0)))) if len(lap) else 0.0
    return {'T': T, 'report': report, 'rms_m': rms, 'inlier_frac': float(inl.mean()) if len(inl) else 0.0,
            'covered': len(fitted) / max(len(report), 1), 'sections_fitted': fitted,
            'sections_missed': [n for n in report if n not in fitted], 'n': int(len(lap)), 'extent_m': ext,
            'median_abs_m': float(np.median(dist)) if len(dist) else float('inf')}


def _thin(pts: List, n: int) -> List:
    if n <= 0 or len(pts) <= n:
        return list(pts)
    k = len(pts) / float(n)
    return [pts[int(i * k)] for i in range(n)]


def _check(key, label, measured, limit, passed, why='', fix='') -> Dict:
    return {'key': key, 'label': label, 'measured': measured, 'limit': limit, 'passed': bool(passed),
            'why': '' if passed else why, 'fix': '' if passed else fix}


class MapUwbAlignmentStep(StepImpl):
    can_keep_previous = True
    ops_while_running = True            # Stop lap while RUNNING

    def __init__(self, cfg: Dict, uwb: Dict, config_dir: str, motion, session_fn: Callable[[], Optional[str]],
                 clock: Callable[[], float] = time.monotonic, pos_cfg: Optional[PositioningCfg] = None):
        super().__init__(cfg)
        self.pos_cfg = pos_cfg or PositioningCfg()
        self.proc, self.pas = cfg.get('procedure') or {}, cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml map_uwb_alignment.procedure')
        _need(self.pas, PASS_KEYS, 'calibration_steps.yaml map_uwb_alignment.pass')
        self.modes = [str(m) for m in (self.proc['modes'] or [])]
        bad = [m for m in self.modes if m not in MODES]
        if bad or not self.modes:
            raise ConfigError(f'calibration_steps.yaml map_uwb_alignment.procedure.modes: {self.proc["modes"]!r} '
                              f'(allowed: {", ".join(MODES)})')
        self.p = {k: float(self.proc[k]) for k in PROC_KEYS if k not in ('modes', 'uwb_lap_init', 'map_tools_dir')}
        self.uwb_lap_init = str(self.proc['uwb_lap_init'])
        if self.uwb_lap_init not in UWB_LAP_INITS:
            raise ConfigError(f'calibration_steps.yaml map_uwb_alignment.procedure.uwb_lap_init: '
                              f'{self.uwb_lap_init!r} (use {" | ".join(UWB_LAP_INITS)})')
        self.map_tools_dir = str(self.proc['map_tools_dir'])
        self.uwb_max_rms = float(self.pas['uwb_lap_max_rms_m'])
        self.uwb_min_inl = float(self.pas['uwb_lap_min_inlier_frac'])
        self.uwb_min_sec = int(self.pas['uwb_lap_min_sections'])
        self.max_rms, self.min_inl = float(self.pas['max_rms_m']), float(self.pas['min_inlier_frac'])
        _need(uwb, ('anchors', 'tag', 'track_to_venue'), 'uwb.yaml')
        self.base, self.config_dir = uwb, config_dir
        self.rec, self.session_fn, self.clock = motion, session_fn, clock
        try:
            self.poses = {k: tuple(float(v) for v in p) for k, p in named_poses(config_dir).items()}
        except Exception as e:  # noqa: BLE001  map / mission files unreadable
            raise ConfigError(f'map poses (track_map.yaml / mission.yaml in {config_dir}/data): {e}')
        if 'lap' in self.modes and 'start_pose' not in self.poses:
            raise ConfigError('track_map.yaml has no start_pose: lap mode needs it')
        self.run: Optional[Dict] = None
        self.points: List[Dict] = []          # recorded points (points mode)
        self.points_basis: Optional[Dict] = None
        self.last: Optional[Dict] = None      # last finished fit: overlay + rows (captures on Save)

    # ------------------------------------------------------------------ session / step 10
    def session_doc(self) -> Tuple[Optional[Dict], str]:
        """This session's data/uwb.yaml from step 10, or (None, why)."""
        session = self.session_fn() if self.session_fn else None
        how = (f'Do {STEP10} first: pass all four stages and press Save (or Keep previous value). '
               'Step 11 fits against the anchors and offsets step 10 saved in THIS session.')
        if not session:
            return None, 'No calibration session yet. ' + how
        path = os.path.join(session, 'data', 'uwb.yaml')
        if not os.path.isfile(path):
            return None, f'{os.path.basename(session)} has no data/uwb.yaml. ' + how
        doc = ct.load_yaml(path)
        if not doc.get('anchors_surveyed') or not doc.get('offsets_calibrated'):
            return None, (f'data/uwb.yaml in {os.path.basename(session)} has anchors_surveyed / offsets_calibrated '
                          'false. ' + how)
        return doc, ''

    def _map_info(self) -> Dict:
        session = self.session_fn() if self.session_fn else None
        own = os.path.join(session, 'data', 'track_map.yaml') if session else ''
        path = own if own and os.path.isfile(own) else os.path.join(self.config_dir, 'data', 'track_map.yaml')
        try:
            fp = file_fingerprints(path)
        except OSError as e:
            return {'file': path, 'error': str(e)}
        return {'file': path, 'sha1': fp['raw'], 'sha1_lf': fp['lf'], 'sha1_crlf': fp['crlf']}

    def _fresh(self, now: float) -> str:
        ages = self.rec.ages(now)
        lim = self.p['max_input_age_s']
        bad = [f'/{k if k == "odom" else "imu/rpy"} ' + ('never received' if a is None else f'{a:.1f} s old')
               for k, a in ages.items() if a is None or a > lim]
        return ', '.join(bad)

    # ------------------------------------------------------------------ StepImpl
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        a, err = parse_argument(inputs.get('argument', ''), self.poses)
        if a is None:
            return err
        if a['mode'] not in self.modes:
            return f'Mode {a["mode"]} is switched off (calibration_steps.yaml map_uwb_alignment.procedure.modes).'
        doc, why = self.session_doc()
        if doc is None:
            return why
        try:
            anchors = wu.anchor_set(doc)
            lever = tuple(float(v) for v in doc['tag']['mount_xy_m'])
        except (KeyError, TypeError, ValueError) as e:
            return f'data/uwb.yaml of this session is broken ({e}): redo {STEP10}.'
        feed = inputs.get(wu.INPUT_KEY)
        if feed is None:
            return 'No UWB feed in the calibration wizard (node started without it): relaunch calibrate.launch.py.'
        b = basis(doc)
        run = {'mode': a['mode'], 't0': now, 'cursor': feed.cursor(), 'rows': [], 'lost': 0, 'anchors': anchors,
               'lever': lever, 'basis': b, 'stop': False, 'live_fit': None, 'live_fit_t': -math.inf}
        if a['mode'] == 'uwb_lap':
            run.update(worker=None, fit_out=None)
        elif a['mode'] == 'lap':
            stale = self._fresh(now)
            if stale:
                return (f'Lap needs the wheel odometry and IMU ({stale}). Is servo_controller running? '
                        'Step 1 must show /odom and /imu/rpy green.')
            sx, sy, sa = self.poses['start_pose']
            run.update(dist=self.rec.dist, yaw0=self.rec.yaw, x=sx, y=sy, a=sa, path=[(now, sx, sy, sa)])
        else:
            if self.points_basis is not None and self.points_basis != b:
                self.points, self.points_basis = [], None          # step 10 changed: old points are stale
            run['pose'] = a['pose']
        self.run = run
        return None

    def cancel(self) -> None:
        self.run = None

    def handle(self, op: str, args: Dict, inputs: Dict) -> Dict:
        if op == 'stop':
            if not self.run or self.run['mode'] not in ('lap', 'uwb_lap'):
                return {'ok': False, 'message': 'No lap is being recorded'}
            self.run['stop'] = True
            return {'ok': True, 'message': 'Lap stopped: fitting…'}
        if op == 'clear_points':
            if self.run and self.run['mode'] == 'points':
                return {'ok': False, 'message': 'Wait for the point being recorded (or Cancel it) first'}
            n = len(self.points)
            self.points, self.points_basis = [], None
            return {'ok': True, 'message': f'Cleared {n} recorded point(s)'}
        return {'ok': False, 'message': f'Unknown operation {op!r}'}

    def progress(self, now: float) -> Dict:
        r = self.run
        if r is None:
            return {}
        el = now - r['t0']
        out = {'mode': r['mode'], 'elapsed_s': round(el, 1), 'samples': len(r['rows'])}
        if r['mode'] == 'points':
            secs = self.p['points_seconds']
            out.update(remaining_s=round(max(0.0, secs - el), 1), fraction=round(min(1.0, el / secs), 2))
        else:
            out.update(remaining_s=round(max(0.0, self.p['lap_max_s'] - el), 1),
                       fraction=round(min(1.0, el / max(self.p['lap_min_s'], 1e-6)), 2))
        return out

    def _advance_pose(self, r: Dict) -> None:
        """Dead reckoning in the track frame: signed wheel distance + IMU yaw change since the start."""
        t = self.rec.odom_t
        if t is None:
            return
        ds = self.rec.dist - r['dist']
        r['dist'] = self.rec.dist
        a_new = wrap(self.poses['start_pose'][2] + math.radians(self.rec.yaw - r['yaw0']))
        am = r['a'] + wrap(a_new - r['a']) / 2.0
        r['x'] += ds * math.cos(am)
        r['y'] += ds * math.sin(am)
        r['a'] = a_new
        if t > r['path'][-1][0]:
            r['path'].append((t, r['x'], r['y'], r['a']))
        else:
            r['path'][-1] = (r['path'][-1][0], r['x'], r['y'], r['a'])

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        feed = inputs.get(wu.INPUT_KEY)
        if feed is not None:
            rows, r['cursor'], lost = feed.rows_since(r['cursor'])
            r['rows'].extend(rows)
            r['lost'] += lost
        el = now - r['t0']
        if r['mode'] == 'points':
            if el < self.p['points_seconds']:
                return None
            self.run = None
            return self._finish_point(r)
        if r['mode'] == 'uwb_lap':
            return self._tick_uwb_lap(r, el)
        self._advance_pose(r)
        stale = self._fresh(now)
        if stale:
            self.run = None
            return self._fail_lap(r, f'odometry / IMU stopped during the lap ({stale})',
                                  'Is servo_controller running? Then start the lap again from the start pose.')
        if not r['stop'] and el < self.p['lap_max_s']:
            return None
        self.run = None
        return self._finish_lap(r, now)

    # ------------------------------------------------------------------ uwb_lap (Haffiz)
    def _current_map(self) -> Tuple[Dict, str]:
        info = self._map_info()
        return ct.load_yaml(info['file']), info['file']

    def _tick_uwb_lap(self, r: Dict, el: float) -> Optional[Dict]:
        if r['worker'] is None:
            if not r['stop'] and el < self.p['lap_max_s']:
                return None
            r['stopped_by'] = 'user' if r['stop'] else 'lap_max_s'
            r['duration_s'] = el
            r['worker'] = threading.Thread(target=self._uwb_lap_work, args=(r,), daemon=True)
            r['worker'].start()
            return None
        if r['worker'].is_alive():
            return None
        self.run = None
        return self._finish_uwb_lap(r)

    def _uwb_lap_work(self, r: Dict) -> None:
        """Worker thread: positions -> rear-axle lap -> map_builder fit. Never raises."""
        out: Dict = {}
        try:
            fixes = wu.positions(r['anchors'], r['rows'], self.pos_cfg)
            lap, dropped = uwb_lap_points(fixes, r['lever'], self.p['uwb_lap_min_speed_mps'])
            out.update(fixes=len(fixes), dropped=dropped, lap=lap)
            if len(lap) < int(self.p['uwb_lap_min_points']):
                out['error'] = (f'only {len(lap)} moving UWB positions (need {int(self.p["uwb_lap_min_points"])}; '
                                f'{dropped} dropped as slower than {self.p["uwb_lap_min_speed_mps"]:g} m/s)')
                return
            doc, path = self._current_map()
            mb = load_map_builder(self.map_tools_dir, self.config_dir)
            from carbot_common.map_geometry import template_from_yaml
            tpl = template_from_yaml(doc)
            init = None
            if self.uwb_lap_init == 'map' and doc.get('venue_transform'):
                vt = doc['venue_transform']
                init = (float(vt['x']), float(vt['y']), math.radians(float(vt['yaw_deg'])))
            out['fit'] = fit_uwb_lap(mb, tpl, lap, init)
            out['map_file'] = path
        except Exception as e:  # noqa: BLE001  shown on the page, never kills the wizard
            out['error'] = f'{type(e).__name__}: {e}'
        finally:
            r['fit_out'] = out

    def _finish_uwb_lap(self, r: Dict) -> Dict:
        o = r['fit_out'] or {}
        dur = float(r.get('duration_s', 0.0))
        info = {'mode': 'uwb_lap', 'duration_s': round(dur, 1), 'uwb_reports': len(r['rows']),
                'lost_reports': r['lost'], 'fixes': o.get('fixes', 0), 'dropped_slow': o.get('dropped', 0),
                'lap_points': int(len(o['lap'])) if 'lap' in o else 0, 'stopped_by': r.get('stopped_by', ''),
                'method': f'{self.pos_cfg.solver} + {self.pos_cfg.filter}', 'init': self.uwb_lap_init}
        checks = [_check('lap_time', 'Lap duration', f'{dur:.0f} s', f'>= {self.p["lap_min_s"]:g} s',
                         dur >= self.p['lap_min_s'], f'the lap took only {dur:.0f} s: too fast or cut short',
                         'Push / drive SLOWLY around the WHOLE track, then press Stop lap.')]
        f = o.get('fit')
        if f is None:
            why = o.get('error', 'no fit')
            checks.append(_check('fit', 'Map fit', why, 'solvable', False, why,
                                 'Is the tag reporting with every anchor (step 10 Link check)? Keep the car moving '
                                 'during the lap; drive the whole track.'))
        else:
            checks.append(_check('extent', 'Area covered', f'{f["extent_m"]:.1f} m', f'>= {self.p["min_extent_m"]:g} m',
                                 f['extent_m'] >= self.p['min_extent_m'], 'the lap covers too small an area',
                                 'Go around the whole track.'))
            checks.append(_check('rms', 'Lap vs map centrelines (RMS)', f'{f["rms_m"] * 100:.1f} cm',
                                 f'<= {self.uwb_max_rms * 100:.0f} cm', f['rms_m'] <= self.uwb_max_rms,
                                 f'the UWB lap is {f["rms_m"] * 100:.1f} cm RMS off the fitted map',
                                 'Drive in the middle of the lane; check step 10 (verify error, anchor heights) and '
                                 'the tag mount; if the road itself differs, fix it with map_builder.py edit '
                                 'captures/step11_uwb_lap.csv, then re-run steps 11 and 12.'))
            checks.append(_check('inliers', 'Lap points on the map', f'{f["inlier_frac"] * 100:.0f} % of {f["n"]}',
                                 f'>= {self.uwb_min_inl * 100:.0f} %', f['inlier_frac'] >= self.uwb_min_inl,
                                 'too much of the lap is off the fitted map (multipath, or a different road shape)',
                                 'Raise the anchors / clear the line of sight; if the road shape differs, fix it with '
                                 'map_builder.py edit captures/step11_uwb_lap.csv.'))
            nsec = len(f['sections_fitted'])
            checks.append(_check('sections', 'Map sections driven', f'{nsec} ({", ".join(f["sections_fitted"])})',
                                 f'>= {self.uwb_min_sec}', nsec >= self.uwb_min_sec,
                                 f'only {nsec} map sections driven: the fit may slide along the track',
                                 'Drive the whole loop: roundabout, lane change, tunnel corner.'))
        passed = all(c['passed'] for c in checks)
        res: Dict = {'step': STEP_ID, 'mode': 'uwb_lap', 'capture': info, 'checks': checks, 'basis': r['basis'],
                     'map': self._map_info()}
        if f is not None:
            T = f['T']
            res['fit'] = {'x_m': round(float(T[0]), 4), 'y_m': round(float(T[1]), 4),
                          'yaw_deg': round(math.degrees(float(T[2])), 3), 'rms_m': round(f['rms_m'], 4),
                          'inlier_frac': round(f['inlier_frac'], 3), 'ranges': f['n'],
                          'extent_m': round(f['extent_m'], 2), 'median_abs_m': round(f['median_abs_m'], 4),
                          'covered': round(f['covered'], 3), 'sections_fitted': f['sections_fitted'],
                          'sections_missed': f['sections_missed']}
        if passed:
            ft = res['fit']
            res['track_to_venue'] = {'x_m': ft['x_m'], 'y_m': ft['y_m'], 'yaw_deg': ft['yaw_deg'], 'aligned': True}
            res['summary'] = (f'uwb_lap: x {ft["x_m"]:+.3f} m, y {ft["y_m"]:+.3f} m, yaw {ft["yaw_deg"]:+.2f} deg; '
                              f'RMS {ft["rms_m"] * 100:.1f} cm, {len(ft["sections_fitted"])} map sections')
            res['message'] = 'Alignment passed: check the lap overlay on the map, then press Save.'
        else:
            bad = [c for c in checks if not c['passed']]
            res['summary'] = 'uwb_lap: ' + '; '.join(c['why'] for c in bad)
            res['message'] = 'Alignment failed: ' + bad[0]['why'] + '.'
        res['passed'] = passed
        lap = o.get('lap')
        self.last = {'mode': 'uwb_lap', 'fit': res.get('fit'), 'uwb_rows': r['rows'], 'pose_rows': None,
                     'points': {}, 'lap_venue': lap.tolist() if lap is not None else [],
                     'overlay': self._lap_overlay(r['anchors'], lap, res.get('fit'))}
        return res

    def _lap_overlay(self, anchors, lap, fit: Optional[Dict]) -> Dict:
        n = int(self.p['overlay_max_points'])
        out: Dict = {'transform': None, 'fixes': [], 'anchors': {}, 'path': [], 'points': [],
                     'poses': {k: [round(v, 3) for v in p] for k, p in self.poses.items()}}
        if fit and lap is not None and len(lap):
            t2v = {'x_m': fit['x_m'], 'y_m': fit['y_m'], 'yaw_deg': fit['yaw_deg']}
            out['transform'] = t2v
            out['anchors'] = {k: [round(c, 3) for c in to_track(t2v, v.x, v.y)] for k, v in anchors.anchors.items()}
            out['fixes'] = [[round(c, 3) for c in to_track(t2v, x, y)] for x, y in _thin(lap.tolist(), n)]
        return out

    # ------------------------------------------------------------------ finish
    def _pose_rows(self, r: Dict) -> List[Dict]:
        return [{'t': t, 'x': x, 'y': y, 'a': a} for t, x, y, a in r['path']]

    def _finish_lap(self, r: Dict, now: float) -> Dict:
        pose_rows = self._pose_rows(r)
        dur = pose_rows[-1]['t'] - pose_rows[0]['t'] if len(pose_rows) > 1 else 0.0
        samples = lap_samples(r['anchors'], r['rows'], pose_rows, r['lever'])
        info = {'mode': 'lap', 'duration_s': round(dur, 1), 'uwb_reports': len(r['rows']), 'poses': len(pose_rows),
                'lost_reports': r['lost'], 'ranges': len(samples), 'stopped_by': 'user' if r['stop'] else 'lap_max_s'}
        return self._evaluate('lap', samples, info, r, pose_rows=pose_rows)

    def _fail_lap(self, r: Dict, why: str, fix: str) -> Dict:
        return {'passed': False, 'mode': 'lap', 'summary': f'lap failed: {why}', 'message': f'Lap failed: {why}.',
                'checks': [_check('inputs', 'Odometry + IMU', why, f'<= {self.p["max_input_age_s"]:g} s old', False,
                                  why, fix)]}

    def _finish_point(self, r: Dict) -> Dict:
        pose = self.poses[r['pose']]
        s = point_samples(r['anchors'], r['rows'], pose, r['lever'])
        self.points = [p for p in self.points if p['pose'] != r['pose']]
        self.points.append({'pose': r['pose'], 'xya': list(pose), 'samples': s, 'rows': r['rows'],
                            'reports': len(r['rows']), 'ranges': len(s)})
        self.points_basis = r['basis']
        samples = [x for p in self.points for x in p['samples']]
        info = {'mode': 'points', 'points': [p['pose'] for p in self.points],
                'ranges_per_point': {p['pose']: p['ranges'] for p in self.points}, 'ranges': len(samples),
                'uwb_reports': sum(p['reports'] for p in self.points)}
        return self._evaluate('points', samples, info, r, just=r['pose'])

    def _evaluate(self, mode: str, samples: List, info: Dict, r: Dict, pose_rows=None, just: str = '') -> Dict:
        anchors = r['anchors']
        checks: List[Dict] = []
        fit, fit_err = None, ''
        try:
            fit = fit_track_to_venue(samples, {k: (v.x, v.y) for k, v in anchors.anchors.items()},
                                     self.p['huber_m'], self.p['inlier_m'])
        except ValueError as e:
            fit_err = str(e)
        if mode == 'lap':
            dur = info['duration_s']
            checks.append(_check('lap_time', 'Lap duration', f'{dur:.0f} s', f'>= {self.p["lap_min_s"]:g} s',
                                 dur >= self.p['lap_min_s'], f'the lap took only {dur:.0f} s: too fast or cut short',
                                 'Push / drive SLOWLY around the WHOLE track, then press Stop lap.'))
            min_ext = self.p['min_extent_m']
        else:
            n, need = len(self.points), int(self.p['min_points'])
            checks.append(_check('points', 'Map poses recorded', f'{n} ({", ".join(info["points"])})', f'>= {need}',
                                 n >= need, f'{n} of {need} poses recorded',
                                 'Park the car EXACTLY on another known pose, far from the others, and record it.'))
            min_ext = self.p['points_min_extent_m']
        if fit is None:
            checks.append(_check('fit', 'Fit', fit_err, 'solvable', False, fit_err,
                                 'Too few ranges: is the tag reporting (step 1 UWB green)? Record longer / more poses.'))
        else:
            checks.append(_check('extent', 'Area covered', f'{fit.track_extent_m:.1f} m', f'>= {min_ext:g} m',
                                 fit.track_extent_m >= min_ext, 'the samples cover too small an area: the yaw is not '
                                 'determined', 'Go around the whole track / use poses far apart.'))
            checks.append(_check('rms', 'Range RMS (inliers)', f'{fit.rms_m * 100:.1f} cm',
                                 f'<= {self.max_rms * 100:.0f} cm', fit.rms_m <= self.max_rms,
                                 f'ranges disagree with the map by {fit.rms_m * 100:.1f} cm RMS',
                                 'Start EXACTLY on the start pose; check step 10 (anchor positions, offsets) and '
                                 'the tag mount; go slower.'))
            checks.append(_check('inliers', 'Inlier ranges', f'{fit.inlier_frac * 100:.0f} % of {fit.n}',
                                 f'>= {self.min_inl * 100:.0f} % within {self.p["inlier_m"] * 100:.0f} cm',
                                 fit.inlier_frac >= self.min_inl, 'too many ranges do not fit (multipath or '
                                 'odometry drift)', 'Raise the anchors / clear the line of sight; redo step 6 if the '
                                 'odometry drifts.'))
        passed = all(c['passed'] for c in checks)
        res: Dict = {'step': STEP_ID, 'mode': mode, 'capture': info, 'checks': checks, 'basis': r['basis'],
                     'map': self._map_info()}
        if fit is not None:
            res['fit'] = {'x_m': round(fit.x, 4), 'y_m': round(fit.y, 4), 'yaw_deg': round(math.degrees(fit.yaw), 3),
                          'rms_m': round(fit.rms_m, 4), 'inlier_frac': round(fit.inlier_frac, 3), 'ranges': fit.n,
                          'extent_m': round(fit.track_extent_m, 2), 'median_abs_m': round(fit.median_abs_m, 4)}
        if passed:
            f = res['fit']
            res['track_to_venue'] = {'x_m': f['x_m'], 'y_m': f['y_m'], 'yaw_deg': f['yaw_deg'], 'aligned': True}
            res['summary'] = (f'{mode}: x {f["x_m"]:+.3f} m, y {f["y_m"]:+.3f} m, yaw {f["yaw_deg"]:+.2f} deg; '
                              f'RMS {f["rms_m"] * 100:.1f} cm, inliers {f["inlier_frac"] * 100:.0f} %')
            res['message'] = 'Alignment passed: check the overlay on the map, then press Save.'
        else:
            bad = [c for c in checks if not c['passed']]
            res['summary'] = f'{mode}: ' + '; '.join(c['why'] for c in bad)
            if mode == 'points' and len(self.points) < int(self.p['min_points']):
                res['message'] = (f'Point {just} recorded ({len(self.points)} of {int(self.p["min_points"])}). '
                                  'Park on another pose far away and record it.')
            else:
                res['message'] = 'Alignment failed: ' + bad[0]['why'] + '.'
        res['passed'] = passed
        # kept in memory for the live overlay and the Save captures (not in the result file)
        rows = r['rows'] if mode == 'lap' else [x for p in self.points for x in p['rows']]
        self.last = {'mode': mode, 'fit': res.get('fit'), 'overlay': self._overlay(anchors, rows, samples,
                                                                                    res.get('fit'), pose_rows),
                     'uwb_rows': rows, 'pose_rows': pose_rows,
                     'points': {p['pose']: p['rows'] for p in self.points} if mode == 'points' else {}}
        return res

    # ------------------------------------------------------------------ overlay (GUI map)
    def _overlay(self, anchors, rows: List[Dict], samples: List, fit: Optional[Dict], pose_rows=None) -> Dict:
        """Everything in the TRACK frame (the map): tag path / poses, UWB fixes moved into the track frame
        with the fitted transform, anchors, named poses. Thinned to overlay_max_points."""
        n = int(self.p['overlay_max_points'])
        out: Dict = {'transform': None, 'fixes': [], 'anchors': {}, 'path': [], 'points': [],
                     'poses': {k: [round(v, 3) for v in p] for k, p in self.poses.items()}}
        if pose_rows:
            out['path'] = [[round(p['x'], 3), round(p['y'], 3)] for p in _thin(pose_rows, n)]
        out['points'] = [{'pose': p['pose'], 'xy': [round(v, 3) for v in tag_position(tuple(p['xya']), self._lever())]}
                         for p in self.points] if not pose_rows else []
        if fit:
            t2v = {'x_m': fit['x_m'], 'y_m': fit['y_m'], 'yaw_deg': fit['yaw_deg']}
            out['transform'] = t2v
            out['anchors'] = {k: [round(c, 3) for c in to_track(t2v, v.x, v.y)] for k, v in anchors.anchors.items()}
            out['fixes'] = [[round(c, 3) for c in to_track(t2v, x, y)] for x, y in _thin(wu.fixes(anchors, rows, self.pos_cfg), n)]
        return out

    def _lever(self) -> Tuple[float, float]:
        doc, _ = self.session_doc()
        try:
            return tuple(float(v) for v in (doc or self.base)['tag']['mount_xy_m'])
        except (KeyError, TypeError, ValueError):
            return (0.0, 0.0)

    # ------------------------------------------------------------------ live view
    def _live_run(self, r: Dict, now: float) -> Dict:
        out = {'mode': r['mode'], 'elapsed_s': round(now - r['t0'], 1), 'reports': len(r['rows']), 'lost': r['lost']}
        if r['mode'] == 'points':
            out['pose'] = r['pose']
            return out
        if r['mode'] == 'uwb_lap':
            out['fitting'] = r.get('worker') is not None
            fx = wu.positions(r['anchors'], r['rows'][-30:], self.pos_cfg)
            if fx:
                out['uwb_xy'] = [round(fx[-1].xy[0], 3), round(fx[-1].xy[1], 3)]
                v = fx[-1].vel
                out['speed_mps'] = round(math.hypot(*v), 3) if v else None
                t2v = (self.session_doc()[0] or {}).get('track_to_venue') or {}
                if t2v.get('aligned'):
                    out['fix_xy'] = [round(c, 3) for c in to_track(t2v, *fx[-1].xy)]
                    out['fix_frame'] = 'saved alignment'
            return out
        path = r['path']
        xs, ys = [p[1] for p in path], [p[2] for p in path]
        out['extent_m'] = round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 2)
        out['pose'] = [round(r['x'], 3), round(r['y'], 3), round(r['a'], 4)]
        tag = tag_position((r['x'], r['y'], r['a']), r['lever'])
        out['tag_xy'] = [round(tag[0], 3), round(tag[1], 3)]
        # running fit every live_fit_period_s (so the page shows RMS / transform while driving)
        if now - r['live_fit_t'] >= self.p['live_fit_period_s']:
            r['live_fit_t'] = now
            samples = lap_samples(r['anchors'], r['rows'], self._pose_rows(r), r['lever'])
            fit = None
            try:
                f = fit_track_to_venue(samples, {k: (v.x, v.y) for k, v in r['anchors'].anchors.items()},
                                       self.p['huber_m'], self.p['inlier_m'])
                fit = {'x_m': round(f.x, 4), 'y_m': round(f.y, 4), 'yaw_deg': round(math.degrees(f.yaw), 3),
                       'rms_m': round(f.rms_m, 4), 'inlier_frac': round(f.inlier_frac, 3), 'ranges': f.n,
                       'extent_m': round(f.track_extent_m, 2)}
            except ValueError:
                pass
            r['live_fit'] = {'samples': len(samples), 'fit': fit,
                             'overlay': self._overlay(r['anchors'], r['rows'], samples, fit, self._pose_rows(r))}
        lf = r['live_fit'] or {}
        out['samples'] = lf.get('samples', 0)
        out['fit'] = lf.get('fit')
        out['overlay'] = lf.get('overlay')
        # current UWB fix vs where the map pose says the tag is (fit so far, else the saved transform)
        t2v = lf.get('fit') or ((self.session_doc()[0] or {}).get('track_to_venue') or {})
        fx = wu.fixes(r['anchors'], r['rows'][-30:], self.pos_cfg)
        if fx and (lf.get('fit') or t2v.get('aligned')):
            cx, cy = to_track(t2v, *fx[-1])
            out['fix_xy'] = [round(cx, 3), round(cy, 3)]
            out['fix_error_m'] = round(math.hypot(cx - tag[0], cy - tag[1]), 3)
            out['fix_frame'] = 'fit so far' if lf.get('fit') else 'saved alignment'
        return out

    def live(self, inputs: Dict) -> Dict:
        now = self.clock()
        doc, why = self.session_doc()
        feed = inputs.get(wu.INPUT_KEY)
        ids = sorted(str(a['id']).upper() for a in ((doc or self.base).get('anchors') or []))
        r = self.run
        out = {
            'modes': self.modes, 'ready': doc is not None, 'blocked': why,
            'step10': ({'anchors': len(doc.get('anchors') or []), 'track_to_venue': doc.get('track_to_venue')}
                       if doc else None),
            'feed': feed.stats(ids) if feed is not None else None,
            'inputs': self.rec.ages(now), 'poses': {k: [round(v, 3) for v in p] for k, p in self.poses.items()},
            'points': [{'pose': p['pose'], 'ranges': p['ranges'], 'reports': p['reports']} for p in self.points],
            'run': self._live_run(r, now) if r else None,
            'last': ({'mode': self.last['mode'], 'fit': self.last['fit'], 'overlay': self.last['overlay']}
                     if self.last else None),
            'limits': {'lap_min_s': self.p['lap_min_s'], 'lap_max_s': self.p['lap_max_s'],
                       'min_extent_m': self.p['min_extent_m'], 'points_min_extent_m': self.p['points_min_extent_m'],
                       'points_seconds': self.p['points_seconds'], 'min_points': int(self.p['min_points']),
                       'max_rms_m': self.max_rms, 'min_inlier_frac': self.min_inl,
                       'inlier_m': self.p['inlier_m'], 'max_input_age_s': self.p['max_input_age_s'],
                       'uwb_lap_max_rms_m': self.uwb_max_rms, 'uwb_lap_min_inlier_frac': self.uwb_min_inl,
                       'uwb_lap_min_sections': self.uwb_min_sec,
                       'uwb_lap_min_speed_mps': self.p['uwb_lap_min_speed_mps']},
            'method': f'{self.pos_cfg.solver} + {self.pos_cfg.filter}',
        }
        return out

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        t2v = res.get('track_to_venue')
        if not res.get('passed') or not t2v:
            raise StepRefused('Nothing to save: the alignment must pass first.')
        doc, why = self.session_doc()
        if doc is None:
            raise StepRefused(why)
        if basis(doc) != res.get('basis'):
            raise StepRefused('Step 10 was saved again after this fit (other anchors / offsets / tag mount): '
                              'the alignment is stale. Run step 11 again.')
        written = [ct.merge_data(session, 'uwb.yaml', self.base, {'track_to_venue': dict(t2v)})]
        last = self.last or {}
        try:                                  # raw captures, for calib_map_uwb --replay
            if last.get('mode') == 'lap' and last.get('pose_rows'):
                for name, rows in (('step11_lap_uwb.jsonl', last['uwb_rows']), ('step11_lap_pose.jsonl',
                                                                                last['pose_rows'])):
                    p = ct.capture_path(session, name)
                    ct.write_jsonl(p, rows)
                    written.append(p)
            if last.get('mode') == 'uwb_lap' and last.get('lap_venue'):
                p = ct.capture_path(session, 'step11_uwb_lap.csv')
                with open(p, 'w', encoding='utf-8') as fh:
                    fh.write('# x,y venue metres, rear axle, Haffiz filtered UWB (map_builder.py edit <this file>)\n')
                    fh.writelines(f'{x:.4f},{y:.4f}\n' for x, y in last['lap_venue'])
                written.append(p)
                p = ct.capture_path(session, 'step11_uwb_lap_uwb.jsonl')
                ct.write_jsonl(p, last['uwb_rows'])
                written.append(p)
            for name, rows in (last.get('points') or {}).items():
                p = ct.capture_path(session, f'step11_point_{name}.jsonl')
                ct.write_jsonl(p, rows)
                written.append(p)
        except OSError:
            pass                              # captures are a convenience; uwb.yaml is what counts
        return written

    def keep_data(self, src_session: str, session: str) -> List[str]:
        name = os.path.basename(src_session)
        path = os.path.join(src_session, 'data', 'uwb.yaml')
        if not os.path.isfile(path):
            raise StepRefused(f'{name} has no data/uwb.yaml with an alignment: run this step again.')
        old = ct.load_yaml(path)
        t2v = old.get('track_to_venue') or {}
        if not t2v.get('aligned'):
            raise StepRefused(f'{name} has no passed alignment (track_to_venue.aligned false): run this step again.')
        doc, why = self.session_doc()
        if doc is None:
            raise StepRefused(why)
        try:
            same = basis(old) == basis(doc)
            upd = {'x_m': float(t2v['x_m']), 'y_m': float(t2v['y_m']), 'yaw_deg': float(t2v['yaw_deg']),
                   'aligned': True}
        except (KeyError, TypeError, ValueError) as e:
            raise StepRefused(f'data/uwb.yaml in {name} is broken ({e}): run this step again.')
        if not same:
            raise StepRefused(f'{name} was aligned with other anchor positions / offsets / tag mount than this '
                              'session\'s step 10: the old alignment would be stale. Run this step again.')
        return [ct.merge_data(session, 'uwb.yaml', self.base, {'track_to_venue': upd})]
