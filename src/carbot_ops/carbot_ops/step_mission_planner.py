"""Calibration step 12 -- mission planner: route + roundabout exits (pure, unit tested).

Runs the team's own planner (tools/map/mission_planner.py, imported from
`procedure.planner_dir`, never copied) on the map this session will race on:

* Map = <session>/data/track_map.yaml if an earlier step (11) wrote one, else the
  repo default config/data/track_map.yaml -- exactly the file race.launch.py
  loads for this session (session copy overrides the repo file, per file).
* Poses P0 (start) .. Pn (finish): the RUN argument, JSON
      {"poses": [[x, y, yaw_deg], ...]}   or   {"poses": [{"x":, "y":, "yaw_deg":}, ...]}
  (rear-axle centre + heading, track frame, as mission.yaml). No argument = the
  poses of this session's mission.yaml, else the repo mission.yaml, else the
  planner's defaults.
* Planning (minutes on the RDK) runs in a thread; one leg at a time, a leg whose
  two poses did not move since the last Run is reused (same map only).

Checks (all must pass):
  pose fits    every car body fully on the road (mission_planner.fit_check,
               margin >= pass.min_pose_margin_m)
  parking bay  a leg whose mission_rules end_behaviour is parking_handoff ends
               inside its parking_bay
  leg route    mission_planner.plan_leg found a route (its own checks)
  block 07     (pass.block07_check) the race-time global planner check
               (carbot_planning.route_core.plan_mission) accepts the new mission:
               map fingerprint, whole-body road check, roundabout exits against
               mission_rules.yaml roundabout_visits. Otherwise race would refuse to arm.
Roundabout exits are chosen HERE by the planner (lane graph); the boom gate never
changes them (mission_rules gate_route_check only logs).

Save writes <session>/data/mission.yaml (mission_planner's own format, which
records map.file + map.sha1 of the track_map it was planned on). Keep previous
copies the older session's mission.yaml, REFUSED when it was planned on a
different track_map than the one this session uses.
"""
import hashlib
import importlib
import json
import math
import os
import shutil
import sys
import tempfile
import threading
from typing import Callable, Dict, List, Optional

import yaml

from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('planner_dir', 'legs', 'track_step_m', 'route_step_m', 'snap')
PASS_KEYS = ('min_pose_margin_m', 'block07_check')


class ConfigError(ValueError):
    pass


def _sha1(path: str) -> str:
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()


def fingerprints(path: str) -> Dict[str, str]:
    """= carbot_common.course.file_fingerprints (raw / LF / CRLF sha1): a map saved on
    Windows (CRLF) and checked out on the robot (LF) is the same map."""
    with open(path, 'rb') as f:
        raw = f.read()
    lf = raw.replace(b'\r\n', b'\n')
    return {'raw': hashlib.sha1(raw).hexdigest(), 'lf': hashlib.sha1(lf).hexdigest(),
            'crlf': hashlib.sha1(lf.replace(b'\n', b'\r\n')).hexdigest()}


def find_planner_dir(planner_dir: str, config_dir: str = '') -> str:
    """Folder holding mission_planner.py + map_builder.py. Absolute, or relative to the
    repo checkout: $CARBOT_REPO, then every parent of this file (real and symlinked
    path: colcon --symlink-install), of config_dir and of the working directory."""
    def ok(d):
        return os.path.isfile(os.path.join(d, 'mission_planner.py')) and \
            os.path.isfile(os.path.join(d, 'map_builder.py'))
    if os.path.isabs(planner_dir):
        return planner_dir if ok(planner_dir) else ''
    starts = [os.environ.get('CARBOT_REPO', ''), os.path.realpath(__file__), os.path.abspath(__file__),
              config_dir, os.getcwd()]
    for s in starts:
        if not s:
            continue
        d = os.path.abspath(s)
        while True:
            if ok(os.path.join(d, planner_dir)):
                return os.path.join(d, planner_dir)
            up = os.path.dirname(d)
            if up == d:
                break
            d = up
    return ''


def load_planner(planner_dir: str, config_dir: str = ''):
    """Import tools/map/mission_planner.py (it imports map_builder by name)."""
    d = find_planner_dir(planner_dir, config_dir)
    if not d:
        raise ConfigError(f'calibration_steps.yaml mission_planner.procedure.planner_dir: {planner_dir} with '
                          'mission_planner.py + map_builder.py not found (looked in $CARBOT_REPO and the parents '
                          'of the package and working directory). Set CARBOT_REPO to the repo checkout.')
    d = os.path.abspath(d)
    if d not in sys.path:
        sys.path.insert(0, d)
    mod = sys.modules.get('mission_planner')
    if mod is not None and os.path.dirname(os.path.abspath(getattr(mod, '__file__', ''))) != d:
        raise ConfigError(f'another mission_planner module is already loaded ({mod.__file__})')
    return importlib.import_module('mission_planner')


# ---------------------------------------------------------------------- block 07 (race-time) check
def _params(path: str, node: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    return ((d.get(node) or {}).get('ros__parameters') or {})


def _merge(dst: Dict, src: Dict) -> Dict:
    for k, v in (src or {}).items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def block07_check(config_dir: str, session: Optional[str], map_path: str, features_path: str,
                  rules_path: str, mission_doc: Dict) -> Dict:
    """The race-time global planner check (route_core.plan_mission) on a mission.yaml
    document that is not written anywhere yet. Parameters: common.yaml vehicle.* and
    planning.yaml global_planner.route_check_tolerance_m / densify_step_m, with this
    session's params_overlay.yaml applied (as the launch does)."""
    from carbot_common.course import Course
    from carbot_common.geometry import geometry
    from carbot_common.mission import load_mission
    from carbot_planning.route_core import PlanCfg, plan_mission

    params = os.path.join(config_dir, 'params')
    common = _params(os.path.join(params, 'common.yaml'), '/**')
    gp = _params(os.path.join(params, 'planning.yaml'), 'global_planner')
    ov = os.path.join(session, 'params_overlay.yaml') if session else ''
    if ov and os.path.isfile(ov):
        _merge(common, _params(ov, '/**'))
        _merge(gp, _params(ov, 'global_planner'))
    for k in ('route_check_tolerance_m', 'densify_step_m'):
        if k not in gp:
            raise ConfigError(f'planning.yaml global_planner.{k} missing')
    if 'vehicle' not in common:
        raise ConfigError('common.yaml vehicle.* missing')
    with open(map_path, 'r', encoding='utf-8') as f:
        tm = yaml.safe_load(f)
    with open(features_path, 'r', encoding='utf-8') as f:
        feats = yaml.safe_load(f)
    poses = {p['id']: (float(p['x']), float(p['y']), math.radians(float(p['yaw_deg'])))
             for p in mission_doc.get('poses', [])}
    course = Course(tm, feats, poses)
    tmp = tempfile.mkdtemp(prefix='carbot_mission_check_')
    try:
        mpath = os.path.join(tmp, 'mission.yaml')
        with open(mpath, 'w', encoding='utf-8') as f:
            yaml.safe_dump(mission_doc, f, sort_keys=False)
        mission = load_mission(mpath, rules_path)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    r = plan_mission(course, mission, geometry(common['vehicle']), PlanCfg(), fingerprints(map_path),
                     float(gp['route_check_tolerance_m']), float(gp['densify_step_m']), log=lambda s: None)
    return {'ok': bool(r.ok), 'reason': r.reason, 'warnings': list(r.warnings),
            'visits': [{'visit': v['visit'], 'leg': v['leg_id'], 'exit': v['exit'], 'label': v.get('label', '')}
                       for v in r.visits]}


# ---------------------------------------------------------------------- the step
class MissionPlannerStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, config_dir: str, session: Callable[[], Optional[str]] = lambda: None,
                 planner=None, race_check: Optional[Callable] = None):
        super().__init__(cfg)
        proc, pas = cfg.get('procedure') or {}, cfg.get('pass') or {}
        for where, d, keys in (('procedure', proc, PROC_KEYS), ('pass', pas, PASS_KEYS)):
            miss = [k for k in keys if k not in d]
            if miss:
                raise ConfigError(f'calibration_steps.yaml mission_planner.{where}: missing {", ".join(miss)}')
        self.proc, self.pas = proc, pas
        self.n_legs = int(proc['legs'])
        if self.n_legs < 1:
            raise ConfigError('calibration_steps.yaml mission_planner.procedure.legs must be >= 1')
        self.config_dir = config_dir
        self.session = session
        self._mp = planner
        self._mp_error = ''
        self.race_check = race_check if race_check is not None else self._default_race_check
        self.lock = threading.Lock()
        self.maps: Dict[str, Dict] = {}          # sha1 -> {tpl, road, graph}
        self.leg_cache: Dict = {}                # (sha1, start, end) -> route
        self.track_cache: Dict = {}              # sha1 -> track json
        self.saved_view: Dict = {}               # (path, mtime) -> routes drawn from a saved mission.yaml
        self.run: Optional[Dict] = None
        self.last: Optional[Dict] = None         # last finished plan (routes for the map)
        self.pending: Optional[Dict] = None      # PASS waiting for Save: {doc, sha1, map_path}

    # ------------------------------------------------------------------ helpers
    @property
    def mp(self):
        if self._mp is None:
            self._mp = load_planner(str(self.proc['planner_dir']), self.config_dir)
        return self._mp

    def data_file(self, name: str, session: Optional[str] = None) -> (str, str):
        """(path, source) of data file `name` as race loads it for the session."""
        session = session if session is not None else self.session()
        own = os.path.join(session, 'data', name) if session else ''
        if own and os.path.isfile(own):
            return own, 'session'
        return os.path.join(self.config_dir, 'data', name), 'repo'

    def _rules(self) -> Dict:
        path = self.data_file('mission_rules.yaml')[0]
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def _default_race_check(self, map_path, mission_doc):
        return block07_check(self.config_dir, self.session(), map_path, self.data_file('track_features.yaml')[0],
                             self.data_file('mission_rules.yaml')[0], mission_doc)

    def default_poses(self) -> (List, str):
        """[[x, y, yaw_deg]] from this session's mission.yaml, else the repo one; [] if none fits."""
        own, src = self.data_file('mission.yaml')
        cands = ([(own, 'session')] if src == 'session' else []) + \
            [(os.path.join(self.config_dir, 'data', 'mission.yaml'), 'repo')]
        for path, src in cands:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    d = yaml.safe_load(f) or {}
                ps = [[float(p['x']), float(p['y']), float(p['yaw_deg'])] for p in d.get('poses', [])]
            except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError):
                continue
            if len(ps) == self.n_legs + 1:
                return ps, f'{src} mission.yaml'
        return [], ''

    def parse_poses(self, arg: str) -> (List, str):
        arg = (arg or '').strip()
        if not arg:
            ps, src = self.default_poses()
            if ps:
                return ps, ''
            try:
                road = self._map(self.data_file('track_map.yaml')[0])['road']
                return [[p[0], p[1], math.degrees(p[2])] for p in self.mp.default_poses(road, self.n_legs)], ''
            except Exception as e:  # noqa: BLE001
                return [], f'No poses given and no mission.yaml to start from ({e}). Place P0..P{self.n_legs} on the map.'
        try:
            d = json.loads(arg)
        except ValueError:
            return [], 'Run argument must be JSON: {"poses": [[x, y, yaw_deg], ...]}'
        raw = d.get('poses') if isinstance(d, dict) else d
        n = self.n_legs + 1
        if not isinstance(raw, list) or len(raw) != n:
            return [], f'Need {n} poses P0..P{n - 1}, got {len(raw) if isinstance(raw, list) else "none"}'
        out = []
        for i, p in enumerate(raw):
            try:
                v = [p['x'], p['y'], p['yaw_deg']] if isinstance(p, dict) else list(p)
                v = [float(x) for x in v]
            except (KeyError, TypeError, ValueError):
                return [], f'P{i}: needs x, y, yaw_deg numbers'
            if len(v) != 3 or not all(math.isfinite(x) for x in v):
                return [], f'P{i}: needs x, y, yaw_deg numbers'
            out.append(v)
        return out, ''

    def _map(self, path: str) -> Dict:
        """Planner objects for one map file, built once per content (lane graph: seconds)."""
        sha = _sha1(path)
        with self.lock:
            m = self.maps.get(sha)
        if m is None:
            try:
                tpl, sha2 = self.mp.load_map(path)
            except SystemExit as e:              # load_map exits on a non-v2 map
                raise ConfigError(str(e))
            road = self.mp.Road(tpl)
            m = {'tpl': tpl, 'sha1': sha2, 'road': road, 'graph': None}
            with self.lock:
                self.maps = {sha: m}            # one map at a time
                self.leg_cache = {k: v for k, v in self.leg_cache.items() if k[0] == sha}
        return m

    def _graph(self, m: Dict):
        if m['graph'] is None:
            m['graph'] = self.mp.LaneGraph(m['road'])
        return m['graph']

    # ------------------------------------------------------------------ StepImpl
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        try:
            self.mp
        except Exception as e:  # noqa: BLE001
            return str(e)
        poses, err = self.parse_poses(inputs.get('argument', ''))
        if err:
            return err
        map_path, source = self.data_file('track_map.yaml')
        if not os.path.isfile(map_path):
            return f'No track_map.yaml: {map_path}'
        r = {'t0': now, 'poses': poses, 'map_path': map_path, 'map_source': source, 'stage': 'loading the map',
             'leg': -1, 'done': [None] * self.n_legs, 'out': None, 'error': '', 'thread': None}
        self.run = r
        self.pending = None
        r['thread'] = threading.Thread(target=self._work, args=(r,), daemon=True)
        r['thread'].start()
        return None

    def cancel(self) -> None:
        self.run = None

    def progress(self, now: float) -> Dict:
        r = self.run
        if r is None:
            return {}
        done = sum(1 for x in r['done'] if x is not None)
        return {'elapsed_s': round(now - r['t0'], 1), 'fraction': round(done / self.n_legs, 2),
                'samples': done, 'stage': r['stage'], 'leg': r['leg'], 'legs': self.n_legs,
                'summary': f'planning: {r["stage"]}'}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None or r['thread'].is_alive():
            return None
        self.run = None
        if r['error'] or r['out'] is None:
            return {'passed': False, 'summary': f'planning failed: {r["error"] or "no result"}',
                    'checks': [{'label': 'Planner', 'measured': 'error', 'limit': 'runs', 'passed': False,
                                'why': r['error'] or 'no result', 'fix': 'Check the map file, then press Redo.'}]}
        res, plan, pending = r['out']
        self.last = plan
        self.pending = pending if res['passed'] else None
        return res

    # ------------------------------------------------------------------ the planning thread
    def _work(self, r: Dict) -> None:
        try:
            r['out'] = self._plan(r)
        except Exception as e:  # noqa: BLE001  shown as a failed result, never kills the wizard
            if self.run is r:
                r['error'] = f'{type(e).__name__}: {e}'

    def _plan(self, r: Dict):
        mp = self.mp
        cancelled = lambda: self.run is not r  # noqa: E731
        m = self._map(r['map_path'])
        road, sha = m['road'], m['sha1']
        poses = [(p[0], p[1], math.radians(p[2])) for p in r['poses']]
        pad = float(self.pas['min_pose_margin_m'])
        rules = self._rules()
        checks, pose_rows = [], []
        for i, p in enumerate(poses):
            ok, margin = mp.fit_check(road, p, pad)
            pose_rows.append({'id': f'P{i}', 'x': round(p[0], 4), 'y': round(p[1], 4),
                              'yaw_deg': round(math.degrees(p[2]), 2), 'fits': bool(ok),
                              'margin_m': round(float(margin), 4), 'bay': road.bay_of_pose(p) or ''})
            checks.append({'label': f'P{i} fits on the road', 'measured': f'{margin * 100:+.1f} cm',
                           'limit': f'≥ {pad * 100:+.1f} cm', 'passed': bool(ok),
                           'why': '' if ok else f'part of the car body at P{i} is {-margin * 100:.1f} cm off the road',
                           'fix': '' if ok else f'Move P{i} onto the lane (snap on) and press Redo.'})
        for i in range(self.n_legs):
            b = ((rules.get('legs') or {}).get(f'leg{i + 1}') or {})
            if b.get('end_behaviour') == 'parking_handoff' and b.get('parking_bay'):
                bay, got = str(b['parking_bay']), pose_rows[i + 1]['bay']
                checks.append({'label': f'P{i + 1} in {bay}', 'measured': got or 'not in a bay', 'limit': bay,
                               'passed': got == bay,
                               'why': '' if got == bay else f'mission_rules.yaml legs.leg{i + 1} parks in {bay}',
                               'fix': '' if got == bay else f'Put P{i + 1} inside {bay} (snap centres it) and press Redo.'})
        routes: List[Optional[Dict]] = [None] * self.n_legs
        for i in range(self.n_legs):
            if cancelled():
                return None
            a, b = poses[i], poses[i + 1]
            key = (sha, tuple(round(v, 4) for v in a), tuple(round(v, 4) for v in b))
            if not (pose_rows[i]['fits'] and pose_rows[i + 1]['fits']):
                route = {'ok': False, 'reason': f'not planned: P{i if not pose_rows[i]["fits"] else i + 1} is off the road',
                         'seconds': 0.0}
            else:
                with self.lock:
                    route = self.leg_cache.get(key)
                if route is None:
                    r['leg'] = i
                    r['stage'] = 'building the lane graph' if m['graph'] is None else f'leg {i + 1} of {self.n_legs}'
                    graph = self._graph(m)
                    r['stage'] = f'leg {i + 1} of {self.n_legs}'
                    try:
                        route = mp.plan_leg(road, graph, a, b, cancel=cancelled)
                    except mp.Cancelled:
                        return None
                    if route.get('ok'):
                        with self.lock:
                            self.leg_cache[key] = route
            routes[i] = route
            r['done'][i] = bool(route.get('ok'))
        r['stage'] = 'checking the whole mission'
        doc = mp.mission_dict(r['map_path'], sha, poses, road, routes)
        leg_rows = []
        for i, rt in enumerate(routes):
            ok = bool(rt and rt.get('ok'))
            row = {'id': f'leg{i + 1}', 'start': f'P{i}', 'end': f'P{i + 1}', 'ok': ok,
                   'reason': (rt or {}).get('reason', ''), 'seconds': round(float((rt or {}).get('seconds', 0.0)), 1)}
            if ok:
                row.update(length_m=round(float(rt['length_m']), 2), roundabout_exits=list(rt['roundabout_exits']),
                           sections=list(rt['sections']),
                           manoeuvre=any(pc['kind'] == 'manoeuvre' for pc in rt['pieces']))
            leg_rows.append(row)
            checks.append({'label': f'leg{i + 1} route P{i} → P{i + 1}',
                           'measured': (f'{row["length_m"]:.2f} m · exits {", ".join(row["roundabout_exits"]) or "none"}'
                                        if ok else 'no route'),
                           'limit': 'route found', 'passed': ok,
                           'why': '' if ok else f'mission_planner: {row["reason"]}',
                           'fix': '' if ok else (f'Move P{i} / P{i + 1} (lane centre, facing the driving direction; '
                                                 'bays: centred) and press Redo.')})
        b07 = None
        if all(x['ok'] for x in leg_rows) and bool(self.pas['block07_check']):
            try:
                b07 = self.race_check(r['map_path'], doc)
            except Exception as e:  # noqa: BLE001
                b07 = {'ok': False, 'reason': f'{type(e).__name__}: {e}', 'warnings': [], 'visits': []}
            checks.append({'label': 'Race route check (block 07)',
                           'measured': 'accepted' if b07['ok'] else 'refused', 'limit': 'accepted',
                           'passed': bool(b07['ok']), 'why': '' if b07['ok'] else b07['reason'],
                           'fix': '' if b07['ok'] else ('Race mode would refuse to arm. For an exit mismatch, move the '
                                                        'poses so the route takes the planned exit (mission_rules.yaml '
                                                        'roundabout_visits), or ask the team to change the plan.')})
        elif bool(self.pas['block07_check']):
            checks.append({'label': 'Race route check (block 07)', 'measured': 'not run', 'limit': 'accepted',
                           'passed': False, 'why': 'a leg has no route', 'fix': 'Fix the failed legs first.'})
        passed = all(c['passed'] for c in checks)
        exits = [f'leg{i + 1}: {", ".join(x.get("roundabout_exits") or []) or "none"}'
                 for i, x in enumerate(leg_rows) if x['ok']]
        n_ok = sum(1 for x in leg_rows if x['ok'])
        summary = (f'{n_ok} of {self.n_legs} legs planned, {sum(x.get("length_m", 0) for x in leg_rows):.1f} m; '
                   f'exits {"; ".join(exits) or "none"}')
        if not passed:
            bad = [c['label'] for c in checks if not c['passed']]
            summary += '; failed: ' + ', '.join(bad[:3]) + (' …' if len(bad) > 3 else '')
        res = {'passed': passed, 'summary': summary, 'checks': checks,
               'map': {'path': os.path.abspath(r['map_path']), 'source': r['map_source'],
                       'file': os.path.basename(r['map_path']), 'sha1': sha},
               'poses': pose_rows, 'legs': leg_rows,
               'block07': b07 or {}, 'seconds': round(sum(x['seconds'] for x in leg_rows), 1)}
        plan = {'poses': r['poses'], 'routes': self._draw_routes(routes), 'from': 'this run', 'sha1': sha}
        pending = {'doc': doc, 'sha1': sha, 'map_path': r['map_path']}
        return res, plan, pending

    # ------------------------------------------------------------------ live view
    def _draw_routes(self, routes) -> List[Dict]:
        """Routes for the map canvas: points [x, y, dir] every procedure.route_step_m."""
        step = float(self.proc['route_step_m'])
        out = []
        for i, rt in enumerate(routes):
            if not rt or not rt.get('ok'):
                out.append({'leg': i, 'ok': False, 'reason': (rt or {}).get('reason', 'not planned'), 'pieces': []})
                continue
            pcs = []
            for pc in rt['pieces']:
                pts = pc['path'] if 'path' in pc else pc['points']
                keep, last = [], None
                for p in pts:
                    if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= step or p[3] != last[3]:
                        keep.append([round(float(p[0]), 3), round(float(p[1]), 3), int(p[3])])
                        last = p
                if pts and keep[-1][:2] != [round(float(pts[-1][0]), 3), round(float(pts[-1][1]), 3)]:
                    keep.append([round(float(pts[-1][0]), 3), round(float(pts[-1][1]), 3), int(pts[-1][3])])
                pcs.append({'kind': pc['kind'], 'bay': pc.get('bay', ''), 'pts': keep})
            out.append({'leg': i, 'ok': True, 'pieces': pcs, 'exits': list(rt.get('roundabout_exits') or []),
                        'length_m': round(float(rt.get('length_m', 0.0)), 2)})
        return out

    def _saved_plan(self) -> Optional[Dict]:
        """Routes of this session's saved mission.yaml (after a wizard restart)."""
        path, src = self.data_file('mission.yaml')
        if src != 'session':
            return None
        key = (path, os.path.getmtime(path))
        if key not in self.saved_view:
            with open(path, 'r', encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
            routes = []
            for lg in d.get('legs', []):
                routes.append({'ok': bool(lg.get('ok')), 'roundabout_exits': lg.get('roundabout_exits') or [],
                               'length_m': lg.get('length_m', 0.0), 'reason': '' if lg.get('ok') else 'no route',
                               'pieces': [{'kind': pc['kind'], 'bay': pc.get('bay', ''), 'points': pc['points']}
                                          for pc in lg.get('pieces') or []]})
            self.saved_view = {key: {'poses': [[p['x'], p['y'], p['yaw_deg']] for p in d.get('poses', [])],
                                     'routes': self._draw_routes(routes), 'from': 'saved mission.yaml',
                                     'sha1': str((d.get('map') or {}).get('sha1', ''))}}
        return self.saved_view[key]

    def track_json(self, path: str) -> Dict:
        """Map in the GUI's Global map format (web/tabs.js drawTrack) + bay headings."""
        sha = _sha1(path)
        if sha not in self.track_cache:
            mp = self.mp
            mb = mp.mb
            tpl, _ = mp.load_map(path)
            step = float(self.proc['track_step_m'])
            r3 = lambda v: round(float(v), 3)  # noqa: E731
            secs = {n: [[r3(x), r3(y)] for x, y in p] for n, p in mb.centrelines(tpl, step).items()}
            ar = mb.areas(tpl)
            areas = {n: {'kind': a['kind'], 'poly': [[r3(x), r3(y)] for x, y in a['poly']],
                         'heading': None if a['heading'] is None else r3(a['heading'])} for n, a in ar.items()}
            paint = [[r3(pa[0]), r3(pa[1]), r3(pb[0]), r3(pb[1]), st] for a in ar.values()
                     for pa, pb, st in mb.paint_segments(a)]
            pts = [q for p in secs.values() for q in p] + [q for a in areas.values() for q in a['poly']]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            self.track_cache = {sha: {
                'lane_width_m': float(mb.LANE_WIDTH), 'sections': secs, 'areas': areas, 'paint': paint,
                'bounds': [r3(min(xs) - 0.3), r3(min(ys) - 0.3), r3(max(xs) + 0.3), r3(max(ys) + 0.3)],
                'exits': {k: [r3(v[0]), r3(v[1])] for k, v in tpl['roundabout'].items()}}}
        return self.track_cache[sha]

    def live(self, inputs: Dict) -> Dict:
        try:
            mp = self.mp
        except Exception as e:  # noqa: BLE001
            return {'error': str(e)}
        map_path, source = self.data_file('track_map.yaml')
        if not os.path.isfile(map_path):
            return {'error': f'No track_map.yaml: {map_path}'}
        fp = fingerprints(map_path)
        out = {'map': {'path': os.path.abspath(map_path), 'source': source, 'sha1': fp['raw']},
               'track': self.track_json(map_path), 'legs': self.n_legs, 'snap': bool(self.proc['snap']),
               'car': {'rear': mp.CAR_REAR, 'front': mp.CAR_FRONT, 'half_w': mp.CAR_HALF_W}}
        ps, src = self.default_poses()
        out['default_poses'], out['default_from'] = ps, src
        plan = self.last
        if plan is None or plan['sha1'] not in fp.values():
            try:
                plan = self._saved_plan()
            except (OSError, yaml.YAMLError, KeyError, TypeError) as e:
                out['saved_error'] = f'cannot read this session\'s mission.yaml: {e}'
                plan = None
        if plan is not None:
            out['plan'] = dict(plan, stale=plan['sha1'] not in fp.values())
        r = self.run
        if r is not None:
            out['running'] = {'stage': r['stage'], 'leg': r['leg'], 'done': r['done'], 'poses': r['poses']}
        return out

    # ------------------------------------------------------------------ save / keep
    def _check_map(self, sha1: str, session: str, what: str) -> str:
        path, src = self.data_file('track_map.yaml', session)
        fp = fingerprints(path)
        if sha1 not in fp.values():
            raise StepRefused(f'{what} was planned on a different track_map.yaml (sha1 {sha1[:10]}) than the one '
                              f'this session races on ({src}: {path}, sha1 {fp["raw"][:10]}). Press Run / Redo '
                              'to plan on this map.')
        return path

    def save_data(self, session: str, result: Dict) -> List[str]:
        p = self.pending
        if p is None:
            raise StepRefused('Nothing planned to save: press Run.')
        self._check_map(p['sha1'], session, 'This plan')
        path = os.path.join(session, 'data', 'mission.yaml')
        if os.path.isfile(path):
            shutil.copy2(path, path + '.bak')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(p['doc'], f, sort_keys=False)
        self.pending = None
        return [path]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        src = os.path.join(src_session, 'data', 'mission.yaml')
        if not os.path.isfile(src):
            raise StepRefused(f'{os.path.basename(src_session)} has no data/mission.yaml to keep: press Run.')
        with open(src, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        self._check_map(str((doc.get('map') or {}).get('sha1', '')), session,
                        f'The mission.yaml of {os.path.basename(src_session)}')
        dst = os.path.join(session, 'data', 'mission.yaml')
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.isfile(dst):
            shutil.copy2(dst, dst + '.bak')
        shutil.copy2(src, dst)
        return [dst]
