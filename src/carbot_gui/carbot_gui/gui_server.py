"""Carbot browser GUI server (phase 7).

One HTTP server (port gui.yaml) serving web/ and a small JSON API. Built on the
base dashboard's approach (threaded http.server, polling, dedicated parameter
helper node) but with lazy topic groups so the GUI never starves autonomy:

  * 'core' (header, Drive progress, events) is always subscribed.
  * every other group is subscribed ONLY while a browser polls a tab that
    needs it, and destroyed idle_unsubscribe_s after the last poll.
  * messages are stored raw and converted only when polled; images are the
    already-downscaled JPEGs from camera_preview / perception debug topics.

API
  GET  /api/config                 mode, tabs, rates, labels
  GET  /api/core                   header + Drive data (poll state_rate_hz)
  GET  /api/tab/<name>[?src=..]    one tab's data (touches its groups)
  GET  /api/img/<key>              latest JPEG (touches 'img:<key>'), 204 if none
  GET  /api/events?since=N
  GET  /api/params                 tuning catalogue (calibrate only)
  POST /api/params/get|set|save    tuning (calibrate only)
  POST /api/estop                  e-stop (race: latched, 0 marks)
  POST /api/estop_release          calibrate only
  POST /api/manual  {on, confirm}  Manual control (race after START: confirm required)
  POST /api/start                  race START (std_srvs/Trigger on /carbot/race/start)
  POST /api/calibration/action     {step, action, argument} -> CalibrationAction (calibrate only, phase 8)
"""
import http.server
import json
import math
import os
import socketserver
import threading
import time
from urllib.parse import parse_qs, urlparse

import rclpy
import yaml
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import (CalibrationState, CandidateArray, CommandOwnerState, Corridor,
                                   DetectionArray, GateRouteMismatch, LocalGrid, LocalizationStatus,
                                   MissionEvent, MissionState, MotionRequest, NodeStatus, PreflightReport,
                                   SafetyStatus, SystemHealth, UwbRanges, UwbStatus)
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Float32, String

from . import gui_core as G

REQUIRED = ['host', 'port', 'mode', 'lazy_subscriptions', 'idle_unsubscribe_s', 'state_rate_hz',
            'map_rate_hz', 'lidar_rate_hz', 'candidates_rate_hz', 'health_rate_hz', 'image_max_fps',
            'grid_rate_hz', 'grid_stride', 'control_history_s', 'control_history_hz', 'events_max',
            'rosout_min_level', 'stale_after_s', 'trail_points', 'scan_max_points', 'path_max_points',
            'drive.obstacle_max_m', 'drive.obstacle_half_width_m', 'race.diagnostics_read_only',
            'race.tuning_enabled', 'race.diagnostic_rate_scale', 'race.estop_label',
            'race.manual_confirm', 'race.split_view', 'calibrate.tuning_enabled', 'legs_refresh_s']
WEB_DIR_DEFAULT = os.path.join(os.path.dirname(__file__), 'web')


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _r(v, n=3):
    if v is None:
        return None
    v = float(v)
    return None if not math.isfinite(v) else round(v, n)


def _pts(path_msg, max_n):
    pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path_msg.poses]
    return [[_r(x), _r(y), _r(z, 0)] for x, y, z in G.decimate(pts, max_n)]


class GuiServer(CarbotNode):

    def __init__(self):
        super().__init__('gui_server', '', REQUIRED)
        self.mode = str(self.p('mode'))
        self.race = self.mode == 'race'
        self.lock = threading.Lock()
        self.raw = {}                      # key -> (monotonic receive time, msg)
        self.nodes = {}                    # NodeStatus by node name
        self.hist = []                     # control history rows
        self.mismatch = []                 # gate vs route log
        self.trails = {'local': [], 'global': [], 'raw': []}
        self.manual = False
        self.estopped = False
        self.images = {}                   # key -> (t, bytes)
        self.events = G.EventLog(int(self.p('events_max')))
        self.lz = G.LazyGroups(float(self.p('idle_unsubscribe_s')), bool(self.p('lazy_subscriptions')))
        self.session = str(self.p('session', ''))
        self.uwb = self._load_yaml(str(self.p('data.uwb', '')))
        self._last_hist = 0.0
        self._subs = {}
        self._legs_cache = None
        self._route_xy = None
        self._build_groups()
        self._live = {s[0] for s in self.groups['core']}
        for spec in self.groups['core']:
            self._subscribe(spec)
        self.pub_estop = self.create_publisher(Bool, T.E_STOP, 10)
        self.pub_manual = self.create_publisher(Bool, T.MANUAL_TAKEOVER, LATCHED)
        self.pub_manual.publish(Bool(data=False))
        from std_srvs.srv import Trigger
        self.start_cli = self.create_client(Trigger, T.RACE_START_SRV) if self.race else None
        from carbot_interfaces.srv import CalibrationAction
        self.calib_cli = None if self.race else self.create_client(CalibrationAction, T.CALIBRATION_ACTION_SRV)
        self.calib_steps = self._load_steps()
        self.create_timer(1.0, self._update_groups)
        self.params = None
        if not self.race and bool(self.p('calibrate.tuning_enabled')):
            from .params_api import ParamClient
            self.params = ParamClient()
        self.set_status(NodeStatus.OK, 'RUNNING', f'mode {self.mode}')

    @staticmethod
    def _load_yaml(path):
        try:
            with open(path, encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            return {}

    def _load_steps(self):
        """Step list for the navigation rail, straight from the data file (works even if the wizard is down)."""
        doc = self._load_yaml(str(self.p('data.calibration_steps', '')))
        out = []
        for st in doc.get('steps', []) or []:
            try:
                out.append({'index': int(st['index']), 'id': str(st['id']), 'title': str(st['title']),
                            'required': bool(st.get('required', True))})
            except (KeyError, TypeError, ValueError):
                continue
        return sorted(out, key=lambda x: x['index'])

    # ------------------------------------------------------------------ topic groups
    def _build_groups(self):
        S, L = SENSOR, LATCHED
        core = [('status', NodeStatus, T.STATUS, 100), ('mission', MissionState, T.MISSION_STATE, L),
                ('owner', CommandOwnerState, T.OWNER_STATE, 10), ('safety', SafetyStatus, T.SAFETY_STATUS, 10),
                ('battery', Float32, T.VEHICLE_BATTERY, 10), ('events', MissionEvent, T.MISSION_EVENTS, 50),
                ('rosout', Log, '/rosout', 100), ('armed', Bool, T.RACE_ARMED, L),
                ('estop', Bool, T.E_STOP, 10), ('cmd_vel', Twist, T.CMD_VEL, 10),
                ('local_pose', Odometry, T.LOCAL_POSE, 10), ('route', Path, T.GLOBAL_ROUTE, L),
                ('route_info', String, T.ROUTE_INFO_JSON, L), ('corridor', Corridor, T.CORRIDOR, 10),
                ('mismatch', GateRouteMismatch, T.GATE_ROUTE_MISMATCH, 10)]
        core.append(('preflight', PreflightReport, T.RACE_PREFLIGHT, 10) if self.race else
                    ('calib', CalibrationState, T.CALIBRATION_STATE, L))
        cand = [('cand_local', CandidateArray, T.LOCAL_CANDIDATES, 5),
                ('cand_parking', CandidateArray, T.PARKING_CANDIDATES, 5),
                ('cand_recovery', CandidateArray, T.RECOVERY_CANDIDATES, 5)]
        self.groups = {
            'core': core,
            'drive': [('local_path', Path, T.LOCAL_PATH, 5), ('detections', DetectionArray, T.DETECTIONS, 5),
                      ('scan', LaserScan, T.SCAN, S)],
            'map': [('track', String, T.TRACK_MAP_JSON, L),
                    ('global_pose', PoseWithCovarianceStamped, T.GLOBAL_POSE, 10),
                    ('active_path', Path, T.ACTIVE_PATH, L)],
            'percplan': [('road_grid', LocalGrid, T.ROAD_GRID, 2), ('local_path', Path, T.LOCAL_PATH, 5),
                         ('parking_path', Path, T.PARKING_PATH, L), ('parking_state', String, T.PARKING_STATE, L),
                         ('recovery_state', String, T.RECOVERY_STATE, L)] + cand,
            'memory': [('memory_grid', LocalGrid, T.MEMORY_GRID, 2), ('scan', LaserScan, T.SCAN, S),
                       ('odom', Odometry, T.ODOM, S), ('tunnel_debug', String, T.TUNNEL_DEBUG, 5),
                       ('tunnel_detected', Bool, T.TUNNEL_DETECTED, 5), ('tunnel_cmd', Twist, T.TUNNEL_CMD, 5),
                       ('req_tunnel', MotionRequest, T.request_topic('TUNNEL'), 5)],
            'loc': [('global_pose', PoseWithCovarianceStamped, T.GLOBAL_POSE, 10),
                    ('local_status', LocalizationStatus, T.LOCAL_STATUS, 5),
                    ('loc_status', LocalizationStatus, T.LOCALIZATION_STATUS, 5),
                    ('uwb_ranges', UwbRanges, T.UWB_RANGES, 10), ('uwb_status', UwbStatus, T.UWB_STATUS, 5),
                    ('raw_fix', PointStamped, T.UWB_RAW_FIX, 10)],
            'det': [('detections', DetectionArray, T.DETECTIONS, 5)],
            'control': [(f'req_{s.lower()}', MotionRequest, T.request_topic(s), 5) for s in T.REQUEST_SOURCES],
            'health': [('health', SystemHealth, T.SYSTEM_HEALTH, 5)],
            # phase 8: wizard pages (open step's live view + sessions); only while a calibration page is open
            'calibration': [('calib_live', String, T.CALIBRATION_LIVE, L)],
        }
        imgs = {'cam_front': T.cam_preview('front'), 'ov_front': T.perception_overlay('front'),
                'stitched': T.PERCEPTION_DEBUG_STITCHED, 'mask': T.PERCEPTION_DEBUG_MASK,
                'warped': T.PERCEPTION_DEBUG_WARPED, 'det': T.DETECTIONS_DEBUG}
        for k, topic in imgs.items():
            self.groups['img:' + k] = [('img:' + k, CompressedImage, topic, 1)]

    def _subscribe(self, spec):
        key, typ, topic, qos = spec
        if (key, topic) in self._subs:
            return
        self._subs[(key, topic)] = self.create_subscription(typ, topic, lambda m, k=key: self._on(k, m), qos)

    def _update_groups(self):
        names = [g for g in self.groups if g != 'core']
        add, rem = self.lz.diff(names)
        keep = {(s[0], s[2]) for s in self.groups['core']}
        for g in self.lz.active:
            keep |= {(s[0], s[2]) for s in self.groups[g]}
        for g in add:
            for spec in self.groups[g]:
                self._subscribe(spec)
        # Never destroy_subscription() while the MultiThreadedExecutor spins: rclpy raises
        # InvalidHandle in the wait set and gui_server dies. Idle groups stay subscribed but are dropped in _on.
        self._live = {kt[0] for kt in keep}
        for kt in list(self._subs):
            if kt not in keep:
                with self.lock:
                    self.raw.pop(kt[0], None)
                    if kt[0].startswith('img:'):
                        self.images.pop(kt[0][4:], None)
        if add or rem:
            self.get_logger().debug(f'GUI groups +{sorted(add)} -{sorted(rem)}')

    # ------------------------------------------------------------------ callbacks
    def _on(self, key, m):
        if key not in self._live:
            return
        t = time.monotonic()
        if key.startswith('img:'):
            with self.lock:
                self.images[key[4:]] = (t, bytes(m.data))
            return
        if key == 'status':
            with self.lock:
                prev = self.nodes.get(m.node)
                self.nodes[m.node] = (t, m)
            if (prev is None or prev[1].level != m.level) and m.level == NodeStatus.ERROR:
                self.events.add('node', 'bad', m.block or m.node, f'{m.node}: {m.detail or m.state}')
            return
        if key == 'rosout':
            if m.level >= int(self.p('rosout_min_level')) and m.name != self.get_name():
                self.events.add('log', 'bad' if m.level >= 40 else 'warn', m.name, m.msg[:200])
            return
        if key == 'events':
            self.events.add('mission', 'info', '08', m.name + (f': {m.detail}' if m.detail else ''))
        elif key == 'mission':
            self.events.change('mode', m.mode, 'mission', 'info', '08', f'Mode {m.mode}')
            self.events.change('hold', m.hold_reason, 'mission', 'warn' if m.hold_reason else 'info', '08',
                               m.hold_reason or 'Hold released')
            self.events.change('challenge', m.challenge_id, 'mission', 'info', '08',
                               f'Challenge {m.challenge_id} {m.challenge_name}'.strip())
        elif key == 'owner':
            self.events.change('winner', m.winner, 'owner', 'bad' if m.winner in ('SAFETY_STOP', 'WATCHDOG')
                               else 'info', '15', f'Winner {m.winner}: {m.reason}')
            if t - self._last_hist >= 1.0 / float(self.p('control_history_hz')):
                self._last_hist = t
                row = [round(t, 2), _r(m.request.speed_mps), _r(m.commanded_speed_mps), _r(m.measured_speed_mps),
                       _r(m.request.steer_rad), _r(m.commanded_steer_rad)]
                with self.lock:
                    self.hist.append(row)
                    horizon = t - float(self.p('control_history_s'))
                    while self.hist and self.hist[0][0] < horizon:
                        self.hist.pop(0)
        elif key == 'safety':
            self.events.change('veto', m.veto_check, 'safety', 'bad' if m.veto_check else 'ok', '14',
                               f'Veto {m.veto_check}: {m.veto_reason}' if m.veto_check else 'Motion allowed')
        elif key == 'estop' and m.data:
            self.estopped = True
            self.events.change('estop', True, 'manual', 'bad', 'gui', 'E-STOP pressed')
        elif key == 'armed':
            if m.data:
                self.events.reset_clock()
            self.events.change('armed', bool(m.data), 'ops', 'ok', 'ops', 'Armed (START)' if m.data else 'Disarmed')
        elif key == 'mismatch':
            with self.lock:
                self.mismatch.append({'t': round(self.events.clock() - self.events.t0, 1), 'visit': m.roundabout_visit,
                                      'planned': m.planned_exit, 'gate': m.gate_state,
                                      'conf': _r(m.gate_confidence, 2), 'note': m.note})
                self.mismatch = self.mismatch[-50:]
            self.events.add('mission', 'warn', '08', f'Gate reads {m.gate_state}, planned exit {m.planned_exit}. '
                                                     'Route kept.')
        elif key in ('local_pose', 'global_pose', 'raw_fix'):
            self._trail(key, m)
        with self.lock:
            self.raw[key] = (t, m)

    def _trail(self, key, m):
        name = {'local_pose': 'local', 'global_pose': 'global', 'raw_fix': 'raw'}[key]
        if key == 'raw_fix':
            x, y = G.venue_to_track(m.point.x, m.point.y, self.uwb.get('track_to_venue', {}))
        else:
            x, y = m.pose.pose.position.x, m.pose.pose.position.y
        with self.lock:
            tr = self.trails[name]
            if not tr or (tr[-1][0] - x) ** 2 + (tr[-1][1] - y) ** 2 > 0.0004:
                tr.append((round(x, 3), round(y, 3)))
                del tr[:-int(self.p('trail_points'))]

    # ------------------------------------------------------------------ helpers
    def get(self, key, max_age=None):
        with self.lock:
            v = self.raw.get(key)
        if v is None or (max_age is not None and time.monotonic() - v[0] > max_age):
            return None
        return v[1]

    def pose(self):
        m = self.get('local_pose')
        if m is None:
            return None
        p = m.pose.pose
        return (p.position.x, p.position.y, _yaw(p.orientation))

    def _route_info(self):
        m = self.get('route_info')
        try:
            return json.loads(m.data) if m else None
        except ValueError:
            return None

    def legs(self):
        # /api/core is polled by every open tab at state_rate_hz; leg progress walks the whole route in Python
        # (30 % of gui_server's time on risabot1). The result only moves with the car: refresh it every
        # legs_refresh_s and rebuild the route point list only when a new route message arrives.
        now = time.monotonic()
        cached = self._legs_cache
        if cached is not None and now - cached[0] < float(self.p('legs_refresh_s')):
            return cached[1]
        info, route, pose = self._route_info(), self.get('route'), self.pose()
        mission = self.get('mission')
        if route is None:
            xy = None
        else:
            if self._route_xy is None or self._route_xy[0] is not route:
                self._route_xy = (route, [(p.pose.position.x, p.pose.position.y) for p in route.poses])
            xy = self._route_xy[1]
        out = G.leg_progress(info, xy, pose[:2] if pose else None, mission.route_leg if mission else 0)
        self._legs_cache = (now, out)
        return out

    # ------------------------------------------------------------------ /api/core
    def core(self):
        now, stale = time.monotonic(), float(self.p('stale_after_s'))
        with self.lock:
            nodes = G.node_rows(self.nodes, now, stale)
            owner_t = self.raw.get('owner', (0.0, None))[0]
        m, o, s = self.get('mission'), self.get('owner'), self.get('safety')
        mission = owner = safety = None
        if m is not None:
            mission = {'mode': m.mode, 'hold_reason': m.hold_reason, 'speed_zone': m.speed_zone,
                       'zone_max': _r(m.zone_max_speed_mps, 2), 'set_max': _r(m.set_max_speed_mps, 2),
                       'challenge_id': m.challenge_id, 'challenge_name': m.challenge_name, 'banner': m.banner,
                       'banner_level': m.banner_level, 'next_exit': m.next_roundabout_exit,
                       'source': m.active_source, 'leg_index': m.route_leg}
        if o is not None:
            owner = {'winner': o.winner, 'reason': o.reason, 'armed': o.armed, 'speed': _r(o.measured_speed_mps, 2),
                     'stale': now - owner_t > stale}
        if s is not None:
            veto = next((c for c in s.checks if c.name == s.veto_check), None)
            safety = {'motion_allowed': s.motion_allowed, 'veto_check': s.veto_check, 'veto_reason': s.veto_reason,
                      'value': _r(veto.value) if veto else None, 'limit': _r(veto.limit) if veto else None}
        legs = self.legs()
        calib = None
        cst = self.get('calib')
        if cst is not None and cst.steps:
            cur = next((x for x in cst.steps if x.index == cst.current_index), cst.steps[0])
            calib = {'index': cur.index, 'text': f'{cur.title} · {cur.status}'}
        running = G.running_now(self.mode, owner, safety, mission, nodes, self.manual, legs, calib)
        calib_flags = None
        if cst is not None:
            calib_flags = [{'i': x.index, 'st': x.status, 'adv': x.can_advance} for x in cst.steps]
        b, armed, cv, cor = self.get('battery', 5.0), self.get('armed'), self.get('cmd_vel', 1.0), \
            self.get('corridor', 1.0)
        out = {'mode': self.mode, 'mission': mission, 'owner': owner, 'safety': safety, 'running': running,
               'battery_v': _r(b.data, 2) if b else None, 'armed': bool(armed.data) if armed else False,
               'manual': self.manual, 'estopped': self.estopped, 'legs': legs,
               'lane_locked': bool(cor.lane_locked) if cor else False,
               'manual_cmd': {'lin': _r(cv.linear.x, 2), 'ang': _r(cv.angular.z, 2)} if cv and self.manual else None,
               'events_seq': self.events.seq, 'calib_steps': calib_flags}
        pf = self.get('preflight')
        if pf is not None:
            out['preflight'] = {'state': pf.state, 'summary': pf.summary, 'session': pf.calibration_session,
                                'missing': list(pf.missing_calibrations),
                                'checks': [{'name': c.name, 'ok': c.ok, 'value': c.value, 'expected': c.expected,
                                            'detail': c.detail} for c in pf.checks]}
        return out

    # ------------------------------------------------------------------ tabs
    def tab(self, name, q):
        for g in G.TAB_GROUPS.get(name, []):
            self.lz.touch(g)
        fn = getattr(self, 'tab_' + name, None)
        return fn(q) if fn else {}

    def _base_path(self, msg, pose):
        if msg is None or pose is None:
            return []
        return [[_r(a), _r(b)] for a, b in (G.to_base(p.pose.position.x, p.pose.position.y, pose)
                                              for p in G.decimate(msg.poses, int(self.p('path_max_points'))))]

    @staticmethod
    def _base_pts(pts, pose):
        return [[_r(a), _r(b)] for a, b in (G.to_base(p.x, p.y, pose) for p in pts)] if pose else []

    def _obstacles(self):
        sc = self.get('scan', 1.0)
        if sc is None:
            return []
        rmax, hw = float(self.p('drive.obstacle_max_m')), float(self.p('drive.obstacle_half_width_m'))
        best = None
        for i, r in enumerate(sc.ranges):
            if not (sc.range_min < r < rmax):
                continue
            a = sc.angle_min + i * sc.angle_increment
            x, y = r * math.cos(a), r * math.sin(a)
            if x > 0 and abs(y) < hw and (best is None or x < best[0]):
                best = (x, y)
        return [{'x': _r(best[0], 2), 'y': _r(best[1], 2), 'dist': _r(best[0], 2)}] if best else []

    @staticmethod
    def _dets(d):
        if d is None:
            return None
        return {'light': d.traffic_light_state, 'gate': d.boom_gate_state, 'bump': d.speed_bump_sign,
                'ms': _r(d.inference_ms, 1),
                'items': [{'cls': x.class_name, 'conf': _r(x.confidence, 2), 'cam': x.camera_role, 'x': x.x,
                           'y': x.y, 'w': x.width, 'h': x.height, 'dist': _r(x.distance_m, 2),
                           'px': _r(x.position.x, 2), 'py': _r(x.position.y, 2)} for x in d.detections]}

    def tab_drive(self, q):
        pose, cor = self.pose(), self.get('corridor', 1.0)
        lanes = None
        if cor is not None and pose is not None:
            lanes = {'left': self._base_pts(cor.left_edge, pose), 'right': self._base_pts(cor.right_edge, pose),
                     'locked': cor.lane_locked, 'mode': cor.mode}
        return {'lanes': lanes, 'path': self._base_path(self.get('local_path', 1.0), pose),
                'det': self._dets(self.get('detections', 1.0)), 'obstacles': self._obstacles()}

    def tab_map(self, q):
        out = {'pose': None, 'global': None, 'legs': self.legs(), 'info': self._route_info()}
        tr = self.get('track')
        fp = ''
        if tr is not None:
            doc = json.loads(tr.data)
            fp = json.dumps(doc.get('fingerprints', {}), sort_keys=True)
            if q.get('have', [''])[0] != fp:
                out['track'] = doc
        out['track_fp'] = fp
        route = self.get('route')
        if route is not None and q.get('have_route', [''])[0] != str(len(route.poses)):
            out['route'] = _pts(route, 4000)
        out['route_n'] = str(len(route.poses)) if route else ''
        p = self.pose()
        if p:
            out['pose'] = [_r(p[0]), _r(p[1]), _r(p[2])]
        g = self.get('global_pose', 2.0)
        if g is not None:
            c = g.pose.covariance
            out['global'] = [_r(g.pose.pose.position.x), _r(g.pose.pose.position.y), _r(_yaw(g.pose.pose.orientation)),
                             [_r(v, 4) for v in G.cov_ellipse(c[0], c[1], c[7])]]
        with self.lock:
            out['mismatch'] = list(self.mismatch)
            out['trail'] = list(self.trails['local'])
        return out

    def tab_percplan(self, q):
        pose = self.pose()
        g = self.get('road_grid', 2.0)
        out = {'grid': None}
        if g is not None:
            out['grid'] = dict(G.encode_grid(g.rows, g.cols, g.kind, g.grown, stride=1),
                               res=_r(g.resolution_m, 4), x0=_r(g.origin_x_m), y0=_r(g.origin_y_m),
                               coverage=_r(g.coverage, 3), connected=g.connected)
        src = q.get('src', ['local'])[0]
        if src not in ('local', 'parking', 'recovery'):
            src = 'local'
        ca = self.get('cand_' + src, 2.0)
        cands = []
        if ca is not None and pose is not None:
            for c in ca.candidates:
                cands.append({'id': c.id, 'offset': _r(c.offset_m), 'cost': _r(c.cost, 2), 'valid': c.valid,
                              'relaxed': c.relaxed, 'sel': bool(c.selected or c.id == ca.selected_id),
                              'clear': _r(c.min_clear_m), 'steer': _r(c.command_steer_rad), 'stage': c.stage,
                              'reject': c.reject, 'pts': self._base_pts(G.decimate(c.points, 40), pose)})
        out.update({'src': src, 'cands': cands, 'selected': ca.selected_id if ca else -1})
        cor = self.get('corridor', 1.0)
        if cor is not None and pose is not None:
            out['corridor'] = {'guide': self._base_path(cor.guide, pose), 'observed': cor.observed,
                               'locked': cor.lane_locked, 'offset': _r(cor.offset_m), 'mode': cor.mode}
        out['path'] = self._base_path(self.get('parking_path' if src == 'parking' else 'local_path', 2.0), pose)
        for k in ('parking_state', 'recovery_state'):
            m = self.get(k)
            try:
                out[k] = json.loads(m.data) if m else None
            except ValueError:
                out[k] = None
        with self.lock:
            rp = self.nodes.get('road_perception')
        out['perception_status'] = {'level': rp[1].level, 'detail': rp[1].detail} if rp else None
        return out

    def tab_memory(self, q):
        out = {}
        g, od = self.get('memory_grid', 3.0), self.get('odom', 1.0)
        st = int(self.p('grid_stride'))
        if g is not None:
            ages = [a for a in g.age_s if a >= 0]
            trav = [a for a in g.travel_since_m if a >= 0]
            out['grid'] = dict(G.encode_grid(g.rows, g.cols, g.kind, g.grown, g.age_s, stride=st),
                               res=_r(g.resolution_m * st, 4), x0=_r(g.origin_x_m), y0=_r(g.origin_y_m),
                               cells=len(ages), oldest=_r(max(ages), 1) if ages else 0,
                               travel=_r(max(trav), 2) if trav else 0, frame=g.header.frame_id)
        if od is not None:
            p = od.pose.pose
            out['odom'] = [_r(p.position.x), _r(p.position.y), _r(_yaw(p.orientation))]
        sc = self.get('scan', 1.0)
        if sc is not None:
            pts, closest = [], None
            for i, r in enumerate(sc.ranges):
                if sc.range_min < r < sc.range_max and math.isfinite(r):
                    a = sc.angle_min + i * sc.angle_increment
                    pts.append([_r(r * math.cos(a)), _r(r * math.sin(a))])
                    if closest is None or r < closest[0]:
                        closest = (r, math.degrees(a))
            out['scan'] = G.decimate(pts, int(self.p('scan_max_points')))
            out['scan_n'] = len(pts)
            out['closest'] = [_r(closest[0], 2), _r(closest[1], 0)] if closest else None
        td = self.get('tunnel_debug', 2.0)
        try:
            out['tunnel'] = json.loads(td.data) if td else None
        except ValueError:
            out['tunnel'] = None
        tdet, tc, rq = self.get('tunnel_detected', 2.0), self.get('tunnel_cmd', 1.0), self.get('req_tunnel', 0.5)
        mission = self.get('mission')
        out['tunnel_trigger'] = bool(tdet.data) if tdet else None
        out['tunnel_cmd'] = [_r(tc.linear.x), _r(tc.angular.z)] if tc else None
        out['tunnel_forwarding'] = rq is not None
        out['tunnel_steer'] = _r(rq.steer_rad) if rq else None
        out['tunnel_mode'] = bool(mission and mission.mode == 'TUNNEL')
        return out

    def tab_loc(self, q):
        out = self.tab_map(q)
        lp = self.get('local_pose', 1.0)
        if lp is not None:
            c = lp.pose.covariance
            out['local_ellipse'] = [_r(v, 4) for v in G.cov_ellipse(c[0], c[1], c[7])]
        t2v = self.uwb.get('track_to_venue', {})
        raw = self.get('raw_fix', 2.0)
        if raw is not None:
            out['raw'] = [_r(v) for v in G.venue_to_track(raw.point.x, raw.point.y, t2v)]
        rng, st = self.get('uwb_ranges', 2.0), self.get('uwb_status', 3.0)
        rmap = {r.anchor_id: r for r in rng.ranges} if rng is not None else {}
        smap = {}
        if st is not None:
            for i, a in enumerate(st.anchor_ids):
                smap[a] = {'seen': bool(st.anchor_seen[i]) if i < len(st.anchor_seen) else False,
                           'age': _r(st.anchor_age_s[i], 2) if i < len(st.anchor_age_s) else None,
                           'fresh_count': int(st.anchor_fresh_count[i]) if i < len(st.anchor_fresh_count) else 0}
        anchors = []
        for a in self.uwb.get('anchors', []):
            aid = str(a.get('id'))
            xyz = a.get('xyz_m', [0, 0, 0])
            tx, ty = G.venue_to_track(xyz[0], xyz[1], t2v)
            r = rmap.get(aid)
            row = {'id': aid, 'x': _r(tx), 'y': _r(ty), 'offset': a.get('range_offset_m', 0.0),
                   'raw': _r(r.range_raw_m) if r else None, 'corr': _r(r.range_corrected_m) if r else None,
                   'fresh': bool(r.fresh) if r else False, 'gated': bool(r.gated_out) if r else False}
            row.update(smap.get(aid, {}))
            anchors.append(row)
        out['anchors'] = anchors
        if st is not None:
            out['uwb'] = {'link': st.agent_link_ok, 'hz': _r(st.rate_hz, 1), 'reboots': st.reboots,
                          'surveyed': st.anchors_surveyed, 'offsets': st.offsets_calibrated,
                          'latency_ms': _r(st.latency_ms, 0), 'unknown': st.last_unknown_id}
        ls = self.get('loc_status', 2.0) or self.get('local_status', 2.0)
        if ls is not None:
            est = {}
            for k in ('state', 'local_sigma_m', 'global_sigma_m', 'global_offset_x_m', 'global_offset_y_m',
                      'uwb_residual_m', 'uwb_gain', 'uwb_accepted', 'uwb_rejected', 'uwb_reacquires',
                      'visual_updates', 'visual_enabled', 'imu_ok', 'heading_rad', 'distance_travelled_m',
                      'uwb_age_s'):
                v = getattr(ls, k)
                est[k] = _r(v) if isinstance(v, float) else v
            out['est'] = est
        with self.lock:
            out['trails'] = {k: list(v) for k, v in self.trails.items()}
        out['aligned'] = bool(t2v.get('aligned', False))
        return out

    def tab_det(self, q):
        with self.lock:
            mm = list(self.mismatch)
        m = self.get('mission')
        return {'det': self._dets(self.get('detections', 1.0)), 'mismatch': mm,
                'challenge': m.challenge_name if m else ''}

    def tab_control(self, q):
        o, s = self.get('owner', 1.0), self.get('safety', 1.0)
        out = {'owner': None, 'safety': None, 'requests': {}}
        now = time.monotonic()
        if o is not None:
            out['owner'] = {'source': o.active_source, 'winner': o.winner, 'reason': o.reason, 'armed': o.armed,
                            'req_age': _r(o.request_age_s, 3), 'cmd_v': _r(o.commanded_speed_mps),
                            'cmd_s': _r(o.commanded_steer_rad), 'meas': _r(o.measured_speed_mps),
                            'lin': _r(o.out_linear_x), 'ang': _r(o.out_angular_z), 'watchdog': o.watchdog_ok}
        if s is not None:
            out['safety'] = {'allowed': s.motion_allowed, 'veto': s.veto_check, 'reason': s.veto_reason,
                             'checks': [{'name': c.name, 'ok': c.ok, 'value': _r(c.value), 'limit': _r(c.limit),
                                         'detail': c.detail} for c in s.checks]}
        for src in T.REQUEST_SOURCES:
            with self.lock:
                v = self.raw.get('req_' + src.lower())
            out['requests'][src] = ({'age': _r(now - v[0], 2), 'v': _r(v[1].speed_mps), 's': _r(v[1].steer_rad),
                                     'reason': v[1].reason} if v else None)
        with self.lock:
            h = list(self.hist)
        t0 = h[-1][0] if h else now
        out['hist'] = [[round(r[0] - t0, 2)] + r[1:] for r in h]
        out['history_s'] = float(self.p('control_history_s'))
        return out

    def tab_health(self, q):
        h = self.get('health', 5.0)
        with self.lock:
            nodes = G.node_rows(self.nodes, time.monotonic(), float(self.p('stale_after_s')))
        out = {'nodes': nodes, 'sys': None}
        if h is not None:
            out['sys'] = {'cpu': _r(h.cpu_percent, 0), 'cores': [_r(c, 0) for c in h.cpu_core_percent],
                          'bpu': _r(h.bpu_percent, 0), 'ram': _r(h.ram_percent, 0), 'temp': _r(h.soc_temp_c, 1),
                          'battery': _r(h.battery_v, 2), 'battery_ok': h.battery_ok, 'agent': h.uwb_agent_running,
                          'procs': [[n, pid] for n, pid in zip(h.camera_process_names, h.camera_process_pids)],
                          'topics': [{'topic': t.topic, 'hz': _r(t.rate_hz, 1), 'expected': _r(t.expected_hz, 1),
                                      'age': _r(t.age_s, 2), 'latency': _r(t.latency_ms, 0), 'ok': t.ok}
                                     for t in h.topics]}
        return out

    def tab_events(self, q):
        return {'events': self.events.since(int(q.get('since', ['0'])[0] or 0))}

    def tab_calibration(self, q):
        c = self.get('calib')
        out = {'session': '', 'steps': [], 'live': None, 'wizard': self._wizard_up()}
        if c is not None:
            out.update({'session': c.session, 'current': c.current_index,
                        'steps': [{'index': s.index, 'id': s.id, 'title': s.title, 'status': s.status,
                                   'required': s.required, 'can_advance': s.can_advance,
                                   'summary': s.result_summary, 'file': s.result_file,
                                   'previous': s.previous_session} for s in c.steps]})
        lv = self.get('calib_live')
        if lv is not None:
            try:
                out['live'] = json.loads(lv.data)
            except ValueError:
                out['live'] = {'error': 'calibration_wizard sent unreadable live data'}
        return out

    def _wizard_up(self):
        with self.lock:
            v = self.nodes.get('calibration_wizard')
        if v is None:
            return {'up': False, 'detail': 'no heartbeat from calibration_wizard yet'}
        stale = time.monotonic() - v[0] > float(self.p('stale_after_s'))
        return {'up': not stale, 'level': v[1].level, 'detail': v[1].detail,
                'state': 'SILENT' if stale else v[1].state}

    def calibration_action(self, body):
        """GUI button -> CalibrationAction. Never blocks the HTTP thread for more than timeout."""
        if self.race or self.calib_cli is None:
            return {'ok': False, 'message': 'Calibration actions exist in calibrate mode only'}
        from carbot_interfaces.srv import CalibrationAction
        action = str(body.get('action', '')).upper()
        if not action:
            return {'ok': False, 'message': 'no action given'}
        if not self.calib_cli.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'message': 'calibration_wizard is not running: look for its error in the launch '
                                            'terminal (ros2 node list | grep calibration_wizard)'}
        req = CalibrationAction.Request(step_id=str(body.get('step', '')), action=action,
                                        argument=str(body.get('argument', '')))
        fut = self.calib_cli.call_async(req)
        end = time.time() + 5.0
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        if not fut.done():
            return {'ok': False, 'message': 'calibration_wizard did not answer within 5 s (busy or stuck)'}
        r = fut.result()
        if r is None:
            return {'ok': False, 'message': 'calibration_wizard call failed'}
        if action not in ('SELECT',):
            self.events.add('calibration', 'info' if r.ok else 'warn', 'wizard', f'{action} {req.step_id}: {r.message}')
        return {'ok': bool(r.ok), 'message': r.message, 'passed': bool(r.passed), 'result_yaml': r.result_yaml}

    # ------------------------------------------------------------------ actions
    def image(self, key):
        if key not in G.IMAGE_KEYS:
            return None
        self.lz.touch('img:' + key)
        with self.lock:
            v = self.images.get(key)
        return v[1] if v and time.monotonic() - v[0] < 3.0 else None

    def estop(self):
        self.get_logger().warn('E-STOP pressed in GUI' + (' (counts as manual intervention)' if self.race else ''))
        self.pub_estop.publish(Bool(data=True))
        self.estopped = True

    def estop_release(self):
        if self.race:
            return False, 'Race mode latches the e-stop: restart race.launch.py'
        self.pub_estop.publish(Bool(data=False))
        self.estopped = False
        self.events.add('manual', 'ok', 'gui', 'E-stop released')
        return True, 'released'

    def set_manual(self, on, confirm):
        armed = self.get('armed')
        after_start = self.race and armed is not None and bool(armed.data)
        if on and after_start and bool(self.p('race.manual_confirm')) and not confirm:
            return False, 'confirm required: counts as manual intervention = 0 marks'
        self.manual = bool(on)
        self.pub_manual.publish(Bool(data=self.manual))
        text = ('Manual control ON' + (' (manual intervention, 0 marks)' if after_start else '')) if on \
            else 'Manual control OFF, handed back'
        self.events.add('manual', 'bad' if on and after_start else 'warn', 'gui', text)
        self.get_logger().warn(text)
        return True, text

    def start(self):
        from std_srvs.srv import Trigger
        if not self.race or self.start_cli is None:
            return False, 'START exists in race mode only'
        if not self.start_cli.wait_for_service(timeout_sec=0.5):
            return False, 'race_supervisor not available'
        fut = self.start_cli.call_async(Trigger.Request())
        end = time.time() + 3.0
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        if not fut.done():
            return False, 'timeout'
        r = fut.result()
        return bool(r.success), r.message

    def config(self):
        race = self.race
        scale = float(self.p('race.diagnostic_rate_scale')) if race else 1.0
        tabs = G.RACE_TABS if race else G.CALIBRATE_TABS
        if not self.params:
            tabs = [t for t in tabs if t != 'tuning']
        return {'mode': self.mode, 'tabs': [{'id': t, 'title': G.TAB_TITLES[t]} for t in tabs],
                'read_only': race and bool(self.p('race.diagnostics_read_only')),
                'tuning': self.params is not None, 'split_view': race and bool(self.p('race.split_view')),
                'estop_label': str(self.p('race.estop_label')) if race else 'Stop motors',
                'manual_confirm': race and bool(self.p('race.manual_confirm')),
                'rates': {'core': float(self.p('state_rate_hz')), 'map': float(self.p('map_rate_hz')) * scale,
                          'lidar': float(self.p('lidar_rate_hz')) * scale,
                          'cand': float(self.p('candidates_rate_hz')) * scale,
                          'grid': float(self.p('grid_rate_hz')) * scale,
                          'health': float(self.p('health_rate_hz')),
                          'image': float(self.p('image_max_fps')) * scale},
                'session': os.path.basename(self.session.rstrip('/')) if self.session else '',
                'calib_steps': [] if race else self.calib_steps}


# ---------------------------------------------------------------------- HTTP
def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001  source checkout without install
        return os.path.join(os.path.dirname(__file__), '..', '..', pkg)


def serve(node: GuiServer, web_dir: str):
    params_dir = os.path.join(_share('carbot_bringup'), 'config', 'params')
    types = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
             '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png'}
    web_dir = os.path.abspath(web_dir)

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, separators=(',', ':'), default=str).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            if body:
                self.wfile.write(body)

        def _json_body(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            try:
                return json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                return {}

        def do_GET(self):
            u = urlparse(self.path)
            q, path = parse_qs(u.query), u.path
            try:
                if path == '/api/config':
                    return self._send(200, node.config())
                if path == '/api/core':
                    return self._send(200, node.core())
                if path.startswith('/api/tab/'):
                    return self._send(200, node.tab(path[9:], q))
                if path.startswith('/api/img/'):
                    b = node.image(path[9:])
                    return self._send(200, b, 'image/jpeg') if b else self._send(204, b'', 'image/jpeg')
                if path == '/api/events':
                    return self._send(200, node.tab_events(q))
                if path == '/api/params':
                    if not node.params:
                        return self._send(403, {'error': 'Tuning is disabled in race mode'})
                    from .params_api import catalogue
                    return self._send(200, {'rows': catalogue(params_dir, node.session), 'session': node.session})
                f = 'index.html' if path in ('/', '') else path.lstrip('/')
                norm = os.path.normpath(f)
                if norm.startswith('..') or os.path.isabs(norm):
                    return self._send(404, b'not found', 'text/plain')
                full = os.path.join(web_dir, norm)
                if os.path.isfile(full):
                    with open(full, 'rb') as fh:
                        return self._send(200, fh.read(), types.get(os.path.splitext(full)[1], 'text/plain'))
                return self._send(404, b'not found', 'text/plain')
            except (BrokenPipeError, ConnectionResetError):
                return None
            except Exception as e:  # noqa: BLE001  one bad topic must not kill the GUI
                node.get_logger().error(f'GET {path}: {e!r}')
                return self._send(500, {'error': str(e)})

        def do_POST(self):
            path = urlparse(self.path).path
            body = self._json_body()
            try:
                if path == '/api/estop':
                    node.estop()
                    return self._send(200, {'ok': True})
                if path == '/api/estop_release':
                    ok, msg = node.estop_release()
                    return self._send(200, {'ok': ok, 'message': msg})
                if path == '/api/manual':
                    ok, msg = node.set_manual(bool(body.get('on')), bool(body.get('confirm')))
                    return self._send(200, {'ok': ok, 'message': msg})
                if path == '/api/start':
                    ok, msg = node.start()
                    return self._send(200, {'ok': ok, 'message': msg})
                if path == '/api/calibration/action':
                    return self._send(200, node.calibration_action(body))
                if path.startswith('/api/params/'):
                    return self._params(path[12:], body)
                return self._send(404, {'error': 'not found'})
            except Exception as e:  # noqa: BLE001
                node.get_logger().error(f'POST {path}: {e!r}')
                return self._send(500, {'ok': False, 'message': str(e)})

        def _params(self, action, body):
            from .params_api import coerce, save
            if not node.params:
                return self._send(403, {'ok': False, 'message': 'Tuning is disabled in race mode'})
            n, k = str(body.get('node', '')), str(body.get('key', ''))
            if action == 'get':
                vals, err = node.params.get_live(n, list(body.get('keys', [])))
                return self._send(200, {'ok': vals is not None, 'values': vals or {}, 'message': err})
            vals, _ = node.params.get_live(n, [k])
            like = (vals or {}).get(k)
            if like is None:
                like = body.get('default')
            try:
                v = coerce(body.get('value'), like)
            except (ValueError, TypeError) as e:
                return self._send(200, {'ok': False, 'message': f'Bad value: {e}'})
            if action == 'set':
                ok, msg = node.params.set_live(n, k, v)
                if ok:
                    node.events.add('tuning', 'info', n, f'{k} = {v} (live)')
                return self._send(200, {'ok': ok, 'message': msg, 'value': v})
            if action == 'save':
                try:
                    path = save(node.session, n, k, v)
                except RuntimeError as e:
                    return self._send(200, {'ok': False, 'message': str(e)})
                node.events.add('tuning', 'info', n, f'{k} = {v} saved')
                return self._send(200, {'ok': True, 'message': f'Saved to {path}', 'value': v})
            return self._send(404, {'ok': False, 'message': 'unknown action'})

    class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    srv = Server((str(node.p('host')), int(node.p('port'))), Handler)
    node.get_logger().info(f'GUI ({node.mode}) at http://<robot_ip>:{int(node.p("port"))}/  web={web_dir}')
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main(args=None):
    rclpy.init(args=args)
    node = GuiServer()
    web = str(node.p('web_root', '')) or os.path.join(_share('carbot_gui'), 'web')
    if not os.path.isdir(web):
        web = WEB_DIR_DEFAULT
    srv = serve(node, web)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        if node.params:
            node.params.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
