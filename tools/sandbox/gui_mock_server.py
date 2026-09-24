#!/usr/bin/env python3
"""GUI mock server: the real carbot_gui web/ files with synthetic data, no ROS.

    python3 tools/sandbox/gui_mock_server.py [--mode race|calibrate] [--scenario drive|stop] [--port 8081]
                                             [--sensors ok|bad] [--data-root DIR] [--skip-to N] [--unlock]

Calibrate mode runs the REAL calibration wizard logic (carbot_ops.wizard_core +
step_sensor_health on the repo YAML) against a synthetic sensor feed; --sensors bad
makes the LiDAR silent and adds a duplicate Astra process so the failure path can be seen.
Step 6 (IMU + wheel odometry) runs against MockCar: a car whose encoder really has
1120 ticks/m (repo YAML 1050) and whose IMU reads 5 % short, so the first distance run
and the first spin get corrected and the second ones verify. The "hand" pushes / turns
it by itself while a test is in progress. Step 7 (servo + steering) drives the same car
from the wizard's CALIBRATION_RAW requests (MockDrive): full-lock radii 0.42 / 0.44 m, and
it drives straight at servo_center 93 (repo YAML 90), so the first straight run corrects.
Step 8 (speed PID) drives it too: its motor needs duty 0.045 to start and 0.8 duty per m/s
(repo YAML 0.08 / 1.0) with a 0.2 s lag; CALIBRATION requests go through command_owner's
real SpeedController (owner_core) with the mock owner's live parameters.
--skip-to N records MOCK passes for steps 1..N-1 in a new session so step N can be run
straight away.
Step 9 (venue thresholds) runs against MockVenue: a synthetic road_perception grid +
stitched colours (venue road lighter than V4, a 35 % dark tunnel) and its classify.*
parameters (--skip-to 9 unlocks the page at once).
Step 12 (mission planner) runs the real tools/map planner on the repo track_map.yaml
(--skip-to 12 unlocks the page at once).
Step 13 (practice runs) sees a synthetic mission: it enters the chosen challenge 3 s after
Start attempt and leaves it 6 s later (--skip-to 13 unlocks the page at once).
Step 10 (UWB survey) gets a synthetic tag at 10 Hz (repo anchor layout, ~1 m range
bias per anchor, 1 cm noise); Measure offsets / Verify move the fake tag to the spot
you type. --unlock makes every step without a mock page optional so later pages
(step 10) can be opened and run without passing steps 3-9.
Step 11 (map-UWB alignment): Start lap makes a mock hand push MockCar clockwise once
around the lane centre at 0.3 m/s (~70 s); the fake tag rides on the car, placed in
the venue with track_map.yaml venue_transform as the TRUE alignment, so the fit
should come back near x 1.23, y -0.40, yaw 1.79 deg. Record point parks the car on
the chosen pose. --skip-to 11 (or more) also writes step 10's data/uwb.yaml (the
mock tag's true offsets) into the new session.
Step 5 (LiDAR-camera alignment) gets a synthetic front image (cam_front) and /scan
of one upright target that moves between 3 spots every 12 s, with the LiDAR really
rotated by MOCK_LIDAR_DELTA_DEG; --skip-to 5 lets its Run be tried.
Sessions are written to --data-root (default: a temp folder).

Open http://localhost:8081/ . Use it to learn the tabs on a laptop, or to test
GUI changes without the car. Data shapes match carbot_gui/gui_server.py; the
header lines, leg progress and grids are computed by the same gui_core
functions the real server uses. Images return 204 ("No image yet").
"""
import argparse
import http.server
import json
import math
import os
import socketserver
import sys
import time
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_gui'))
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_common'))
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_ops'))
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_control'))
sys.path.insert(0, os.path.join(REPO, 'src', 'uwb_localization'))
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_localization'))
from carbot_gui import gui_core as G  # noqa: E402

sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_perception'))
WEB = os.path.join(REPO, 'src', 'carbot_gui', 'web')
T0 = time.time()
MOCK_LIDAR_DELTA_DEG = 3.0                      # step 5: how far the mock LiDAR is really rotated
MOCK_TARGETS = ((0.80, 0.26), (0.95, 0.0), (0.75, -0.24))


def t():
    return time.time() - T0


# ----------------------------------------------------------------------------- synthetic track + route
def loop_pts(step=0.02):
    """Rounded rectangle 6 x 4 m, anticlockwise, plus a spur to a parking bay."""
    pts = []
    W, Hh, r = 6.0, 4.0, 0.8
    corners = [(W - r, r, -90), (W - r, Hh - r, 0), (r, Hh - r, 90), (r, r, 180)]
    x, y = r, 0.0
    for cx, cy, a0 in corners:
        ex, ey = cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0))
        n = max(2, int(math.hypot(ex - x, ey - y) / step))
        pts += [(x + (ex - x) * i / n, y + (ey - y) * i / n) for i in range(n)]
        for k in range(int(r * math.pi / 2 / step)):
            a = math.radians(a0 + 90 * k / (r * math.pi / 2 / step))
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        x, y = cx + r * math.cos(math.radians(a0 + 90)), cy + r * math.sin(math.radians(a0 + 90))
    return pts


LOOP = loop_pts()
N = len(LOOP)
ROUTE = [(x, y, 1.0) for x, y in LOOP] + [(LOOP[-1][0], LOOP[-1][1] + k * 0.02, -1.0) for k in range(1, 30)]
cut = [0, N // 3, 2 * N // 3, len(ROUTE) - 1]
INFO = {'ok': True, 'reason': '', 'warnings': [],
        'pieces': [{'index': 0, 'leg': 0, 'kind': 'road', 'start': 0, 'end': cut[1], 'length_m': 6.1},
                   {'index': 1, 'leg': 1, 'kind': 'road', 'start': cut[1] + 1, 'end': cut[2], 'length_m': 6.0},
                   {'index': 2, 'leg': 2, 'kind': 'road', 'start': cut[2] + 1, 'end': N - 1, 'length_m': 5.4},
                   {'index': 3, 'leg': 2, 'kind': 'manoeuvre', 'bay': 'parallel', 'start': N, 'end': len(ROUTE) - 1,
                    'length_m': 0.6}],
        'legs': [{'index': 0, 'id': 'leg1', 'exits': []}, {'index': 1, 'id': 'leg2', 'exits': ['north']},
                 {'index': 2, 'id': 'leg3', 'exits': ['west'], 'parking_bay': 'parallel'}],
        'visits': [{'visit': 1, 'piece': 1, 'exit': 'north', 'label': 'north'},
                   {'visit': 2, 'piece': 2, 'exit': 'west', 'label': 'west'}]}
TRACK = {'version': 2, 'frame': 'track', 'fingerprints': {'track_map': 'mock3f9a2c1e'}, 'lane_width_m': 0.38,
         'ring': {'x': 3.0, 'y': 4.0, 'r': 0.45}, 'exits': {'north': [3.0, 4.6], 'west': [2.4, 4.0]},
         'sections': {'loop': [[round(x, 3), round(y, 3)] for x, y in LOOP[::5]]},
         'areas': {'tunnel': {'kind': 'tunnel', 'poly': [[5.75, 1.3], [6.25, 1.3], [6.25, 2.6], [5.75, 2.6]]},
                   'parking': {'kind': 'parking', 'poly': [[-0.3, 3.0], [0.3, 3.0], [0.3, 3.8], [-0.3, 3.8]]}},
         'paint': [], 'bounds': [-0.8, -0.6, 6.8, 5.0]}
ANCHORS = [('1782', 0.0, 0.0), ('1786', 7.5, 0.0), ('1783', 7.5, 4.83)]


def pose():
    i = int(t() * 25) % N
    x, y = LOOP[i]
    x2, y2 = LOOP[(i + 3) % N]
    return x, y, math.atan2(y2 - y, x2 - x), i


def leg_index(i):
    return 0 if i <= cut[1] else 1 if i <= cut[2] else 2


# ----------------------------------------------------------------------------- payloads
class Mock:
    def __init__(self, mode, scenario):
        self.mode, self.scn, self.manual, self.estop = mode, scenario, False, False
        self.events = G.EventLog(300)
        for k, (lvl, src, txt) in enumerate([('ok', 'ops', 'Armed (START)'), ('info', '08', 'Mode ROAD'),
                                             ('info', '08', 'Challenge 1 Lane change'),
                                             ('warn', '08', 'TRAFFIC HOLD'), ('bad', '14', 'Veto front_clearance: 0.12 m')]):
            self.events.add('mission' if src == '08' else 'safety' if src == '14' else 'ops', lvl, src, txt)
        self.events.add('log', 'warn', 'global_pose', 'anchor 1783 gated 0.41 m')

    def stop(self):
        return self.scn == 'stop' and not self.manual

    def config(self):
        race = self.mode == 'race'
        tabs = G.RACE_TABS if race else G.CALIBRATE_TABS
        return {'mode': self.mode, 'tabs': [{'id': x, 'title': G.TAB_TITLES[x]} for x in tabs], 'read_only': race,
                'tuning': not race, 'split_view': race, 'manual_confirm': race,
                'estop_label': 'E-STOP - counts as manual intervention = 0 marks' if race else 'Stop motors',
                'rates': {'core': 5, 'map': 2, 'lidar': 3, 'cand': 3, 'grid': 2, 'health': 1, 'image': 2},
                'session': '' if race else '20260924_141208',
                'calib_steps': [] if race else [{'index': x.index, 'id': x.id, 'title': x.title, 'required': x.required}
                                                for x in self.wiz.wiz.slots]}

    def legs(self):
        x, y, a, i = pose()
        return G.leg_progress(INFO, [(p[0], p[1]) for p in ROUTE], (x, y), leg_index(i))

    def core(self):
        stop = self.stop()
        mission = {'mode': 'HOLD' if stop else 'ROAD', 'hold_reason': 'TRAFFIC HOLD' if stop else '',
                   'speed_zone': '' if stop else 'bump', 'zone_max': 0.0 if stop else 0.35, 'set_max': 0.6,
                   'challenge_id': 7 if stop else 6, 'challenge_name': 'Traffic light' if stop else 'Speed bump',
                   'banner': 'Waiting for green' if stop else '', 'banner_level': 1, 'next_exit': 'north',
                   'source': 'ROAD', 'leg_index': 1}
        owner = {'winner': 'SAFETY_STOP' if stop else 'ROAD', 'reason': 'front_clearance' if stop else 'ok',
                 'armed': True, 'speed': 0.0 if stop else 0.42 + 0.02 * math.sin(t()), 'stale': False}
        safety = ({'motion_allowed': False, 'veto_check': 'front_clearance', 'veto_reason': 'obstacle',
                   'value': 0.12, 'limit': 0.15} if stop else {'motion_allowed': True, 'veto_check': ''})
        legs = self.legs()
        calib = flags = None
        if self.mode == 'calibrate':
            self.wiz.tick()
            st = self.wiz.wiz.state()
            cur = st['steps'][st['current'] - 1]
            calib = {'index': cur['index'], 'text': f"{cur['title']} · {cur['status']}"}
            flags = [{'i': x['index'], 'st': x['status'], 'adv': x['can_advance']} for x in st['steps']]
        out = {'mode': self.mode, 'mission': mission, 'owner': owner, 'safety': safety, 'battery_v': 11.62,
               'running': G.running_now(self.mode, owner, safety, mission, [], self.manual, legs, calib),
               'armed': self.mode == 'race', 'manual': self.manual, 'estopped': self.estop, 'legs': legs,
               'lane_locked': True, 'manual_cmd': {'lin': 0.32, 'ang': -0.18} if self.manual else None,
               'events_seq': self.events.seq, 'calib_steps': flags}
        if self.mode == 'race':
            out['preflight'] = {'state': 5, 'summary': 'running', 'session': '20260924_141208', 'missing': [],
                                'checks': [{'name': n, 'ok': True, 'value': 'ok', 'expected': 'ok', 'detail': ''}
                                           for n in ('cam_front_rate', 'lidar_rate',
                                                     'uwb_anchor_1782', 'uwb_anchor_1786', 'uwb_anchor_1783',
                                                     'battery', 'start_pose')]}
        return out

    def tab(self, name, q):
        x, y, a, i = pose()
        stop = self.stop()
        bend = 0.25 * math.sin(t() / 3)
        if name == 'drive':
            left = [[k * 0.1, 0.19 + bend * (k * 0.1) ** 2] for k in range(0, 30)]
            right = [[k * 0.1, -0.19 + bend * (k * 0.1) ** 2] for k in range(0, 30)]
            items = [{'cls': 'traffic_light_red' if stop else 'traffic_light_green', 'conf': 0.97, 'cam': 'front',
                      'px': 1.6, 'py': -0.45, 'dist': 1.6},
                     {'cls': 'speed_bump_sign', 'conf': 0.91, 'cam': 'front', 'px': 1.3, 'py': 0.45, 'dist': 1.3},
                     {'cls': 'boom_gate_open', 'conf': 0.88, 'cam': 'front', 'px': 2.4, 'py': -0.25, 'dist': 2.4}]
            return {'lanes': {'left': left, 'right': right, 'locked': True, 'mode': 'CAMERA CORRIDOR'},
                    'path': [[k * 0.1, bend * (k * 0.1) ** 2] for k in range(0, 25)],
                    'det': {'light': 'RED' if stop else 'GREEN', 'gate': 'OPEN', 'bump': True, 'ms': 11.8, 'items': items},
                    'obstacles': [{'x': 0.12 if stop else 0.84, 'y': 0.02, 'dist': 0.12 if stop else 0.84}]}
        if name in ('map', 'loc'):
            out = {'pose': [x, y, a], 'global': [x + 0.03, y - 0.02, a, [0.06, 0.03, 0.4]], 'legs': self.legs(),
                   'info': INFO, 'track_fp': json.dumps(TRACK['fingerprints'], sort_keys=True), 'route_n': str(len(ROUTE)),
                   'mismatch': [{'t': 41.2, 'visit': 1, 'planned': 'north', 'gate': 'CLOSED', 'conf': 0.61, 'note': ''}],
                   'trail': [[p[0], p[1]] for p in LOOP[max(0, i - 120):i:4]]}
            if q.get('have', [''])[0] != out['track_fp']:
                out['track'] = TRACK
            if q.get('have_route', [''])[0] != out['route_n']:
                out['route'] = [[round(p[0], 3), round(p[1], 3), p[2]] for p in G.decimate(ROUTE, 4000)]
            if name == 'loc':
                out['local_ellipse'] = [0.02, 0.012, 0.3]
                out['raw'] = [x + 0.08, y + 0.05]
                out['anchors'] = [{'id': aid, 'x': ax, 'y': ay, 'offset': 0.92, 'raw': math.hypot(ax - x, ay - y) + 0.92,
                                   'corr': math.hypot(ax - x, ay - y), 'fresh': True, 'gated': aid == '1783',
                                   'seen': True, 'age': 0.08, 'fresh_count': 1200} for aid, ax, ay in ANCHORS]
                out['uwb'] = {'link': True, 'hz': 9.8, 'reboots': 0, 'surveyed': True, 'offsets': True,
                              'latency_ms': 14, 'unknown': ''}
                out['est'] = {'state': 'RUNNING', 'local_sigma_m': 0.021, 'global_sigma_m': 0.064,
                              'global_offset_x_m': 0.031, 'global_offset_y_m': -0.018, 'uwb_residual_m': 0.047,
                              'uwb_gain': 0.18, 'uwb_accepted': 1284, 'uwb_rejected': 67, 'uwb_reacquires': 0,
                              'visual_updates': 412, 'imu_ok': True}
                out['trails'] = {'local': out['trail'], 'global': [[p[0] + 0.03, p[1] - 0.02] for p in out['trail']],
                                 'raw': [[p[0] + 0.07 * math.sin(k), p[1] + 0.06 * math.cos(k)] for k, p in enumerate(out['trail'])]}
                out['aligned'] = True
            return out
        if name == 'percplan':
            rows, cols = 100, 100
            kind, grown = [], []
            for r in range(rows):
                xx = -0.65 + r * 0.018
                c0 = bend * xx * xx
                for c in range(cols):
                    yy = -0.9 + c * 0.018
                    road = abs(yy - c0) < 0.2
                    paint = 0.2 <= abs(yy - c0) < 0.23
                    kind.append(1 if road else 2 if paint else 3)
                    grown.append(1 if road else 0)
            grid = dict(G.encode_grid(rows, cols, kind, grown), res=0.018, x0=-0.65, y0=-0.9, coverage=0.62, connected=3412)
            cands = []
            for k in range(-7, 8):
                pts = [[s * 1.1, k * 0.04 * s + bend * (s * 1.1) ** 2] for s in [j / 20 for j in range(21)]]
                bad = abs(k) >= 5
                cands.append({'id': k + 7, 'offset': k * 0.04, 'cost': 1.24 + abs(k) * 0.52, 'valid': not bad,
                              'relaxed': k in (-4, 3), 'sel': k == 0, 'clear': 0.03 if bad else 0.09, 'steer': 0.08,
                              'stage': '', 'reject': 'clearance 0.03 < 0.06 m' if bad else '', 'pts': pts})
            return {'grid': grid, 'src': q.get('src', ['local'])[0], 'cands': cands, 'selected': 7,
                    'corridor': {'guide': [[s * 0.1, bend * (s * 0.1) ** 2] for s in range(12)], 'observed': 4,
                                 'locked': True, 'offset': -0.02, 'mode': 'CAMERA CORRIDOR'},
                    'path': [], 'parking_state': {'state': 'IDLE', 'replans': 0}, 'recovery_state': {'state': 'IDLE'},
                    'perception_status': {'level': 0, 'detail': '3 cameras calibrated'}}
        if name == 'memory':
            rows = cols = 60
            kind, age = [], []
            for r in range(rows):
                for c in range(cols):
                    xx, yy = x - 1.5 + r * 0.05, y - 1.5 + c * 0.05
                    near = min((xx - p[0]) ** 2 + (yy - p[1]) ** 2 for p in LOOP[::10]) < 0.04
                    kind.append(1 if near else 0)
                    age.append(((r * 7 + c) % 30) / 10 if near else -1)
            grid = dict(G.encode_grid(rows, cols, kind, [], age), res=0.05, x0=x - 1.5, y0=y - 1.5, cells=sum(kind),
                        oldest=2.8, travel=1.12, frame='odom')
            scan = [[round(0.3 * math.cos(math.radians(d)) + 1.2 * math.cos(math.radians(d)), 3),
                     round(1.4 * math.sin(math.radians(d)), 3)] for d in range(0, 360, 2)]
            return {'grid': grid, 'odom': [x, y, a], 'scan': scan, 'scan_n': 452, 'closest': [0.12 if stop else 0.6, 2],
                    'tunnel': None, 'tunnel_trigger': False, 'tunnel_cmd': None, 'tunnel_forwarding': False,
                    'tunnel_steer': None, 'tunnel_mode': False}
        if name == 'det':
            return {'det': self.tab('drive', q)['det'],
                    'mismatch': [{'t': 41.2, 'visit': 1, 'planned': 'north', 'gate': 'CLOSED', 'conf': 0.61, 'note': ''}],
                    'challenge': 'Traffic light'}
        if name == 'control':
            now = t()
            hist = [[round(-k / 10, 2), 0.4, 0.4 * (1 - math.exp(-(120 - k) / 20)), 0.38 * (1 - math.exp(-(120 - k) / 25)),
                     0.14 * math.sin((now - k / 10) / 1.4), 0.13 * math.sin((now - k / 10 - 0.1) / 1.4)] for k in range(120, -1, -1)]
            if stop:
                hist = [r[:2] + [0.0, 0.0] + r[4:] if r[0] > -1.4 else r for r in hist]
            checks = [('camera_fresh', True, 0.04, 0.3), ('motion_fresh', True, 0.04, 0.25), ('local_sigma', True, 0.021, 0.08),
                      ('front_clearance', not stop, 0.12 if stop else 0.84, 0.15), ('road_mask', True, 96, 60)]
            return {'owner': {'source': 'ROAD', 'winner': 'SAFETY_STOP' if stop else 'ROAD', 'reason': 'ok', 'armed': True,
                              'req_age': 0.04, 'cmd_v': 0 if stop else 0.4, 'cmd_s': 0.081, 'meas': 0 if stop else 0.41,
                              'lin': 0 if stop else 0.318, 'ang': 0.204, 'watchdog': True},
                    'safety': {'allowed': not stop, 'veto': 'front_clearance' if stop else '', 'reason': 'obstacle',
                               'checks': [{'name': n, 'ok': o, 'value': v, 'limit': lim, 'detail': ''} for n, o, v, lim in checks]},
                    'requests': {'ROAD': {'age': 0.04, 'v': 0.4, 's': 0.081, 'reason': ''}, 'TUNNEL': None, 'PARKING': None,
                                 'RECOVERY': None}, 'hist': hist, 'history_s': 12.0}
        if name == 'health':
            tops = [('/camera/color/image_raw', 14.9, 15),
                    ('/scan', 10.1, 10), ('/odom', 20.0, 20), ('/carbot/owner/state', 50.0, 50)]
            return {'sys': {'cpu': 58, 'cores': [64, 58, 71, 49, 55, 62, 73, 40], 'bpu': 38, 'ram': 61, 'temp': 64.5,
                            'battery': 11.62, 'battery_ok': True, 'agent': True,
                            'procs': [['astra_camera_node', 2314]],
                            'topics': [{'topic': a, 'hz': b, 'expected': c, 'age': 0.03, 'latency': 40, 'ok': b > 0.8 * c}
                                       for a, b, c in tops]},
                    'nodes': [{'node': 'road_perception', 'block': '03', 'level': 0, 'state': 'RUNNING', 'detail': '1 camera',
                               'oldest_input_s': 0.05, 'never': [], 'stale': False},
                              {'node': 'mission_logic', 'block': '08', 'level': 1 if stop else 0, 'state': 'HOLD' if stop else 'ROAD',
                               'detail': 'TRAFFIC HOLD' if stop else '', 'oldest_input_s': 0.02, 'never': [], 'stale': False},
                              {'node': 'safety_monitor', 'block': '14', 'level': 1 if stop else 0, 'state': 'VETO' if stop else 'OK',
                               'detail': 'front_clearance' if stop else '', 'oldest_input_s': 0.03, 'never': [], 'stale': False}]}
        if name == 'events':
            return {'events': self.events.since(int(q.get('since', ['0'])[0] or 0))}
        if name == 'calibration':
            return self.wiz.tab()
        return {}


class MockServo:
    """servo_controller (steps 6-7) / command_owner (steps 7-8) parameter services: immediate replies."""

    def __init__(self, values=None):
        self.values = values or {'ticks_per_meter': 1050.0, 'odom_reverse_polarity': False, 'imu_yaw_scale': 1.0,
                                 'servo_center': 90, 'servo_range_left': 50, 'servo_range_right': 70}

    def get(self, names, done):
        done({n: self.values.get(n) for n in names})

    def set(self, values, done):
        self.values.update(values)
        done(True)


class MockDrive:
    """/carbot/calibration/request stream (steps 7-8)."""

    def __init__(self):
        self.cmd = None

    def command(self, source, speed, steer, reason):
        self.cmd = (source, speed, steer, reason)

    def stop(self):
        self.cmd = None


class MockCar:
    """Pushed / turned by a mock hand while step 6 has a test in progress, driven by the step 7
    and step 8 requests; publishes like servo_controller (/odom integrated per increment,
    /imu/rpy = normalise(raw * scale)). Motor: steady speed = (|duty| - STATIC_DUTY) / PER_MPS,
    first-order lag TAU; CALIBRATION (m/s) goes through owner_core.SpeedController."""
    TPM, IMU_GAIN, DRIFT_DEG_MIN = 1120.0, 0.95, 0.4
    RL, RR, TRUE_CENTRE = 0.42, 0.44, 93
    STATIC_DUTY, PER_MPS, TAU = 0.045, 0.8, 0.2

    def __init__(self, rec, servo, drive, owner):
        from carbot_control import owner_core
        self.rec, self.servo, self.drive, self.owner, self.oc = rec, servo, drive, owner, owner_core
        self.ctrl, self.v = owner_core.SpeedController(owner_core.SpeedCfg()), 0.0
        self.x, self.y, self.th, self.raw_yaw, self.t = 0.0, 0.0, 0.0, 37.0, time.monotonic()
        self.test, self.done_m, self.done_deg = None, 0.0, 0.0
        self.hand11, self.lap = None, None      # step 11: the "hand" pushes the car once around the track

    LAP_V, LAP_R = 0.3, 0.75                    # m/s, corner radius (track lane centre, clockwise)

    def _lap_segments(self, start):
        """Start pose (bottom lane, facing west) -> clockwise lap along the lane centre:
        [(seconds, v, w)]. Lane corners at x 0.25 / 6.75, y 0.75 / 4.75 (track_map.yaml)."""
        v, r = self.LAP_V, self.LAP_R
        turn = (math.pi / 2 * r / v, v, -v / r)
        out = []
        for L in (start[0] - 1.0, 4.0 - 1.5, 6.0 - 1.0, 4.0 - 1.5, 6.0 - start[0]):
            if L > 0:
                out.append((L / v, v, 0.0))
            out.append(turn)
        return out[:-1]

    def _lap_hand(self, dt):
        """While step 11 records a lap: (dm, dd deg) for this increment; the true track pose is kept
        in self.lap['hist'] so the fake UWB tag reports from where the car really is."""
        h = self.hand11
        run = h.run if h is not None else None
        if run is None or run['mode'] != 'lap':
            return None
        if self.lap is None or self.lap['run'] is not run:
            p = h.poses['start_pose']
            self.lap = {'run': run, 't0': self.t, 'pose': list(p), 'segs': self._lap_segments(p),
                        'hist': [(self.t, p[0], p[1], p[2])]}
        el = self.t - self.lap['t0']
        v = w = 0.0
        for d, vv, ww in self.lap['segs']:
            if el < d:
                v, w = vv, ww
                break
            el -= d
        q = self.lap['pose']
        q[2] += w * dt
        q[0] += v * dt * math.cos(q[2])
        q[1] += v * dt * math.sin(q[2])
        self.lap['hist'].append((self.t, q[0], q[1], q[2]))
        del self.lap['hist'][:-400]
        return v * dt, math.degrees(w * dt)

    def lap_pose_at(self, t):
        if not self.lap:
            return None
        best = self.lap['hist'][0]
        for e in self.lap['hist']:
            if e[0] <= t:
                best = e
        return best[1:]

    def _curvature(self, z):
        if z < 0:                         # steer_sign -1: negative angular.z = LEFT
            return 1.0 / self.RL
        if z > 0:
            return -1.0 / self.RR
        v = self.servo.values
        return (self.TRUE_CENTRE - v['servo_center']) * (1.0 / self.RR) / v['servo_range_right']

    def _duty(self, c, dt):
        if c is None:
            self.ctrl.reset()
            return 0.0
        if c[0] == 'CALIBRATION_RAW':
            return c[1]
        o = self.owner.values
        self.ctrl.cfg = self.oc.SpeedCfg(kp=o['speed_pid.kp'], ki=o['speed_pid.ki'], kd=o['speed_pid.kd'],
                                         integral_limit=o['speed_pid.integral_limit'],
                                         duty_per_mps=o['feedforward.duty_per_mps'],
                                         static_duty=o['feedforward.static_duty'])
        return self.ctrl.update(c[1], self.v, dt)

    def update(self, step):
        from carbot_ops.step_imu_odometry import wrap_deg
        now = time.monotonic()
        a = step.active if step is not None else None
        if a is not self.test:
            self.test, self.done_m, self.done_deg = a, 0.0, 0.0
        while self.t < now:
            dt = min(0.05, now - self.t)
            self.t += dt
            dm = dd = 0.0
            if a and a['phase'] == 'measuring' and self.t >= self.rec.ignore_until + 0.5:
                if a['test'] == 'distance' and self.done_m < 2.0:
                    dm = min(0.3 * dt, 2.0 - self.done_m)
                if a['test'] == 'spin' and self.done_deg < 360.0:
                    dd = min(45.0 * dt, 360.0 - self.done_deg)
            self.done_m += dm
            self.done_deg += dd
            c = self.drive.cmd
            if c is not None or abs(self.v) > 1e-4:             # steps 7-8: the car drives itself
                duty = self._duty(c, dt)
                v_ss = math.copysign(max(0.0, abs(duty) - self.STATIC_DUTY) / self.PER_MPS, duty)
                self.v += (v_ss - self.v) * min(1.0, dt / self.TAU)
                dm = self.v * dt
                dd = math.degrees(dm * self._curvature(c[2] if c else 0.0))
            lap = self._lap_hand(dt)
            if lap is not None:                                 # step 11: pushed around the track
                dm, dd = lap
            v = self.servo.values
            ds = (-1.0 if v['odom_reverse_polarity'] else 1.0) * dm * self.TPM / v['ticks_per_meter']
            self.th += math.radians(dd)
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
            self.raw_yaw += self.IMU_GAIN * dd + self.DRIFT_DEG_MIN * dt / 60.0
            self.rec.on_odom(self.t, self.x, self.y, self.th, self.v)
            self.rec.on_imu(self.t, wrap_deg(wrap_deg(self.raw_yaw) * v['imu_yaw_scale']))


class MockVenue:
    """Step 9: road_perception's grid + stitched colours and its classify.* parameters, synthetic.
    Venue road lighter than V4 (luma ~125), cream tape at +-0.15 m, coloured floor outside;
    the tunnel sample sees the same scene at 35 % brightness."""
    N, RES, X0, Y0 = 100, 0.018, -0.65, -0.9

    def __init__(self, steps, cams):
        from carbot_ops.step_venue_thresholds import VenueThresholdsStep
        cfg = next(x for x in steps['steps'] if x['id'] == 'venue_thresholds')
        self.step = VenueThresholdsStep(cfg, cams)
        self.vals = {'classify.road_max_luma': 105, 'classify.road_max_chroma': 50, 'classify.paint_min_luma': 190}
        self.seq = 0

    # fake ServoLink on road_perception (answers at once)
    def get(self, names, done):
        done({n: self.vals.get(n) for n in names})

    def set(self, values, done):
        self.vals.update(values)
        done(True)

    def _scene(self):
        import numpy as np
        from carbot_ops import step_venue_thresholds as svt
        n = self.N
        rng = np.random.RandomState(self.seq % 50)
        c = self.X0 + (np.arange(n) + 0.5) * self.RES
        gx, gy = np.meshgrid(c, self.Y0 + (np.arange(n) + 0.5) * self.RES, indexing='ij')
        seen = (gx > 0.466) & (np.abs(gy) < 0.15 + 0.5 * (gx - 0.466))
        img = np.full((n, n, 3), 125.0)
        img[(np.abs(gy) > 0.14) & (np.abs(gy) < 0.175)] = 215
        img[np.abs(gy) >= 0.30] = (90, 150, 190)
        img += rng.randn(n, n, 1) * 4 + rng.randn(n, n, 3) * 1.5
        r = self.step.run
        dark = 0.35 if r and r['phase'] == 'tunnel' else 1.0
        img = np.clip(img * dark, 0, 255).astype(np.uint8)
        luma, chroma = svt.luma_chroma(img)
        kind = svt.classify(luma, chroma, {k.split('.')[1]: v for k, v in self.vals.items()})
        kind[~seen] = svt.UNSEEN
        return img, {'rows': n, 'cols': n, 'res': self.RES, 'x0': self.X0, 'y0': self.Y0, 'kind': kind, 'age_s': 0.1}

    def pair(self):
        self.seq += 1
        img, g = self._scene()
        return self.seq, g, img

    def inputs(self):
        return {'road_grid': lambda: self._scene()[1], 'road_pair': self.pair, 'road_params': self}


class MockWizard:
    """Real wizard_core and built steps against synthetic sensor and mission data."""

    def __init__(self, sensors, root, skip_to=0, unlock=False):
        import yaml
        from carbot_common import calibration_store as cs
        from carbot_ops import wizard_core as wc
        from carbot_ops import wizard_uwb as wu
        from carbot_ops.step_camera_identity import CameraIdentityStep
        from carbot_ops.step_imu_odometry import ImuOdometryStep, MotionRecorder
        from carbot_ops.step_servo_steering import ServoSteeringStep
        from carbot_ops.step_mission_planner import MissionPlannerStep
        from carbot_ops.step_practice_runs import PracticeRunsStep
        from carbot_ops.step_lidar_camera import LidarCameraStep
        from carbot_ops.step_extrinsics_ipm import ExtrinsicsIpmStep
        from carbot_ops.step_sensor_health import SensorHealthStep
        from carbot_ops.step_speed_pid import SpeedPidStep
        data = os.path.join(REPO, 'src', 'carbot_bringup', 'config', 'data')
        from carbot_ops.step_uwb_survey import UwbSurveyStep
        from carbot_ops.step_map_uwb_alignment import MapUwbAlignmentStep
        ld = lambda n: yaml.safe_load(open(os.path.join(data, n)))  # noqa: E731
        self.steps, self.cams, self.uwb = ld('calibration_steps.yaml'), ld('cameras.yaml'), ld('uwb.yaml')
        self.step4_frames = {}
        if skip_to == 4:
            self._make_step4_frames(root)
        step1 = next(x for x in self.steps['steps'] if x['id'] == 'sensor_health')
        step2 = next(x for x in self.steps['steps'] if x['id'] == 'camera_identity')
        step6 = next(x for x in self.steps['steps'] if x['id'] == 'imu_odometry')
        step7 = next(x for x in self.steps['steps'] if x['id'] == 'servo_steering')
        step12 = next(x for x in self.steps['steps'] if x['id'] == 'mission_planner')
        sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_planning'))      # step 12 race-time route check
        step13 = next(x for x in self.steps['steps'] if x['id'] == 'practice_runs')
        step8 = next(x for x in self.steps['steps'] if x['id'] == 'speed_pid')
        self.motion, self.servo, self.drive = MotionRecorder(), MockServo(), MockDrive()
        owner = MockServo({'mode': 'calibrate', 'steering.steer_sign': -1.0,       # = control.yaml
                           'speed_pid.kp': 0.8, 'speed_pid.ki': 0.4, 'speed_pid.kd': 0.0, 'speed_pid.integral_limit': 0.15,
                           'feedforward.duty_per_mps': 1.0, 'feedforward.static_duty': 0.08})
        self.car = MockCar(self.motion, self.servo, self.drive, owner)
        self.step6 = ImuOdometryStep(step6, self.motion, self.servo)
        self.step7 = ServoSteeringStep(step7, self.motion, self.servo, owner, self.drive, 0.216)
        self.practice = PracticeRunsStep(step13, ld('challenges.yaml'))
        self.mission_events, self.mission_seq, self.mission_mark = [], 0, None
        self.venue = MockVenue(self.steps, self.cams)       # step 9 (venue thresholds)
        with open(os.path.join(REPO, 'src', 'carbot_bringup', 'config', 'params', 'drivers.yaml')) as fh:
            drv = yaml.safe_load(fh)
        self.laser = [float(x) for x in drv['carbot_tf']['ros__parameters']['base_to_laser']]
        step5 = next(x for x in self.steps['steps'] if x['id'] == 'lidar_camera')
        self.lidar_step = LidarCameraStep(step5, self.cams, lambda: self.cams, lambda: list(self.laser),
                                          clock=time.time)
        step4 = next(x for x in self.steps['steps'] if x['id'] == 'extrinsics_ipm')
        self.step4 = ExtrinsicsIpmStep(step4, self.cams, lambda: self.cams,
                                       os.path.join(REPO, 'src', 'carbot_bringup', 'config'), work_root=root)
        step10 = next(x for x in self.steps['steps'] if x['id'] == 'uwb_survey')
        impls = {'sensor_health': SensorHealthStep(step1, self.cams, self.uwb),
                 'camera_identity': CameraIdentityStep(step2, self.cams),
                 'extrinsics_ipm': self.step4,
                 'lidar_camera': self.lidar_step,
                 'imu_odometry': self.step6, 'servo_steering': self.step7,
                 'speed_pid': SpeedPidStep(step8, self.motion, owner, self.drive),
                 'venue_thresholds': self.venue.step,
                 'mission_planner': MissionPlannerStep(step12, os.path.dirname(data),
                                                       lambda: self.wiz.session),
                 'practice_runs': self.practice,
                 'uwb_survey': UwbSurveyStep(step10, self.uwb, os.path.join(REPO, 'src', 'carbot_bringup', 'config'))}
        step11 = next(x for x in self.steps['steps'] if x['id'] == 'map_uwb_alignment')
        self.step11 = MapUwbAlignmentStep(step11, self.uwb, os.path.join(REPO, 'src', 'carbot_bringup', 'config'),
                                          self.motion, lambda: self.wiz.session)
        impls['map_uwb_alignment'] = self.step11
        self.car.hand11 = self.step11
        tv = ld('track_map.yaml')['venue_transform']       # the mock venue's TRUE track -> venue
        self.t2v_true = (float(tv['x']), float(tv['y']), math.radians(float(tv['yaw_deg'])))
        self.parked = None                                 # step 11 points: pose the car is parked on
        if unlock:
            for x in self.steps['steps']:
                if x['id'] not in impls:
                    x['required'] = False
        self.wiz = wc.Wizard(self.steps, root, {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True,
                                                'resume_max_age_h': 12.0, 'page_watch_s': 8.0}, impls)
        if skip_to > 1:
            sess = self.wiz._ensure_session()
            for x in self.wiz.slots:
                if x.index < skip_to:
                    cs.update_step(sess, x.id, 'PASS', '', summary='MOCK pass (--skip-to)')
            self.wiz.refresh()
            self.wiz.current = skip_to
            if skip_to > 6:        # as if step 6 had calibrated the mock car's encoder and IMU
                self.servo.values.update(ticks_per_meter=MockCar.TPM, imu_yaw_scale=round(1 / MockCar.IMU_GAIN, 4))
        # synthetic UWB tag (step 10): true anchors = repo layout, ~1 m uncalibrated range bias
        self.wu = wu
        self.uwb_feed = wu.UwbFeed(30.0, 3.0)
        truth = {str(a['id']): tuple(a['xyz_m']) for a in self.uwb['anchors']}
        self.uwb_tag = wu.SyntheticTag(truth, float(self.uwb['tag']['z_m']),
                                       {a: 0.9 + 0.05 * k for k, a in enumerate(sorted(truth))}, 0.01, xy=(5.0, 1.5))
        self.uwb_t = time.monotonic()
        if skip_to > 10:           # as if step 10 had saved the survey + the mock tag's real offsets
            from carbot_common import calib_tools as ct
            ct.merge_data(self.wiz._ensure_session(), 'uwb.yaml', self.uwb, {
                'anchors': [dict(a, range_offset_m=round(self.uwb_tag.bias[str(a['id'])], 4))
                            for a in self.uwb['anchors']],
                'anchors_surveyed': True, 'offsets_calibrated': True})
        self.sensors, self.seq, self.t_last = sensors, 0, 0.0
        self.task = {'name': 'restart_cameras', 'state': 'idle', 'message': '', 'log': []}
        self.fixed_at = None

    def _make_step4_frames(self, root):
        """Render the real printed floor boards through synthetic calibrated cameras."""
        import cv2
        from carbot_perception.calib_core import FloorBoard
        from carbot_perception.camera_model import camera_setup, save_intrinsics
        from run_calib_extrinsics import FloorWorld
        import sandbox_common as sb

        from carbot_common.calib_tools import floor_board_dicts
        boards = [FloorBoard.from_yaml(b) for b in floor_board_dicts(next(
            s for s in self.steps['steps'] if s['id'] == 'extrinsics_ipm')['target'])]
        world = FloorWorld(boards)
        for role, sensor in self.cams['roles'].items():
            if not self.cams['sensors'][sensor].get('enabled', True):
                continue
            _, s, mount, hfov = camera_setup(self.cams, role)
            w = int(s.get('image_width', s.get('width')))
            h = int(s.get('image_height', s.get('height')))
            intr = sb.synth_lens('pinhole', w, h, hfov)
            intr.source = 'calibrated'
            path = os.path.abspath(os.path.join(root, 'mock_intrinsics', sensor + '.yaml'))
            save_intrinsics(path, intr, sensor)
            self.cams['sensors'][sensor]['intrinsics_file'] = path
            camera = sb.VirtualCamera.build(role, intr.scaled(w * 2, h * 2), mount)
            image = cv2.resize(camera.render(world, (0.0, 0.0, 0.0)), (w, h), interpolation=cv2.INTER_AREA)
            self.step4_frames[sensor] = (1, image)

    def inputs(self):
        now = time.time()
        if now - self.t_last >= 1.0:
            self.seq, self.t_last = self.seq + 1, now
        bad = self.sensors == 'bad' and not (self.fixed_at and now > self.fixed_at)
        j = lambda v: v + 0.3 * math.sin(now + v)  # noqa: E731
        topics = {'/camera/color/image_raw': {'hz': j(14.9), 'age': 0.03, 'latency': 41},
                  '/scan': {'hz': 0.0, 'age': -1.0, 'latency': -1} if bad else {'hz': j(10.0), 'age': 0.08, 'latency': 22},
                  '/odom': {'hz': j(20.0), 'age': 0.05, 'latency': 4}, '/imu/rpy': {'hz': j(19.8), 'age': 0.05, 'latency': -1},
                  '/uwb3/input_json': {'hz': j(9.7), 'age': 0.1, 'latency': -1}}
        procs = [('astra_camera /', 2314)]
        if bad:
            procs.append(('astra_camera /', 1877))
        snap = {'health_age_s': 0.3, 'topics': topics, 'procs': procs, 'agent': True, 'battery_v': 11.62,
                'uwb': {'link': True, 'hz': 9.7, 'unknown': '',
                        'anchors': {a['id']: {'seen': True, 'age': 0.1} for a in self.uwb['anchors']}},
                'env': {'domain_id': '1', 'localhost_only': '0', 'ok': True, 'problems': []}}
        while self.uwb_t + 0.1 <= time.monotonic():         # 10 Hz synthetic tag reports
            self.uwb_t += 0.1
            self._tag_to_car(self.uwb_t)
            self.uwb_feed.add(self.uwb_tag.report(), self.uwb_t)
        return dict({'snap': snap, 'health_seq': self.seq, self.wu.INPUT_KEY: self.uwb_feed,
                     'scans': self.scans(now), 'now': now,
                     'frame': lambda sensor: self.step4_frames.get(sensor)},
                    **self.venue.inputs(), **self.mission())

    def mission(self):
        """Synthetic mission for step 13: enter the attempted challenge after 3 s, leave 6 s later."""
        cur, now = self.practice.cur, time.time()
        st = {'mode': 'ROAD', 'challenge_id': 0, 'challenge_name': '', 'hold_reason': '', 'banner': '', 'age_s': 0.1}
        if cur is None:
            self.mission_mark = None
        else:
            if self.mission_mark is None or self.mission_mark[0] != cur['n']:
                self.mission_mark = (cur['n'], now)
            dt = now - self.mission_mark[1]
            if 3.0 <= dt < 9.0:
                st.update(challenge_id=cur['challenge'], challenge_name=cur['name'])
                if self.mission_seq == 0 or self.mission_events[-1]['n'] != cur['n']:
                    self.mission_seq += 1
                    self.mission_events.append({'seq': self.mission_seq, 'n': cur['n'], 'name': 'CHALLENGE',
                                                'detail': f'Entered challenge {cur["challenge"]}: {cur["name"]}',
                                                'challenge_id': cur['challenge']})
        return {'mission': st, 'mission_events': self.mission_events[-50:], 'armed': True, 'manual': False}

    def _tag_to_car(self, t):
        """Step 11: the fake tag sits on the car (lap: pushed around; points: parked on a named pose)."""
        from carbot_localization.alignment import tag_position
        pose = self.car.lap_pose_at(t) if self.car.lap and self.step11.run is self.car.lap['run'] else self.parked
        if pose is None:
            return
        tx, ty = tag_position(tuple(pose), self.step11._lever())
        x, y, a = self.t2v_true
        self.uwb_tag.xy = [math.cos(a) * tx - math.sin(a) * ty + x, math.sin(a) * tx + math.cos(a) * ty + y]
    def target(self, now):
        return MOCK_TARGETS[int((now - T0) // 12) % len(MOCK_TARGETS)]

    def true_laser(self):
        m = list(self.laser)
        m[3] += math.radians(MOCK_LIDAR_DELTA_DEG)
        return m

    def scans(self, now, n=450, radius=0.03):
        """5 ray-cast 360 deg scans (10 Hz) of the current target, a little range noise."""
        import random
        L, (tx, ty) = self.true_laser(), self.target(now)
        inc = 2 * math.pi / n
        out = []
        for k in range(5):
            rng = random.Random(int(now * 10) - k)
            ranges = []
            for i in range(n):
                a = L[3] - math.pi + i * inc
                dx, dy = math.cos(a), math.sin(a)
                ox, oy = L[0] - tx, L[1] - ty
                b = ox * dx + oy * dy
                disc = b * b - (ox * ox + oy * oy - radius ** 2)
                r = -b - math.sqrt(disc) if disc >= 0 else 0.0
                ranges.append(r + rng.gauss(0, 0.004) if r > 0 else 0.0)
            out.append({'t': now - 0.1 * (4 - k), 'ranges': ranges, 'angle_min': -math.pi, 'angle_increment': inc,
                        'range_min': 0.02, 'range_max': 16.0})
        return out

    def front_jpeg(self):
        """Synthetic front camera: floor grid and the target as an upright bar (camera_model projection)."""
        import cv2
        import numpy as np
        g = self.lidar_step.geometry()
        w, h = g['w'], g['h']
        img = np.full((h, w, 3), (70, 70, 72), np.uint8)
        for x in np.arange(0.25, 2.01, 0.25):
            pts = np.array([[x, y, 0.0] for y in np.linspace(-1.2, 1.2, 25)])
            uv, ok, _ = g['cm'].project_ground(g['intr'], g['cam'], pts)
            pv = [tuple(int(v) for v in p) for p, o in zip(uv, ok) if o]
            for a, b in zip(pv, pv[1:]):
                cv2.line(img, a, b, (95, 95, 98), 1)
        tx, ty = self.target(time.time())
        fu, fv = g['cm'].project_ground(g['intr'], g['cam'], np.array([[tx, ty, 0.0]]))[0][0]
        tu, tv = g['cm'].project_ground(g['intr'], g['cam'], np.array([[tx, ty, 0.30]]))[0][0]
        half = max(3, int(abs(0.03 * g['intr'].K[0, 0] / max(0.2, tx - g['cam'].x))))
        cv2.rectangle(img, (int(fu) - half, int(tv)), (int(fu) + half, int(fv)), (30, 30, 200), -1)
        cv2.putText(img, 'mock front camera', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else b''

    def tick(self):
        self.car.update(self.step6)
        self.wiz.tick(self.inputs())
        if self.task['state'] == 'running' and time.time() > self.task['until']:
            self.task.update(state='done', message='Drivers restarted. Check the camera rows turn green (about 5 s).')
            self.fixed_at = time.time() + 2

    def action(self, body):
        a = str(body.get('action', '')).upper()
        if a == 'RESTART_CAMERAS':
            self.task = {'name': 'restart_cameras', 'state': 'running', 'message': 'kill stale camera processes',
                         'log': ['kill stale camera processes: sudo -n /usr/local/lib/carbot/kill_stale.sh ...',
                                 '  [kill_stale] stopping stale processes: 1877 astra_camera_container'], 'until': time.time() + 4}
            return {'ok': True, 'message': 'Restarting camera drivers (about 10 s): stale ones are killed first'}
        self.car.update(self.step6)
        try:                                    # the user carries the (fake) tag to the typed spot
            spot = json.loads(str(body.get('argument', '')) or '{}').get('spot')
            if a in ('RUN', 'REDO') and spot:
                self.uwb_tag.xy = [float(spot[0]), float(spot[1])]
            arg = json.loads(str(body.get('argument', '')) or '{}')
            if a in ('RUN', 'REDO') and arg.get('mode') == 'points':      # step 11: car parked on the pose
                self.parked = self.step11.poses.get(str(arg.get('pose')))
            elif a in ('RUN', 'REDO') and arg.get('mode') == 'lap':
                self.parked = None
        except (ValueError, AttributeError, TypeError, IndexError):
            pass
        return self.wiz.action(str(body.get('step', '')), a, str(body.get('argument', '')), self.inputs())

    def tab(self):
        self.tick()
        st = self.wiz.state()
        return {'session': st['session'] or '(not started)', 'current': st['current'], 'steps': st['steps'],
                'live': json.loads(json.dumps(self.wiz.live(self.inputs(), self.task), default=str)),
                'wizard': {'up': True, 'level': 0, 'state': 'RUNNING', 'detail': ''}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='race', choices=['race', 'calibrate'])
    ap.add_argument('--scenario', default='drive', choices=['drive', 'stop'])
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--sensors', default='ok', choices=['ok', 'bad'])
    ap.add_argument('--data-root', default='')
    ap.add_argument('--skip-to', type=int, default=0, help='calibrate: record MOCK passes for the steps before N')
    ap.add_argument('--unlock', action='store_true', help='calibrate: steps without a mock page are optional')
    a = ap.parse_args()
    mock = Mock(a.mode, a.scenario)
    if a.mode == 'calibrate':
        import tempfile
        root = a.data_root or tempfile.mkdtemp(prefix='carbot_mock_data_')
        mock.wiz = MockWizard(a.sensors, root, a.skip_to, a.unlock)
        print(f'calibration sessions -> {root}')

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *x):
            pass

        def _send(self, code, body, ctype='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q, p = parse_qs(u.query), u.path
            if p == '/api/config':
                return self._send(200, mock.config())
            if p == '/api/core':
                return self._send(200, mock.core())
            if p.startswith('/api/tab/'):
                return self._send(200, mock.tab(p[9:], q))
            if p.startswith('/api/img/'):
                if p == '/api/img/cam_front' and mock.mode == 'calibrate':
                    return self._send(200, mock.wiz.front_jpeg(), 'image/jpeg')
                return self._send(204, b'', 'image/jpeg')
            if p == '/api/params':
                return self._send(200, {'session': '/data/calibration/20260924_141208',
                                        'rows': [{'file': 'planning.yaml', 'node': 'local_planner', 'key': k, 'default': v,
                                                  'saved': v, 'overlay': False}
                                                 for k, v in (('candidate_count', 15), ('min_clearance_m', 0.06),
                                                              ('lookahead_m', 0.35))]})
            f = 'index.html' if p == '/' else p.lstrip('/')
            full = os.path.join(WEB, f)
            if os.path.isfile(full):
                ct = {'.js': 'text/javascript', '.css': 'text/css', '.html': 'text/html'}.get(os.path.splitext(f)[1], 'text/plain')
                with open(full, 'rb') as fh:
                    return self._send(200, fh.read(), ct)
            return self._send(404, b'nf', 'text/plain')

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(n) or b'{}')
            p = urlparse(self.path).path
            if p == '/api/manual':
                if body.get('on') and mock.mode == 'race' and not body.get('confirm'):
                    return self._send(200, {'ok': False, 'message': 'confirm required'})
                mock.manual = bool(body.get('on'))
                mock.events.add('manual', 'bad' if mock.manual else 'warn', 'gui', 'Manual control ' + ('ON' if mock.manual else 'OFF'))
                return self._send(200, {'ok': True, 'message': 'Manual control ' + ('ON' if mock.manual else 'OFF')})
            if p == '/api/estop':
                mock.estop = True
                if mock.mode == 'calibrate':                 # as calibration_wizard._on_estop
                    mock.wiz.drive.stop()
                    mock.wiz.wiz.cancel_running('STOP MOTORS pressed')
                return self._send(200, {'ok': True})
            if p == '/api/calibration/action':
                if mock.mode != 'calibrate':
                    return self._send(200, {'ok': False, 'message': 'Calibration actions exist in calibrate mode only'})
                return self._send(200, mock.wiz.action(body))
            if p == '/api/params/get':
                return self._send(200, {'ok': True, 'values': {'min_clearance_m': 0.05}})
            return self._send(200, {'ok': True, 'message': 'mock: ' + p})

    class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    print(f'GUI mock ({a.mode}, {a.scenario}) at http://localhost:{a.port}/')
    Server(('0.0.0.0', a.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
