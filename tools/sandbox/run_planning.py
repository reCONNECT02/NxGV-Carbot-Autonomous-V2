#!/usr/bin/env python3
"""Phase 4 sandbox: drive the whole mission on a laptop (no ROS, no car).

The REAL block 07-13 cores from src/carbot_planning (route check / V4 hybrid
A*, corridor, local planner, tracker, mission state machine) drive a simple
V4-style car around the map in config/data. Sensors are ideal (camera road
grid rendered from the map, true pose); the traffic light turns GREEN after
--red seconds of waiting; manoeuvres follow mission_planner's preview as a
stand-in for block 11 (phase 5).

  python tools/sandbox/run_planning.py                  # team map + mission (v2)
  python tools/sandbox/run_planning.py --v4             # V4 reference (v1, plans with hybrid A*)
  python tools/sandbox/run_planning.py --gate-closed    # Challenge 4 gate never opens -> GATE HOLD
  python tools/sandbox/run_planning.py --data ~/carbot_data/calibration/<session>/data

Writes tools/sandbox/out/planning/run.png (map, route, driven path coloured by
mode) and prints the mission events plus a report: minimum body margin per
piece (negative = body over the lane edge), tracking error, feasible candidates.
"""
import argparse
import collections
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
for p in (os.path.join(REPO, 'src', 'carbot_common'), os.path.join(REPO, 'src', 'carbot_planning')):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from carbot_common.course import file_fingerprints, load_course  # noqa: E402
from carbot_common.geometry import geometry  # noqa: E402
from carbot_common.mission import load_mission  # noqa: E402
from carbot_planning.route_core import PlanCfg, plan_mission  # noqa: E402
from carbot_planning.sim_core import simulate  # noqa: E402

CONFIG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
OUT = os.path.join(HERE, 'out', 'planning')
COLOURS = {'ROAD': (0.1, 0.45, 0.95), 'TUNNEL': (0.55, 0.3, 0.8), 'PARKING': (0.95, 0.55, 0.1),
           'HOLD': (0.9, 0.1, 0.1), 'RECOVERY': (0.2, 0.7, 0.3), 'SAFETY_STOP': (0.0, 0.0, 0.0)}


def draw(course, route, log, path):
    import cv2
    s = 120                                       # px per metre
    x0, y0 = course.x0, course.y0
    W, H = int(course.w * s), int(course.h * s)
    X, Y = np.meshgrid(x0 + (np.arange(W) + 0.5) / s, y0 + (H - np.arange(H) - 0.5) / s)
    d = course.clearance(X, Y)
    img = np.full((H, W, 3), (80, 130, 85), np.uint8)
    img[d >= 0] = (62, 60, 58)
    img[(d < 0) & (d > -0.03)] = (235, 235, 235)
    px = lambda P: np.column_stack([(P[:, 0] - x0) * s, H - (P[:, 1] - y0) * s]).astype(np.int32)  # noqa: E731
    for p in route.pieces:
        if len(p['points']):
            cv2.polylines(img, [px(p['points'])], False, (200, 200, 120), 1, cv2.LINE_AA)
    P = np.column_stack([log.x, log.y])
    modes = [m.split(':')[0] for m in log.mode]
    for i in range(1, len(P)):
        c = COLOURS.get(modes[i], (0.5, 0.5, 0.5))
        cv2.line(img, tuple(px(P[i - 1:i])[0]), tuple(px(P[i:i + 1])[0]),
                 tuple(int(255 * v) for v in c[::-1]), 2, cv2.LINE_AA)
    y = 20
    for k, c in COLOURS.items():
        cv2.putText(img, k, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tuple(int(255 * v) for v in c[::-1]), 2)
        y += 18
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default=os.path.join(CONFIG, 'data'), help='folder with track_map/mission(+rules/features)')
    ap.add_argument('--v4', action='store_true', help='use config/data/v4_reference (V4 simulator, v1 files)')
    ap.add_argument('--red', type=float, default=2.0, help='seconds of RED at the traffic light')
    ap.add_argument('--gate-closed', action='store_true', help='Challenge 4 gate stays CLOSED')
    ap.add_argument('--t-max', type=float, default=600.0)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    data = os.path.join(CONFIG, 'data', 'v4_reference') if a.v4 else os.path.expanduser(a.data)
    common = yaml.safe_load(open(os.path.join(CONFIG, 'params', 'common.yaml')))['/**']['ros__parameters']
    g = geometry(common['vehicle'])
    challenges = yaml.safe_load(open(os.path.join(CONFIG, 'data', 'challenges.yaml')))['challenges']
    course = load_course(os.path.join(data, 'track_map.yaml'))
    mission = load_mission(os.path.join(data, 'mission.yaml'))
    t0 = time.time()
    route = plan_mission(course, mission, g, PlanCfg(), file_fingerprints(os.path.join(data, 'track_map.yaml')),
                         0.005, log=lambda s: None)
    print(f'block 07: {"OK" if route.ok else "FAIL"} ({route.source}) {time.time() - t0:.1f} s  {route.reason}')
    for w in route.warnings:
        print('  warning:', w)
    if not route.ok:
        sys.exit(1)
    print('  pieces:', ', '.join(f'{p["leg_id"]}:{p["kind"]}' for p in route.pieces))
    print('  roundabout exits:', ', '.join(f'visit {v["visit"]} {v["exit"]}' for v in route.visits))
    t0 = time.time()
    log = simulate(course, route, mission.rules, challenges, g, common['limits'], t_max=a.t_max,
                   light_red_s=a.red, gate_open=not a.gate_closed, verbose=not a.quiet)
    print(f'\nsimulated {log.t[-1]:.0f} s in {time.time() - t0:.0f} s wall')
    M, Pc, E = np.array(log.margin), np.array(log.piece), np.array(log.track_error)
    for pi in sorted(set(Pc)):
        sel = Pc == pi
        p = route.pieces[pi]
        print(f'  piece {pi} {p["leg_id"]:5s} {p["kind"]:9s} min body margin {M[sel].min() * 100:6.2f} cm   '
              f'max tracking error {E[sel].max() * 100:5.1f} cm' + ('   (block 11 stand-in)' if p['kind'] == 'manoeuvre' else ''))
    print('  feasible local candidates per plan: min', min(log.candidates_valid), 'mean',
          round(float(np.mean(log.candidates_valid)), 1))
    print('  time per mode:', dict(collections.Counter(m.split(':')[0] for m in log.mode)))
    done = any(e[1] == 'MISSION COMPLETE' for e in log.events)
    print('  RESULT:', 'MISSION COMPLETE' if done else f'not complete, last mode {log.mode[-1]}')
    out = os.path.join(OUT, 'run.png')
    try:
        draw(course, route, log, out)
        print('  picture:', out)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
