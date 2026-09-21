#!/usr/bin/env python3
"""Blocks 05 + 06 sandbox (no ROS): the REAL estimators from src/ on

  sim  (default) a simulated slow lap of the outer loop: wheel-scale error,
       IMU noise + lag, camera road grid rendered from the map, UWB ranges with
       uncalibrated offsets removed, noise, multipath spikes, a missing anchor.
  bag  --bag <rosbag2 folder> recorded on the car (see the phase-3 notes):
       /odom, /imu/rpy, /carbot/perception/road_grid, /uwb3/input_json.
       Prints the numbers used to tune localization.yaml (range_sigma_m, gates,
       latency, visual matching). Needs `pip install rosbags`.

Output: tools/sandbox/out/localization.png (truth / local / global / raw UWB)
and a text report.

  python tools/sandbox/run_localization.py
  python tools/sandbox/run_localization.py --start-error 0.3 --spikes 0.15 --drop 1783
  python tools/sandbox/run_localization.py --bag ~/carbot_data/bags/run_0925_0930
  python tools/sandbox/run_localization.py --bag BAG --session ~/carbot_data/calibration/20260925_0815
"""
import argparse
import json
import math
import os
import random
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
for pkg in ('carbot_common', 'carbot_localization', 'uwb_localization'):
    sys.path.insert(0, os.path.join(REPO, 'src', pkg))

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from carbot_common.course import Course  # noqa: E402
from carbot_localization.estimator_core import (GlobalCfg, GlobalEstimator, LocalCfg,  # noqa: E402
                                                LocalEstimator, PoseHistory, TrackToVenue,
                                                grid_axes, odom_increment)
from uwb_localization.uwb_core import AnchorSet, RangeProcessor, parse_report, trilaterate  # noqa: E402

CONFIG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
OUT = os.path.join(HERE, 'out')


def load_params(session=''):
    """localization.yaml (+ the session's params_overlay) and uwb.yaml (session copy wins)."""
    def rd(p):
        with open(p) as f:
            return yaml.safe_load(f) or {}
    loc = rd(os.path.join(CONFIG, 'params', 'localization.yaml'))
    uwb_path = os.path.join(CONFIG, 'data', 'uwb.yaml')
    if session:
        ov = os.path.join(session, 'params_overlay.yaml')
        if os.path.isfile(ov):
            for node, v in rd(ov).items():
                if node in loc:
                    loc[node]['ros__parameters'].update((v or {}).get('ros__parameters') or {})
        if os.path.isfile(os.path.join(session, 'data', 'uwb.yaml')):
            uwb_path = os.path.join(session, 'data', 'uwb.yaml')
    return loc, rd(uwb_path)


def getter(d):
    def p(name):
        cur = d
        for k in name.split('.'):
            cur = cur[k]
        return cur
    return p


class Pipeline:
    """Same wiring as the local_pose / global_pose / uwb_ranges nodes."""

    def __init__(self, loc, uwb, course, start):
        lp, gp, up = (getter(loc[n]['ros__parameters']) for n in ('local_pose', 'global_pose', 'uwb_ranges'))
        self.lp, self.gp = lp, gp
        self.course = course
        self.local = LocalEstimator(LocalCfg.from_params(lp), *start)
        self.glob = GlobalEstimator(GlobalCfg.from_params(gp))
        self.anchors = AnchorSet.from_yaml(uwb)
        self.t2v = TrackToVenue.from_yaml(uwb)
        self.lever = tuple(uwb['tag'].get('mount_xy_m', [0.0, 0.0]))
        self.proc = RangeProcessor(self.anchors, up('max_range_age_ms'), up('min_range_m'), up('max_range_m'),
                                   up('latency.mode'), up('latency.window_s'), up('latency.base_transit_ms'),
                                   up('latency.max_extra_ms'))
        self.odom_hist = PoseHistory(lp('pose_history_s'))
        self.local_hist = PoseHistory(gp('pose_history_s'))
        self.prev_odom = self.prev_t = None
        self.imu = None
        self.imu_t = -1e9
        self.last_gpose = None
        self.last_vis = 0
        self.log = defaultdict(list)

    def odom(self, t, x, y, yaw):
        ds, dyaw = odom_increment(self.prev_odom, (x, y, yaw))
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_odom, self.prev_t = (x, y, yaw), t
        if abs(ds) > self.lp('max_odom_step_m'):
            ds = dyaw = 0.0
        imu = self.imu if t - self.imu_t < self.lp('imu_timeout_s') else None
        self.local.predict(ds, dt, imu, dyaw)
        self.odom_hist.add(t, *self.local.odom)
        lx, ly, la = self.local.pose
        self.glob.predict(dt, abs(ds))
        self.local_hist.add(t, lx, ly, la)
        g = self.glob.pose(self.local.pose)
        self.log['t'].append(t)
        self.log['local'].append(self.local.pose)
        self.log['global'].append(g)
        self.log['sigma'].append(self.local.sigma)
        self.log['gsigma'].append(self.glob.sigma)

    def imu_yaw(self, t, yaw_deg):
        self.imu, self.imu_t = math.radians(yaw_deg), t

    def grid(self, t, kind, grown, lx, ly, pitch_deg=0.0):
        if abs(math.radians(pitch_deg)) > self.lp('visual.disable_pitch_rad'):
            return
        od = self.odom_hist.at(t, self.lp('max_stamp_gap_s'))
        if od is None:
            self.log['vis_skipped'].append(t)
            return
        r = self.local.visual_update(kind, grown, lx, ly,
                                     (od[0] + self.local.tx, od[1] + self.local.ty, od[2]), self.course)
        self.log['vis_matches'].append(r.matches)
        self.log['vis_rank'].append(r.rank)
        if r.applied:
            self.glob.landmark_update(r.rank)

    def uwb(self, t, text):
        rep = parse_report(text)
        if rep is None:
            return
        res = self.proc.process(rep, t)
        self.log['latency_ms'].append(res.latency_ms)
        fix_in = {r.anchor: r.corrected_m for r in res.ranges if r.reason in ('', 'repeat')}
        p = trilaterate(self.anchors, fix_in)
        if p is not None:
            self.log['raw_fix'].append(self.t2v.to_track(*p))
        for r in res.ranges:
            if not r.fresh:
                self.log['repeat' if r.reason == 'repeat' else 'dropped'].append(r.anchor)
                continue
            loc = self.local_hist.at(r.stamp, self.gp('max_uwb_age_s'))
            if loc is None:
                continue
            c, s = math.cos(loc[2]), math.sin(loc[2])
            tag = (loc[0] + self.lever[0] * c - self.lever[1] * s, loc[1] + self.lever[0] * s + self.lever[1] * c)
            a = self.anchors.anchors[r.anchor]
            u = self.glob.range_update(r.anchor, tag, (a.x, a.y), r.corrected_m, self.t2v)
            if u is not None:
                self.log[f'innov_{r.anchor}'].append(u.innovation)
                self.log[f'gate_{r.anchor}'].append(u.accepted)


# --------------------------------------------------------------------------- simulation
def demo_route(course):
    """A continuous clockwise lap from the start pose (track_map.yaml geometry):
    west along the start lane, lane change down into the roundabout, clockwise
    half ring, west exit past the gate, tunnel corner, up the left side, along
    the top, down the right side and back into the start lane."""
    from carbot_common.course import sample_arc, sample_line
    pi = math.pi
    parts = [sample_line(7.18, 1.30, 4.35, 1.30), sample_line(4.35, 1.30, 3.55, 0.80),
             sample_line(3.55, 0.80, 3.00, 0.80), sample_arc(2.40, 0.80, 0.60, 0.0, -pi),
             sample_line(1.80, 0.75, 1.00, 0.75), sample_arc(1.00, 1.50, 0.75, 1.5 * pi, pi),
             sample_line(0.25, 1.50, 0.25, 4.00), sample_arc(1.00, 4.00, 0.75, pi, 0.5 * pi),
             sample_line(1.00, 4.75, 6.00, 4.75), sample_arc(6.00, 4.00, 0.75, 0.5 * pi, 0.0),
             sample_line(6.75, 4.00, 6.75, 2.05), sample_arc(6.00, 2.05, 0.75, 0.0, -0.5 * pi),
             sample_line(6.00, 1.30, 5.00, 1.30)]
    return np.concatenate(parts) * np.array([course.sx, course.sy])


def simulate(a, loc, uwb, course):
    rng = random.Random(a.seed)
    pts = demo_route(course)
    start = course.start_pose()
    pipe = Pipeline(loc, uwb, course, start)
    t2v, anchors = TrackToVenue.from_yaml(uwb), AnchorSet.from_yaml(uwb)
    lever = pipe.lever
    speed, dt = a.speed, 0.05
    # dense truth path (0.025 m samples) resampled at the odom rate
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s_path = np.concatenate([[0], np.cumsum(seg)])
    total = s_path[-1]
    truth, odom_pose = [], [0.0, 0.0, 0.0]
    imu_lag = 0.0
    seq = {}
    tmp = 0.0
    last_grid = last_uwb = -1
    true_prev = None
    n = int(total / (speed * dt))
    for i in range(n):
        t = i * dt
        s = min(total, i * speed * dt)
        k = min(int(np.searchsorted(s_path, s)), len(pts) - 1)
        k0 = max(k - 1, 0)
        f = 0.0 if k == k0 else (s - s_path[k0]) / max(s_path[k] - s_path[k0], 1e-9)
        x = pts[k0][0] + f * (pts[k][0] - pts[k0][0])
        y = pts[k0][1] + f * (pts[k][1] - pts[k0][1])
        yaw = math.atan2(pts[k][1] - pts[k0][1], pts[k][0] - pts[k0][0]) if k != k0 else start[2]
        if i == 0:
            yaw = start[2]
        truth.append((x, y, yaw))
        # wheel odometry in its own frame (servo_controller): scale error + yaw from steering
        if true_prev is not None:
            ds = math.hypot(x - true_prev[0], y - true_prev[1]) * (1 + a.encoder_error)
            dyaw = math.atan2(math.sin(yaw - true_prev[2]), math.cos(yaw - true_prev[2])) * (1 + a.steer_yaw_error)
            odom_pose[2] += dyaw
            odom_pose[0] += ds * math.cos(odom_pose[2])
            odom_pose[1] += ds * math.sin(odom_pose[2])
        true_prev = (x, y, yaw)
        # IMU: power-on zero, EMA lag like servo_controller (alpha 0.15 at 20 Hz), noise
        imu_true = yaw - start[2] + 0.7
        if i == 0:
            imu_lag = imu_true          # servo_controller seeds its EMA with the first reading
        imu_lag = imu_lag + 0.15 * math.atan2(math.sin(imu_true - imu_lag), math.cos(imu_true - imu_lag))
        pipe.imu_yaw(t, math.degrees(imu_lag + rng.gauss(0, math.radians(a.imu_noise_deg))))
        pipe.odom(t, *odom_pose)
        if a.start_error and i == 0:
            pipe.local.reset(start[0] + a.start_error, start[1], start[2])
        # camera grid at ~15 Hz, 60 ms old
        if not a.no_camera and int(t * 15) != last_grid:
            last_grid = int(t * 15)
            tt = max(0.0, t - 0.06)
            tp = truth[max(0, int(tt / dt))]
            kind, grown, lx, ly = render(course, tp)
            pipe.grid(tt, kind, grown, lx, ly)
        # UWB at 10 Hz, per-anchor 7 Hz fresh, WiFi delay 40-130 ms, offsets already calibrated
        if int(t * 10) != last_uwb:
            last_uwb = int(t * 10)
            links = []
            c, s_ = math.cos(yaw), math.sin(yaw)
            v = t2v.to_venue(x + lever[0] * c - lever[1] * s_, y + lever[0] * s_ + lever[1] * c)
            for aid, an in anchors.anchors.items():
                if aid in a.drop and 0.3 * n * dt < t < 0.6 * n * dt:
                    continue
                if rng.random() < 0.7:
                    seq[aid] = seq.get(aid, 0) + 1
                r = math.hypot(v[0] - an.x, v[1] - an.y) + rng.gauss(0, a.uwb_noise)
                if rng.random() < a.spikes:
                    r += rng.uniform(0.3, 0.8)
                r3 = math.sqrt(r * r + (an.z - anchors.tag_z) ** 2) + an.offset
                links.append({'A': aid, 'R': r3, 'age_ms': 40, 'sample_seq': seq.get(aid, 0)})
            tmp = t + rng.uniform(0.04, 0.13)
            pipe.uwb(tmp, json.dumps({'tag': 'SIM', 'boot_id': 'SIM', 'seq': i, 't_ms': int(t * 1000),
                                      'links': links}))
    return pipe, truth


def render(course, pose, rows=100, res=0.018, x0=-0.65, y0=-0.9):
    lx, ly = grid_axes(rows, rows, res, x0, y0)
    X, Y = np.meshgrid(lx, ly, indexing='ij')
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = course.clearance(pose[0] + X * c - Y * s, pose[1] + X * s + Y * c)
    kind = np.where(d >= 0, 1, np.where(d >= -0.10, 2, 3)).astype(np.uint8)
    return kind, (kind == 1).astype(np.uint8), lx, ly


# --------------------------------------------------------------------------- bag replay
LOCALGRID_MSG = os.path.join(REPO, 'src', 'carbot_interfaces', 'msg', 'LocalGrid.msg')


def replay_bag(a, loc, uwb, course):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ImportError:
        sys.exit('bag mode needs: pip install rosbags')
    ts = get_typestore(Stores.ROS2_HUMBLE)
    ts.register(get_types_from_msg(open(LOCALGRID_MSG).read(), 'carbot_interfaces/msg/LocalGrid'))
    pipe = Pipeline(loc, uwb, course, course.start_pose())
    want = {'/odom', '/imu/rpy', '/imu/pitch', '/carbot/perception/road_grid', '/uwb3/input_json'}
    pitch = 0.0
    counts = defaultdict(int)
    with AnyReader([__import__('pathlib').Path(os.path.expanduser(a.bag))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic in want]
        missing = want - {c.topic for c in conns} - {'/imu/pitch'}
        if missing:
            print('WARNING: bag has no', sorted(missing))
        for c, t_ns, raw in rd.messages(connections=conns):
            m = rd.deserialize(raw, c.msgtype)
            t = t_ns * 1e-9
            counts[c.topic] += 1
            if c.topic == '/odom':
                q = m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                ts_ = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                pipe.odom(ts_, m.pose.pose.position.x, m.pose.pose.position.y, yaw)
            elif c.topic == '/imu/rpy':
                try:
                    pipe.imu_yaw(pipe.prev_t if pipe.prev_t is not None else t, float(json.loads(m.data)['yaw']))
                except (ValueError, KeyError):
                    pass
            elif c.topic == '/imu/pitch':
                pitch = float(m.data)
            elif c.topic == '/carbot/perception/road_grid':
                kind = np.asarray(m.kind, np.uint8).reshape(m.rows, m.cols)
                grown = np.asarray(m.grown, np.uint8).reshape(m.rows, m.cols)
                lx, ly = grid_axes(m.rows, m.cols, m.resolution_m, m.origin_x_m, m.origin_y_m)
                pipe.grid(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, kind, grown, lx, ly, pitch)
            elif c.topic == '/uwb3/input_json':
                pipe.uwb(t, m.data)
    print('messages:', dict(counts))
    return pipe, None


# --------------------------------------------------------------------------- report
def report(pipe, truth, a):
    L = pipe.log
    print('\n==== localization report ====')
    if truth:
        tr = np.array([p[:2] for p in truth])
        lo = np.array([p[:2] for p in L['local']])
        gl = np.array([p[:2] for p in L['global']])
        el, eg = np.hypot(*(lo - tr).T), np.hypot(*(gl - tr).T)
        print(f'local  error: median {np.median(el) * 100:5.1f} cm  p95 {np.percentile(el, 95) * 100:5.1f}  '
              f'end {el[-1] * 100:5.1f}')
        print(f'global error: median {np.median(eg) * 100:5.1f} cm  p95 {np.percentile(eg, 95) * 100:5.1f}  '
              f'end {eg[-1] * 100:5.1f}')
        steps = np.hypot(*np.diff(lo, axis=0).T)
        tsteps = np.hypot(*np.diff(tr, axis=0).T)
        print(f"largest local step beyond odometry: {np.max((steps - tsteps)[1:]) * 1000:.1f} mm (no jumps)")
    if L['vis_matches']:
        vm, vr = np.array(L['vis_matches']), np.array(L['vis_rank'])
        print(f'visual: {len(vm)} grids, matches median {np.median(vm):.0f}, '
              f'used {np.mean(vm >= pipe.local.c.vis_min_rows) * 100:.0f} %, '
              f'rank > landmark {np.mean(vr > pipe.glob.c.landmark_min_rank) * 100:.0f} %, '
              f'skipped (no odom at stamp) {len(L["vis_skipped"])}')
    lat = np.array(L['latency_ms']) if L['latency_ms'] else np.zeros(1)
    print(f'uwb latency above fastest packet: median {np.median(lat):.0f} ms, p95 {np.percentile(lat, 95):.0f} ms')
    print(f'repeated samples skipped {len(L["repeat"])}, dropped (old/out of range) {len(L["dropped"])}')
    print('per anchor:      n   accepted   innov median   robust sd   (m)')
    sds = []
    for aid in pipe.anchors.ids:
        inn, gate = np.array(L[f'innov_{aid}']), np.array(L[f'gate_{aid}'])
        if not len(inn):
            print(f'  {aid}:  no updates')
            continue
        sd = 1.4826 * np.median(np.abs(inn - np.median(inn)))
        sds.append(sd)
        print(f'  {aid}: {len(inn):6d}   {gate.mean() * 100:6.1f} %    {np.median(inn):+8.3f}      {sd:7.3f}')
    g = pipe.glob
    print(f'global: accepted {g.accepted}, gated {g.rejected}, reacquires {g.reacquires}, '
          f'final offset {math.hypot(*g.off) * 100:.1f} cm, sigma {g.sigma * 100:.1f} cm')
    if sds:
        cur = pipe.glob.c.range_sigma
        sug = float(np.clip(np.median(sds), 0.02, 0.3))
        print(f'\nSUGGEST range_sigma_m: {sug:.3f} (now {cur:.3f})')
        med = [np.median(L[f'innov_{a}']) for a in pipe.anchors.ids if L[f'innov_{a}']]
        if med and max(abs(m) for m in med) > 0.05:
            print('SUGGEST: an anchor has a median innovation > 5 cm -> redo step 10 offsets '
                  'or re-survey that anchor')
    if g.reacquires > 2:
        print('SUGGEST: many reacquires -> start pose / step 11 alignment is off')


def plot(pipe, truth, course, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib not installed: no plot)')
        return
    fig, ax = plt.subplots(figsize=(11, 7.5))
    X, Y = np.meshgrid(np.arange(course.nx) * course.res, np.arange(course.ny) * course.res)
    ax.contourf(X, Y, course.field, levels=[0, 10], colors=['#e8e8e8'])
    ax.contour(X, Y, course.field, levels=[0], colors=['#999'], linewidths=0.6)
    if truth:
        tr = np.array([p[:2] for p in truth])
        ax.plot(tr[:, 0], tr[:, 1], 'k-', lw=1, label='truth')
    lo = np.array([p[:2] for p in pipe.log['local']])
    gl = np.array([p[:2] for p in pipe.log['global']])
    if pipe.log['raw_fix']:
        rf = np.array(pipe.log['raw_fix'])
        ax.plot(rf[:, 0], rf[:, 1], '.', ms=2, color='#e8a33c', label='raw UWB fix')
    ax.plot(lo[:, 0], lo[:, 1], '-', color='#2f7de1', lw=1.2, label='block 05 local')
    ax.plot(gl[:, 0], gl[:, 1], '--', color='#2bb673', lw=1.2, label='block 06 global')
    for an in pipe.anchors.anchors.values():
        x, y = pipe.t2v.to_track(an.x, an.y)
        ax.plot(x, y, 'r^')
        ax.annotate(an.id, (x, y), fontsize=8)
    ax.set_aspect('equal')
    ax.legend(loc='upper center', ncol=4, fontsize=8)
    ax.set_title('blocks 05/06 (track frame, m)')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print('wrote', path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', default='')
    ap.add_argument('--session', default='', help='calibration session folder (uwb.yaml + overlay)')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--encoder-error', type=float, default=0.02, help='wheel distance scale error')
    ap.add_argument('--steer-yaw-error', type=float, default=0.10, help='/odom yaw error (fallback only)')
    ap.add_argument('--imu-noise-deg', type=float, default=0.35)
    ap.add_argument('--uwb-noise', type=float, default=0.04)
    ap.add_argument('--spikes', type=float, default=0.08)
    ap.add_argument('--drop', nargs='*', default=[], help='anchor ids missing for the middle third')
    ap.add_argument('--start-error', type=float, default=0.0, help='car placed this far (m) off the start')
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--aligned', action='store_true', default=True)
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    loc, uwb = load_params(os.path.expanduser(a.session))
    with open(os.path.join(CONFIG, 'data', 'v4_reference', 'track_map.yaml')) as f:
        course = Course(yaml.safe_load(f))
    if not a.bag:
        # the sim assumes steps 10 + 11 are done (offsets applied, map aligned)
        uwb['track_to_venue']['aligned'] = True
        for an in uwb['anchors']:
            an['range_offset_m'] = 0.0
    pipe, truth = replay_bag(a, loc, uwb, course) if a.bag else simulate(a, loc, uwb, course)
    report(pipe, truth, a)
    plot(pipe, truth, course, os.path.join(OUT, 'localization.png'))


if __name__ == '__main__':
    main()
