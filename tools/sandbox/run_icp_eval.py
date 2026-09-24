#!/usr/bin/env python3
"""Offline evaluation of the ICP-style local correction (no ROS): odometry drift + noisy front-camera road edges.

Runs the REAL estimator code from src/ (LocalEstimator + icp_core) on a simulated lap of the outer loop and
compares four modes on the same data:

  odom    wheel odometry + IMU heading only (no camera)
  map     + V4 visual update: camera edges -> PRIOR MAP boundary (translation only) = what the car runs today
  icp     + ICP against the accumulated camera-edge memory (icp_core), no map
  both    map + icp

for four scenarios (the camera grid is rendered from the map at the TRUE pose, 60 ms old, 8 Hz):

  nominal     wheel scale error 2 %, IMU noise 0.35 deg, camera grid exact
  imu_drift   nominal + an IMU yaw bias that keeps growing (default 0.3 deg/s)
  map_offset  the prior map is off by (2 cm, -1.5 cm, 0.3 deg) w.r.t. the real venue: the camera sees the real
              venue, so map matching is pulled toward the wrong boundary; memory-ICP does not use the map
  noisy       10 % of the paint-edge cells drop out, 0.3 % false paint cells, edge cells jitter by 1 cell

Prints position / heading error (against the physical truth), the largest single-step correction, how often the
ICP accepted / rejected, and the CPU time of one ICP update in Python (multiply by ~6-10 for the RDK X5).

  python tools/sandbox/run_icp_eval.py                       # all scenarios, all modes
  python tools/sandbox/run_icp_eval.py --scenarios imu_drift --modes map icp both
  python tools/sandbox/run_icp_eval.py --set icp.gain=1.0 icp.max_step_rad=0.002
"""
import argparse
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
for pkg in ('carbot_common', 'carbot_localization'):
    sys.path.insert(0, os.path.join(REPO, 'src', pkg))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from carbot_common.course import Course  # noqa: E402
from carbot_localization.estimator_core import (LocalCfg, LocalEstimator, PoseHistory,  # noqa: E402
                                                grid_axes, odom_increment)
from carbot_localization.icp_core import IcpCfg, IcpCorrector  # noqa: E402
from run_localization import demo_route, render  # noqa: E402

CONFIG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
MODES = ('odom', 'map', 'icp', 'both')
SCENARIOS = ('nominal', 'imu_drift', 'map_offset', 'noisy')


def load_local_params(overrides):
    with open(os.path.join(CONFIG, 'params', 'localization.yaml'), encoding='utf-8') as f:
        d = yaml.safe_load(f)['local_pose']['ros__parameters']
    for item in overrides:
        k, v = item.split('=', 1)
        cur = d
        parts = k.split('.')
        for q in parts[:-1]:
            cur = cur[q]
        if parts[-1] not in cur:
            raise KeyError(f'--set {k}: no such key')
        cur[parts[-1]] = yaml.safe_load(v)
    d['icp']['enabled'] = True

    def p(name):
        cur = d
        for k in name.split('.'):
            cur = cur[k]
        return cur
    return p


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def smooth_route(pts, half_window=12):
    """demo_route is a polyline with sharp corners between pieces: a real car cannot turn 90 deg in one sample.
    A Hann-window average (~0.6 m) gives a drivable truth path (heading changes at a finite rate)."""
    k = np.hanning(2 * half_window + 1)
    k /= k.sum()
    pad = np.concatenate([np.repeat(pts[:1], half_window, axis=0), pts, np.repeat(pts[-1:], half_window, axis=0)])
    out = np.stack([np.convolve(pad[:, 0], k, mode='valid'), np.convolve(pad[:, 1], k, mode='valid')], axis=1)
    return out


def perturb(kind, rng, dropout, false_paint, jitter):
    """Camera noise on the rendered grid: paint dropouts, false paint, one-cell edge jitter."""
    kind = kind.copy()
    paint = kind == 2
    if dropout > 0:
        drop = paint & (np.random.default_rng(rng.randrange(1 << 30)).random(kind.shape) < dropout)
        kind[drop] = 3
    if false_paint > 0:
        g = np.random.default_rng(rng.randrange(1 << 30))
        fp = (kind == 3) & (g.random(kind.shape) < false_paint)
        kind[fp] = 2
    if jitter:
        g = np.random.default_rng(rng.randrange(1 << 30))
        sh = g.integers(-1, 2, 2)
        kind = np.roll(kind, (int(sh[0]), int(sh[1])), axis=(0, 1))
    return kind


def simulate(mode, scenario, p, a):
    rng = random.Random(a.seed)
    with open(os.path.join(CONFIG, 'data', 'v4_reference', 'track_map.yaml')) as f:
        course = Course(yaml.safe_load(f))
    pts = smooth_route(demo_route(course))
    start = course.start_pose()
    est = LocalEstimator(LocalCfg.from_params(p), *start)
    icp = IcpCorrector(IcpCfg.from_params(p))
    hist = PoseHistory(p('pose_history_s'))
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s_path = np.concatenate([[0], np.cumsum(seg)])
    total = s_path[-1]
    dt, gdt = 0.05, 1.0 / 8.0
    speed = a.speed
    drift = math.radians(a.imu_drift_deg_s) if scenario == 'imu_drift' else 0.0
    map_off = (0.02, -0.015, math.radians(0.3)) if scenario == 'map_offset' else (0.0, 0.0, 0.0)
    noisy = scenario == 'noisy'
    truth, err_p, err_h, steps, err_lat, err_al = [], [], [], [], [], []
    icp_ms, prev = [], None
    odom_pose, imu_lag, last_grid = [0.0, 0.0, 0.0], 0.0, -1
    prev_odom = None
    prev_est = None
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
        if prev is not None:
            ds = math.hypot(x - prev[0], y - prev[1]) * (1 + a.encoder_error)
            dyaw = wrap(yaw - prev[2])
            odom_pose[2] += dyaw
            odom_pose[0] += ds * math.cos(odom_pose[2])
            odom_pose[1] += ds * math.sin(odom_pose[2])
        prev = (x, y, yaw)
        imu_true = yaw - start[2] + 0.7 + drift * t
        if i == 0:
            imu_lag = imu_true
        imu_lag = imu_lag + a.imu_alpha * wrap(imu_true - imu_lag)
        imu = imu_lag + rng.gauss(0, math.radians(a.imu_noise_deg))
        ds_o, dyaw_o = odom_increment(prev_odom, tuple(odom_pose))
        dyaw_o *= 1.0 + a.odom_yaw_error                       # /odom yaw comes from steering kinematics: small bias
        prev_odom = tuple(odom_pose)
        est.predict(ds_o, dt, imu, dyaw_o)
        hist.add(t, *est.odom)
        if mode != 'odom' and int(t / gdt) != last_grid:
            last_grid = int(t / gdt)
            tt = max(0.0, t - 0.06)
            tp = truth[max(0, int(tt / dt))]
            kind, grown, lx, ly = render(course, tp if scenario != 'map_offset' else _venue_pose(tp, map_off))
            if noisy:
                kind = perturb(kind, rng, 0.10, 0.003, True)
                grown = (kind == 1).astype(np.uint8)
            od = hist.at(tt, p('max_stamp_gap_s'))
            if od is not None:
                if mode in ('map', 'both'):
                    est.visual_update(kind, grown, lx, ly, (od[0] + est.tx, od[1] + est.ty, od[2] + est.ta), course)
                if mode in ('icp', 'both'):
                    t0 = time.perf_counter()
                    st = icp.on_grid(kind, grown, lx, ly, od, tt, est.distance, speed, od[2] + est.ta)
                    dtm = (time.perf_counter() - t0) * 1000.0
                    if st is not None:
                        icp_ms.append(dtm)
                        if st.applied:
                            est.apply_body_step(st.dx, st.dy, st.dth)
        ex, ey, ea = est.pose
        err_p.append(math.hypot(ex - x, ey - y))
        err_al.append((ex - x) * math.cos(yaw) + (ey - y) * math.sin(yaw))        # along the truth heading
        err_lat.append(-(ex - x) * math.sin(yaw) + (ey - y) * math.cos(yaw))      # sideways (what steering feels)
        err_h.append(wrap(ea - yaw))
        if prev_est is not None:
            steps.append(math.hypot(ex - prev_est[0], ey - prev_est[1]) - math.hypot(x - truth[-2][0], y - truth[-2][1]))
        prev_est = (ex, ey)
    e, h = np.array(err_p), np.degrees(np.array(err_h))
    ty = np.array([q[2] for q in truth])
    rate = np.abs(np.degrees(np.array([wrap(v) for v in (ty[10:] - ty[:-10])]))) / (10 * dt)   # deg/s over 0.5 s
    st = np.zeros(len(ty), bool)
    st[5:5 + len(rate)] = rate < 3.0                       # straight-ish sections (IMU EMA lag is ~0 there)
    if st.sum() < 10:
        st[:] = True
    return dict(mean_cm=e.mean() * 100, rms_cm=math.sqrt((e ** 2).mean()) * 100, p95_cm=np.percentile(e, 95) * 100,
                max_cm=e.max() * 100, h_rms=math.sqrt((h ** 2).mean()), h_max=np.abs(h).max(),
                s_rms_cm=math.sqrt((e[st] ** 2).mean()) * 100, s_h_rms=math.sqrt((h[st] ** 2).mean()),
                lat_rms=math.sqrt(float(np.mean(np.array(err_lat) ** 2))) * 100,
                lat_max=float(np.abs(err_lat).max()) * 100,
                al_rms=math.sqrt(float(np.mean(np.array(err_al) ** 2))) * 100,
                s_h_end=float(np.mean(h[st][-30:])),
                step_mm=max(steps) * 1000.0 if steps else 0.0, acc=icp.accepted, att=icp.attempts,
                rej=icp.rejected, ms_mean=float(np.mean(icp_ms)) if icp_ms else 0.0,
                ms_max=float(np.max(icp_ms)) if icp_ms else 0.0, trim=math.degrees(est.ta),
                icp_net_cm=float(np.hypot(*icp.net)) * 100)


def _venue_pose(true_pose, off):
    """The camera sees the REAL venue: express the true pose in prior-map coordinates (the map is off by `off`)."""
    x, y, a = true_pose
    c, s = math.cos(off[2]), math.sin(off[2])
    return (c * x - s * y + off[0], s * x + c * y + off[1], a + off[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenarios', nargs='*', default=list(SCENARIOS), choices=SCENARIOS)
    ap.add_argument('--modes', nargs='*', default=list(MODES), choices=MODES)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--encoder-error', type=float, default=0.02)
    ap.add_argument('--imu-noise-deg', type=float, default=0.35)
    ap.add_argument('--imu-drift-deg-s', type=float, default=0.3)
    ap.add_argument('--odom-yaw-error', type=float, default=0.05, help='scale error of the wheel-derived /odom yaw rate')
    ap.add_argument('--imu-alpha', type=float, default=0.15, help='servo_controller IMU EMA gain per 20 Hz sample (1 = no lag)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE', help='override local_pose params, e.g. icp.gain=1.0')
    a = ap.parse_args()
    p = load_local_params(a.set)
    print(f'{"scenario":11s} {"mode":5s} {"pos rms":>7s} {"max":>6s} | {"sideways":>8s} {"max":>6s} {"along":>6s} (cm rms) | '
          f'{"head rms":>8s} (deg) | {"step":>6s} | icp acc/att  ms mean/max  trim(deg) net(cm)')
    for sc in a.scenarios:
        for m in a.modes:
            r = simulate(m, sc, p, a)
            icp_txt = f'{r["acc"]:4d}/{r["att"]:<4d}  {r["ms_mean"]:5.1f}/{r["ms_max"]:<5.1f}  {r["trim"]:+7.3f}  {r["icp_net_cm"]:5.2f}' \
                if m in ('icp', 'both') else ''
            print(f'{sc:11s} {m:5s} {r["rms_cm"]:7.2f} {r["max_cm"]:6.2f} | {r["lat_rms"]:8.2f} {r["lat_max"]:6.2f} {r["al_rms"]:6.2f}            | '
                  f'{r["h_rms"]:8.3f}       | {r["step_mm"]:6.2f} | {icp_txt}')


if __name__ == '__main__':
    main()
