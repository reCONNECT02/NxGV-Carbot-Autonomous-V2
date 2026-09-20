#!/usr/bin/env python3
"""Block 04 local_memory without ROS.

Drives the virtual car along the demo route. Every camera cycle the block-03
grid is integrated into LocalMemory at the odom pose interpolated at the
camera timestamp (exactly like the node: odom arrives at odom_rate_hz, the
camera frame is `latency` seconds old). Odom can drift (--drift-pct,
--yaw-drift-deg-s) so you can see memory smear when odometry is bad.

Window: left = published memory window (3 m, odom frame, north up) shaded by
age (bright = just seen, dark = old; cyan = paint), car arrow in the middle;
right = what the V4 driving rule would still accept (age < drive_max_age_s
and uncertainty base + per_m * travel_since <= limit).

Keys: q quit, space pause.

  python tools/sandbox/run_local_memory.py
  python tools/sandbox/run_local_memory.py --drift-pct 3 --yaw-drift-deg-s 1.5
"""
import argparse
import contextlib
import io
import math

import cv2
import numpy as np
import sandbox_common as sb
from carbot_perception.memory_core import LocalMemory, OdomHistory
from run_road_perception import Perception


def render_window(kind, age, travel, drive_max_age, prm, scale=3, pose=None, x0=0.0, y0=0.0, res=0.025):
    n = kind.shape[0]
    img = np.zeros((n, n, 3), np.uint8)
    seen = age >= 0
    shade = np.clip(1.0 - age / float(prm['display_max_age_s']), 0.15, 1.0)
    img[seen & (kind == 1)] = (np.array([60, 200, 90]) * shade[seen & (kind == 1)][:, None]).astype(np.uint8)
    img[seen & (kind == 2)] = (np.array([230, 230, 60]) * shade[seen & (kind == 2)][:, None]).astype(np.uint8)
    unc = prm['uncertainty_base_m'] + prm['uncertainty_per_m'] * travel     # V4 guidance.evidence
    ok = seen & (age < drive_max_age) & (unc <= prm['uncertainty_limit_m'])
    drive = np.zeros_like(img)
    drive[ok & (kind == 1)] = (60, 200, 90)
    drive[ok & (kind == 2)] = (230, 230, 60)
    out = []
    for im in (img, drive):
        v = cv2.flip(np.transpose(im, (1, 0, 2)), 0)       # rows=x -> screen right, cols=y -> screen up
        v = cv2.resize(v, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        if pose is not None:
            px = int(((pose[0] - x0) / res) * scale)
            py = int((n - (pose[1] - y0) / res) * scale)
            q = (int(px + 18 * math.cos(pose[2])), int(py - 18 * math.sin(pose[2])))
            cv2.arrowedLine(v, (px, py), q, (0, 0, 255), 2, tipLength=0.4)
        out.append(v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=0.18)
    ap.add_argument('--cam-rate', type=float, default=0.0, help='default perception.yaml process_rate_hz')
    ap.add_argument('--odom-rate', type=float, default=50.0)
    ap.add_argument('--latency', type=float, default=0.05, help='camera frame age at processing (s)')
    ap.add_argument('--drift-pct', type=float, default=0.0, help='odom distance scale error (%%)')
    ap.add_argument('--yaw-drift-deg-s', type=float, default=0.0, help='odom heading drift')
    ap.add_argument('--lens', choices=['auto', 'pinhole', 'fisheye'], default='auto')
    ap.add_argument('--cameras', default='')
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--frames', type=int, default=0)
    a = ap.parse_args()

    prm = sb.params('local_memory')
    pprm = sb.params('road_perception')
    veh = sb.common()['vehicle']
    cams = sb.cameras(a.cameras)
    cam_rate = a.cam_rate or float(pprm['process_rate_hz'])
    world = sb.TrackWorld()
    rig = sb.camera_rig(cams, 480, a.lens)
    with contextlib.redirect_stdout(io.StringIO()):
        per = Perception(cams, pprm, veh, {r: c.intr for r, c in rig.items()})
    mem = LocalMemory(prm['resolution_m'], prm['ring_cells'], prm['max_cells'], prm['display_max_age_s'])
    hist = OdomHistory(prm['odom_history_s'])
    route = world.demo_route()
    gx, gy = per.grid.centres()
    gx, gy = gx.ravel(), gy.ravel()

    t, s = 0.0, 0.0                      # sim time, true distance along route
    ox, oy, oa = route[0]                # odom pose (drifts), starts at the true pose
    last_true = route[0]
    dt_odom = 1.0 / a.odom_rate
    next_cam = 0.0
    pending = []                         # (capture time, frames) waiting for `latency`
    n, paused = 0, False
    out = sb.out_dir('local_memory')
    win = 'local_memory (block 04)'
    while True:
        if paused:
            if cv2.waitKey(30) & 0xFF == ord(' '):
                paused = False
            continue
        # ---- odom tick
        t += dt_odom
        s += a.speed * dt_odom
        i = int(s / 0.01)
        if i >= len(route) - 1:
            break
        true = route[i]
        d = math.hypot(true[0] - last_true[0], true[1] - last_true[1]) * (1 + a.drift_pct / 100.0)
        da = math.atan2(math.sin(true[2] - last_true[2]), math.cos(true[2] - last_true[2]))
        oa += da + math.radians(a.yaw_drift_deg_s) * dt_odom
        ox += d * math.cos(oa)
        oy += d * math.sin(oa)
        last_true = true
        hist.add(t, ox, oy, oa)
        # ---- camera capture (rendered at the TRUE pose) and delayed processing
        if t >= next_cam:
            next_cam += 1.0 / cam_rate
            pending.append((t, {r: rig[r].render(world, true) for r in sb.ROLES}))
        while pending and t - pending[0][0] >= a.latency:
            stamp, frames = pending.pop(0)
            res, _, _ = per.process(frames)
            pose = hist.at(stamp, prm['max_stamp_gap_s'])
            if pose is None:
                print('[04] skipped grid: no odom at its stamp')
                continue
            mem.integrate(res.kind.ravel(), gx, gy, pose, stamp)
            n += 1
            _, cur = hist.latest()
            x0, y0, nn, kind, age, dist = mem.window(cur.x, cur.y, prm['window_m'] / 2, t,
                                                     prm['display_max_age_s'])
            travel = np.where(age >= 0, cur.distance - dist, -1.0)
            left, right = render_window(kind, age, travel, prm['drive_max_age_s'], prm,
                                        pose=(cur.x, cur.y, cur.a), x0=x0, y0=y0, res=mem.res)
            view = np.hstack([sb.label(left, f'memory window {prm["window_m"]} m (age shaded)'),
                              sb.label(right, f'usable for driving (<{prm["drive_max_age_s"]} s, '
                                              f'unc <= {prm["uncertainty_limit_m"]} m)')])
            err = math.hypot(cur.x - true[0], cur.y - true[1])
            txt = f't {t:5.1f}s  cells {mem.size(t)}  fresh {mem.fresh}  odom error {err * 100:.1f} cm'
            cv2.putText(view, txt, (6, view.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            key = sb.show(win, view, a.headless, f'{out}/frame_{n:04d}.png' if a.headless else None)
            if a.headless:
                print('[04]', txt)
            if key == ord('q') or (a.frames and n >= a.frames):
                cv2.destroyAllWindows()
                return
            if key == ord(' '):
                paused = True
    print('[04] end of route')
    if not a.headless:
        cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
