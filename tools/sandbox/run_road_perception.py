#!/usr/bin/env python3
"""Block 03 road_perception without ROS.

Same algorithm and the same YAML values as the node (road_mask.py,
camera_model.py, perception.yaml road_perception section, cameras.yaml).

Sources
  --source synth   (default) virtual cameras on the competition track
  --source images  --images front=a.png left_rear=b.png right_rear=c.png
  --source video   --video front=a.mp4 left_rear=b.mp4 right_rear=c.mp4
                   (e.g. recordings from the car; any role may be left out)

Keys in the window
  q quit   space pause   o overlay on/off   s save snapshot to tools/sandbox/out/
  Trackbars: the three classify thresholds (live, like calibration step 9)

Examples
  python tools/sandbox/run_road_perception.py
  python tools/sandbox/run_road_perception.py --drive
  python tools/sandbox/run_road_perception.py --lens fisheye --mismatch
      (true lenses are fisheye but perception assumes the uncalibrated
       pinhole -> shows what skipping calibration step 3 does)
"""
import argparse
import math
import time

import cv2
import numpy as np
import sandbox_common as sb
from carbot_perception.camera_model import camera_setup, intrinsics_for
from carbot_perception.road_mask import (BodyBox, Classify, GridSpec, OverlayLookup, RoadMask,
                                         Seed, bev_view, build_camera_map, mask_colours,
                                         warped_strip)
from carbot_perception.ros_image import resize_to_width


class Perception:
    """The node's _process() without ROS: resize, build maps once, run RoadMask."""

    def __init__(self, cams, prm, veh, intr_override=None):
        self.cams, self.prm = cams, prm
        g = prm['grid']
        self.grid = GridSpec(g['cells'], g['resolution_m'], g['origin_x_m'], g['origin_y_m'])
        s = prm['seed']
        self.mask = RoadMask(self.grid, Seed(s['x_min_m'], s['x_max_m'], s['half_width_m']))
        self.body = BodyBox(veh['rear_overhang_m'], veh['car_length_m'] - veh['rear_overhang_m'],
                            veh['car_width_m'] / 2) if prm['stitch']['exclude_body'] else None
        self.override = intr_override or {}
        self.overlays = {}
        self.cls = Classify(**{k: float(v) for k, v in prm['classify'].items()})

    def _map(self, role, w, h):
        m = self.mask.maps.get(role)
        if m is not None and (m.width, m.height) == (w, h):
            return m
        intr = self.override[role].scaled(w, h) if role in self.override else \
            intrinsics_for(self.cams, role, w, h)
        _, _, mount, _ = camera_setup(self.cams, role)
        st = self.prm['stitch']
        m = build_camera_map(role, self.grid, intr, mount, self.body, st['min_depth_m'],
                             st['weight_eps'], st['border_px'])
        self.mask.maps[role] = m
        print(f'[03] {role}: {w}x{h} {intr.model} ({intr.source}) hfov {intr.hfov_deg():.1f} deg, '
              f'sees {int(m.valid.sum())} grid cells')
        return m

    def process(self, frames):
        small = {}
        for role, img in frames.items():
            if img is None:
                continue
            s = resize_to_width(img, int(self.prm['input_downscale_width']))
            self._map(role, s.shape[1], s.shape[0])
            small[role] = s
        t0 = time.perf_counter()
        res = self.mask.process(small, self.cls)
        return res, small, (time.perf_counter() - t0) * 1000.0

    def overlay(self, role, img, res, width):
        lk = self.overlays.get(role)
        m = self.mask.maps[role]
        if lk is None:
            lk = self.overlays[role] = OverlayLookup(self.grid, m.intr, m.mount, width)
        return lk.draw(img, res)


def sources(a, cams):
    """Generator of (frames dict, pose or None)."""
    if a.source == 'images':
        pairs = dict(kv.split('=', 1) for kv in a.images)
        frames = {r: cv2.imread(p) for r, p in pairs.items()}
        while True:
            yield frames, None
    if a.source == 'video':
        caps = {r: cv2.VideoCapture(p) for r, p in (kv.split('=', 1) for kv in a.video)}
        while True:
            frames = {}
            for r, c in caps.items():
                ok, f = c.read()
                if not ok:
                    c.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, f = c.read()
                frames[r] = f if ok else None
            yield frames, None
    world = sb.TrackWorld()
    rig = sb.camera_rig(cams, a.render_width, a.lens)
    sources.rig, sources.world = rig, world
    route = world.demo_route() if a.drive else None
    pose = tuple(a.pose) if a.pose else world.start_pose()
    i = 0.0
    while True:
        if route:
            p = route[int(i) % len(route)]
            pose = (p[0] + a.offset * -math.sin(p[2]), p[1] + a.offset * math.cos(p[2]), p[2] + math.radians(a.yaw_error))
            i += a.speed / 0.01 / a.rate
        yield {r: rig[r].render(world, pose) for r in sb.ROLES}, pose


def minimap(world, pose, scale=0.12):
    img = world.top_view(scale)
    if pose is not None:
        x, y, a = pose
        h = img.shape[0]
        p = (int(x / world.res * scale), int(h - y / world.res * scale))
        q = (int(p[0] + 12 * math.cos(a)), int(p[1] - 12 * math.sin(a)))
        cv2.arrowedLine(img, p, q, (0, 0, 255), 2, tipLength=0.4)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', choices=['synth', 'images', 'video'], default='synth')
    ap.add_argument('--images', nargs='*', default=[])
    ap.add_argument('--video', nargs='*', default=[])
    ap.add_argument('--cameras', default='', help='cameras.yaml (default repo; or a session data/cameras.yaml)')
    ap.add_argument('--lens', choices=['auto', 'pinhole', 'fisheye'], default='auto',
                    help='synth: lens of the virtual cameras')
    ap.add_argument('--mismatch', action='store_true',
                    help='synth: perception uses cameras.yaml intrinsics instead of the true virtual lens')
    ap.add_argument('--render-width', type=int, default=480)
    ap.add_argument('--pose', nargs=3, type=float, metavar=('X', 'Y', 'A'), help='synth: track pose')
    ap.add_argument('--drive', action='store_true', help='synth: drive the demo route')
    ap.add_argument('--speed', type=float, default=0.18, help='m/s (V4 max 0.18)')
    ap.add_argument('--offset', type=float, default=0.0, help='synth: lateral offset from the centreline (m)')
    ap.add_argument('--yaw-error', type=float, default=0.0, help='synth: heading error (deg)')
    ap.add_argument('--rate', type=float, default=0.0, help='Hz (default perception.yaml process_rate_hz)')
    ap.add_argument('--headless', action='store_true', help='no window; save frames instead')
    ap.add_argument('--frames', type=int, default=0, help='stop after N frames (0 = run until q)')
    a = ap.parse_args()

    prm = sb.params('road_perception')
    veh = sb.common()['vehicle']
    cams = sb.cameras(a.cameras)
    a.rate = a.rate or float(prm['process_rate_hz'])
    gen = sources(a, cams)
    frames, pose = next(gen)
    override = None
    if a.source == 'synth' and not a.mismatch:
        override = {r: c.intr for r, c in sources.rig.items()}
    per = Perception(cams, prm, veh, override)
    win = 'road_perception (block 03)'
    if not a.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        for key, top in (('road_max_luma', 255), ('road_max_chroma', 255), ('paint_min_luma', 255)):
            cv2.createTrackbar(key, win, int(prm['classify'][key]), top, lambda v: None)
    overlay, paused, n = True, False, 0
    out = sb.out_dir('road_perception')
    while True:
        if not paused:
            frames, pose = next(gen)
        if not a.headless:
            per.cls = Classify(*(float(cv2.getTrackbarPos(k, win)) for k in
                                 ('road_max_luma', 'road_max_chroma', 'paint_min_luma')))
        res, small, ms = per.process(frames)
        cams_row = []
        for r in sb.ROLES:
            if r in small:
                im = per.overlay(r, small[r], res, int(prm['debug']['overlay_width'])) if overlay else small[r]
                cams_row.append(sb.label(im, f'{r}' + (' + mask' if overlay else '')))
        scale = int(prm['debug']['bev_scale'])
        bev_row = [sb.label(warped_strip(res, sb.ROLES, 2), 'warped: front | left | right'),
                   sb.label(bev_view(res.bgr, scale), 'stitched (fwd up)'),
                   sb.label(bev_view(mask_colours(res), scale),
                            f'mask cov {res.coverage:.2f} road {res.connected}')]
        view = np.vstack([sb.tile(cams_row, 3, 360), sb.tile(bev_row, 3, 360)])
        if a.source == 'synth':
            mm = minimap(sources.world, pose)
            view[-mm.shape[0]:, -mm.shape[1]:] = mm
        text = f'proc {ms:.1f} ms  coverage {res.coverage:.2f}  connected {res.connected}'
        cv2.putText(view, text, (6, view.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        n += 1
        save = f'{out}/frame_{n:04d}.png' if a.headless else None
        key = sb.show(win, view, a.headless, save)
        if a.headless:
            print(f'[03] frame {n}: {text}')
        if key == ord('q') or (a.frames and n >= a.frames):
            break
        if key == ord('o'):
            overlay = not overlay
        if key == ord(' '):
            paused = not paused
        if key == ord('s'):
            p = f'{out}/snapshot_{int(time.time())}.png'
            cv2.imwrite(p, view)
            print('saved', p)
        if not a.headless:
            time.sleep(max(0.0, 1.0 / a.rate - ms / 1000.0))
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
