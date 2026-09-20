#!/usr/bin/env python3
"""camera_preview without ROS: what the GUI (preview) and the rosbag (record)
streams look like and cost, using the perception.yaml camera_preview values.

Sources (any role may be left out):
  --source synth                       virtual cameras on the track (default)
  --source webcam --webcam front=0     laptop / USB cameras by index
  --source video  --video front=a.mp4 left_rear=b.mp4

Shows each role's preview after the real JPEG encode -> decode round trip and
prints KB per frame and the bandwidth the GUI and the rosbag would use.
Keys: q quit, p toggle preview/record view.

  python tools/sandbox/run_camera_preview.py
  python tools/sandbox/run_camera_preview.py --source webcam --webcam front=0
"""
import argparse
import time

import cv2
import numpy as np
import sandbox_common as sb
from carbot_perception.ros_image import resize_to_width


def open_sources(a, cams):
    if a.source == 'synth':
        world = sb.TrackWorld()
        rig = sb.camera_rig(cams, 960)
        route = world.demo_route()
        state = {'i': 0}

        def grab():
            p = route[state['i'] % len(route)]
            state['i'] += 3
            return {r: rig[r].render(world, p) for r in sb.ROLES}
        return grab
    pairs = dict(kv.split('=', 1) for kv in (a.webcam if a.source == 'webcam' else a.video))
    caps = {r: cv2.VideoCapture(int(v) if a.source == 'webcam' else v) for r, v in pairs.items()}

    def grab():
        out = {}
        for r, c in caps.items():
            ok, f = c.read()
            if not ok and a.source == 'video':
                c.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, f = c.read()
            out[r] = f if ok else None
        return out
    return grab


def roundtrip(img, width, q):
    small = resize_to_width(img, width)
    ok, enc = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), int(q)])
    return cv2.imdecode(enc, cv2.IMREAD_COLOR), len(enc)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', choices=['synth', 'webcam', 'video'], default='synth')
    ap.add_argument('--webcam', nargs='*', default=[])
    ap.add_argument('--video', nargs='*', default=[])
    ap.add_argument('--cameras', default='')
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--frames', type=int, default=0)
    a = ap.parse_args()
    prm = sb.params('camera_preview')
    grab = open_sources(a, sb.cameras(a.cameras))
    mode, n, misses = 'preview', 0, 0
    sizes = {'preview': [], 'record': []}
    out = sb.out_dir('camera_preview')
    while True:
        t0 = time.monotonic()
        frames = grab()
        w = int(prm[f'{mode}_width'])
        q = int(prm[f'{mode}_jpeg_quality'])
        fps = float(prm[f'{mode}_max_fps'])
        tiles = []
        for r in sb.ROLES:
            f = frames.get(r)
            if f is None:
                continue
            img, nbytes = roundtrip(f, w, q)
            sizes[mode].append(nbytes)
            tiles.append(sb.label(img, f'{r} {mode} {img.shape[1]}x{img.shape[0]} q{q} {nbytes / 1024:.1f} KB'))
        if not tiles:
            misses += 1
            print('no frames (check the webcam index / video path)')
            if misses >= 6:
                break
            time.sleep(0.5)
            continue
        misses = 0
        view = sb.tile(tiles, 3, w)          # Astra is 4:3, the MIPI cameras 16:9
        kb = np.mean(sizes[mode][-30:]) / 1024
        txt = f'{mode}: {len(tiles)} cams x {kb:.1f} KB x {fps:g} fps = {len(tiles) * kb * fps:.0f} KB/s'
        view = cv2.copyMakeBorder(view, 0, 24, 0, 0, cv2.BORDER_CONSTANT)
        cv2.putText(view, txt, (6, view.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        n += 1
        key = sb.show('camera_preview', view, a.headless, f'{out}/{mode}_{n:04d}.jpg' if a.headless else None)
        if a.headless:
            print('[preview]', txt)
        if key == ord('q') or (a.frames and n >= a.frames):
            break
        if key == ord('p'):
            mode = 'record' if mode == 'preview' else 'preview'
        time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - t0)))
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
