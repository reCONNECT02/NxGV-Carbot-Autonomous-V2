"""Calibration step 3 - camera intrinsics, one sensor per run.

    ros2 run carbot_perception calib_intrinsics --sensor ov5647 --session 20260921_1400
    ros2 run carbot_perception calib_intrinsics --sensor astra  --session 20260921_1400
    ros2 run carbot_perception calib_intrinsics --sensor ov5647 --images DIR   (offline)

Hold the step-3 chessboard (calibration_steps.yaml: 9x6 inner corners, 25 mm)
in front of the camera. A view is captured automatically when the board is
held STILL and is in a new position/size/tilt. Move it into every part of the
image, especially the corners and edges: lens distortion is only measured
where the board has been. Progress is printed and the latest annotated frame
is written to <session>/captures/intrinsics/<sensor>/live.jpg.

Both the standard and the fisheye lens model are fitted; the result keeps the
better one. Calibrates at the resolution the camera publishes (960x544 for the
MIPI cameras) - block 03 scales it to its processing size.
"""
import argparse
import datetime
import glob
import os
import sys
import time

import cv2
import numpy as np
from carbot_common import calibration_store as cs

from .calib_core import ViewCollector, calibrate_intrinsics, detect_chessboard
from .calib_io import (FrameGrabber, bringup_config_dir, common_args, draw_corners,
                       effective_cameras, merge_result, save_cameras, step_cfg)
from .camera_model import save_intrinsics

STEP = 'camera_intrinsics'
RESULT = '03_camera_intrinsics.yaml'


def collect_views(next_frame, cols, rows, target_views, min_cells, out_dir, timeout, novelty,
                  still_px, label='camera', on_frame=None):
    """Auto-capture loop, independent of where frames come from.

    next_frame() -> BGR image or None (timeout). on_frame(img, corners, state,
    collector) is called for every frame (the VS Code sandbox shows a window);
    returning False stops early."""
    col = None
    prev = None
    t_end = time.monotonic() + timeout
    last_print = 0.0
    while time.monotonic() < t_end:
        img = next_frame()
        if img is None:
            print(f'[calib] no frames from {label} (is the camera running?)')
            continue
        if col is None:
            col = ViewCollector(img.shape[1], img.shape[0], novelty)
            print(f'[calib] {label}: {img.shape[1]}x{img.shape[0]}')
        c = detect_chessboard(img, cols, rows, fast=True)
        still = c is not None and prev is not None and \
            float(np.mean(np.linalg.norm(c - prev, axis=1))) < still_px
        prev = c
        added = still and col.offer(c, cols, rows)
        if added:
            cv2.imwrite(os.path.join(out_dir, f'view_{len(col.views):02d}.png'), img)
        state = 'NEW VIEW' if added else ('board: hold still' if c is not None and not still
                                           else 'board seen' if c is not None else 'no board')
        if on_frame is not None and on_frame(img, c, state, col) is False:
            break
        now = time.monotonic()
        if added or now - last_print > 2.0:
            last_print = now
            cv2.imwrite(os.path.join(out_dir, 'live.jpg'), draw_corners(img, c, cols, rows, c is not None))
            print(f'[calib] views {len(col.views)}/{target_views}  coverage '
                  f'{col.coverage_cells()}/9  ({state})\n{col.coverage_text()}')
        if len(col.views) >= target_views and col.coverage_cells() >= min_cells:
            break
    return col


def collect_live(topic, cols, rows, target_views, min_cells, out_dir, timeout, novelty, still_px):
    grab = FrameGrabber({'cam': topic})
    try:
        return collect_views(lambda: grab.next('cam', 2.0), cols, rows, target_views, min_cells,
                             out_dir, timeout, novelty, still_px, label=topic)
    finally:
        grab.close()


def collect_files(folder, cols, rows, novelty):
    files = sorted(glob.glob(os.path.join(folder, '*.png')) + glob.glob(os.path.join(folder, '*.jpg')))
    files = [f for f in files if not os.path.basename(f).startswith('live')]
    col = None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        if col is None:
            col = ViewCollector(img.shape[1], img.shape[0], novelty)
        c = detect_chessboard(img, cols, rows)
        ok = c is not None and col.offer(c, cols, rows)
        print(f'[calib] {os.path.basename(f)}: {"used" if ok else "board not found" if c is None else "duplicate"}')
    return col


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sensor', required=True, help='sensor name in cameras.yaml: astra | ov5647 | imx219')
    ap.add_argument('--images', default='', help='calibrate from saved images instead of live capture')
    ap.add_argument('--views', type=int, default=0, help='views to collect (default: pass.min_views + 10)')
    ap.add_argument('--timeout', type=float, default=300.0, help='live capture time limit (s)')
    ap.add_argument('--novelty', type=float, default=0.12, help='how different a new view must be')
    ap.add_argument('--still-px', type=float, default=1.5, help='max corner motion between frames')
    common_args(ap)
    a = ap.parse_args(argv)

    root = cs.data_root(a.data_root)
    cfg_dir = bringup_config_dir(a.config_dir)
    step = step_cfg(cfg_dir, STEP)
    cols, rows = step['target']['inner_corners']
    square = float(step['target']['square_m'])
    pas = step['pass']
    session = cs.open_session(root, a.session)
    cameras = effective_cameras(session, cfg_dir, root)
    if a.sensor not in cameras['sensors']:
        sys.exit(f'unknown sensor {a.sensor}; cameras.yaml has {list(cameras["sensors"])}')
    sensor = cameras['sensors'][a.sensor]
    out_dir = os.path.join(session, 'captures', 'intrinsics', a.sensor)
    os.makedirs(out_dir, exist_ok=True)
    target = a.views or int(pas['min_views']) + 10
    min_cells = int(pas.get('min_coverage_cells', 9))
    print(f'[calib] session {session}\n[calib] board {cols}x{rows} inner corners, {square * 1000:.0f} mm squares')

    if a.images:
        col = collect_files(a.images, cols, rows, a.novelty)
    else:
        col = collect_live(sensor['image_topic'], cols, rows, target, min_cells, out_dir,
                           a.timeout, a.novelty, a.still_px)
    n = 0 if col is None else len(col.views)
    if n < 6:
        sys.exit(f'[calib] only {n} usable views - nothing calibrated')

    out = calibrate_intrinsics(col.views, cols, rows, square, (col.width, col.height))
    fit = out['chosen']
    intr = fit.intr
    intr.meta.update({'sensor': a.sensor, 'date': datetime.datetime.now().isoformat(timespec='seconds'),
                      'coverage_cells': col.coverage_cells()})
    checks = {'views': len(out['used_views']) >= int(pas['min_views']),
              'reprojection': fit.rms_px <= float(pas['max_reprojection_px']),
              'coverage': col.coverage_cells() >= min_cells}
    status = 'PASS' if all(checks.values()) else 'FAIL'

    ipath = os.path.join(session, 'intrinsics', f'{a.sensor}.yaml')
    save_intrinsics(ipath, intr, a.sensor)
    if status == 'PASS':
        cameras['sensors'][a.sensor]['intrinsics_file'] = os.path.abspath(ipath)
        save_cameras(session, cameras)
    result = {'status': status, 'checks': checks, 'model': intr.model,
              'rms_px': round(fit.rms_px, 4),
              'pinhole_rms_px': round(out['pinhole'].rms_px, 4),
              'fisheye_rms_px': None if out['fisheye'] is None else round(out['fisheye'].rms_px, 4),
              'views_used': len(out['used_views']), 'coverage_cells': col.coverage_cells(),
              'image_size': [col.width, col.height], 'hfov_deg': round(intr.hfov_deg(), 2),
              'fx': round(float(intr.K[0, 0]), 2), 'fy': round(float(intr.K[1, 1]), 2),
              'cx': round(float(intr.K[0, 2]), 2), 'cy': round(float(intr.K[1, 2]), 2),
              'file': os.path.abspath(ipath), 'captures': out_dir}
    doc = merge_result(session, RESULT, 'sensors', {a.sensor: result})
    per = step.get('per_sensor', [a.sensor])
    done = doc.get('sensors', {})
    all_pass = all((done.get(s) or {}).get('status') == 'PASS' for s in per)
    missing = [s for s in per if s not in done]
    cs.update_step(session, STEP, 'PASS' if all_pass else ('INCOMPLETE' if missing else 'FAIL'), RESULT,
                   sensors={s: (done.get(s) or {}).get('status', 'TODO') for s in per})
    if a.activate:
        cs.set_active(root, session)

    fish_txt = 'n/a' if out['fisheye'] is None else f"{out['fisheye'].rms_px:.3f}"
    print(f'\n[calib] {a.sensor}: {status}  model={intr.model}  rms={fit.rms_px:.3f}px '
          f'(pinhole {out["pinhole"].rms_px:.3f}, fisheye {fish_txt})  hfov={intr.hfov_deg():.1f}deg  '
          f'views={len(out["used_views"])}  coverage={col.coverage_cells()}/9')
    for k, v in checks.items():
        print(f'        {k:13s} {"ok" if v else "FAILED"}')
    state = 'all sensors PASS' if all_pass else (f'still to do: {missing}' if missing else 'a sensor FAILED')
    print(f'[calib] wrote {ipath}\n[calib] step {STEP}: {state}')
    return 0 if status == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
