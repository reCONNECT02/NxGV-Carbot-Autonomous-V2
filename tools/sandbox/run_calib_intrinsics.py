#!/usr/bin/env python3
"""Calibration step 3 (calib_intrinsics) without ROS.

Runs the real calib_intrinsics code; results go to a sandbox session in
tools/sandbox/out/carbot_data/calibration/<session>/ (never the car's data).

Sources
  --synth pinhole|fisheye   render views of the step-3 board through a known
                            virtual lens, calibrate, compare with the truth
  --webcam 0                live auto-capture with your laptop camera (show the
                            printed board, or the PDF on a second screen)
  --video file.mp4          same auto-capture from a recording
  --images DIR              calibrate from saved images

Window (webcam/video): corners drawn when the board is found, 3x3 grid of
the image regions already covered, status text. Hold the board still in a
new place to capture; q stops early and calibrates what was collected.

  python tools/sandbox/run_calib_intrinsics.py --synth fisheye
  python tools/sandbox/run_calib_intrinsics.py --webcam 0
"""
import argparse
import os
import shutil
import sys

import cv2
import numpy as np
import sandbox_common as sb
import yaml
from carbot_perception import calib_intrinsics as ci
from carbot_perception.calib_core import board_points
from carbot_perception.camera_model import load_intrinsics


def step3():
    steps = sb.load_yaml(os.path.join(sb.CONFIG, 'data', 'calibration_steps.yaml'))['steps']
    return [s for s in steps if s['id'] == 'camera_intrinsics'][0]


def render_board_view(intr, rvec, tvec, cols, rows, sq, ss=2):
    """Image of a chessboard (white margin) at pose (rvec, tvec) seen through intr."""
    big = intr.scaled(intr.width * ss, intr.height * ss)
    u, v = np.meshgrid(np.arange(big.width, dtype=float), np.arange(big.height, dtype=float))
    rays = big.rays(np.column_stack([u.ravel(), v.ravel()]))
    R, _ = cv2.Rodrigues(rvec)
    n = R[:, 2]                                   # board normal in camera frame
    denom = rays @ n
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (tvec @ n) / denom
    p = rays * s[:, None] - tvec                  # camera -> board frame
    b = p @ R                                     # board coordinates
    img = np.full(u.size, 150, np.uint8)
    hit = np.isfinite(s) & (s > 0)
    bx, by = b[:, 0], b[:, 1]
    w, h = (cols + 1) * sq, (rows + 1) * sq
    paper = hit & (bx > -sq * 1.6) & (bx < w - sq * 0.4) & (by > -sq * 1.6) & (by < h - sq * 0.4)
    img[paper] = 235
    inside = hit & (bx > -sq) & (bx < w - sq) & (by > -sq) & (by < h - sq)
    ix = np.floor((bx + sq) / sq).astype(int)
    iy = np.floor((by + sq) / sq).astype(int)
    img[inside & ((ix + iy) % 2 == 0)] = 20
    img = img.reshape(big.height, big.width)
    img = cv2.resize(img, (intr.width, intr.height), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def synth_views(kind, folder, cols, rows, sq, n=28, seed=1):
    truth = sb.synth_lens(kind, 960, 544, 110.0 if kind == 'fisheye' else 70.0)
    rng = np.random.default_rng(seed)
    obj = board_points(cols, rows, sq).astype(float)
    centre = obj.mean(axis=0)
    k = 0
    while k < n:
        rvec = np.array([rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.4)])
        R, _ = cv2.Rodrigues(rvec)
        target = np.array([[rng.uniform(0.12, 0.88) * truth.width, rng.uniform(0.12, 0.88) * truth.height]])
        tvec = truth.rays(target)[0] * rng.uniform(0.18, 0.4) - R @ centre
        uv, ok = truth.project(obj @ R.T + tvec)
        if not ok.all() or uv.min() < 8 or (uv[:, 0] > truth.width - 8).any() or (uv[:, 1] > truth.height - 8).any():
            continue
        cv2.imwrite(os.path.join(folder, f'synth_{k:02d}.png'), render_board_view(truth, rvec, tvec, cols, rows, sq))
        k += 1
    return truth


def live(a, cols, rows, folder, target, min_cells):
    cap = cv2.VideoCapture(int(a.webcam) if a.webcam != '' else a.video)
    if not cap.isOpened():
        sys.exit('cannot open the camera / video')
    win = 'calib_intrinsics (step 3)'

    def next_frame():
        ok, f = cap.read()
        return f if ok else None

    def on_frame(img, corners, state, col):
        if a.headless:
            return True
        v = img.copy()
        if corners is not None:
            cv2.drawChessboardCorners(v, (cols, rows), corners.reshape(-1, 1, 2), True)
        h, w = v.shape[:2]
        for i in range(3):
            for j in range(3):
                if col.coverage[i, j]:
                    cv2.rectangle(v, (j * w // 3 + 2, i * h // 3 + 2), ((j + 1) * w // 3 - 2, (i + 1) * h // 3 - 2),
                                  (0, 200, 0), 2)
        cv2.putText(v, f'{state}  views {len(col.views)}/{target}  coverage {col.coverage_cells()}/{min_cells}',
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(win, v)
        return (cv2.waitKey(1) & 0xFF) != ord('q')

    try:
        ci.collect_views(next_frame, cols, rows, target, min_cells, folder, a.timeout, 0.12, 1.5,
                         label=str(a.webcam or a.video), on_frame=on_frame)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--synth', choices=['pinhole', 'fisheye'])
    src.add_argument('--webcam', default='')
    src.add_argument('--video', default='')
    src.add_argument('--images', default='')
    ap.add_argument('--sensor', default='astra', help='name the result is saved under (sandbox only)')
    ap.add_argument('--session', default='sandbox')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--headless', action='store_true')
    a = ap.parse_args()

    st = step3()
    cols, rows = st['target']['inner_corners']
    sq = float(st['target']['square_m'])
    target = int(st['pass']['min_views']) + 10
    min_cells = int(st['pass'].get('min_coverage_cells', 9))
    root = sb.out_dir('carbot_data')
    folder = a.images or sb.out_dir('calib_intrinsics', 'views')
    truth = None
    if not a.images:
        shutil.rmtree(folder, ignore_errors=True)
        os.makedirs(folder)
    if a.synth:
        print(f'[step3] rendering views through a virtual {a.synth} lens ...')
        truth = synth_views(a.synth, folder, cols, rows, sq)
    elif not a.images:
        live(a, cols, rows, folder, target, min_cells)

    rc = ci.main(['--sensor', a.sensor, '--images', folder, '--session', a.session,
                  '--data-root', root, '--config-dir', sb.CONFIG])
    res_file = os.path.join(root, 'calibration', a.session, 'intrinsics', f'{a.sensor}.yaml')
    if os.path.isfile(res_file):
        got = load_intrinsics(res_file)
        print(f'\n[step3] result file: {res_file}')
        print(yaml.safe_dump({'model': got.model, 'fx': float(got.K[0, 0]), 'fy': float(got.K[1, 1]),
                              'cx': float(got.K[0, 2]), 'cy': float(got.K[1, 2]),
                              'D': [float(x) for x in got.D], 'hfov_deg': round(got.hfov_deg(), 2)},
                             sort_keys=False))
        if truth is not None:
            print(f'[step3] TRUTH: model {truth.model} fx {truth.K[0, 0]:.1f} cx {truth.K[0, 2]:.1f} '
                  f'cy {truth.K[1, 2]:.1f} hfov {truth.hfov_deg():.1f} deg')
            err = abs(got.K[0, 0] - truth.K[0, 0]) / truth.K[0, 0] * 100
            print(f'[step3] focal length error {err:.2f} %  ({"OK" if err < 2 else "CHECK"})')
        if not a.headless and a.synth is None:
            # undistorted preview of the last view: straight edges must be straight
            imgs = sorted(f for f in os.listdir(folder) if f.startswith(('view', 'synth')))
            if imgs:
                img = cv2.imread(os.path.join(folder, imgs[-1]))
                und = undistort(got, img)
                cv2.imshow('step 3: original | undistorted', np.hstack([img, und]))
                cv2.waitKey(0)
    return rc


def undistort(intr, img):
    K = intr.K
    if intr.fisheye:
        m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, intr.D.reshape(4, 1), np.eye(3), K,
                                                     (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    else:
        m1, m2 = cv2.initUndistortRectifyMap(K, intr.D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)


if __name__ == '__main__':
    sys.exit(main())
