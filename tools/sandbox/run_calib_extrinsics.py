#!/usr/bin/env python3
"""Calibration step 4 (calib_extrinsics) without ROS.

Runs the real calib_extrinsics code in a sandbox session
(tools/sandbox/out/carbot_data/calibration/<session>/).

  --synth      render the three floor boards (calibration_steps.yaml step 4)
               through virtual cameras whose TRUE mounts are the cameras.yaml
               mounts plus a random error (--perturb-cm, --perturb-deg), then
               check that the tool finds the true mounts
  --images front=f.png
               real photos from the car; the session must already hold the
               intrinsics (run run_calib_intrinsics.py or copy intrinsics/*.yaml
               and data/cameras.yaml from the car's session)

Shows each camera's detection and the IPM check (red = true board outlines).

  python tools/sandbox/run_calib_extrinsics.py --synth
  python tools/sandbox/run_calib_extrinsics.py --synth --lens fisheye --perturb-deg 4
"""
import argparse
import copy
import math
import os
import sys

import cv2
import numpy as np
import sandbox_common as sb
import yaml
from carbot_perception import calib_extrinsics as ce
from carbot_perception.calib_core import FloorBoard
from carbot_perception.camera_model import Mount, camera_setup, save_intrinsics


class FloorWorld:
    """Grey floor with the step-4 boards (car at the origin)."""

    def __init__(self, boards):
        self.boards = boards

    def sample(self, x, y):
        out = np.full(x.shape + (3,), 95, np.uint8)
        for b in self.boards:
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            u = (x - b.cx) * c + (y - b.cy) * s
            v = -(x - b.cx) * s + (y - b.cy) * c
            hu, hv = (b.cols + 1) * b.square / 2, (b.rows + 1) * b.square / 2
            out[(np.abs(u) < hu + 0.035) & (np.abs(v) < hv + 0.023)] = 240     # A3 paper margin
            inside = (np.abs(u) < hu) & (np.abs(v) < hv)
            iu = np.floor((u + hu) / b.square).astype(int)
            iv = np.floor((v + hv) / b.square).astype(int)
            out[inside & ((iu + iv) % 2 == 0)] = 18
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--synth', action='store_true')
    src.add_argument('--images', nargs='+')
    ap.add_argument('--lens', choices=['pinhole', 'fisheye'], default='pinhole', help='synth: lens type')
    ap.add_argument('--perturb-cm', type=float, default=1.0, help='synth: true mount position error')
    ap.add_argument('--perturb-deg', type=float, default=2.0, help='synth: true mount angle error')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--session', default='sandbox')
    ap.add_argument('--headless', action='store_true')
    a = ap.parse_args()

    root = sb.out_dir('carbot_data')
    session = os.path.join(root, 'calibration', a.session)
    steps = sb.load_yaml(os.path.join(sb.CONFIG, 'data', 'calibration_steps.yaml'))['steps']
    target = [s for s in steps if s['id'] == 'extrinsics_ipm'][0]['target']
    boards = [FloorBoard.from_yaml(b) for b in target['boards']]
    truth = {}
    if a.synth:
        rng = np.random.default_rng(a.seed)
        cams = copy.deepcopy(sb.cameras())
        world = FloorWorld(boards)
        args = []
        img_dir = sb.out_dir('calib_extrinsics', 'synth')
        for role in sb.ROLES:
            sensor, s, nominal, hfov = camera_setup(cams, role)
            w = int(s.get('image_width', s.get('width')))
            h = int(s.get('image_height', s.get('height')))
            intr = sb.synth_lens('pinhole' if role == 'front' else a.lens, w, h, hfov)
            intr.source = 'calibrated'
            path = os.path.join(session, 'intrinsics', f'{sensor}.yaml')
            save_intrinsics(path, intr, sensor)
            cams['sensors'][sensor]['intrinsics_file'] = path
            d = rng.normal(0, a.perturb_cm / 100 / math.sqrt(3), 3)
            r = np.radians(rng.normal(0, a.perturb_deg / math.sqrt(3), 3))
            t = Mount(nominal.x + d[0], nominal.y + d[1], nominal.z + d[2],
                      nominal.yaw + r[0], nominal.pitch_down + r[1], nominal.roll + r[2])
            truth[role] = t
            big = sb.VirtualCamera.build(role, intr.scaled(w * 2, h * 2), t)
            img = cv2.resize(big.render(world, (0.0, 0.0, 0.0)), (w, h), interpolation=cv2.INTER_AREA)
            p = os.path.join(img_dir, f'{role}.png')
            cv2.imwrite(p, img)
            args.append(f'{role}={p}')
        os.makedirs(os.path.join(session, 'data'), exist_ok=True)
        with open(os.path.join(session, 'data', 'cameras.yaml'), 'w', encoding='utf-8') as f:
            yaml.safe_dump(cams, f, sort_keys=False)
    else:
        args = a.images
        if not os.path.isfile(os.path.join(session, 'data', 'cameras.yaml')):
            print(f'[step4] WARNING: {session}/data/cameras.yaml missing: intrinsics will be ASSUMED')

    rc = ce.main(['--session', a.session, '--data-root', root, '--config-dir', sb.CONFIG, '--images'] + args)

    if truth:
        got = sb.load_yaml(os.path.join(session, 'data', 'cameras.yaml'))['mounts']
        print('\n[step4] recovered vs TRUE mount:')
        for role, t in truth.items():
            dpos, dang = Mount.from_yaml(got[role]).difference(t)
            print(f'  {role:10s} position error {dpos * 1000:5.1f} mm   angle error {dang:4.2f} deg   '
                  f'{"OK" if dpos < 0.01 and dang < 1.0 else "CHECK"}')
    cap = os.path.join(session, 'captures', 'extrinsics')
    views = [sb.label(cv2.imread(os.path.join(cap, f'{r}_detect.jpg')), r) for r in sb.ROLES
             if os.path.isfile(os.path.join(cap, f'{r}_detect.jpg'))]
    ipm = cv2.imread(os.path.join(cap, 'ipm_check.png'))
    if views and not a.headless:
        cv2.imshow('step 4: detections', sb.tile(views, 3, 420))
        if ipm is not None:
            cv2.imshow('step 4: IPM check (red = true boards)', ipm)
        print('[step4] press any key in a window to close')
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return rc


if __name__ == '__main__':
    sys.exit(main())
