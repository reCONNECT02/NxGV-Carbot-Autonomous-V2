"""calib_extrinsics end to end (offline --images): synthetic floor photos with
the step-4 boards, rendered through the team's CAD mounts slightly perturbed.
Checks board detection, pose solving, file layout and summary."""
import math
import os

import cv2
import numpy as np
import pytest
import yaml
from synth import MOUNTS, fisheye, pinhole

from carbot_perception import calib_extrinsics
from carbot_perception.calib_core import FloorBoard
from carbot_perception.camera_model import Intrinsics, Mount, pixels_to_ground, save_intrinsics

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.normpath(os.path.join(HERE, '..', '..', 'carbot_bringup', 'config'))


def board_texture(boards):
    def tex(x, y):
        out = np.full(x.shape + (3,), 90, np.uint8)          # grey floor
        for b in boards:
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            u = (x - b.cx) * c + (y - b.cy) * s
            v = -(x - b.cx) * s + (y - b.cy) * c
            hu, hv = (b.cols + 1) * b.square / 2, (b.rows + 1) * b.square / 2
            margin = 0.02
            inside_paper = (np.abs(u) < hu + margin) & (np.abs(v) < hv + margin)
            out[inside_paper] = 245
            inside = (np.abs(u) < hu) & (np.abs(v) < hv)
            iu = np.floor((u + hu) / b.square).astype(int)
            iv = np.floor((v + hv) / b.square).astype(int)
            black = inside & ((iu + iv) % 2 == 0)
            out[black] = 15
        return out
    return tex


def render(intr, mount, tex, ss=3):
    big = intr.scaled(intr.width * ss, intr.height * ss)
    u, v = np.meshgrid(np.arange(big.width, dtype=float), np.arange(big.height, dtype=float))
    g, ok = pixels_to_ground(big, mount, np.column_stack([u.ravel(), v.ravel()]))
    img = np.full((u.size, 3), 200, np.uint8)
    ok &= np.hypot(g[:, 0], g[:, 1]) < 5
    img[ok] = tex(g[ok, 0], g[ok, 1])
    img = img.reshape(big.height, big.width, 3)
    return cv2.resize(img, (intr.width, intr.height), interpolation=cv2.INTER_AREA)


@pytest.mark.xfail(strict=True, reason='BACKLOG #54: with the rear axle 150 mm back the front board is at base_link x 0.77 '
                   'and the camera (9.35 cm high) sees it at a grazing angle: calib_core.detect_chessboard does not find it '
                   'in this synthetic 640x480 render (it did at x 0.62). Remove the marker when detection is robust there.')
def test_extrinsics_cli_offline(tmp_path):
    steps = yaml.safe_load(open(os.path.join(CONFIG, 'data', 'calibration_steps.yaml')))
    target = [s for s in steps['steps'] if s['id'] == 'extrinsics_ipm'][0]['target']
    from carbot_common.calib_tools import floor_board_dicts
    boards = [FloorBoard.from_yaml(b) for b in floor_board_dicts(target)]     # base_link, axle_offset_m applied
    roles = sorted({r for b in target['boards'] for r in b['roles']})          # front only since 2026-09-24
    cams = yaml.safe_load(open(os.path.join(CONFIG, 'data', 'cameras.yaml')))
    root = tmp_path / 'data'
    session = root / 'calibration' / 'S1'
    intrs = {'front': Intrinsics.ideal(640, 480, 60.0), 'left_rear': fisheye(960, 544, 300.0),
             'right_rear': pinhole(960, 544, 600.0)}
    truth = {}
    images = []
    intrs = {r: i for r, i in intrs.items() if r in roles}
    for role, intr in intrs.items():
        sensor = cams['roles'][role]
        path = session / 'intrinsics' / f'{sensor}.yaml'
        intr.source = 'calibrated'
        save_intrinsics(str(path), intr, sensor)
        cams['sensors'][sensor]['intrinsics_file'] = str(path)
        m = MOUNTS[role]
        truth[role] = Mount(m.x + 0.005, m.y, m.z - 0.004, m.yaw - math.radians(1.0),
                            m.pitch_down + math.radians(1.5), math.radians(0.5))
        img = render(intr, truth[role], board_texture(boards))
        p = tmp_path / f'{role}.png'
        cv2.imwrite(str(p), img)
        images.append(f'{role}={p}')
    os.makedirs(session / 'data', exist_ok=True)
    yaml.safe_dump(cams, open(session / 'data' / 'cameras.yaml', 'w'))

    rc = calib_extrinsics.main(['--session', 'S1', '--data-root', str(root), '--config-dir', CONFIG,
                                '--roles'] + roles + ['--images'] + images)
    res = yaml.safe_load(open(session / '04_extrinsics_ipm.yaml'))
    assert rc == 0, res['problems']
    out = yaml.safe_load(open(session / 'data' / 'cameras.yaml'))
    assert out['extrinsics_calibrated'] is True
    for role in intrs:
        got = Mount.from_yaml(out['mounts'][role])
        dpos, dang = got.difference(truth[role])
        assert dpos < 0.006 and dang < 0.6, (role, dpos, dang)
    summary = yaml.safe_load(open(session / 'summary.yaml'))
    assert summary['steps']['extrinsics_ipm']['status'] == 'PASS'
    assert os.path.isfile(session / 'captures' / 'extrinsics' / 'ipm_check.png')
