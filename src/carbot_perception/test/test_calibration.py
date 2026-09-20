"""Steps 3/4 maths on synthetic views with known answers."""
import math

import cv2
import numpy as np
import pytest
from synth import MOUNTS, fisheye, pinhole

from carbot_perception.calib_core import (FloorBoard, board_points, calibrate_intrinsics,
                                          solve_mount)
from carbot_perception.camera_model import Intrinsics, Mount, project_ground


def synth_views(intr: Intrinsics, cols=9, rows=6, square=0.025, n=30, seed=0):
    rng = np.random.default_rng(seed)
    obj = board_points(cols, rows, square).astype(np.float64)
    obj -= obj.mean(axis=0)
    views = []
    while len(views) < n:
        rvec = rng.uniform(-0.6, 0.6, 3)
        rvec[2] = rng.uniform(-0.4, 0.4)
        R, _ = cv2.Rodrigues(rvec)
        target = np.array([[rng.uniform(0.1, 0.9) * intr.width, rng.uniform(0.1, 0.9) * intr.height]])
        t = intr.rays(target)[0] * rng.uniform(0.15, 0.35)   # spread views over the whole image
        pc = obj @ R.T + t
        uv, ok = intr.project(pc)
        if not ok.all():
            continue
        if (uv[:, 0].min() < 5 or uv[:, 1].min() < 5 or uv[:, 0].max() > intr.width - 5
                or uv[:, 1].max() > intr.height - 5):
            continue
        views.append((uv + rng.normal(0, 0.1, uv.shape)).astype(np.float32))
    return views


@pytest.mark.parametrize('make,expected', [(fisheye, 'equidistant'), (pinhole, 'plumb_bob')])
def test_intrinsics_choose_model(make, expected):
    truth = make()
    views = synth_views(truth)
    out = calibrate_intrinsics(views, 9, 6, 0.025, (truth.width, truth.height))
    got = out['chosen']
    assert got.intr.model == expected
    assert got.rms_px < 0.3
    assert got.intr.K[0, 0] == pytest.approx(truth.K[0, 0], rel=0.02)
    assert got.intr.K[0, 2] == pytest.approx(truth.K[0, 2], abs=2.0)


BOARDS = {
    'front': dict(role='front', inner_corners=[6, 4], square_m=0.050, centre_m=[0.62, 0.0], yaw_deg=90.0),
    'left_rear': dict(role='left_rear', inner_corners=[6, 4], square_m=0.050, centre_m=[0.10, 0.47], yaw_deg=0.0),
    'right_rear': dict(role='right_rear', inner_corners=[6, 4], square_m=0.050, centre_m=[0.10, -0.47], yaw_deg=0.0),
}


@pytest.mark.parametrize('role', ['front', 'left_rear', 'right_rear'])
@pytest.mark.parametrize('order', ['identity', 'rot180'])
def test_extrinsics_recover_mount(role, order):
    intr = Intrinsics.ideal(640, 480, 60.0) if role == 'front' else fisheye(960, 544, 300.0)
    board = FloorBoard.from_yaml(BOARDS[role])
    truth = Mount(MOUNTS[role].x + 0.006, MOUNTS[role].y - 0.004, MOUNTS[role].z + 0.003,
                  MOUNTS[role].yaw + math.radians(1.5), MOUNTS[role].pitch_down + math.radians(2.0),
                  math.radians(0.7))
    g = board.ground_points()
    uv, ok, _ = project_ground(intr, truth, g)
    assert ok.all()
    assert (uv[:, 0] > 0).all() and (uv[:, 0] < intr.width).all()
    assert (uv[:, 1] > 0).all() and (uv[:, 1] < intr.height).all(), 'default board not visible'
    if order == 'rot180':
        uv = uv[::-1]
    rng = np.random.default_rng(3)
    uv = uv + rng.normal(0, 0.15, uv.shape)
    fit = solve_mount(role, intr, uv.astype(np.float32), board, MOUNTS[role])
    assert fit is not None
    dpos, dang = fit.mount.difference(truth)
    assert dpos < 0.004 and dang < 0.5
    assert fit.ground_rms_m < 0.004
