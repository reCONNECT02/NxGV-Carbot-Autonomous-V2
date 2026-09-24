"""A REAL front-camera frame of the step 4 floor board (risabot5, 640x480, 2026-09-24, camera 9.35 cm above
the floor, board at base_link x 0.77): rows only 10-20 px apart. The plain detectors give up (BACKLOG #54);
the vertical-stretch fallback (calibration_steps.yaml extrinsics_ipm.target.detect_vertical_stretch) finds it."""
import os
import sys

import cv2
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from carbot_perception import calib_core as cc  # noqa: E402

FRAME = os.path.join(HERE, 'data', 'step4_front_640x480.png')
STEPS = os.path.join(HERE, '..', '..', 'carbot_bringup', 'config', 'data', 'calibration_steps.yaml')


def _stretch():
    doc = yaml.safe_load(open(STEPS, encoding='utf-8'))
    return [float(x) for x in [s for s in doc['steps'] if s['id'] == 'extrinsics_ipm'][0]['target']['detect_vertical_stretch']]


def test_plain_detector_misses_the_foreshortened_board():
    img = cv2.imread(FRAME)
    assert img is not None and img.shape[:2] == (480, 640)
    assert cc.detect_chessboard(img, 6, 4) is None


def test_stretch_fallback_finds_all_corners_in_grid_order():
    img = cv2.imread(FRAME)
    c = cc.detect_chessboard(img, 6, 4, stretch=_stretch())
    assert c is not None and c.shape == (24, 2)
    g = c.reshape(4, 6, 2)
    assert (np.diff(g[:, :, 0], axis=1) > 0).all() or (np.diff(g[:, :, 0], axis=1) < 0).all()   # columns run one way
    assert (np.diff(g[:, :, 1], axis=0) > 0).all() or (np.diff(g[:, :, 1], axis=0) < 0).all()   # rows run one way
    # the corners are on the original image, inside the frame, in the lower half (board on the floor)
    assert (c[:, 0] > 0).all() and (c[:, 0] < 640).all() and (c[:, 1] > 240).all() and (c[:, 1] < 480).all()
    # plausibility of the physical grid (50 mm squares, camera ~0.8 m away): rows 5-20 px apart, columns 30-70 px
    rows_px = np.abs(np.diff(g[:, :, 1], axis=0)).mean()
    cols_px = np.abs(np.diff(g[:, :, 0], axis=1)).mean()
    assert 5.0 < rows_px < 20.0 and 30.0 < cols_px < 70.0, (rows_px, cols_px)
    # A homography from the ideal grid fits to a few pixels only: the raw frame still has lens distortion
    # (the rows bend); this is why step 3 intrinsics come before step 4. A wrong detection is far worse.
    grid = np.array([[x * 0.05, y * 0.05] for y in range(4) for x in range(6)], np.float32)
    H, _ = cv2.findHomography(grid, c, 0)
    proj = cv2.perspectiveTransform(grid[None], H)[0]
    assert float(np.linalg.norm(proj - c, axis=1).max()) < 10.0


def test_no_stretch_means_old_behaviour():
    img = cv2.imread(FRAME)
    assert cc.detect_chessboard(img, 6, 4, stretch=()) is None
    assert cc.detect_chessboard(img, 6, 4, stretch=(1.0, 0.5)) is None          # factors <= 1 are ignored
