"""Tests for the step-4 floor sheet: the printed layout must put every board
exactly where calib_core.FloorBoard (and so calib_extrinsics) expects it.

    python3 -m pytest -q tools/calibration
"""
import copy
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(REPO, 'src', 'carbot_perception'), os.path.join(REPO, 'src', 'carbot_common')]

import make_boards as mb  # noqa: E402
from carbot_perception.calib_core import FloorBoard, detect_chessboard  # noqa: E402

STEPS, VEHICLE = mb.load_yaml()
BOARDS = STEPS['extrinsics_ipm']['target']['boards']


def _check_image(img, lay, ppm):
    """Detect each board in its own crop of a top-view image (forward = up,
    left = left, pixel centres at +0.5) and return the max corner error, m."""
    x0, x1, y0, y1 = lay['bounds']
    h, w = img.shape
    worst = 0.0
    for bd in BOARDS:
        fb = FloorBoard.from_yaml(bd)
        lb = next(b for b in lay['boards'] if b['name'] == fb.name)
        q = np.array(lb['quiet'])
        us, vs = (y1 - q[:, 1]) * ppm, (x1 - q[:, 0]) * ppm
        pad = int(0.03 * ppm)
        ua, ub = max(0, int(us.min()) - pad), min(w, int(us.max()) + pad)
        va, vb = max(0, int(vs.min()) - pad), min(h, int(vs.max()) + pad)
        c = detect_chessboard(img[va:vb, ua:ub], fb.cols, fb.rows)
        assert c is not None, f'board {fb.name} not detected'
        pts = np.column_stack([x1 - (c[:, 1] + va + 0.5) / ppm, y1 - (c[:, 0] + ua + 0.5) / ppm])
        err = min(float(np.linalg.norm(pts - g[:, :2], axis=1).max()) for _, g in fb.orderings())
        worst = max(worst, err)
    return worst


def test_layout_matches_floorboard_geometry():
    lay = mb.sheet_layout(BOARDS, VEHICLE)
    assert lay['problems'] == []
    ppm = 2000.0                                             # 0.5 mm per pixel
    x0, x1, y0, y1 = lay['bounds']
    img = np.full((int((x1 - x0) * ppm), int((y1 - y0) * ppm)), 255, np.uint8)
    shift = 4
    for b in lay['boards']:
        for sq in b['squares']:
            poly = np.array([[(y1 - y) * ppm - 0.5, (x1 - x) * ppm - 0.5] for x, y in sq]) * (1 << shift)
            cv2.fillPoly(img, [np.round(poly).astype(np.int32)], 0, cv2.LINE_AA, shift)
    assert _check_image(img, lay, ppm) < 0.001


def test_alignment_lines_present_and_clear_of_boards():
    lay = mb.sheet_layout(BOARDS, VEHICLE)
    assert len(lay['lines']['centre']) == 2 and len(lay['lines']['axle']) == 2
    for a, b in lay['lines']['centre'] + lay['lines']['axle']:
        for t in np.linspace(0, 1, 50):
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            assert not any(mb._inside(p, bd['quiet']) for bd in lay['boards'])


def test_overlapping_boards_are_rejected():
    boards = copy.deepcopy(BOARDS)
    boards[1]['centre_m'] = [boards[0]['centre_m'][0] - 0.05, 0.05]     # left board onto the front one
    assert any('overlap' in p for p in mb.sheet_layout(boards, VEHICLE)['problems'])
    boards = copy.deepcopy(BOARDS)
    boards[0]['centre_m'] = [0.15, 0.0]                                   # front board under the car
    assert any('under the car' in p for p in mb.sheet_layout(boards, VEHICLE)['problems'])


def test_roll_fit():
    assert mb.roll_fit(1310, 940) == 1067
    assert mb.roll_fit(1310, 880) == 914
    assert mb.roll_fit(1300, 1200) is None


@pytest.mark.skipif(shutil.which('pdftoppm') is None, reason='needs poppler-utils (pdftoppm)')
def test_printed_pdf_matches_geometry(tmp_path):
    pytest.importorskip('reportlab')
    assert mb.main(['--sheet', '--out', str(tmp_path)]) == 0
    subprocess.run(['pdftoppm', '-r', '100', '-gray', '-png', str(tmp_path / 'floor_sheet.pdf'),
                    str(tmp_path / 'r')], check=True)
    img = cv2.imread(str(next(tmp_path.glob('r*.png'))), cv2.IMREAD_GRAYSCALE)
    lay = mb.sheet_layout(BOARDS, VEHICLE)
    assert _check_image(img, lay, 100 / 0.0254) < 0.001
