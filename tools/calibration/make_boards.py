#!/usr/bin/env python3
"""Printable calibration boards at exact scale (reportlab).

    python3 tools/calibration/make_boards.py            -> docs/calibration/*.pdf

intrinsics_board_A4.pdf  step 3: 9 x 6 inner corners, 25 mm squares (hand-held)
floor_board_A3.pdf       step 4: 6 x 4 inner corners, 50 mm squares (print 3)

Sizes come from calibration_steps.yaml, so the PDFs always match the tools.
PRINT AT 100 % / "Actual size" (never "fit to page"), then measure the 100 mm
scale bar. If it is not 100 mm, edit square_m in calibration_steps.yaml to the
measured square size. Glue the hand-held board to foam board: it must be flat.
Matte paper: glossy prints reflect the lights and break corner detection.
"""
import os
import sys

import yaml
from reportlab.lib.pagesizes import A3, A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STEPS = os.path.join(REPO, 'src', 'carbot_bringup', 'config', 'data', 'calibration_steps.yaml')


def board(path, page, cols, rows, square_mm, title, centre_marks):
    w, h = landscape(page)
    c = canvas.Canvas(path, pagesize=(w, h))
    nx, ny = cols + 1, rows + 1                      # squares
    bw, bh = nx * square_mm * mm, ny * square_mm * mm
    x0, y0 = (w - bw) / 2, (h - bh) / 2
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                c.rect(x0 + i * square_mm * mm, y0 + j * square_mm * mm, square_mm * mm, square_mm * mm,
                       stroke=0, fill=1)
    if centre_marks:   # ticks on the paper margin through the board centre (for measuring centre_m)
        cx, cy = w / 2, h / 2
        c.setLineWidth(0.6)
        for a, b in (((cx, 1 * mm), (cx, y0 - 2 * mm)), ((cx, y0 + bh + 2 * mm), (cx, h - 1 * mm)),
                     ((1 * mm, cy), (x0 - 2 * mm, cy)), ((x0 + bw + 2 * mm, cy), (w - 1 * mm, cy))):
            c.line(a[0], a[1], b[0], b[1])
        c.setFont('Helvetica', 7)
        c.drawRightString(w - 4 * mm, 4 * mm, 'ticks = board centre; long side = board x (yaw_deg)')
    c.setFont('Helvetica', 7)
    c.drawString(4 * mm, h - 5 * mm, f'{title}: {cols} x {rows} inner corners, {square_mm:g} mm squares. '
                 'PRINT AT 100 % (actual size). Check the scale bar.')
    c.setLineWidth(1.0)
    c.line(4 * mm, 4 * mm, 104 * mm, 4 * mm)
    for t in (4, 104):
        c.line(t * mm, 3 * mm, t * mm, 6 * mm)
    c.drawString(40 * mm, 6 * mm, '100 mm scale bar')
    c.showPage()
    c.save()
    print('wrote', path)


def main():
    steps = {s['id']: s for s in yaml.safe_load(open(STEPS))['steps']}
    out = os.path.join(REPO, 'docs', 'calibration')
    os.makedirs(out, exist_ok=True)
    t3 = steps['camera_intrinsics']['target']
    board(os.path.join(out, 'intrinsics_board_A4.pdf'), A4, *t3['inner_corners'],
          t3['square_m'] * 1000, 'Step 3 intrinsics board', False)
    fb = steps['extrinsics_ipm']['target']['boards'][0]
    board(os.path.join(out, 'floor_board_A3.pdf'), A3, *fb['inner_corners'],
          fb['square_m'] * 1000, 'Step 4 floor board (print 3)', True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
