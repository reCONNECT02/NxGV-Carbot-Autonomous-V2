#!/usr/bin/env python3
"""Printable calibration boards at exact scale (reportlab).

    python3 tools/calibration/make_boards.py            -> docs/calibration/*_A4/A3.pdf
    python3 tools/calibration/make_boards.py --sheet    -> docs/calibration/floor_sheet.pdf

intrinsics_board_A4.pdf  step 3: 9 x 6 inner corners, 25 mm squares (hand-held)
floor_board_A3.pdf       step 4: 6 x 4 inner corners, 50 mm squares (print 1: front camera only)
floor_sheet.pdf          step 4 ALTERNATIVE: the floor board(s) on ONE large
                         sheet at their exact calibration_steps.yaml positions,
                         plus the car's centre line and rear-axle line to line
                         the car up with. Nothing to measure except that the
                         print is to scale. Needs a large-format (roll) printer;
                         the script prints the sheet size and which roll fits.

Sizes AND the sheet's board positions come from calibration_steps.yaml, so the
PDFs always match the tools. The boards are drawn in base_link: centre_m plus
target.axle_offset_m in x (the rear axle sits that far behind the layout's
reference line, see carbot_common.calib_tools.floor_board_dicts). If you change a
board's centre_m / yaw_deg / axle_offset_m, regenerate and reprint the sheet.

PRINT AT 100 % / "Actual size" (never "fit to page"), then measure the scale
bars. If a bar is not its printed length, the print is scaled: for the A4/A3
boards edit square_m to the measured square size; for the sheet, reprint (a
roll print stretched along the feed direction cannot be fixed in the YAML).
Matte paper or matte vinyl: gloss reflects the lights and breaks detection.
Glue the hand-held board to foam board: it must be flat.
"""
import argparse
import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_common'))
from carbot_common.calib_tools import floor_board_dicts  # noqa: E402
STEPS = os.path.join(CFG, 'data', 'calibration_steps.yaml')
COMMON = os.path.join(CFG, 'params', 'common.yaml')
MM = 72.0 / 25.4                        # PDF points per millimetre
ROLLS_MM = (610, 914, 1067, 1118)       # common large-format roll widths (24/36/42/44 in)
ROLL_EDGE_MM = 10                       # unprintable edge a plotter keeps on each side


# --------------------------------------------------------------------------- A4 / A3 boards
def board(path, page, cols, rows, square_mm, title, centre_marks):
    from reportlab.lib.pagesizes import landscape
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
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


# --------------------------------------------------------------------------- sheet geometry
# Everything below is in base_link metres (x forward, y left, origin = rear-axle
# centre on the floor), the frame calib_core.FloorBoard uses. The PDF is only a
# drawing of this layout, and the test rasterises the same layout and checks
# the detected corners against FloorBoard.ground_points.

def _rot(u, v, cx, cy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return cx + u * c - v * s, cy + u * s + v * c


def _rect(cx, cy, yaw, half_u, half_v):
    return [_rot(u, v, cx, cy, yaw) for u, v in
            ((-half_u, -half_v), (half_u, -half_v), (half_u, half_v), (-half_u, half_v))]


def _inside(p, poly):
    """Point strictly inside a convex polygon (either winding)."""
    sign = 0
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        cr = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
        if abs(cr) < 1e-12:
            return False
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


def _overlap(a, b):
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            ax, ay = y1 - y2, x2 - x1
            pa = [ax * x + ay * y for x, y in a]
            pb = [ax * x + ay * y for x, y in b]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


def _clip(p0, p1, obstacles, step=0.001):
    """Parts of segment p0-p1 that lie outside every obstacle polygon."""
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(length / step))
    pts = [(p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n) for k in range(n + 1)]
    free = [not any(_inside(p, o) for o in obstacles) for p in pts]
    segs, start = [], None
    for k, f in enumerate(free):
        if f and start is None:
            start = k
        if (not f or k == n) and start is not None:
            end = k if f else k - 1
            if end - start >= 5:                       # drop stubs shorter than ~5 mm
                segs.append((pts[start], pts[end]))
            start = None
    return segs


def sheet_layout(boards, vehicle, margin_m=0.04, quiet_m=0.02):
    """Boards (calibration_steps.yaml step-4 dicts) + vehicle dims -> layout dict."""
    out, problems = [], []
    for b in boards:
        cols, rows = (int(v) for v in b['inner_corners'])
        sq = float(b['square_m'])
        cx, cy = (float(v) for v in b['centre_m'])
        yaw = math.radians(float(b['yaw_deg']))
        nx, ny = cols + 1, rows + 1
        squares = []
        for i in range(nx):
            for j in range(ny):
                if (i + j) % 2 == 0:                  # same parity as the A3 board
                    u0, v0 = (i - nx / 2.0) * sq, (j - ny / 2.0) * sq
                    squares.append([_rot(u, v, cx, cy, yaw) for u, v in
                                    ((u0, v0), (u0 + sq, v0), (u0 + sq, v0 + sq), (u0, v0 + sq))])
        out.append({'name': b.get('name', b['roles'][0]), 'roles': list(b.get('roles') or [b['role']]),
                    'cols': cols, 'rows': rows, 'square_m': sq, 'centre_m': (cx, cy),
                    'yaw_deg': float(b['yaw_deg']), 'squares': squares,
                    'outline': _rect(cx, cy, yaw, nx * sq / 2, ny * sq / 2),
                    'quiet': _rect(cx, cy, yaw, nx * sq / 2 + quiet_m, ny * sq / 2 + quiet_m)})

    L, W, R = float(vehicle['car_length_m']), float(vehicle['car_width_m']), float(vehicle['rear_overhang_m'])
    car = [(-R, -W / 2), (L - R, -W / 2), (L - R, W / 2), (-R, W / 2)]

    for i in range(len(out)):
        for j in range(i + 1, len(out)):
            if _overlap(out[i]['quiet'], out[j]['quiet']):
                problems.append(f'boards "{out[i]["name"]}" and "{out[j]["name"]}" overlap '
                                f'(including the {quiet_m * 1000:.0f} mm white border)')
        if _overlap(out[i]['quiet'], car):
            problems.append(f'board "{out[i]["name"]}" is under the car outline')

    xs = [p[0] for b in out for p in b['quiet']] + [p[0] for p in car]
    ys = [p[1] for b in out for p in b['quiet']] + [p[1] for p in car]
    bounds = (min(xs) - margin_m, max(xs) + margin_m, min(ys) - margin_m, max(ys) + margin_m)
    x0, x1, y0, y1 = bounds

    pad = 0.005                                        # lines stop 5 mm short of the car / borders
    car_pad = [(-R - pad, -W / 2 - pad), (L - R + pad, -W / 2 - pad),
               (L - R + pad, W / 2 + pad), (-R - pad, W / 2 + pad)]
    obstacles = [b['quiet'] for b in out] + [car_pad]
    inset = margin_m * 0.5                             # keep guide lines out of the text margins
    xa, xb, ya, yb = -R - pad, L - R + pad, -W / 2 - pad, W / 2 + pad

    def touches_car(seg):   # keep only the pieces that start at the car: those are what you align to
        return any(abs(p[0] - xa) < 0.003 or abs(p[0] - xb) < 0.003 or
                   abs(p[1] - ya) < 0.003 or abs(p[1] - yb) < 0.003 for p in seg)

    lines = {k: [s for s in _clip(p0, p1, obstacles) if touches_car(s)] for k, (p0, p1) in
             {'centre': ((x0 + inset, 0.0), (x1 - inset, 0.0)),
              'axle': ((0.0, y0 + inset), (0.0, y1 - inset))}.items()}
    if len(lines['centre']) < 2 or len(lines['axle']) < 2:
        problems.append('no room for the centre line (front and back) or the rear-axle line '
                        '(both sides): a board covers it next to the car')
    return {'bounds': bounds, 'boards': out, 'car': car, 'lines': lines, 'problems': problems,
            'margin_m': margin_m, 'quiet_m': quiet_m, 'vehicle': {'L': L, 'W': W, 'R': R}}


def roll_fit(width_mm, height_mm):
    """Smallest common roll that fits the sheet's shorter side (None if none)."""
    short = min(width_mm, height_mm)
    for r in ROLLS_MM:
        if short <= r - 2 * ROLL_EDGE_MM:
            return r
    return None


# --------------------------------------------------------------------------- sheet PDF
def draw_sheet(path, lay):
    from reportlab.pdfgen import canvas
    x0, x1, y0, y1 = lay['bounds']
    pw, ph = (y1 - y0) * 1000 * MM, (x1 - x0) * 1000 * MM

    def P(x, y):   # base_link -> page: forward = up, left = left (a rotation, not a mirror)
        return (y1 - y) * 1000 * MM, (x - x0) * 1000 * MM

    c = canvas.Canvas(path, pagesize=(pw, ph), invariant=1)
    c.setTitle('RISA Bot calibration step 4 floor sheet')

    def poly(pts, fill, stroke):
        p = c.beginPath()
        p.moveTo(*P(*pts[0]))
        for q in pts[1:]:
            p.lineTo(*P(*q))
        p.close()
        c.drawPath(p, fill=fill, stroke=stroke)

    # car outline (guide only) and the rear-axle centre
    c.setDash(6, 4)
    c.setLineWidth(0.8)
    c.setStrokeGray(0.45)
    poly(lay['car'], 0, 1)
    c.setDash()
    ox, oy = P(0.0, 0.0)
    c.setStrokeGray(0.0)
    c.setLineWidth(1.2)
    c.line(ox - 12 * MM, oy, ox + 12 * MM, oy)
    c.line(ox, oy - 12 * MM, ox, oy + 12 * MM)
    c.setFont('Helvetica', 9)
    c.setFillGray(0.3)
    c.drawString(ox + 3 * MM, oy + 3 * MM, 'rear-axle centre (under the car)')
    c.drawString(*P(lay['vehicle']['L'] - lay['vehicle']['R'] - 0.03, lay['vehicle']['W'] / 2 - 0.01),
                 'car outline (guide only)')

    # alignment lines
    c.setLineWidth(1.5)
    c.setStrokeGray(0.0)
    for a, b in lay['lines']['centre'] + lay['lines']['axle']:
        c.line(*P(*a), *P(*b))
    c.setFillGray(0.0)
    c.setFont('Helvetica-Bold', 11)
    fwd = lay['vehicle']['L'] - lay['vehicle']['R'] + 0.02
    front_line = [s for s in lay['lines']['centre'] if s[1][0] > fwd]
    if front_line:                                      # arrowhead on the forward end of the centre line
        tip = min(front_line[0][1][0], fwd + 0.12)
        tx, ty = P(tip, 0.0)
        p = c.beginPath()
        p.moveTo(tx, ty)
        p.lineTo(tx - 5 * MM, ty - 10 * MM)
        p.lineTo(tx + 5 * MM, ty - 10 * MM)
        p.close()
        c.drawPath(p, fill=1, stroke=0)
        c.drawString(tx + 4 * MM, ty - 8 * MM, 'FORWARD  (car centre line)')
    for a, b in lay['lines']['axle']:
        far = a if abs(a[1]) > abs(b[1]) else b
        near = b if far is a else a
        mx, my = P(0.0, (far[1] + near[1]) / 2)
        c.setFont('Helvetica-Bold', 9)
        c.drawCentredString(mx, my + 3 * MM, 'REAR AXLE')

    # boards: white border first, then the squares
    for b in lay['boards']:
        c.setFillGray(1.0)
        poly(b['quiet'], 1, 0)
        c.setFillGray(0.0)
        for sq in b['squares']:
            poly(sq, 1, 0)

    # scale bars in the margins: across (top margin) and along (left margin)
    m = lay['margin_m']
    c.setLineWidth(1.0)
    c.setStrokeGray(0.0)
    c.setFillGray(0.0)

    def bar(p_start, p_end, n_dm, label_at):
        c.line(*p_start, *p_end)
        for k in range(n_dm + 1):
            f = k / n_dm
            px = p_start[0] + (p_end[0] - p_start[0]) * f
            py = p_start[1] + (p_end[1] - p_start[1]) * f
            t = (3 if k in (0, n_dm) else 1.5) * MM
            if p_start[1] == p_end[1]:
                c.line(px, py - t, px, py + t)
            else:
                c.line(px - t, py, px + t, py)
        c.setFont('Helvetica', 8)
        text = f'{n_dm * 100} mm scale bar - measure it; if it is not exactly {n_dm * 100} mm the print is scaled: reprint at 100 %'
        if p_start[1] == p_end[1]:
            c.drawString(*label_at, text)
        else:                          # the along-the-car bar: text runs up the left margin, clear of the boards
            c.saveState()
            c.translate(*label_at)
            c.rotate(90)
            c.drawString(0, 0, text)
            c.restoreState()

    n_across = min(10, int(((y1 - y0) - 2 * m) * 10))
    n_along = min(10, int(((x1 - x0) - 2 * m) * 10))
    ytop = ph - m * 1000 * MM * 0.35
    xs = m * 1000 * MM
    bar((xs, ytop), (xs + n_across * 100 * MM, ytop), n_across, (xs, ytop + 4 * MM))
    xl = m * 1000 * MM * 0.35
    y_s = m * 1000 * MM
    bar((xl, y_s), (xl, y_s + n_along * 100 * MM), n_along, (xl + 8 * MM, y_s + 4 * MM))

    # title + board list + instructions in the bottom margin
    lines = ['RISA Bot - calibration step 4 floor sheet (generated from calibration_steps.yaml). '
             'Print at 100 % / actual size on matte paper or vinyl. Tape flat: no bubbles, no curl.',
             'Place the car: centre line under the middle of the front and rear bumpers, rear wheel hubs '
             'directly above the REAR AXLE lines. Do not let the wheels touch a board.']
    lines.append('Boards (base_link, m): ' + '   '.join(
        f'{b["name"]}: centre ({b["centre_m"][0]:.3f}, {b["centre_m"][1]:.3f}) yaw {b["yaw_deg"]:.0f} deg, '
        f'{b["cols"]}x{b["rows"]} inner, {b["square_m"] * 1000:.0f} mm' for b in lay['boards']))
    off = lay['axle_offset_m']
    lines.append(f'Rear axle placed {off * 1000:.0f} mm further BACK relative to the boards than on the first sheet '
                 f'(axle_offset_m {off:g}): the boards above are drawn in base_link, i.e. centre_m + {off:g} m in x.')
    lines.append('If you edit centre_m / yaw_deg / axle_offset_m in calibration_steps.yaml, regenerate '
                 '(make_boards.py --sheet) and reprint - the sheet and the YAML must match.')
    c.setFont('Helvetica', 8)
    c.setFillGray(0.0)
    for k, t in enumerate(lines):
        c.drawString(m * 1000 * MM, (m * 1000 * MM) * 0.75 - k * 3.6 * MM, t)
    c.showPage()
    c.save()


# --------------------------------------------------------------------------- main
def load_yaml():
    steps = {s['id']: s for s in yaml.safe_load(open(STEPS))['steps']}
    vehicle = yaml.safe_load(open(COMMON))['/**']['ros__parameters']['vehicle']
    return steps, vehicle


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sheet', action='store_true',
                    help='write only docs/calibration/floor_sheet.pdf (one large step-4 sheet)')
    ap.add_argument('--margin', type=float, default=0.04, help='sheet margin around everything, m')
    ap.add_argument('--quiet', type=float, default=0.02, help='white border kept around each board, m')
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'calibration'))
    a = ap.parse_args(argv)
    steps, vehicle = load_yaml()
    os.makedirs(a.out, exist_ok=True)

    if not a.sheet:
        from reportlab.lib.pagesizes import A3, A4
        t3 = steps['camera_intrinsics']['target']
        board(os.path.join(a.out, 'intrinsics_board_A4.pdf'), A4, *t3['inner_corners'],
              t3['square_m'] * 1000, 'Step 3 intrinsics board', False)
        fb = steps['extrinsics_ipm']['target']['boards'][0]
        board(os.path.join(a.out, 'floor_board_A3.pdf'), A3, *fb['inner_corners'],
              fb['square_m'] * 1000, 'Step 4 floor board (print 1)', True)
        return 0

    target = steps['extrinsics_ipm']['target']
    lay = sheet_layout(floor_board_dicts(target), vehicle, a.margin, a.quiet)      # base_link, axle_offset_m applied
    lay['axle_offset_m'] = float(target['axle_offset_m'])
    if lay['problems']:
        for p in lay['problems']:
            print('ERROR:', p)
        print('Fix centre_m / yaw_deg in calibration_steps.yaml (step 4) and run again.')
        return 1
    path = os.path.join(a.out, 'floor_sheet.pdf')
    draw_sheet(path, lay)
    x0, x1, y0, y1 = lay['bounds']
    w_mm, h_mm = (y1 - y0) * 1000, (x1 - x0) * 1000
    print(f'wrote {path}')
    print(f'sheet size: {w_mm:.0f} mm across x {h_mm:.0f} mm along the car')
    r = roll_fit(w_mm, h_mm)
    if r is None:
        print('WARNING: too big for common rolls (up to 1118 mm / 44 in). Move the boards closer '
              'together in calibration_steps.yaml or use --margin 0.02.')
    else:
        print(f'fits a {r} mm roll ({r / 25.4:.0f} in) with the {min(w_mm, h_mm):.0f} mm side across the roll')
    return 0


if __name__ == '__main__':
    sys.exit(main())
