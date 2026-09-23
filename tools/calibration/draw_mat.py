#!/usr/bin/env python3
"""Draw docs/images/calib_mat.png (step-4 floor layout) from the YAML, so the
picture always matches calibration_steps.yaml and cameras.yaml.

    python3 tools/calibration/draw_mat.py
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import FancyArrow, Polygon  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
sys.path.insert(0, os.path.join(REPO, 'src', 'carbot_common'))
from carbot_common.calib_tools import floor_board_dicts  # noqa: E402


def rot(x, y, a):
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def main():
    steps = {s['id']: s for s in yaml.safe_load(open(os.path.join(CFG, 'data', 'calibration_steps.yaml')))['steps']}
    target = steps['extrinsics_ipm']['target']
    cams = yaml.safe_load(open(os.path.join(CFG, 'data', 'cameras.yaml')))
    veh = yaml.safe_load(open(os.path.join(CFG, 'params', 'common.yaml')))['/**']['ros__parameters']['vehicle']
    fig, ax = plt.subplots(figsize=(8, 9))
    # plot with forward = up, left = left: screen x = -y, screen y = x
    P = lambda x, y: (-y, x)  # noqa: E731
    L, W, R = veh['car_length_m'], veh['car_width_m'], veh['rear_overhang_m']
    car = [P(-R, -W / 2), P(L - R, -W / 2), P(L - R, W / 2), P(-R, W / 2)]
    ax.add_patch(Polygon(car, closed=True, fc='#dfe6ef', ec='#334', lw=1.5))
    ax.plot(*P(0, 0), marker='+', ms=22, mew=2.5, color='#d33')
    off = float(target['axle_offset_m'])
    if off:   # where the rear axle would sit with no offset: the layout's reference line
        ax.plot([-0.85, 0.85], [off, off], ls=':', color='#888', lw=1)
        ax.text(-0.84, off + 0.006, f'layout reference line ({off * 1000:.0f} mm ahead of the rear axle)',
                fontsize=6, color='#666')
    ax.annotate('TAPE CROSS = rear-axle centre\n(midpoint between rear wheels, on the floor)',
                P(0, 0), xytext=P(-0.28, 0.02), fontsize=8, color='#d33',
                arrowprops=dict(arrowstyle='->', color='#d33'))
    ax.plot([0, 0], [-0.35, 1.0], ls='--', color='#d33', lw=1)
    ax.text(0.01, 0.97, 'TAPE LINE = car centre line (+x forward)', color='#d33', fontsize=8)
    boarded = {r for b in target['boards'] for r in b['roles']}      # only cameras that have a floor board
    for role, m in cams['mounts'].items():
        if role not in boarded:
            continue
        x, y = P(m['x_m'], m['y_m'])
        a = math.radians(m['yaw_deg'])
        dx, dy = P(0.09 * math.cos(a), 0.09 * math.sin(a))
        ax.add_patch(FancyArrow(x, y, dx, dy, width=0.004, head_width=0.02, color='#1f6fd1'))
        ax.text(x + dx * 1.4, y + dy * 1.4 + 0.02, f'{role}\n({cams["roles"][role]})', fontsize=7,
                color='#1f6fd1', ha='center')
    for b in floor_board_dicts(target):
        cols, rows = b['inner_corners']
        sq = b['square_m']
        hu, hv = (cols + 1) * sq / 2, (rows + 1) * sq / 2
        a = math.radians(b['yaw_deg'])
        cx, cy = b['centre_m']
        pts = [P(cx + rot(u, v, a)[0], cy + rot(u, v, a)[1]) for u, v in
               ((-hu, -hv), (hu, -hv), (hu, hv), (-hu, hv))]
        ax.add_patch(Polygon(pts, closed=True, fc='#fff6d6', ec='k', lw=1.2))
        ax.plot(*P(cx, cy), marker='x', color='k')
        ax.text(*P(cx, cy), f'  {b["name"]} board\n  centre ({cx:.2f}, {cy:.2f}) m\n  long side yaw {b["yaw_deg"]:.0f} deg',
                fontsize=7, va='top')
    ax.set_aspect('equal')
    ax.set_xlim(-0.85, 0.85)
    ax.set_ylim(-0.35, 1.0)
    ax.set_xlabel('left  <-   y (m)   ->  right')
    ax.set_ylabel('x (m) forward')
    ax.set_xticks([-0.8, -0.4, 0, 0.4, 0.8])
    ax.set_xticklabels(['0.8', '0.4', '0', '-0.4', '-0.8'])
    ax.grid(alpha=0.3)
    ax.set_title('Calibration step 4: floor boards (top view, not to print)\n'
                 'Boards are drawn in base_link (axle_offset_m applied); edit centre_m if you move a board',
                 fontsize=9)
    out = os.path.join(REPO, 'docs', 'images', 'calib_mat.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print('wrote', out)


if __name__ == '__main__':
    main()
