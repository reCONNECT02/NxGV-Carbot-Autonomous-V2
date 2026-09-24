"""Calibration step 4 - 3-camera extrinsics + IPM.

    ros2 run carbot_perception calib_extrinsics --session 20260921_1400
    ros2 run carbot_perception calib_extrinsics --session ... --images front=f.png left_rear=l.png right_rear=r.png

Needs step 3 (intrinsics) in the same session first.

Floor set-up (calibration_steps.yaml step 4 target.boards, picture
docs/images/calib_mat.png): tape a cross on the floor where the REAR-AXLE
CENTRE goes and a straight line for the car's centre line. Lay each floor
board flat at its position in base_link = centre_m + target.axle_offset_m in x
(carbot_common.calib_tools.floor_board_dicts; the rear axle sits axle_offset_m
BEHIND the layout's reference line, 0.150 m since 2026-09-24), measured from the
cross along / across the line, with its long side along yaw_deg.
If a board is not fully visible in its camera, move it, MEASURE the new
centre and edit centre_m - the measurement is what matters, not the default.

Or use the single floor sheet (tools/calibration/make_boards.py --sheet ->
docs/calibration/floor_sheet.pdf): the board printed at its exact position, with
the car's centre line and rear-axle line on the sheet. Tape it flat, put the car
on the lines, check the scale bars - nothing to measure. The sheet must be
regenerated whenever centre_m / yaw_deg / axle_offset_m change.

For every camera the tool averages the board corners over several frames,
solves the camera pose, reports how well the pose maps the board back onto
the floor (ground error) and how far it is from the CAD mount, writes the
mounts into <session>/data/cameras.yaml and saves
<session>/captures/extrinsics/ipm_check.png: the stitched top-down view with
the boards' true outlines drawn on top - they must line up.
"""
import argparse
import datetime
import os
import sys

import cv2
import numpy as np
from carbot_common import calibration_store as cs
from carbot_common.calib_tools import floor_board_dicts

from .calib_core import FloorBoard, detect_chessboard, seam_error, solve_mount
from .calib_io import (FrameGrabber, bringup_config_dir, common_args, draw_corners,
                       effective_cameras, load_yaml, save_cameras, step_cfg)
from .camera_model import camera_setup, intrinsics_for
from .road_mask import BodyBox, Classify, GridSpec, RoadMask, Seed, bev_view, build_camera_map

STEP = 'extrinsics_ipm'
RESULT = '04_extrinsics_ipm.yaml'


def capture(grab, role, board, frames, tries, stretch=()):
    """Average corners over `frames` detections of a still board."""
    got, img_last = [], None
    for _ in range(tries):
        img = grab.next(role, 2.0)
        if img is None:
            continue
        img_last = img
        c = detect_chessboard(img, board.cols, board.rows, stretch=stretch)
        if c is None:
            continue
        if got and float(np.mean(np.linalg.norm(c - got[-1], axis=1))) > 1.0:
            got = []                                  # moved: start again
        got.append(c)
        if len(got) >= frames:
            break
    if not got:
        return None, img_last
    return np.mean(got, axis=0).astype(np.float32), img_last


def ipm_check(cameras, images, grid, body, boards, out_path, scale=4):
    rm = RoadMask(grid, Seed())
    small = {}
    for role, img in images.items():
        if img is None:
            continue
        w = 480
        s = cv2.resize(img, (w, int(round(img.shape[0] * w / img.shape[1]))), interpolation=cv2.INTER_AREA)
        intr = intrinsics_for(cameras, role, s.shape[1], s.shape[0])
        _, _, mount, _ = camera_setup(cameras, role)
        rm.maps[role] = build_camera_map(role, grid, intr, mount, body, 0.025, 0.1, 2.0)
        small[role] = s
    res = rm.process(small, Classify())
    view = bev_view(res.bgr, scale)
    n = grid.cells

    def to_px(x, y):   # base_link -> display pixel (forward up, left on the left)
        r = (x - grid.x0) / grid.res
        c = (y - grid.y0) / grid.res
        return int(round((n - c) * scale)), int(round((n - r) * scale))

    for b in boards:
        g = b.ground_points().reshape(b.rows, b.cols, 3)
        corners = [g[0, 0], g[0, -1], g[-1, -1], g[-1, 0]]
        pts = np.array([to_px(p[0], p[1]) for p in corners], np.int32)
        cv2.polylines(view, [pts], True, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.drawMarker(view, to_px(0.0, 0.0), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
    cv2.imwrite(out_path, view)
    return out_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', nargs='*', default=[], help='role=path pairs (offline)')
    ap.add_argument('--frames', type=int, default=8, help='frames averaged per camera')
    ap.add_argument('--tries', type=int, default=60, help='frames to try per camera')
    ap.add_argument('--roles', nargs='*', default=['front', 'left_rear', 'right_rear'])
    common_args(ap)
    a = ap.parse_args(argv)

    root = cs.data_root(a.data_root)
    cfg_dir = bringup_config_dir(a.config_dir)
    step = step_cfg(cfg_dir, STEP)
    boards = [FloorBoard.from_yaml(b) for b in floor_board_dicts(step['target'])]      # base_link, axle_offset_m applied
    pas = step['pass']
    if 'detect_vertical_stretch' not in step['target']:
        raise KeyError('missing YAML key calibration_steps.yaml extrinsics_ipm.target.detect_vertical_stretch')
    stretch = [float(x) for x in step['target']['detect_vertical_stretch']]     # foreshortened floor board, BACKLOG #54
    session = cs.open_session(root, a.session)
    cameras = effective_cameras(session, cfg_dir, root)
    out_dir = os.path.join(session, 'captures', 'extrinsics')
    os.makedirs(out_dir, exist_ok=True)
    common = load_yaml(os.path.join(cfg_dir, 'params', 'common.yaml'))['/**']['ros__parameters']
    v = common['vehicle']
    body = BodyBox(v['rear_overhang_m'], v['car_length_m'] - v['rear_overhang_m'], v['car_width_m'] / 2)
    perc = load_yaml(os.path.join(cfg_dir, 'params', 'perception.yaml'))['road_perception']['ros__parameters']
    grid = GridSpec(perc['grid']['cells'], perc['grid']['resolution_m'],
                    perc['grid']['origin_x_m'], perc['grid']['origin_y_m'])

    images, corners, fits, intrs, problems = {}, {}, {}, {}, []
    offline = dict(kv.split('=', 1) for kv in a.images)
    grab = None
    if not offline:
        from carbot_common.data import camera_role_topics
        grab = FrameGrabber({r: t for r, t in camera_role_topics(cameras).items() if r in a.roles})
    try:
        for role in a.roles:
            sensor, sdef, nominal, _ = camera_setup(cameras, role)
            mine = [b for b in boards if role in b.roles]
            if not mine:
                problems.append(f'{role}: no floor board lists this role')
                continue
            board = mine[0]
            if offline:
                img = cv2.imread(offline.get(role, ''))
                c = detect_chessboard(img, board.cols, board.rows, stretch=stretch) if img is not None else None
            else:
                print(f'[calib] {role}: looking for board "{board.name}" ...')
                c, img = capture(grab, role, board, a.frames, a.tries, stretch)
            images[role] = img
            if img is None:
                problems.append(f'{role}: no image')
                continue
            cv2.imwrite(os.path.join(out_dir, f'{role}.png'), img)
            cv2.imwrite(os.path.join(out_dir, f'{role}_detect.jpg'),
                        draw_corners(img, c, board.cols, board.rows, c is not None))
            if c is None:
                problems.append(f'{role}: board "{board.name}" not found (see {role}_detect.jpg)')
                continue
            corners[(board.name, role)] = c
            intr = intrinsics_for(cameras, role, img.shape[1], img.shape[0])
            intrs[role] = intr
            if intr.source != 'calibrated':
                problems.append(f'{role}: sensor {sensor} has no intrinsics (run step 3 first)')
            fit = solve_mount(role, intr, c, board, nominal)
            if fit is None:
                problems.append(f'{role}: pose could not be solved')
                continue
            fits[role] = fit
            # also collect corners of other boards this camera can see (seam check)
            for other in boards:
                if other is not board and role in other.roles:
                    oc = detect_chessboard(img, other.cols, other.rows, stretch=stretch)
                    if oc is not None:
                        corners[(other.name, role)] = oc
    finally:
        if grab is not None:
            grab.close()

    seams = seam_error(fits, intrs, corners, boards)
    limit_g = float(pas['max_ground_error_m'])
    limit_s = float(pas['max_seam_error_m'])
    warn_pos = float(step['target'].get('warn_nominal_offset_m', 0.03))
    warn_ang = float(step['target'].get('warn_nominal_angle_deg', 5.0))
    per_role = {}
    for role in a.roles:
        f = fits.get(role)
        if f is None:
            per_role[role] = {'status': 'FAIL'}
            continue
        ok = f.ground_rms_m <= limit_g
        per_role[role] = {
            'status': 'PASS' if ok else 'FAIL',
            'mount': f.mount.to_yaml(),
            'nominal': camera_setup(cameras, role)[2].to_yaml(),
            'ground_rms_m': round(f.ground_rms_m, 4), 'ground_max_m': round(f.ground_max_m, 4),
            'reprojection_px': round(f.reproj_px, 3), 'corner_order': f.ordering,
            'offset_from_nominal_m': round(f.nominal_dpos_m, 4),
            'angle_from_nominal_deg': round(f.nominal_dang_deg, 2),
            'intrinsics': intrs[role].source}
        if f.nominal_dpos_m > warn_pos or f.nominal_dang_deg > warn_ang:
            problems.append(f'{role}: {f.nominal_dpos_m * 100:.1f} cm / {f.nominal_dang_deg:.1f} deg from the '
                            f'CAD mount - check the board centre_m / yaw_deg you measured')
    seam_ok = all(e <= limit_s for e in seams.values())
    all_ok = (all(per_role[r]['status'] == 'PASS' for r in a.roles) and seam_ok
              and all(intrs.get(r) is not None and intrs[r].source == 'calibrated' for r in a.roles))

    if fits:
        for role, f in fits.items():
            hfov = round(intrs[role].hfov_deg(), 2)
            cameras['mounts'][role] = f.mount.to_yaml({'hfov_deg': hfov})
        cameras['extrinsics_calibrated'] = bool(all_ok)
        cameras['extrinsics_date'] = datetime.datetime.now().isoformat(timespec='seconds')
        save_cameras(session, cameras)
        ipm = ipm_check(cameras, images, grid, body, boards, os.path.join(out_dir, 'ipm_check.png'))
    else:
        ipm = ''

    doc = {'status': 'PASS' if all_ok else 'FAIL', 'cameras': per_role,
           'seam_error_m': {k: round(v, 4) for k, v in seams.items()} or 'not measured (no board seen by two cameras)',
           'limits': {'max_ground_error_m': limit_g, 'max_seam_error_m': limit_s},
           'boards': floor_board_dicts(step['target']), 'problems': problems, 'ipm_check': ipm}
    cs.write_yaml(os.path.join(session, RESULT), doc)
    cs.update_step(session, STEP, doc['status'], RESULT)
    if a.activate:
        cs.set_active(root, session)

    print(f'\n[calib] step {STEP}: {doc["status"]}')
    for role, r in per_role.items():
        if 'mount' in r:
            m = r['mount']
            print(f'  {role:10s} {r["status"]}  ground rms {r["ground_rms_m"] * 1000:.1f} mm  '
                  f'x={m["x_m"]:.3f} y={m["y_m"]:.3f} z={m["z_m"]:.3f} yaw={m["yaw_deg"]:.1f} '
                  f'pitch_down={m["pitch_down_deg"]:.1f} roll={m["roll_deg"]:.1f}  '
                  f'(CAD delta {r["offset_from_nominal_m"] * 100:.1f} cm / {r["angle_from_nominal_deg"]:.1f} deg)')
        else:
            print(f'  {role:10s} FAIL')
    print(f'  seams: {doc["seam_error_m"]}')
    for p in problems:
        print(f'  ! {p}')
    if ipm:
        print(f'[calib] check {ipm}: red = true inner-corner rectangle of each board')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
