"""Calibration step 7 - servo centre + steering limits (calibration_steps.yaml id servo_steering).

Runs with calibrate.launch.py up. The car DRIVES ITSELF slowly (through the
command owner, CALIBRATION_RAW): keep the e-stop ready and give it a clear,
flat floor of about 1.5 x 2.5 m. Tests, in order:

  circles   full LEFT lock, then full RIGHT lock, until the IMU has turned
            circle_yaw_deg -> radius each side -> command_owner.steering
            left/right_max_rad and vehicle.min_turning_radius_m (also copied
            to tunnel_bridge.command_owner_steering). A lock that turns the
            wrong way flips steering.steer_sign.
  straight  angular.z = 0 for straight_run_m: residual curvature -> integer
            servo_controller.servo_center correction (set LIVE), repeated until
            the drift per metre passes (max_runs).

  ros2 run carbot_control calib_steering
  ros2 run carbot_control calib_steering --replay <session>/captures/steering.json
"""
import argparse
import json
import math
import os

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs

from . import calib_core as cc

STEP_ID = 'servo_steering'
STEERING_KEYS = ('steer_sign', 'left_max_rad', 'right_max_rad', 'trim_rad', 'angular_limit')


def analyse(cap: dict, cfg: dict, wheelbase: float) -> dict:
    left = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], wheelbase)
    right = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], wheelbase)
    runs = [cc.straight_result(r['distance'], r['yaw']) for r in cap.get('straight', [])]
    last = runs[-1] if runs else None
    summary = cc.steering_summary(left, right, last, cfg['pass'])
    return {'left': left.as_dict(), 'right': right.as_dict(), 'straight_runs': [r.as_dict() for r in runs],
            **summary}


def steering_params(res: dict, steer_sign: float) -> dict:
    """command_owner.steering.* from an analyse() result (also used by the phase-8 wizard page)."""
    return {'steer_sign': float(steer_sign), 'left_max_rad': round(res['left_max_rad'], 4),
            'right_max_rad': round(res['right_max_rad'], 4), 'trim_rad': 0.0, 'angular_limit': 1.0}


# params_overlay entries this step writes: (node, parameter) -- calibration_steps.yaml `writes`
OVERLAY_KEYS = ([('servo_controller', 'servo_center')]
                + [('command_owner', f'steering.{k}') for k in STEERING_KEYS]
                + [('tunnel_bridge', f'command_owner_steering.{k}') for k in STEERING_KEYS]
                + [('/**', 'vehicle.min_turning_radius_m')])


def write_overlay(session: str, res: dict, steering: dict, servo_center: int) -> str:
    """Write the step's params_overlay.yaml entries (CLI and wizard page alike)."""
    ct.merge_overlay(session, 'servo_controller', {'servo_center': int(servo_center)})
    ct.merge_overlay(session, 'command_owner', {f'steering.{k}': v for k, v in steering.items()})
    ct.merge_overlay(session, 'tunnel_bridge', {f'command_owner_steering.{k}': v for k, v in steering.items()})
    return ct.merge_overlay(session, '/**', {'vehicle.min_turning_radius_m': round(res['min_turning_radius_m'], 4)})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ct.common_args(ap)
    ap.add_argument('--replay', default='', help='capture JSON from an earlier run (no ROS)')
    ap.add_argument('--wheelbase', type=float, default=0.0, help='default: common.yaml vehicle.wheelbase_m')
    a = ap.parse_args(argv)
    cfg_dir = ct.bringup_config_dir(a.config_dir)
    cfg = ct.step_cfg(cfg_dir, STEP_ID)
    proc = cfg['procedure']
    common = ct.load_yaml(os.path.join(cfg_dir, 'params', 'common.yaml'))['/**']['ros__parameters']
    wb = a.wheelbase or float(common['vehicle']['wheelbase_m'])
    root = cs.data_root(a.data_root)
    session = ct.open_session_from_args(a)

    if a.replay:
        cap = json.load(open(a.replay))
    else:
        cap = capture(a, proc, cfg, wb, session)
        if cap is None:
            return 1
        with open(ct.capture_path(session, 'steering.json'), 'w') as f:
            json.dump({k: v for k, v in cap.items() if k != 'rows'}, f, indent=1)
    res = analyse(cap, cfg, wb)
    steer_sign = float(cap.get('steer_sign', -1.0))
    servo_center = int(cap.get('servo_center', 90))
    for k, v in res['checks'].items():
        print(f'  {k:10s} {"PASS" if v else "FAIL"}')
    print(f'  radius L {res["left"]["radius_m"]:.3f} m  R {res["right"]["radius_m"]:.3f} m  '
          f'-> min_turning_radius {res["min_turning_radius_m"]:.3f} m; servo_center {servo_center}')
    steering = steering_params(res, steer_sign)
    write_overlay(session, res, steering, servo_center)
    doc = dict(res, step=STEP_ID, wheelbase_m=wb, servo_center=servo_center, steering=steering)
    fname = ct.write_step(session, int(cfg['index']), STEP_ID, cc.plain(doc))
    ct.finish(session, root, STEP_ID, bool(res['passed']), fname, a.activate)
    return 0 if res['passed'] else 2


def capture(a, proc, cfg, wb, session):
    import rclpy
    from .calib_drive import Driver
    rclpy.init()
    drv = Driver(rclpy, 'carbot_calib_steering')
    owner = ct.RemoteParams(drv.param_node, 'command_owner')
    servo = ct.RemoteParams(drv.param_node, 'servo_controller')
    try:
        mode = (owner.get(['mode']) or {}).get('mode')
        if mode != 'calibrate':
            print('command_owner is not in calibrate mode: start  ros2 launch carbot_bringup calibrate.launch.py')
            return None
        sc = servo.get(['servo_center', 'servo_range_left', 'servo_range_right']) or {}
        centre = int(sc.get('servo_center') or 90)
        own = owner.get(['steering.steer_sign']) or {}
        sign = float(own.get('steering.steer_sign') or -1.0)
        duty = float(proc['raw_duty'])
        target = math.radians(float(proc['circle_yaw_deg']))
        cap = {'servo_center': centre, 'steer_sign': sign}
        ct.pause('Car on a clear flat floor (about 1.5 x 2.5 m free), e-stop ready. The car will drive a '
                 'LEFT circle', a.yes)
        for side in ('left', 'right'):
            # base angular.z for a physical LEFT lock = steer_sign (default -1: negative = left)
            z = sign if side == 'left' else -sign
            r = drv.run('CALIBRATION_RAW', duty, z, f'step 7 {side} lock',
                        lambda d, y, el: abs(y) >= target or abs(d) >= float(proc['circle_max_distance_m']),
                        float(proc['timeout_s']))
            if r['aborted']:
                print(f'[{side}] aborted: {r["aborted"]}')
                return None
            cap[side] = {'distance': r['distance'], 'yaw': r['yaw']}
            res = cc.circle_result(side, r['distance'], r['yaw'], wb)
            print(f'[{side}] {r["distance"]:.2f} m, {math.degrees(r["yaw"]):.0f} deg -> R {res.radius_m:.3f} m '
                  f'({"ok" if res.turned_correct_way else "WRONG WAY"})')
            if side == 'right':
                ct.pause('Turn the car around if needed. Next: STRAIGHT run', a.yes)
        L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], wb)
        R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], wb)
        if not L.turned_correct_way and not R.turned_correct_way:
            sign = -sign
            cap['steer_sign'] = sign
            for s in ('left', 'right'):
                cap[s]['yaw'] = -cap[s]['yaw']
            print(f'steering reversed: steer_sign -> {sign}')
            L = cc.circle_result('left', cap['left']['distance'], cap['left']['yaw'], wb)
            R = cc.circle_result('right', cap['right']['distance'], cap['right']['yaw'], wb)
        kl = (1.0 / L.radius_m) / max(float(sc.get('servo_range_left') or 50), 1)
        kr = (1.0 / R.radius_m) / max(float(sc.get('servo_range_right') or 70), 1)
        cap['straight'] = []
        for run in range(int(proc['max_runs'])):
            ct.pause(f'Straight run {run + 1}: line the car up with {float(proc["straight_run_m"]):.1f} m '
                     'free ahead', a.yes)
            r = drv.run('CALIBRATION_RAW', duty, 0.0, 'step 7 straight',
                        lambda d, y, el: abs(d) >= float(proc['straight_run_m']), float(proc['timeout_s']))
            if r['aborted']:
                print(f'[straight] aborted: {r["aborted"]}')
                return None
            cap['straight'].append({'distance': r['distance'], 'yaw': r['yaw'], 'servo_center': centre})
            s = cc.straight_result(r['distance'], r['yaw'])
            print(f'[straight] drift {s.drift_m_per_m * 100:.2f} cm/m (limit '
                  f'{float(cfg["pass"]["straight_drift_m_per_m"]) * 100:.1f})')
            if s.drift_m_per_m <= float(cfg['pass']['straight_drift_m_per_m']):
                break
            du = cc.centre_correction(s.curvature, kl, kr)
            if du == 0:
                print('[straight] below one servo unit: cannot correct further')
                break
            centre += du
            servo.set({'servo_center': centre})
            print(f'[straight] servo_center -> {centre} (set live)')
        cap['servo_center'] = centre
        return cap
    finally:
        drv.close()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
