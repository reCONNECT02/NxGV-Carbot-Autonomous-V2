"""Calibration step 6 - IMU + wheel odometry (calibration_steps.yaml id imu_odometry).

Runs with calibrate.launch.py up (servo_controller publishing /odom, /imu/rpy).
Three tests, in order:

  distance  Tape two marks straight_run_m apart. Rear axle on the first mark,
            push the car (or drive slowly) straight to the second. Compares
            the /odom distance with the tape.
            * negative distance -> odom_reverse_polarity is toggled
            * error > max_distance_error_pct -> ticks_per_meter is corrected
            Corrections are set LIVE on servo_controller and written to the
            session overlay; the test repeats until a run passes (verify run).
  drift     Car still for drift_test_s: IMU yaw drift must be below
            max_heading_drift_deg_per_min.
  spin      Turn the car by hand one full turn to the LEFT (counter-clockwise
            seen from above) back to a tape mark. The IMU yaw must change by
            +360 deg: a negative change flips the sign of imu_yaw_scale, a
            wrong magnitude rescales it (block 05 needs yaw increasing to the left).

  ros2 run carbot_localization calib_odometry                 # all three
  ros2 run carbot_localization calib_odometry --tests spin    # just one
"""
import argparse
import json
import math
import os
import threading
import time

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_common import topics as T

from .estimator_core import odom_increment, wrap

STEP_ID = 'imu_odometry'
SERVO = 'servo_controller'


class Recorder:
    """Background subscriber: signed wheel distance and unwrapped IMU yaw."""

    def __init__(self, rclpy):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.node = rclpy.create_node('carbot_calib_odometry')
        self.lock = threading.Lock()
        self.prev = None
        self.dist = 0.0
        self.odom_n = 0
        self.yaw_prev = None
        self.yaw_unwrapped = 0.0
        self.imu_n = 0
        self.node.create_subscription(Odometry, T.ODOM, self._odom, qos_profile_sensor_data)
        self.node.create_subscription(String, T.IMU_RPY, self._imu, qos_profile_sensor_data)
        self._stop = False
        self.th = threading.Thread(target=self._spin, args=(rclpy,), daemon=True)
        self.th.start()

    def _spin(self, rclpy):
        while not self._stop and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _odom(self, m):
        q = m.pose.pose
        o = q.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        cur = (q.position.x, q.position.y, yaw)
        with self.lock:
            ds, _ = odom_increment(self.prev, cur)
            self.prev = cur
            self.dist += ds
            self.odom_n += 1

    def _imu(self, m):
        try:
            y = math.radians(float(json.loads(m.data)['yaw']))
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if self.yaw_prev is not None:
                self.yaw_unwrapped += wrap(y - self.yaw_prev)
            self.yaw_prev = y
            self.imu_n += 1

    def zero(self):
        with self.lock:
            self.dist = 0.0
            self.yaw_unwrapped = 0.0

    def read(self):
        with self.lock:
            return self.dist, self.yaw_unwrapped, self.odom_n, self.imu_n

    def close(self):
        self._stop = True
        self.th.join(timeout=1.0)
        self.node.destroy_node()


def test_distance(rec, params, cfg, a, results):
    true_m = float(a.true_m or cfg['procedure']['straight_run_m'])
    limit = float(cfg['pass']['max_distance_error_pct'])
    cur = params.get(['ticks_per_meter', 'odom_reverse_polarity']) or {}
    tpm = float(cur.get('ticks_per_meter') or 0.0)
    pol = bool(cur.get('odom_reverse_polarity'))
    if tpm <= 0:
        print('[distance] cannot read servo_controller parameters: is calibrate.launch.py running?')
        results['distance'] = {'status': 'FAIL', 'reason': 'servo_controller not reachable'}
        return False
    runs = []
    for attempt in range(1, int(a.max_runs) + 1):
        print(f'\n[distance] run {attempt}: ticks_per_meter={tpm:.1f} reverse_polarity={pol}')
        ct.pause(f'Put the REAR AXLE on the first tape mark (marks {true_m:.2f} m apart)', a.yes)
        rec.zero()
        n0 = rec.read()[2]
        ct.pause('Now push the car STRAIGHT FORWARD until the rear axle is on the second mark', a.yes)
        d, _, n1, _ = rec.read()
        if n1 - n0 < 5:
            print('[distance] no /odom received during the run')
            runs.append({'odom_m': d, 'status': 'NO_ODOM'})
            break
        err = (d - true_m) / true_m * 100.0
        runs.append({'odom_m': round(d, 4), 'true_m': true_m, 'error_pct': round(err, 2),
                     'ticks_per_meter': tpm, 'reverse_polarity': pol})
        print(f'[distance] odom {d:.3f} m vs tape {true_m:.3f} m -> error {err:+.1f} %')
        if d < 0:
            pol = not pol
            print(f'[distance] distance is NEGATIVE: setting odom_reverse_polarity={pol}; redo the run')
            params.set({'odom_reverse_polarity': pol})
            continue
        if abs(err) <= limit:
            results['distance'] = {'status': 'PASS', 'runs': runs, 'ticks_per_meter': tpm,
                                   'odom_reverse_polarity': pol}
            return True
        if abs(d) < 0.2 * true_m:
            print('[distance] far too short: wheel not turning, wrong drive_motor_index or car lifted?')
            continue
        tpm = tpm * d / true_m
        print(f'[distance] correcting ticks_per_meter -> {tpm:.1f}; redo the run to VERIFY')
        params.set({'ticks_per_meter': tpm})
    results['distance'] = {'status': 'FAIL', 'runs': runs, 'ticks_per_meter': tpm,
                           'odom_reverse_polarity': pol}
    return False


def test_drift(rec, cfg, a, results):
    secs = float(a.drift_s or cfg['procedure']['drift_test_s'])
    limit = float(cfg['pass']['max_heading_drift_deg_per_min'])
    ct.pause(f'\n[drift] Leave the car completely STILL for {secs:.0f} s (hands off)', a.yes)
    rec.zero()
    n0 = rec.read()[3]
    time.sleep(secs)
    _, yaw, _, n1 = rec.read()
    if n1 - n0 < secs * 5:
        results['drift'] = {'status': 'FAIL', 'reason': f'/imu/rpy only {n1 - n0} messages'}
        print('[drift] /imu/rpy is not publishing')
        return False
    rate = math.degrees(yaw) / secs * 60.0
    ok = abs(rate) <= limit
    results['drift'] = {'status': 'PASS' if ok else 'FAIL', 'seconds': secs,
                        'drift_deg_per_min': round(rate, 3), 'limit': limit}
    print(f'[drift] yaw drift {rate:+.2f} deg/min (limit {limit}) -> {"PASS" if ok else "FAIL"}')
    if not ok:
        print('[drift] keep the car still and away from motors/magnets; run the IMU hardware '
              f'calibration (ros2 topic pub --once {T.IMU_CALIBRATE} std_msgs/String) and redo')
    return ok


def test_spin(rec, params, cfg, a, results):
    limit = float(cfg['pass']['max_spin_error_pct'])
    cur = params.get(['imu_yaw_scale']) or {}
    scale = float(cur.get('imu_yaw_scale') or 1.0)
    runs = []
    for attempt in range(1, int(a.max_runs) + 1):
        print(f'\n[spin] run {attempt}: imu_yaw_scale={scale:+.4f}')
        ct.pause('Line the car up with a tape mark on the floor', a.yes)
        rec.zero()
        ct.pause('Turn the car by hand ONE FULL TURN to the LEFT (counter-clockwise seen from '
                 'above), slowly, back onto the mark', a.yes)
        _, yaw, _, _ = rec.read()
        deg = math.degrees(yaw)
        runs.append({'yaw_change_deg': round(deg, 2), 'imu_yaw_scale': scale})
        print(f'[spin] IMU yaw changed {deg:+.1f} deg (expected +360)')
        if abs(deg) < 180:
            print('[spin] less than half a turn measured: IMU not updating or turn not completed')
            continue
        err = (abs(deg) - 360.0) / 360.0 * 100.0
        if deg > 0 and abs(err) <= limit:
            results['spin'] = {'status': 'PASS', 'runs': runs, 'imu_yaw_scale': scale}
            return True
        scale = scale * 360.0 / deg
        why = 'sign flipped (yaw decreased turning left)' if deg < 0 else f'scale error {err:+.1f} %'
        print(f'[spin] {why}: imu_yaw_scale -> {scale:+.4f}; redo to VERIFY')
        params.set({'imu_yaw_scale': scale})
    results['spin'] = {'status': 'FAIL', 'runs': runs, 'imu_yaw_scale': scale}
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ct.common_args(ap)
    ap.add_argument('--tests', default='distance,drift,spin')
    ap.add_argument('--true-m', type=float, default=0.0, help='tape distance (default: steps yaml)')
    ap.add_argument('--drift-s', type=float, default=0.0)
    ap.add_argument('--max-runs', type=int, default=4)
    a = ap.parse_args(argv)
    config_dir = ct.bringup_config_dir(a.config_dir)
    root = cs.data_root(a.data_root)
    cfg = ct.step_cfg(config_dir, STEP_ID)
    session = ct.open_session_from_args(a)
    print(f'[{STEP_ID}] session {session}')

    import rclpy
    rclpy.init()
    rec = Recorder(rclpy)
    helper = rclpy.create_node('carbot_calib_odometry_params')
    params = ct.RemoteParams(helper, SERVO)
    time.sleep(1.0)
    results = {}
    tests = [t.strip() for t in a.tests.split(',') if t.strip()]
    ok = True
    try:
        if 'distance' in tests:
            ok &= test_distance(rec, params, cfg, a, results)
        if 'drift' in tests:
            ok &= test_drift(rec, cfg, a, results)
        if 'spin' in tests:
            ok &= test_spin(rec, params, cfg, a, results)
    finally:
        rec.close()
        helper.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # A partial run keeps earlier results of the other tests in this session.
    fname = f'{int(cfg["index"]):02d}_{STEP_ID}.yaml'
    path = os.path.join(session, fname)
    prev = ct.load_yaml(path) if os.path.isfile(path) else {}
    merged = dict(prev.get('tests') or {})
    merged.update(results)
    overlay = {}
    if 'distance' in results:
        overlay['ticks_per_meter'] = float(results['distance']['ticks_per_meter'])
        overlay['odom_reverse_polarity'] = bool(results['distance']['odom_reverse_polarity'])
    if 'spin' in results:
        overlay['imu_yaw_scale'] = float(results['spin']['imu_yaw_scale'])
    if overlay:
        ct.merge_overlay(session, SERVO, overlay)
    passed = all((merged.get(t) or {}).get('status') == 'PASS' for t in ('distance', 'drift', 'spin'))
    ct.write_step(session, int(cfg['index']), STEP_ID, {'step': STEP_ID, 'tests': merged,
                                                        'overlay': {SERVO: overlay}})
    missing = [t for t in ('distance', 'drift', 'spin') if t not in merged]
    ct.finish(session, root, STEP_ID, passed, fname, a.activate,
              detail='missing: ' + ','.join(missing) if missing else '')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
