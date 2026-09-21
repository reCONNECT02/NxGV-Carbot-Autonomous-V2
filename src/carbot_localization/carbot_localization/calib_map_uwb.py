"""Calibration step 11 - map-to-UWB alignment (id map_uwb_alignment).

Finds uwb.yaml track_to_venue (p_venue = R(yaw) p_track + [x, y]) so the prior
map (track frame) and the UWB anchors (venue frame) line up. Needs step 10.

  lap     (best) car on the map start pose -> block 05 is reset there -> push or
          drive the car SLOWLY once around the track. Tag positions come from
          block 05 (+ the tag's mount position), ranges from the raw tag JSON with
          THIS session's step-10 offsets (the running uwb_ranges node may still
          hold the old ones).
  points  (quick) park the car on 2+ named map poses far apart, 10 s each.

The fit (alignment.py) uses the ranges directly with a robust loss.

  ros2 run carbot_localization calib_map_uwb                       # lap
  ros2 run carbot_localization calib_map_uwb --mode points --points start_pose light_goal_pose
  ros2 run carbot_localization calib_map_uwb --replay <session>/captures
"""
import argparse
import math
import os
import threading
import time
from typing import Dict, List, Tuple

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_common import topics as T

from .alignment import Sample, fit_track_to_venue, tag_position
from .estimator_core import PoseHistory

STEP_ID = 'map_uwb_alignment'


def named_poses(config_dir: str) -> Dict[str, Tuple[float, float, float]]:
    """start_pose, light_goal_pose and every mission checkpoint with x/y/a."""
    tm = ct.load_yaml(os.path.join(config_dir, 'data', 'track_map.yaml'))
    mi = ct.load_yaml(os.path.join(config_dir, 'data', 'mission.yaml'))
    out = {}
    for k in ('start_pose', 'light_goal_pose'):
        p = tm[k]
        out[k] = (float(p['x']), float(p['y']), float(p['a']))
    for leg in mi.get('legs', []):
        for c in leg.get('checkpoints') or []:
            if 'x' in c:
                out.setdefault(c['name'], (float(c['x']), float(c['y']), float(c['a'])))
    return out


class Recorder:
    """Raw tag JSON + block-05 local pose, both stamped with wall time on receipt."""

    def __init__(self, want_pose: bool):
        import rclpy
        from carbot_common.qos import UWB
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('carbot_calib_map_uwb')
        self.lock = threading.Lock()
        self.uwb: List[Dict] = []
        self.pose: List[Dict] = []
        self.recording = False
        self.node.create_subscription(String, T.UWB_INPUT_JSON, self._uwb, UWB)
        if want_pose:
            self.node.create_subscription(Odometry, T.LOCAL_POSE, self._pose, 10)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.reset_pub = self.node.create_publisher(PoseWithCovarianceStamped, T.LOCALIZATION_RESET, 10)
        self._stop = False
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while not self._stop and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def _uwb(self, m):
        if self.recording:
            with self.lock:
                self.uwb.append({'t': time.time(), 'json': m.data})

    def _pose(self, m):
        if self.recording:
            q = m.pose.pose
            o = q.orientation
            a = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            with self.lock:
                self.pose.append({'t': time.time(), 'x': q.position.x, 'y': q.position.y, 'a': a})

    def reset_to(self, pose):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'track'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        for _ in range(3):
            self.reset_pub.publish(m)
            time.sleep(0.1)

    def take(self):
        with self.lock:
            u, p = self.uwb, self.pose
            self.uwb, self.pose = [], []
            return u, p

    def close(self):
        self._stop = True
        self.th.join(timeout=1.0)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# --------------------------------------------------------------------------- analysis (pure)
def ranges_from_rows(anchors, rows):
    """Raw JSON rows -> [(stamp, anchor, corrected flattened range)] using the
    session's anchors/offsets, fresh samples only, latency-compensated."""
    from uwb_localization.uwb_core import RangeProcessor, parse_report
    proc = RangeProcessor(anchors, 400, 0.05, 30.0, 'min_filter')
    out = []
    for r in rows:
        rep = parse_report(r['json'])
        if rep is None:
            continue
        for x in proc.process(rep, r['t']).ranges:
            if x.fresh:
                out.append((x.stamp, x.anchor, x.corrected_m))
    return out


def lap_samples(anchors, uwb_rows, pose_rows, lever) -> List[Sample]:
    hist = PoseHistory(1e9)
    for p in pose_rows:
        hist.add(p['t'], p['x'], p['y'], p['a'])
    out = []
    for t, aid, r in ranges_from_rows(anchors, uwb_rows):
        loc = hist.at(t, 0.2)
        if loc is None:
            continue
        tx, ty = tag_position(loc, lever)
        out.append(Sample(tx, ty, aid, r))
    return out


def point_samples(anchors, uwb_rows, pose, lever) -> List[Sample]:
    tx, ty = tag_position(pose, lever)
    return [Sample(tx, ty, aid, r) for _, aid, r in ranges_from_rows(anchors, uwb_rows)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ct.common_args(ap)
    ap.add_argument('--mode', choices=['lap', 'points'], default='lap')
    ap.add_argument('--points', nargs='*', default=[], help='named map poses (points mode)')
    ap.add_argument('--seconds', type=float, default=0.0, help='lap: stop after N s instead of Enter')
    ap.add_argument('--replay', default='', help='captures folder of an earlier run (no ROS needed)')
    a = ap.parse_args(argv)
    config_dir = ct.bringup_config_dir(a.config_dir)
    root = cs.data_root(a.data_root)
    cfg = ct.step_cfg(config_dir, STEP_ID)
    pr, ps = cfg['procedure'], cfg['pass']
    session = ct.open_session_from_args(a)
    print(f'[{STEP_ID}] session {session}  mode {a.mode}')

    from uwb_localization.uwb_core import AnchorSet
    doc = ct.effective_data(session, config_dir, root, 'uwb.yaml')
    if not doc.get('offsets_calibrated'):
        print('WARN: uwb.yaml offsets_calibrated=false (step 10 not passed): ranges read ~1 m long, '
              'the fit will be poor')
    if not a.replay and ct.check_uwb_env(doc):
        return 2
    anchors = AnchorSet.from_yaml(doc)
    lever = tuple(float(v) for v in doc['tag'].get('mount_xy_m', [0.0, 0.0]))
    poses = named_poses(config_dir)
    capdir = a.replay or os.path.dirname(ct.capture_path(session, 'x'))
    samples: List[Sample] = []
    info: Dict = {'mode': a.mode}

    rec = None if a.replay else Recorder(want_pose=(a.mode == 'lap'))
    try:
        if a.mode == 'lap':
            f_uwb, f_pose = os.path.join(capdir, 'step11_lap_uwb.jsonl'), os.path.join(capdir, 'step11_lap_pose.jsonl')
            if a.replay:
                uwb_rows, pose_rows = ct.read_jsonl(f_uwb), ct.read_jsonl(f_pose)
            else:
                start = poses['start_pose']
                ct.pause(f'Put the car on the map START pose ({start[0]:.2f}, {start[1]:.2f}, '
                         f'{math.degrees(start[2]):.0f} deg): bottom-right lane, facing west', a.yes)
                rec.reset_to(start)
                time.sleep(0.5)
                rec.take()
                rec.recording = True
                if a.seconds:
                    print(f'Recording {a.seconds:.0f} s: push / drive SLOWLY around the whole track')
                    time.sleep(a.seconds)
                else:
                    ct.pause('Recording: push / drive SLOWLY once around the whole track, then', a.yes)
                rec.recording = False
                uwb_rows, pose_rows = rec.take()
                ct.write_jsonl(f_uwb, uwb_rows)
                ct.write_jsonl(f_pose, pose_rows)
            dur = (pose_rows[-1]['t'] - pose_rows[0]['t']) if len(pose_rows) > 1 else 0.0
            info.update({'duration_s': round(dur, 1), 'uwb_reports': len(uwb_rows), 'poses': len(pose_rows)})
            if not pose_rows:
                print('No local pose received: is calibrate.launch.py running (local_pose node)?')
            samples = lap_samples(anchors, uwb_rows, pose_rows, lever)
            if dur < float(pr['lap_min_s']):
                print(f'WARN: lap took only {dur:.0f} s (expected >= {pr["lap_min_s"]} s): too fast or cut short')
        else:
            names = a.points or []
            if not names and not a.yes:
                print('Known poses:', ', '.join(sorted(poses)))
                names = ct.ask('Pose names to use (space separated)', 'start_pose light_goal_pose').split()
            names = names or ['start_pose', 'light_goal_pose']
            used = []
            for n in names:
                if n not in poses:
                    print(f'unknown pose {n}; known: {sorted(poses)}')
                    continue
                f_uwb = os.path.join(capdir, f'step11_point_{n}.jsonl')
                if a.replay:
                    rows = ct.read_jsonl(f_uwb)
                else:
                    p = poses[n]
                    ct.pause(f'Park the car EXACTLY on {n} ({p[0]:.2f}, {p[1]:.2f}, '
                             f'{math.degrees(p[2]):.0f} deg)', a.yes)
                    rec.take()
                    rec.recording = True
                    time.sleep(float(pr['points_seconds']))
                    rec.recording = False
                    rows, _ = rec.take()
                    ct.write_jsonl(f_uwb, rows)
                s = point_samples(anchors, rows, poses[n], lever)
                print(f'  {n}: {len(s)} ranges')
                samples += s
                used.append(n)
            info['points'] = used
            if len(used) < int(pr['min_points']):
                print(f'need at least {pr["min_points"]} poses')
    finally:
        if rec:
            rec.close()

    result = {'step': STEP_ID, 'capture': info}
    passed = False
    try:
        fit = fit_track_to_venue(samples, {k: (v.x, v.y) for k, v in anchors.anchors.items()},
                                 float(pr['huber_m']), float(pr['inlier_m']))
        min_ext = float(pr['min_extent_m']) if a.mode == 'lap' else 1.5
        ok_ext = fit.track_extent_m >= min_ext
        passed = fit.rms_m <= float(ps['max_rms_m']) and fit.inlier_frac >= float(ps['min_inlier_frac']) \
            and ok_ext and (a.mode == 'lap' or len(info.get('points', [])) >= int(pr['min_points']))
        print(f'\nfit: x {fit.x:+.3f} m  y {fit.y:+.3f} m  yaw {math.degrees(fit.yaw):+.2f} deg\n'
              f'     range RMS {fit.rms_m * 100:.1f} cm (limit {float(ps["max_rms_m"]) * 100:.0f}), '
              f'inliers {fit.inlier_frac * 100:.0f} % (min {float(ps["min_inlier_frac"]) * 100:.0f}), '
              f'{fit.n} ranges over {fit.track_extent_m:.1f} m (min {min_ext})')
        if not ok_ext:
            print('     the samples cover too small an area: the yaw is not determined')
        result['fit'] = {'x_m': round(fit.x, 4), 'y_m': round(fit.y, 4),
                         'yaw_deg': round(math.degrees(fit.yaw), 3), 'rms_m': round(fit.rms_m, 4),
                         'inlier_frac': round(fit.inlier_frac, 3), 'ranges': fit.n,
                         'extent_m': round(fit.track_extent_m, 2), 'median_abs_m': round(fit.median_abs_m, 4)}
        if passed:
            doc['track_to_venue'] = {'x_m': round(fit.x, 4), 'y_m': round(fit.y, 4),
                                     'yaw_deg': round(math.degrees(fit.yaw), 3), 'aligned': True}
            print(f'wrote {ct.save_data(session, "uwb.yaml", doc)}')
    except ValueError as e:
        print(f'fit failed: {e}')
        result['fit'] = {'error': str(e)}
    fname = ct.write_step(session, int(cfg['index']), STEP_ID, result)
    ct.finish(session, root, STEP_ID, passed, fname, a.activate)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
