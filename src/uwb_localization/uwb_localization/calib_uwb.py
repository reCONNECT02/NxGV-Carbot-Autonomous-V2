"""Calibration step 10 - UWB anchor survey + per-anchor offsets (id uwb_survey).

Replaces running tools/uwb/uwb_calib.py by hand (same method: offset =
median(raw 3-D range) - tape-measured 3-D distance) and writes the result into
the calibration session's data/uwb.yaml instead of into a script.

Stages (all in one run; the wizard runs the same functions):
  1 layout   enter / confirm anchor positions + heights (metres), tag height and
             the tag's position on the car. Checks spacing, collinearity, anchors
             on the floor, and position accuracy (HDOP) over the track.
  2 link     5 s listen: every anchor in uwb.yaml must be heard; an ID heard that
             is not in the list is printed (add it to RangeProtocol.h + uwb.yaml).
  3 offsets  tag still at a measured spot (> 1 m from every anchor), 20 s.
  4 verify   tag at a DIFFERENT measured spot: corrected fix within 15 cm; also
             reports the stationary flip-flop (multipath) distance.

Environment: the micro-ROS agent must be running (calibrate.launch.py starts it),
ROS_DOMAIN_ID=1, ROS_LOCALHOST_ONLY=0.

  ros2 run uwb_localization calib_uwb
  ros2 run uwb_localization calib_uwb --spot 5.0 1.5 --verify 3.5 2.5 --yes
  ros2 run uwb_localization calib_uwb --replay <session>/captures   # re-evaluate offline
"""
import argparse
import copy
import math
import os
import statistics
import threading
import time
from typing import Dict, List, Tuple

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs
from carbot_common import topics as T

from .uwb_core import (AnchorSet, RangeProcessor, compute_offsets, fix_clusters, hdop_coverage,
                       layout_checks, parse_report, trilaterate)

STEP_ID = 'uwb_survey'


class Listener:
    """Background BEST_EFFORT subscriber keeping every raw tag message."""

    def __init__(self):
        import rclpy
        from carbot_common.qos import UWB
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('carbot_calib_uwb')
        self.lock = threading.Lock()
        self.buf: List[Dict] = []
        self.node.create_subscription(String, T.UWB_INPUT_JSON, self._cb, UWB)
        self._stop = False
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while not self._stop and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def _cb(self, msg):
        with self.lock:
            self.buf.append({'t': time.time(), 'json': msg.data})

    def collect(self, seconds: float) -> List[Dict]:
        with self.lock:
            self.buf = []
        end = time.time() + seconds
        while time.time() < end:
            time.sleep(0.2)
            left = int(end - time.time())
            print(f'\r  collecting... {left:3d} s  ({len(self.buf)} reports)', end='', flush=True)
        print()
        with self.lock:
            return list(self.buf)

    def close(self):
        self._stop = True
        self.th.join(timeout=1.0)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# --------------------------------------------------------------------------- analysis (pure)
def raw_fresh_samples(rows: List[Dict]) -> Tuple[Dict[str, List[float]], List[str], int]:
    """Raw R per anchor, one per NEW sample_seq. -> (samples, unknown ids, reports)."""
    out: Dict[str, List[float]] = {}
    last: Dict[str, int] = {}
    unknown = set()
    n = 0
    boot = None
    for r in rows:
        rep = parse_report(r['json'])
        if rep is None:
            continue
        n += 1
        if rep.boot_id != boot:
            boot, last = rep.boot_id, {}
        if rep.last_unknown_id not in ('0000', ''):
            unknown.add(rep.last_unknown_id)
        for ln in rep.links:
            if last.get(ln.anchor) == ln.sample_seq:
                continue
            last[ln.anchor] = ln.sample_seq
            out.setdefault(ln.anchor, []).append(ln.r)
    return out, sorted(unknown), n


def fixes(anchors: AnchorSet, rows: List[Dict]) -> List[Tuple[float, float]]:
    proc = RangeProcessor(anchors, 400, 0.05, 30.0, 'arrival')
    out = []
    for r in rows:
        rep = parse_report(r['json'])
        if rep is None:
            continue
        res = proc.process(rep, r['t'])
        rng = {x.anchor: x.corrected_m for x in res.ranges if x.reason in ('', 'repeat')}
        p = trilaterate(anchors, rng)
        if p is not None:
            out.append(p)
    return out


def venue_points(doc: Dict, anchors: AnchorSet, config_dir: str) -> Tuple[List[Tuple[float, float]], str]:
    """Where accuracy matters: the track (if aligned) else the anchors' bounding box."""
    t = doc.get('track_to_venue') or {}
    if t.get('aligned'):
        try:
            from carbot_common.course import load_course
            import numpy as np
            course = load_course(os.path.join(config_dir, 'data', 'track_map.yaml'))
            c, s = math.cos(math.radians(t['yaw_deg'])), math.sin(math.radians(t['yaw_deg']))
            pts = np.concatenate(course.paths)[::4]
            return [(c * x - s * y + t['x_m'], s * x + c * y + t['y_m']) for x, y in pts], 'track centrelines'
        except Exception:  # noqa: BLE001
            pass
    xs = [a.x for a in anchors.anchors.values()]
    ys = [a.y for a in anchors.anchors.values()]
    pts = [(xs_ + 0.0, ys_ + 0.0)
           for xs_ in [min(xs) + (max(xs) - min(xs)) * i / 10 for i in range(1, 10)]
           for ys_ in [min(ys) + (max(ys) - min(ys)) * j / 10 for j in range(1, 10)]]
    return pts, 'anchor bounding box (track not aligned yet)'


# --------------------------------------------------------------------------- stages
def stage_layout(doc: Dict, a, proc_cfg) -> Dict:
    print('\n=== 1. Anchor layout (metres; venue origin = anchor 1782) ===')
    anchors = {str(x['id']).upper(): x for x in doc['anchors']}
    for spec in a.anchor or []:
        aid, xyz = spec.split(':')
        anchors[aid.upper()] = {'id': aid.upper(), 'xyz_m': [float(v) for v in xyz.split(',')],
                                'range_offset_m': 0.0}
    for aid in (a.add_anchor or []):
        anchors.setdefault(aid.upper(), {'id': aid.upper(), 'xyz_m': [0.0, 0.0, 0.0],
                                         'range_offset_m': 0.0})
        print(f'  added anchor {aid.upper()} (remember RangeProtocol.h ANCHOR_IDS / ANCHOR_COUNT)')
    for aid in sorted(anchors):
        cur = anchors[aid]['xyz_m']
        v = ct.ask(f'  anchor {aid} x y z', f'{cur[0]:.3f} {cur[1]:.3f} {cur[2]:.3f}', a.yes)
        anchors[aid]['xyz_m'] = [float(t) for t in v.replace(',', ' ').split()]
    tag = doc['tag']
    tag['z_m'] = float(ct.ask('  tag antenna height above floor z', f'{float(tag["z_m"]):.3f}', a.yes))
    mxy = tag.get('mount_xy_m', [0.0, 0.0])
    v = ct.ask('  tag position on car from rear-axle centre (forward, left)',
               f'{float(mxy[0]):.3f} {float(mxy[1]):.3f}', a.yes)
    tag['mount_xy_m'] = [float(t) for t in v.replace(',', ' ').split()]
    doc['anchors'] = [anchors[k] for k in sorted(anchors)]
    return doc


def check_layout(doc, anchors: AnchorSet, cfg, config_dir) -> Dict:
    pr, ps = cfg['procedure'], cfg['pass']
    problems = layout_checks(anchors, float(pr['min_spacing_m']), float(pr['min_triangle_angle_deg']))
    warnings = [p for p in problems if 'on the floor' in p]
    errors = [p for p in problems if p not in warnings]
    pts, where = venue_points(doc, anchors, config_dir)
    cov, worst = hdop_coverage(anchors, pts, float(pr['hdop_limit']))
    ok = not errors and cov >= float(ps['min_hdop_coverage'])
    for p in errors:
        print('  ERROR:', p)
    for p in warnings:
        print('  WARN :', p)
    print(f'  HDOP <= {pr["hdop_limit"]} over {cov * 100:.0f} % of the {where} (worst {worst:.1f})')
    if cov < float(ps['min_hdop_coverage']):
        print('  -> move anchors so they surround the track, or add a 4th anchor')
    return {'status': 'PASS' if ok else 'FAIL', 'errors': errors, 'warnings': warnings,
            'hdop_coverage': round(cov, 3), 'hdop_worst': round(worst, 2), 'hdop_area': where}


def check_link(rows, anchors: AnchorSet, secs: float) -> Dict:
    samples, unknown, n = raw_fresh_samples(rows)
    seen = {aid: len(samples.get(aid, [])) for aid in anchors.ids}
    missing = [aid for aid, k in seen.items() if k < max(3, secs)]
    print(f'  {n} reports in {secs:.0f} s ({n / max(secs, 1e-6):.1f} Hz); fresh samples per anchor {seen}')
    if unknown:
        print(f'  heard anchor IDs NOT in uwb.yaml: {unknown} -> add to RangeProtocol.h and uwb.yaml')
    if n == 0:
        print('  NOTHING received: agent running? tag powered? ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=1?')
    for aid in missing:
        print(f'  anchor {aid} missing or weak: powered? same radio mode? line of sight?')
    return {'status': 'PASS' if n and not missing else 'FAIL', 'reports': n,
            'fresh_per_anchor': seen, 'unknown_ids': unknown}


def spot_ok(anchors: AnchorSet, x, y, min_d) -> List[str]:
    bad = []
    for aid in anchors.ids:
        d = anchors.true_range_3d(aid, x, y)
        if d < min_d:
            bad.append(f'{aid} ({d:.2f} m)')
    return bad


def analyse_offsets(anchors: AnchorSet, rows, x, y, cfg) -> Tuple[Dict, Dict[str, float]]:
    pr = cfg['procedure']
    samples, _, n = raw_fresh_samples(rows)
    res = compute_offsets(anchors, samples, x, y, int(pr['min_samples']))
    print(f'  {"anchor":>6} {"samples":>7} {"measured":>9} {"true":>7} {"offset":>7} {"spread":>7}   (m)')
    table, off = {}, {}
    for aid in anchors.ids:
        r = res.get(aid)
        if r is None:
            print(f'  {aid:>6} {len(samples.get(aid, [])):>7}   too few samples -- is this anchor on?')
            continue
        flag = '  <- noisy (multipath?)' if r.spread_m > float(pr['max_spread_m']) else ''
        print(f'  {aid:>6} {r.samples:>7} {r.measured_m:9.3f} {r.true_m:7.3f} {r.offset_m:+7.3f} '
              f'{r.spread_m:7.3f}{flag}')
        table[aid] = {'samples': r.samples, 'measured_m': round(r.measured_m, 4),
                      'true_m': round(r.true_m, 4), 'offset_m': round(r.offset_m, 4),
                      'spread_m': round(r.spread_m, 4)}
        off[aid] = r.offset_m
    ok = len(off) == len(anchors.ids)
    return {'status': 'PASS' if ok else 'FAIL', 'spot_m': [x, y], 'reports': n, 'anchors': table}, off


def analyse_verify(anchors: AnchorSet, rows, x, y, cfg) -> Dict:
    fx = fixes(anchors, rows)
    if len(fx) < 10:
        print(f'  only {len(fx)} fixes (need all anchors in a report)')
        return {'status': 'FAIL', 'spot_m': [x, y], 'fixes': len(fx)}
    mx = statistics.median(p[0] for p in fx)
    my = statistics.median(p[1] for p in fx)
    err = math.hypot(mx - x, my - y)
    jitter, flip = fix_clusters(fx)
    lim = float(cfg['pass']['max_verify_error_m'])
    print(f'  median fix ({mx:.3f}, {my:.3f}) vs true ({x:.3f}, {y:.3f}): error {err * 100:.1f} cm '
          f'(limit {lim * 100:.0f}); jitter {jitter * 100:.1f} cm')
    if flip > 0:
        print(f'  WARN: fixes flip between two clusters {flip * 100:.0f} cm apart (multipath): '
              'raise the anchors / clear line of sight')
    return {'status': 'PASS' if err <= lim else 'FAIL', 'spot_m': [x, y], 'fixes': len(fx),
            'median_fix_m': [round(mx, 4), round(my, 4)], 'error_m': round(err, 4),
            'jitter_m': round(jitter, 4), 'flip_flop_m': round(flip, 4)}


def ask_spot(label, default, a):
    v = ct.ask(f'  {label} tag spot x y (m)', f'{default[0]:.2f} {default[1]:.2f}', a.yes)
    x, y = (float(t) for t in v.replace(',', ' ').split())
    return x, y


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ct.common_args(ap)
    ap.add_argument('--anchor', action='append', help='ID:x,y,z (metres), repeatable')
    ap.add_argument('--add-anchor', action='append', help='add a new anchor ID (e.g. a 4th)')
    ap.add_argument('--spot', nargs=2, type=float, help='offset spot x y (m)')
    ap.add_argument('--verify', nargs=2, type=float, help='verify spot x y (m)')
    ap.add_argument('--seconds', type=float, default=0.0)
    ap.add_argument('--replay', default='', help='captures folder of an earlier run (no ROS needed)')
    a = ap.parse_args(argv)
    config_dir = ct.bringup_config_dir(a.config_dir)
    root = cs.data_root(a.data_root)
    cfg = ct.step_cfg(config_dir, STEP_ID)
    pr = cfg['procedure']
    session = ct.open_session_from_args(a)
    print(f'[{STEP_ID}] session {session}')
    secs = float(a.seconds or pr['seconds'])

    doc = ct.effective_data(session, config_dir, root, 'uwb.yaml')
    if not a.replay and ct.check_uwb_env(doc):
        return 2
    doc = stage_layout(copy.deepcopy(doc), a, pr)
    zero = AnchorSet.from_yaml({**doc, 'anchors': [{**x, 'range_offset_m': 0.0} for x in doc['anchors']]})
    result = {'step': STEP_ID}
    result['layout'] = check_layout(doc, zero, cfg, config_dir)

    listener = None
    cap = {k: ct.capture_path(session, f'uwb_step10_{k}.jsonl') for k in ('link', 'spot', 'verify')}
    if a.replay:
        cap = {k: os.path.join(a.replay, f'uwb_step10_{k}.jsonl') for k in cap}

    def get(stage, seconds):
        if a.replay:
            return ct.read_jsonl(cap[stage])
        rows = listener.collect(seconds)
        ct.write_jsonl(cap[stage], rows)
        return rows

    try:
        if not a.replay:
            listener = Listener()
            time.sleep(0.5)
        print('\n=== 2. Link check ===')
        result['link'] = check_link(get('link', float(pr['link_check_s'])), zero, float(pr['link_check_s']))

        print('\n=== 3. Offsets ===')
        x, y = a.spot or ask_spot('OFFSET', (5.0, 1.5), a)
        bad = spot_ok(zero, x, y, float(pr['min_distance_from_anchor_m']))
        if bad:
            print(f'  WARN: spot is closer than {pr["min_distance_from_anchor_m"]} m to {bad}')
        ct.pause(f'  Put the tag STILL at ({x:.2f}, {y:.2f}), antenna as on the car, '
                 f'nobody in line of sight. Collect {secs:.0f} s', a.yes)
        result['offsets'], off = analyse_offsets(zero, get('spot', secs), x, y, cfg)

        print('\n=== 4. Verify ===')
        vx, vy = a.verify or ask_spot('VERIFY (different spot)', (3.5, 2.5), a)
        if math.hypot(vx - x, vy - y) < 0.5:
            print('  WARN: verify spot is < 0.5 m from the offset spot: not a real check')
        for xa in doc['anchors']:
            xa['range_offset_m'] = round(float(off.get(str(xa['id']).upper(), 0.0)), 4)
        corrected = AnchorSet.from_yaml(doc)
        ct.pause(f'  Move the tag to ({vx:.2f}, {vy:.2f}). Collect {secs:.0f} s', a.yes)
        result['verify'] = analyse_verify(corrected, get('verify', secs), vx, vy, cfg)
    finally:
        if listener:
            listener.close()

    passed = all(result[k]['status'] == 'PASS' for k in ('layout', 'link', 'offsets', 'verify'))
    doc['anchors_surveyed'] = True
    doc['offsets_calibrated'] = bool(passed)
    path = ct.save_data(session, 'uwb.yaml', doc)
    print(f'\n[{STEP_ID}] wrote {path}')
    fname = ct.write_step(session, int(cfg['index']), STEP_ID, result)
    failed = [k for k in ('layout', 'link', 'offsets', 'verify') if result[k]['status'] != 'PASS']
    ct.finish(session, root, STEP_ID, passed, fname, a.activate,
              detail='failed: ' + ','.join(failed) if failed else '')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
