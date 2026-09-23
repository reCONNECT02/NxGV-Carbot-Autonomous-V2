"""Calibration step 5 maths -- LiDAR-camera alignment (pure numpy, unit tested).

Frames: base_link (+x forward, +y left, rear-axle centre on the ground) and
laser_frame, placed by the static TF base_link -> laser_frame
[x, y, z, yaw, pitch, roll] (drivers.yaml carbot_tf.base_to_laser). A scan
point at scan angle a and range r is, in base_link,
    L + r * (cos(yaw + a), sin(yaw + a))          L = (x, y)
-- the same convention as ros_util.scan_hits and the base tunnel follower
(angle = scan angle + lidar_angle_offset), which is why both keys get the
same corrected yaw.

Method: an upright target stands on the floor ahead. The user clicks its foot
in the front camera image; the camera ray hits the floor at g (base_link, from
the step 3/4 calibration). The LiDAR sees the target as a small cluster; its
centroid p is kept in LASER coordinates (independent of the yaw being
calibrated). Seen from the LiDAR origin L:
    camera bearing  (laser frame) = atan2(g - L) - yaw
    LiDAR  bearing  (laser frame) = atan2(p)
    error = camera - LiDAR        -> new yaw = yaw + mean(error)
The residual per capture after the correction is the pass metric
(max_bearing_error_deg). Ranges |g - L| vs |p| are compared only as a warning
(they depend on the LiDAR x offset and the camera pitch, not on the yaw).
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def wrap(a: float) -> float:
    """Angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def circ_mean(angles: Sequence[float]) -> float:
    a = np.asarray(angles, float)
    return float(math.atan2(np.sin(a).mean(), np.cos(a).mean()))


# --------------------------------------------------------------------------- scans
def scan_points(ranges: Sequence[float], angle_min: float, angle_inc: float,
                range_min: float, range_max: float) -> Tuple[np.ndarray, np.ndarray]:
    """Valid returns as laser-frame points (N,2), plus their scan index (N,)."""
    r = np.asarray(ranges, float)
    idx = np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(range_min, 1e-3)) & (r <= range_max)
    a = angle_min + idx[ok] * angle_inc
    return np.column_stack([r[ok] * np.cos(a), r[ok] * np.sin(a)]), idx[ok]


def laser_to_base(pts: np.ndarray, mount: Sequence[float]) -> np.ndarray:
    """(N,2) laser-frame points -> base_link. mount = base_to_laser [x, y, z, yaw, ...]."""
    x, y, yaw = float(mount[0]), float(mount[1]), float(mount[3])
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(pts, float).reshape(-1, 2)
    return np.column_stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]])


def clusters(pts: np.ndarray, idx: np.ndarray, gap_m: float, min_points: int,
             max_width_m: float, n_beams: int = 0) -> List[Dict]:
    """Split consecutive scan returns where neighbours are further apart than
    gap_m (or a scan index is skipped). Keeps clusters with >= min_points and a
    chord (first-last point distance) <= max_width_m. Points in laser frame.
    n_beams > 0 = a full 360 deg scan of that many beams: the last and first
    beams are neighbours. With the reversed mount (yaw pi) straight ahead IS
    that seam, so a target in the middle of the view must not be cut in two."""
    out: List[Dict] = []
    n = len(pts)
    if n == 0:
        return out
    segs, start = [], 0
    for i in range(1, n + 1):
        if i == n or idx[i] != idx[i - 1] + 1 or float(np.hypot(*(pts[i] - pts[i - 1]))) > gap_m:
            segs.append(pts[start:i])
            start = i
    if n_beams and len(segs) > 1 and idx[0] == 0 and idx[-1] == n_beams - 1 and \
            float(np.hypot(*(pts[0] - pts[-1]))) <= gap_m:
        segs = [np.vstack([segs[-1], segs[0]])] + segs[1:-1]
    for seg in segs:
        if len(seg) < min_points:
            continue
        width = float(np.hypot(*(seg[-1] - seg[0])))
        if width > max_width_m:
            continue
        c = seg.mean(axis=0)
        out.append({'laser_xy': [float(c[0]), float(c[1])], 'n': int(len(seg)), 'width_m': width})
    return out


def full_circle(scan: Dict) -> int:
    """Beam count if the scan covers 360 deg (first and last beams adjacent), else 0."""
    n = len(scan['ranges'])
    span = abs(float(scan['angle_increment'])) * n
    return n if n > 1 and abs(span - 2.0 * math.pi) <= 1.5 * abs(float(scan['angle_increment'])) else 0


def in_roi(base_xy: np.ndarray, mount: Sequence[float], roi_min_m: float, roi_max_m: float,
           half_angle_rad: float) -> np.ndarray:
    """Mask: ahead of the LiDAR, range from it in [roi_min_m, roi_max_m], within
    +-half_angle_rad of straight ahead (base_link +x)."""
    d = np.asarray(base_xy, float).reshape(-1, 2) - np.array([float(mount[0]), float(mount[1])])
    r = np.hypot(d[:, 0], d[:, 1])
    b = np.arctan2(d[:, 1], d[:, 0])
    return (r >= roi_min_m) & (r <= roi_max_m) & (np.abs(b) <= half_angle_rad)


def front_clusters(scan: Dict, mount: Sequence[float], proc: Dict, half_angle_rad: float) -> List[Dict]:
    """Target candidates of one scan (dict ranges/angle_min/angle_increment/range_min/range_max),
    with base_link position, bearing and range from the LiDAR under `mount`."""
    pts, idx = scan_points(scan['ranges'], scan['angle_min'], scan['angle_increment'],
                           scan['range_min'], scan['range_max'])
    cl = clusters(pts, idx, float(proc['cluster_gap_m']), int(proc['min_cluster_points']),
                  float(proc['max_cluster_width_m']), full_circle(scan))
    if not cl:
        return []
    lxy = np.array([c['laser_xy'] for c in cl])
    bxy = laser_to_base(lxy, mount)
    keep = in_roi(bxy, mount, float(proc['roi_min_m']), float(proc['roi_max_m']), half_angle_rad)
    out = []
    for c, b, k in zip(cl, bxy, keep):
        if not k:
            continue
        d = b - np.array([float(mount[0]), float(mount[1])])
        out.append(dict(c, base_xy=[float(b[0]), float(b[1])],
                        bearing_deg=math.degrees(math.atan2(d[1], d[0])), range_m=float(np.hypot(*d))))
    return out


def match_capture(scans: Sequence[Dict], ground_xy: Sequence[float], mount: Sequence[float],
                  proc: Dict, half_angle_rad: float) -> Tuple[Optional[Dict], str]:
    """Nearest cluster to the clicked floor point in each scan, averaged over
    the scans where one is within match_max_m. Returns (capture, '') or (None, why)."""
    g = np.asarray(ground_xy, float)
    lim = float(proc['match_max_m'])
    hits, nearest = [], None
    for sc in scans:
        best = None
        for c in front_clusters(sc, mount, proc, half_angle_rad):
            d = float(np.hypot(*(np.asarray(c['base_xy']) - g)))
            if nearest is None or d < nearest:
                nearest = d
            if d <= lim and (best is None or d < best[0]):
                best = (d, c)
        if best is not None:
            hits.append(best[1])
    need = max(1, int(math.ceil(len(scans) / 2.0)))
    if len(hits) < need:
        if nearest is None:
            why = 'the LiDAR sees no target-sized object in the front area'
        else:
            why = (f'the nearest LiDAR object is {nearest:.2f} m from the clicked point (limit {lim:.2f} m)'
                   if not hits else f'the target was found in only {len(hits)} of {len(scans)} scans')
        return None, why
    p = np.array([h['laser_xy'] for h in hits]).mean(axis=0)
    return {'laser_xy': [round(float(p[0]), 4), round(float(p[1]), 4)],
            'ground_xy': [round(float(g[0]), 4), round(float(g[1]), 4)],
            'n_scans': len(hits), 'n_points': int(round(np.mean([h['n'] for h in hits])))}, ''


# --------------------------------------------------------------------------- estimate
def capture_errors(caps: Sequence[Dict], mount: Sequence[float]) -> List[Dict]:
    """Per capture: camera / LiDAR bearing (laser frame), error, ranges."""
    L = np.array([float(mount[0]), float(mount[1])])
    yaw = float(mount[3])
    out = []
    for c in caps:
        d = np.asarray(c['ground_xy'], float) - L
        p = np.asarray(c['laser_xy'], float)
        cam = wrap(math.atan2(d[1], d[0]) - yaw)
        lid = math.atan2(p[1], p[0])
        out.append({'cam_bearing': cam, 'lidar_bearing': lid, 'error': wrap(cam - lid),
                    'cam_range_m': float(np.hypot(*d)), 'lidar_range_m': float(np.hypot(*p)),
                    'base_bearing_deg': math.degrees(math.atan2(d[1], d[0]))})
    return out


def estimate(caps: Sequence[Dict], mount: Sequence[float]) -> Optional[Dict]:
    """Yaw correction from the captures (None without captures)."""
    if not caps:
        return None
    rows = capture_errors(caps, mount)
    off = circ_mean([r['error'] for r in rows])
    for r in rows:
        r['residual'] = wrap(r['error'] - off)
    bb = [r['base_bearing_deg'] for r in rows]
    # not wrapped: stays next to the old yaw (3.1416 + 0.05, not -3.09) so the YAML reads as a small change
    return {'offset': off, 'new_yaw': float(mount[3]) + off, 'rows': rows,
            'max_residual_deg': max(abs(math.degrees(r['residual'])) for r in rows),
            'spread_deg': max(bb) - min(bb),
            'max_range_diff_m': max(abs(r['cam_range_m'] - r['lidar_range_m']) for r in rows)}


def evaluate(caps: Sequence[Dict], mount: Sequence[float], proc: Dict, pass_cfg: Dict) -> Dict:
    """The step result: {'passed', 'summary', 'checks', 'estimate', ...}."""
    est = estimate(caps, mount)
    n = len(caps)
    need_n, need_spread = int(proc['min_captures']), float(proc['min_spread_deg'])
    max_res, max_corr = float(pass_cfg['max_bearing_error_deg']), float(pass_cfg['max_correction_deg'])
    warn_rd = float(pass_cfg['warn_range_diff_m'])
    checks = []

    def chk(key, label, ok, measured, limit, why='', fix=''):
        checks.append({'key': key, 'label': label, 'passed': bool(ok), 'measured': measured,
                       'limit': limit, 'why': '' if ok else why, 'fix': '' if ok else fix})

    chk('captures', 'Target positions captured', n >= need_n, str(n), f'≥ {need_n}',
        f'only {n} capture(s)', 'Move the target and press Capture again (left, centre and right of the view).')
    spread = est['spread_deg'] if est else 0.0
    chk('spread', 'Spread of the positions', n >= 2 and spread >= need_spread, f'{spread:.1f}°', f'≥ {need_spread:.0f}°',
        'the captures are too close together to separate a rotation from noise',
        'Capture the target once near the left edge of the camera view and once near the right edge.')
    corr = math.degrees(est['offset']) if est else 0.0
    chk('correction', 'Yaw correction', est is not None and abs(corr) <= max_corr, f'{corr:+.2f}°', f'≤ ±{max_corr:.0f}°',
        'the LiDAR and the camera disagree by more than a mounting tolerance',
        'Check drivers.yaml ydlidar reversion/inverted and carbot_tf.base_to_laser yaw (3.1416 for the reversed mount), '
        'and that step 4 (camera extrinsics) passed. Then Clear and capture again.')
    res = est['max_residual_deg'] if est else 0.0
    chk('residual', 'Bearing error after correction', est is not None and res <= max_res, f'{res:.2f}°', f'≤ {max_res:.1f}°',
        'the captures do not agree on one rotation',
        'Look at the per-capture table: redo the capture with the largest error (click exactly where the target '
        'meets the floor, keep the target still and away from walls).')
    passed = all(c['passed'] for c in checks)
    warn = ''
    if est and est['max_range_diff_m'] > warn_rd:
        warn = (f'LiDAR and camera distances differ by up to {est["max_range_diff_m"]:.2f} m: measure the LiDAR '
                'x position (Controls) or recheck step 4. The yaw result is still valid.')
    if passed:
        summary = f'yaw correction {corr:+.2f}°, residual {res:.2f}° over {n} positions'
    else:
        summary = next(c['label'] + ': ' + c['why'] for c in checks if not c['passed'])
    return {'passed': passed, 'summary': summary, 'checks': checks, 'warning': warn,
            'estimate': est, 'n_captures': n}
