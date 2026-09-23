"""Step 5 maths (lidar_align): synthetic 360 deg scans of round targets."""
import math

import numpy as np

from carbot_ops import lidar_align as la

NOMINAL = [0.09, 0.0, 0.22, math.pi, 0.0, 0.0]      # repo TF (reversed mount)
PROC = {'roi_min_m': 0.25, 'roi_max_m': 1.6, 'cluster_gap_m': 0.05, 'min_cluster_points': 3,
        'max_cluster_width_m': 0.15, 'match_max_m': 0.25, 'min_captures': 3, 'min_spread_deg': 20.0}
PASS = {'max_bearing_error_deg': 2.0, 'max_correction_deg': 10.0, 'warn_range_diff_m': 0.08}
HALF = math.radians(30.0 + 10.0)            # hfov/2 + max_correction_deg, as the step uses


def fake_scan(targets, true_mount, n=450, radius=0.03, noise=0.0, seed=0):
    """Ray-cast a 360 deg scan (angle_min -pi) against cylinders at base_link xy."""
    rng = np.random.default_rng(seed)
    L = np.array(true_mount[:2])
    inc = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = true_mount[3] - math.pi + i * inc
        d = np.array([math.cos(a), math.sin(a)])
        best = 0.0
        for t in targets:
            oc = L - np.asarray(t, float)
            b = float(oc @ d)
            disc = b * b - (float(oc @ oc) - radius ** 2)
            if disc < 0:
                continue
            s = -b - math.sqrt(disc)
            if s > 0 and (best == 0.0 or s < best):
                best = s
        ranges.append(best + (rng.normal(0, noise) if best and noise else 0.0) if best else 0.0)
    return {'ranges': ranges, 'angle_min': -math.pi, 'angle_increment': inc, 'range_min': 0.02,
            'range_max': 16.0, 't': 0.0}


def capture(target, true_mount, **kw):
    scans = [fake_scan([target], true_mount, seed=k, **kw) for k in range(5)]
    cap, why = la.match_capture(scans, target, NOMINAL, PROC, HALF)
    assert cap is not None, why
    return cap


def test_seam_target_is_one_cluster():
    # straight ahead with yaw pi = scan angle +-pi: the seam of the scan
    sc = fake_scan([(0.8, 0.0)], NOMINAL)
    assert la.full_circle(sc) == 450
    tg = la.front_clusters(sc, NOMINAL, PROC, HALF)
    assert len(tg) == 1
    assert abs(tg[0]['bearing_deg']) < 0.5 and abs(tg[0]['range_m'] - (0.71 - 0.03)) < 0.03


def test_no_merge_without_full_circle():
    pts = np.array([[1.0, 0.0], [1.0, 0.01], [1.0, 0.02], [1.0, -0.02], [1.0, -0.01]])
    idx = np.array([0, 1, 2, 7, 8])
    assert len(la.clusters(pts, idx, 0.05, 2, 0.2)) == 2
    assert len(la.clusters(pts, idx, 0.05, 2, 0.2, n_beams=9)) == 1


def test_wide_objects_rejected():
    sc = fake_scan([(0.8, 0.0)], NOMINAL, radius=0.20)
    assert la.front_clusters(sc, NOMINAL, PROC, HALF) == []


def test_recovers_yaw_error():
    for delta_deg in (-4.0, 0.0, 2.5, 6.0):
        true = list(NOMINAL)
        true[3] = NOMINAL[3] + math.radians(delta_deg)
        caps = [capture(t, true) for t in ((0.8, 0.30), (0.9, 0.0), (0.7, -0.28))]
        est = la.estimate(caps, NOMINAL)
        # the cluster centroid is the target's front face, the click its centre: same ray
        assert abs(math.degrees(est['offset']) - delta_deg) < 0.4, (delta_deg, math.degrees(est['offset']))
        assert est['max_residual_deg'] < 0.6
        assert abs(la.wrap(est['new_yaw'] - true[3])) < math.radians(0.4)
        res = la.evaluate(caps, NOMINAL, PROC, PASS)
        assert res['passed'], res['summary']


def test_fails_with_few_or_clustered_captures():
    caps = [capture((0.8, 0.0), NOMINAL), capture((0.85, 0.02), NOMINAL)]
    res = la.evaluate(caps, NOMINAL, PROC, PASS)
    assert not res['passed']
    failed = {c['key'] for c in res['checks'] if not c['passed']}
    assert failed == {'captures', 'spread'}
    assert all(c['fix'] for c in res['checks'] if not c['passed'])


def test_inconsistent_capture_fails_residual():
    caps = [capture(t, NOMINAL) for t in ((0.8, 0.30), (0.9, 0.0), (0.7, -0.28))]
    caps[1] = dict(caps[1], ground_xy=[0.9, 0.06])            # a click 4 deg off
    res = la.evaluate(caps, NOMINAL, PROC, PASS)
    assert not res['passed']
    assert [c['key'] for c in res['checks'] if not c['passed']] == ['residual']


def test_large_correction_fails():
    true = list(NOMINAL)
    true[3] += math.radians(14.0)
    caps = [capture(t, true) for t in ((0.8, 0.30), (0.9, 0.0), (0.7, -0.28))]
    res = la.evaluate(caps, NOMINAL, dict(PROC), PASS)
    assert 'correction' in {c['key'] for c in res['checks'] if not c['passed']}


def test_click_far_from_any_object_is_refused():
    scans = [fake_scan([(0.8, 0.3)], NOMINAL)]
    cap, why = la.match_capture(scans, (0.8, -0.3), NOMINAL, PROC, HALF)
    assert cap is None and 'from the clicked point' in why
    cap, why = la.match_capture([fake_scan([], NOMINAL)], (0.8, 0.0), NOMINAL, PROC, HALF)
    assert cap is None and 'no target-sized object' in why


def test_range_warning():
    caps = [capture(t, NOMINAL) for t in ((0.8, 0.30), (0.9, 0.0), (0.7, -0.28))]
    far = [dict(c, ground_xy=[c['ground_xy'][0] * 1.25, c['ground_xy'][1] * 1.25]) for c in caps]
    assert la.evaluate(caps, NOMINAL, PROC, PASS)['warning'] == ''
    assert 'distances differ' in la.evaluate(far, NOMINAL, PROC, PASS)['warning']
