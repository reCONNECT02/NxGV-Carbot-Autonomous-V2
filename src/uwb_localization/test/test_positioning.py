"""uwb_localization.positioning vs Haffiz's ORIGINAL code.

The reference classes are pulled out of tools/uwb/haffiz/*.py with `ast` (no copy
in this file, no rclpy / turtle needed), so if someone edits his files this test
compares against the edited version."""
import ast
import json
import math
import os
import random
from collections import deque

import numpy as np
import pytest

from uwb_localization.positioning import (KEYS, CvKalman, Positioner, PositioningCfg, PositioningConfigError,
                                          cov_ellipse, flatten, positions_from_rows, rear_axle_from_tag)
from uwb_localization.uwb_core import AnchorSet, parse_report, solve_linear, trilaterate

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
HAFFIZ = os.path.join(REPO, 'tools', 'uwb', 'haffiz')

# Haffiz's anchor layout (turtle_uwb_visualizer.py ANCHORS), metres
H_ANCHORS = {'1786': (-2.5, 0.0), '1782': (6.0, -0.6), '1783': (2.5, 8.0)}


class FakeTime:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now


def _load(fname, names):
    """Exec the named top-level classes / UWBROSNode methods of one Haffiz file."""
    path = os.path.join(HAFFIZ, fname)
    if not os.path.isfile(path):
        pytest.skip(f'{path} not in the checkout')
    tree = ast.parse(open(path, encoding='utf-8').read())
    clock = FakeTime()
    # float(): his code converts a 1x1 array with float(), an error on numpy >= 2 (fine on the
    # RDK's numpy 1.21). Same value, only the conversion is made version-proof here.
    def _float(v):
        return float(np.asarray(v).reshape(-1)[0]) if isinstance(v, np.ndarray) else float(v)
    ns = {'np': np, 'time': clock, 'deque': deque, 'ANCHORS': dict(H_ANCHORS), 'float': _float}
    try:
        from scipy.optimize import least_squares
        ns['least_squares'] = least_squares
    except ImportError:
        pass
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in names:
            exec(compile(ast.Module([node], []), path, 'exec'), ns)
        if isinstance(node, ast.ClassDef) and node.name == 'UWBROSNode':
            for f in node.body:
                if isinstance(f, ast.FunctionDef) and f.name in names:
                    exec(compile(ast.Module([f], []), path, 'exec'), ns)
    return ns, clock


def anchors_haffiz(z=0.0):
    return AnchorSet.from_yaml({'anchors': [{'id': k, 'xyz_m': [v[0], v[1], z], 'range_offset_m': 0.0}
                                            for k, v in H_ANCHORS.items()],
                                'tag': {'z_m': 0.0}, 'anchors_surveyed': True, 'offsets_calibrated': True})


def cfg(**kw):
    c = PositioningCfg()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def true_ranges(x, y, noise=0.0, rng=None):
    out = {}
    for k, (ax, ay) in H_ANCHORS.items():
        out[k] = math.hypot(x - ax, y - ay) + (rng.gauss(0, noise) if rng else 0.0)
    return out


# --------------------------------------------------------------------------- solver
def test_linear_solver_equals_haffiz_solve_trilateration():
    ns, _ = _load('turtle_uwb_visualizer.py', {'solve_trilateration'})
    a = anchors_haffiz()
    rng = random.Random(3)
    for _ in range(200):
        x, y = rng.uniform(-2, 6), rng.uniform(-0.5, 7.5)
        r = true_ranges(x, y, 0.05, rng)
        mine = solve_linear(a, r)
        his = ns['solve_trilateration'](None, r)
        assert mine == pytest.approx(his, abs=1e-9)
    assert solve_linear(a, true_ranges(1.0, 2.0)) == pytest.approx((1.0, 2.0), abs=1e-9)


def test_linear_solver_needs_three_and_rejects_collinear():
    a = anchors_haffiz()
    assert solve_linear(a, {'1786': 2.0, '1782': 3.0}) is None             # never (0, 0)
    line = AnchorSet.from_yaml({'anchors': [{'id': str(i), 'xyz_m': [i, 0, 0]} for i in range(3)],
                                'tag': {'z_m': 0.0}})
    assert solve_linear(line, {'0': 1.0, '1': 1.0, '2': 1.5}) is None


def test_linear_solver_four_anchors_least_squares():
    doc = {'anchors': [{'id': k, 'xyz_m': [v[0], v[1], 0.0]} for k, v in H_ANCHORS.items()] +
           [{'id': '1790', 'xyz_m': [6.0, 8.0, 0.0]}], 'tag': {'z_m': 0.0}}
    a = AnchorSet.from_yaml(doc)
    r = {k: math.hypot(2.0 - an.x, 3.0 - an.y) for k, an in a.anchors.items()}
    assert solve_linear(a, r) == pytest.approx((2.0, 3.0), abs=1e-9)


def test_nlls_matches_haffiz_robust_trilateration():
    pytest.importorskip('scipy')
    ns, _ = _load('turtle_uwb_visualizer_noEKF.py', {'RobustTrilateration'})
    his = ns['RobustTrilateration']()
    a = anchors_haffiz()
    pos = Positioner(a, cfg(solver='nlls', filter='none'))
    rng = random.Random(5)
    for _ in range(40):
        r = true_ranges(rng.uniform(0, 4), rng.uniform(1, 6), 0.04, rng)
        hx, hy = his.solve(dict(r))
        mine = pos.update(r, 0.0)
        assert mine.raw == pytest.approx((hx, hy), abs=1e-6)


def test_trilaterate_dispatch_and_pairwise_kept():
    a = anchors_haffiz()
    r = true_ranges(1.2, 3.4)
    for m in ('linear', 'pairwise', 'nlls'):
        assert trilaterate(a, r, m) == pytest.approx((1.2, 3.4), abs=1e-4)
    with pytest.raises(ValueError):
        trilaterate(a, r, 'magic')


# --------------------------------------------------------------------------- filter
def test_cv_kalman_equals_haffiz_ekf_step_by_step():
    ns, clock = _load('turtle_uwb_visualizer.py', {'ExtendedKalmanFilter2D'})
    his = ns['ExtendedKalmanFilter2D'](process_noise=0.2, measurement_noise=0.25)
    mine = CvKalman(0.2, 0.25, 16.0, 1.0, 0)
    rng = random.Random(7)
    t = clock.now
    x, y = 1.0, 2.0
    for k in range(400):
        t += rng.uniform(0.04, 0.134)                    # WiFi jitter seen on the car
        x += 0.02
        y += 0.01 * math.sin(k / 20)
        zx, zy = x + rng.gauss(0, 0.06), y + rng.gauss(0, 0.06)
        if k in (100, 250):
            zx += 2.5                                    # multipath spike: gated by both
        clock.now = t
        hx, hy = his.process(zx, zy)
        mx, my = mine.process(zx, zy, t)
        assert (mx, my) == pytest.approx((hx, hy), abs=1e-12)
        assert np.allclose(mine.P, his.P, atol=1e-12)
    assert mine.rejected >= 2


def test_gate_rejects_spike_and_reacquire_escapes_a_jump():
    haffiz = CvKalman(0.2, 0.25, 16.0, 1.0, 0)
    ours = CvKalman(0.2, 0.25, 16.0, 1.0, 10)
    t = 0.0
    for _ in range(100):                                 # settle at (1, 1)
        t += 0.1
        haffiz.process(1.0, 1.0, t)
        ours.process(1.0, 1.0, t)
    t += 0.1
    assert haffiz.process(4.0, 1.0, t) == pytest.approx((1.0, 1.0), abs=0.01)    # 3 m spike gated
    ours.process(4.0, 1.0, t)
    for _ in range(20):                                  # the tag really is at (4, 1) now
        t += 0.1
        h = haffiz.process(4.0, 1.0, t)
        o = ours.process(4.0, 1.0, t)
    assert o == pytest.approx((4.0, 1.0), abs=0.05)      # re-acquired after 10 rejects
    assert ours.reacquires == 1
    assert abs(h[0] - 4.0) > 1.0                         # Haffiz filter still stuck (documented)


def test_positioner_filtered_is_less_noisy_than_raw():
    a = anchors_haffiz()
    pos = Positioner(a, cfg())
    rng = random.Random(11)
    raw_err, filt_err = [], []
    for k in range(300):
        f = pos.update(true_ranges(2.0, 3.0, 0.05, rng), 0.1 * k)
        if k > 50:
            raw_err.append(math.hypot(f.raw[0] - 2.0, f.raw[1] - 3.0))
            filt_err.append(math.hypot(f.xy[0] - 2.0, f.xy[1] - 3.0))
    assert np.median(filt_err) < 0.5 * np.median(raw_err)
    assert f.cov[0] > 0 and f.cov[2] > 0 and f.accepted and f.n_anchors == 3


def test_moving_average_equals_haffiz_noekf_filter():
    ns, _ = _load('turtle_uwb_visualizer_noEKF.py', {'MovingAverageFilter'})
    his = ns['MovingAverageFilter'](window_size=10)
    pos = Positioner(anchors_haffiz(), cfg(filter='moving_average', solver='linear'))
    rng = random.Random(2)
    for k in range(50):
        r = true_ranges(1.0 + 0.01 * k, 2.0, 0.03, rng)
        f = pos.update(r, 0.1 * k)
        assert f.xy == pytest.approx(his.filter(*f.raw), abs=1e-12)


def test_min_anchors_and_missing_anchor():
    pos = Positioner(anchors_haffiz(), cfg())
    assert pos.update({'1786': 2.0, '1782': 3.0}, 0.0) is None
    assert pos.unsolved == 1


# --------------------------------------------------------------------------- config / rows
def common_yaml_section():
    import yaml
    path = os.path.join(REPO, 'src', 'carbot_bringup', 'config', 'params', 'common.yaml')
    return yaml.safe_load(open(path))['/**']['ros__parameters']['uwb_positioning']


def test_common_yaml_has_every_key_and_haffiz_defaults():
    d = common_yaml_section()
    flat = flatten(d)
    assert set(KEYS) <= set(flat)
    c = PositioningCfg.from_dict(d)
    assert (c.solver, c.filter) == ('linear', 'cv_kf')
    assert (c.process_noise, c.measurement_noise_m, c.gate_mahalanobis2, c.initial_variance_m2) == \
        (0.2, 0.25, 16.0, 1.0)
    assert c.ma_window == 10
    assert PositioningCfg.from_dict(flat) == c                      # params_under() form


def test_config_errors_are_loud():
    with pytest.raises(PositioningConfigError):
        PositioningCfg.from_dict({'solver': 'linear'})
    d = common_yaml_section()
    with pytest.raises(PositioningConfigError):
        PositioningCfg.from_dict(dict(d, solver='bogus'))


def _report(seq, ranges, boot='B1', t_ms=None):
    return json.dumps({'tag': 'T', 'boot_id': boot, 'seq': seq, 't_ms': t_ms or seq * 100,
                       'last_unknown_id': '0000',
                       'links': [{'A': k, 'R': round(v, 4), 'age_ms': 30, 'sample_seq': seq}
                                 for k, v in ranges.items()]})


def test_positions_from_rows_and_echo_prefix():
    a = anchors_haffiz()
    rows = [{'t': 0.1 * k, 'json': _report(k, true_ranges(2.0, 3.0))} for k in range(1, 30)]
    fx = positions_from_rows(a, rows, cfg())
    assert len(fx) == 29 and fx[-1].xy == pytest.approx((2.0, 3.0), abs=1e-3)
    # Haffiz accepts a pasted `ros2 topic echo` line
    assert parse_report("data: '" + rows[0]['json'] + "'") is not None


def test_helpers():
    a, b, ang = cov_ellipse(0.04, 0.0, 0.01)
    assert (a, b) == pytest.approx((0.4, 0.2)) and ang == pytest.approx(0.0)
    assert rear_axle_from_tag((1.0, 1.0), math.pi / 2, (0.1, 0.0)) == pytest.approx((1.0, 0.9))


def test_node_required_lists_every_positioning_key():
    tree = ast.parse(open(os.path.join(REPO, 'src', 'uwb_localization', 'uwb_localization', 'uwb_ranges.py')).read())
    req = next(ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'REQUIRED')
    assert {'uwb_positioning.' + k for k in KEYS} <= set(req)
