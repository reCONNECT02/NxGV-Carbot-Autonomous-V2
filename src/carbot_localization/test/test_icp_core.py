"""icp_core: edge memory, point-to-line ICP, and the slew-limited corrector (pure numpy, no ROS)."""
import math
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), '..', 'carbot_common'))

from carbot_common.course import Course  # noqa: E402
from carbot_localization import icp_core as ic  # noqa: E402
from carbot_localization.estimator_core import LocalCfg, LocalEstimator  # noqa: E402
from synth_loc import CONFIG, render_grid, track_map  # noqa: E402


def cfg_dict():
    with open(os.path.join(CONFIG, 'params', 'localization.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['local_pose']['ros__parameters']


def getter(d):
    def p(name):
        cur = d
        for k in name.split('.'):
            cur = cur[k]
        return cur
    return p


def make_cfg(**over):
    d = cfg_dict()
    d = yaml.safe_load(yaml.safe_dump(d))
    d['icp']['enabled'] = True
    for k, v in over.items():
        if '.' in k:
            a, b = k.split('.')
            d['icp'][a][b] = v
        else:
            d['icp'][k] = v
    return ic.IcpCfg.from_params(getter(d))


def lane_points(x0=-0.5, x1=0.7, half=0.30, step=0.02, jitter=0.0, seed=0):
    rng = np.random.default_rng(seed)
    xs = np.arange(x0, x1, step)
    pts = np.concatenate([np.stack([xs, np.full_like(xs, half)], 1), np.stack([xs, np.full_like(xs, -half)], 1)])
    return pts + rng.normal(0, jitter, pts.shape)


def corner_points(seed=0):
    """A wall along x plus a wall across it (an L): all three degrees of freedom are observable."""
    xs = np.arange(-0.5, 0.7, 0.02)
    ys = np.arange(-0.5, 0.3, 0.02)
    return np.concatenate([np.stack([xs, np.full_like(xs, 0.30)], 1), np.stack([np.full_like(ys, 0.60), ys], 1)])


def transform(pts, dx, dy, dth):
    c, s = math.cos(dth), math.sin(dth)
    return np.stack([c * pts[:, 0] - s * pts[:, 1] + dx, s * pts[:, 0] + c * pts[:, 1] + dy], 1)


def inverse(pts, dx, dy, dth):
    c, s = math.cos(dth), math.sin(dth)
    q = pts - np.array([dx, dy])
    return np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)


# --------------------------------------------------------------------------- pieces
def test_normals_of_a_straight_edge_are_perpendicular_to_it():
    pts = lane_points()
    n = ic.normals(pts, 8)
    assert np.abs(np.abs(n[:, 1]) - 1.0).max() < 1e-6 and np.abs(n[:, 0]).max() < 1e-6


def test_edge_points_find_the_road_paint_boundary():
    rows = 100
    lx = -0.65 + (np.arange(rows) + 0.5) * 0.018
    ly = -0.9 + (np.arange(rows) + 0.5) * 0.018
    X, Y = np.meshgrid(lx, ly, indexing='ij')
    kind = np.where(np.abs(Y) < 0.30, 1, np.where(np.abs(Y) < 0.33, 2, 3)).astype(np.uint8)
    pts = ic.edge_points(kind, (kind == 1).astype(np.uint8), lx, ly, 0.70, 1)
    assert len(pts) > 50 and np.abs(np.abs(pts[:, 1]) - 0.31).max() < 0.02
    assert np.hypot(pts[:, 0], pts[:, 1]).max() <= 0.70 + 1e-9


def test_edge_memory_age_window_and_motion_uncertainty():
    cfg = make_cfg(**{'memory.min_age_s': 0.5, 'memory.max_age_s': 3.0})
    mem = ic.EdgeMemory()
    pts = lane_points(step=0.05)
    n = ic.normals(pts, 8)
    for t, dist in ((0.0, 0.0), (1.0, 0.15), (2.5, 0.40)):
        mem.add(pts, n, (dist, 0.0, 0.0), t, dist)
    pose = (0.6, 0.0, 0.0)
    ref, _, mean_age = mem.reference(pose, 3.0, 0.6, cfg)
    assert len(ref) > 0 and mean_age > 0.5
    # V4 motion-uncertainty rule: 0.004 + 0.006 * travel <= limit; a tight limit drops the far-travelled scans
    ref2, _, _ = mem.reference(pose, 3.0, 0.6, make_cfg(**{'memory.min_age_s': 0.5, 'memory.max_age_s': 3.0,
                                                           'memory.uncertainty_limit_m': 0.0065}))
    assert 0 < len(ref2) < len(ref)
    # a scan younger than min_age (t = 2.5, 0.2 s old at now = 2.7) is the same view: never used
    only_young, _, _ = mem.reference((0.4, 0.0, 0.0), 2.7, 0.4, make_cfg(**{'memory.min_age_s': 0.5, 'memory.max_age_s': 0.3}))
    assert len(only_young) == 0
    # max_age 0.6 s still keeps the scan that is 0.5 s old
    assert len(mem.reference(pose, 3.0, 0.6, make_cfg(**{'memory.min_age_s': 0.5, 'memory.max_age_s': 0.6}))[0]) > 0


# --------------------------------------------------------------------------- registration
def test_register_recovers_sideways_and_heading_on_straight_edges_and_leaves_along_track_alone():
    cfg = make_cfg()
    ref = lane_points()
    nrm = ic.normals(ref, 8)
    truth = (0.0, 0.006, 0.012)                                       # dx, dy, dth that maps src onto ref
    src = inverse(lane_points(x0=-0.4, x1=0.6, jitter=0.0015, seed=3), *truth)
    r = ic.register(src, ref, nrm, cfg)
    assert r.ok and r.constrained == 2                                # sideways + heading, along-track unobservable
    assert abs(r.dy - truth[1]) < 0.0012 and abs(r.dth - truth[2]) < 0.0025
    assert abs(r.dx) < 0.004                                          # not invented from the noise
    assert r.rms_m < 0.004 and r.inliers >= cfg.min_inliers


def test_register_recovers_all_three_on_a_corner():
    cfg = make_cfg()
    ref = corner_points()
    nrm = ic.normals(ref, 8)
    truth = (0.004, -0.005, -0.010)
    src = inverse(ref[::2], *truth)                                   # every other point, so not the same samples
    r = ic.register(src, ref, nrm, cfg)
    assert r.ok and r.constrained == 3
    assert abs(r.dx - truth[0]) < 0.0012 and abs(r.dy - truth[1]) < 0.0012 and abs(r.dth - truth[2]) < 0.003


def test_register_is_robust_to_outliers():
    cfg = make_cfg()
    ref = lane_points()
    nrm = ic.normals(ref, 8)
    truth = (0.0, -0.004, 0.008)
    src = inverse(lane_points(x0=-0.4, x1=0.6, jitter=0.001, seed=4), *truth)
    rng = np.random.default_rng(5)
    junk = rng.uniform([-0.4, -0.5], [0.6, 0.5], (30, 2))             # false paint cells
    r = ic.register(np.concatenate([src, junk]), ref, nrm, cfg)
    assert r.ok and abs(r.dy - truth[1]) < 0.002 and abs(r.dth - truth[2]) < 0.004


def test_register_refuses_thin_data():
    cfg = make_cfg()
    ref = lane_points()
    assert not ic.register(ref[:5], ref, ic.normals(ref, 8), cfg).ok
    assert not ic.register(ref, ref[:10], ic.normals(ref[:10], 8), cfg).ok


# --------------------------------------------------------------------------- estimator step
def test_apply_body_step_moves_the_transform_and_trims_the_heading():
    est = LocalEstimator(LocalCfg.from_params(getter(cfg_dict())), 1.0, 2.0, math.pi / 2)   # facing +y
    before = est.pose
    est.apply_body_step(0.001, 0.0, 0.0002)                           # 1 mm "forward" = +y in the track frame
    x, y, a = est.pose
    assert abs(x - before[0]) < 1e-12 and abs(y - before[1] - 0.001) < 1e-12
    assert abs(a - before[2] - 0.0002) < 1e-12
    est.reset(0.0, 0.0, 0.0)
    assert est.ta == 0.0 and est.pose == (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- corrector on rendered grids
def straight_grids(cfg, n=90, heading_drift=0.0, speed=0.3, dt=0.125):
    """A straight drive along the start lane; odom heading drifts by `heading_drift` rad/s (an IMU bias).
    Returns (corrector, steps, err_trace) where err is (lateral, heading) of the pose vs the truth."""
    course = Course(track_map())
    corr = ic.IcpCorrector(cfg)
    x0, y0, a0 = course.start_pose()
    est = LocalEstimator(LocalCfg.from_params(getter(cfg_dict())), x0, y0, a0)
    steps, err = [], []
    dist = 0.0
    for i in range(n):
        t = i * dt
        true = (x0 - speed * t, y0, a0)                               # west along the lane
        dist = speed * t
        odom_h = a0 + heading_drift * t
        odom = (x0 - speed * t * math.cos(heading_drift * t / 2), y0 - speed * t * math.sin(heading_drift * t / 2), odom_h)
        kind, grown, lx, ly = render_grid(course, true)
        ax, ay = np.arange(kind.shape[0]), np.arange(kind.shape[1])
        lxv = -0.65 + (ax + 0.5) * 0.018
        lyv = -0.9 + (ay + 0.5) * 0.018
        st = corr.on_grid(kind, grown, lxv, lyv, odom, t, dist, speed, odom_h + est.ta)
        if st is not None:
            steps.append(st)
            if st.applied:
                est.apply_body_step(st.dx, st.dy, st.dth)
        est.ox, est.oy, est.oa = odom
        err.append((est.pose[1] - true[1], wrap_(est.pose[2] - true[2])))
    return corr, steps, err


def wrap_(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_corrector_never_exceeds_its_caps_and_reduces_heading_drift():
    cfg = make_cfg(gain=1.0, max_step_rad=0.003, max_total_rad=0.5, max_total_m=0.5)
    drift = math.radians(0.3)                                         # 0.3 deg/s IMU bias: big enough to see over the 3 s memory
    corr, steps, err = straight_grids(cfg, n=140, heading_drift=drift)
    applied = [s for s in steps if s.applied]
    assert len(applied) > 5 and corr.accepted == len(applied)
    for s in applied:
        assert math.hypot(s.dx, s.dy) <= cfg.max_step_m + 1e-12 and abs(s.dth) <= cfg.max_step_rad + 1e-12
    assert corr.total_th <= cfg.max_total_rad + 1e-12 and np.hypot(*corr.net) <= cfg.max_total_m + 1e-12
    baseline = abs(drift * 140 * 0.125)                               # heading error with no correction at the end
    assert abs(err[-1][1]) < 0.8 * baseline                           # the trim cancels part of it (measured ~35 %: docs/LOCALIZATION_MEMORY.md)


def test_corrector_rejects_when_standing_or_without_memory():
    cfg = make_cfg()
    course = Course(track_map())
    corr = ic.IcpCorrector(cfg)
    kind, grown, _, _ = render_grid(course, course.start_pose())
    lx = -0.65 + (np.arange(100) + 0.5) * 0.018
    ly = -0.9 + (np.arange(100) + 0.5) * 0.018
    out = [corr.on_grid(kind, grown, lx, ly, course.start_pose(), 0.1 * i, 0.0, 0.0) for i in range(1, 9)]
    assert all(s is None or (not s.applied) for s in out)
    assert corr.accepted == 0 and corr.rejected >= 1
    assert 'standing' in corr.last_reason or 'few' in corr.last_reason or 'too few' in corr.last_reason


def test_corrector_total_bound_stops_further_heading_trim():
    cfg = make_cfg(max_total_rad=0.0004, max_step_rad=0.0003, gain=1.0)
    corr, steps, err = straight_grids(cfg, n=140, heading_drift=math.radians(0.05))
    assert abs(corr.total_th) <= 0.0004 + 1e-12
    applied = [s for s in steps if s.applied]
    # once the bound is reached the corrector keeps applying sideways steps but no more heading
    assert any(s.dth == 0.0 for s in applied[2:]) or corr.rejected > 0


def test_disabled_config_key_is_loud():
    d = cfg_dict()
    del d['icp']['gain']
    with pytest.raises(KeyError):
        ic.IcpCfg.from_params(getter(d))


# --------------------------------------------------------------------------- IMU lag compensation (heading_lead_s)
def _turn_error(lead, rate_deg_s=25.0, seconds=6.0, alpha=0.15):
    """Steady turn: /imu/rpy yaw is servo_controller's EMA of the truth (alpha 0.15 per 20 Hz sample)."""
    d = yaml.safe_load(yaml.safe_dump(cfg_dict()))
    d['heading_lead_s'] = lead
    est = LocalEstimator(LocalCfg.from_params(getter(d)), 0.0, 0.0, 0.0)
    dt, rate = 0.05, math.radians(rate_deg_s)
    truth = imu = 0.0
    err = []
    for i in range(int(seconds / dt)):
        truth += rate * dt
        imu += alpha * (truth - imu)
        est.predict(0.015, dt, imu, rate * dt)               # odom yaw increment = true (wheel kinematics)
        err.append(ic.wrap(est.pose[2] - truth))
    return float(np.mean(err[-40:]))


def test_heading_lead_off_is_v4_and_lead_cancels_the_imu_lag():
    lagged = _turn_error(0.0)
    assert abs(math.degrees(lagged)) > 5.0 and lagged < 0          # the heading trails a 25 deg/s turn by ~10 deg
    led = _turn_error(0.28)
    assert abs(led) < 0.5 * abs(lagged)
    assert abs(_turn_error(0.0, rate_deg_s=0.0)) < 1e-6           # no turn: nothing changes
