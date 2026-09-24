"""Blocks 05/06 algorithms (no ROS)."""
import math

import numpy as np
import pytest

import synth_loc as S
from carbot_common.course import Course
from carbot_localization.estimator_core import (GlobalCfg, GlobalEstimator, LocalCfg, LocalEstimator,
                                                PoseHistory, TrackToVenue, covariance_ellipse,
                                                odom_increment)
from uwb_localization.uwb_core import AnchorSet


@pytest.fixture(scope='module')
def course():
    return Course(S.track_map())


# ---------------------------------------------------------------- block 05
def test_dead_reckoning_straight():
    e = LocalEstimator(LocalCfg(), 7.18, 1.30, math.pi)
    for _ in range(100):
        e.predict(0.01, 0.05, imu_yaw=0.0, odom_dyaw=0.0)
    x, y, a = e.pose
    assert (x, y) == pytest.approx((6.18, 1.30), abs=1e-9)
    assert a == pytest.approx(math.pi)                  # IMU zero taken as the start heading


def test_imu_offset_and_blend():
    e = LocalEstimator(LocalCfg(), 0, 0, 1.0)
    e.predict(0.0, 0.05, imu_yaw=0.3, odom_dyaw=0.0)   # first sample: offset = 0.7
    e.predict(0.0, 0.05, imu_yaw=0.5, odom_dyaw=0.0)   # IMU turned +0.2
    assert e.pose[2] == pytest.approx(1.0 + 0.25 * 0.2)


def test_heading_falls_back_to_odom_yaw():
    e = LocalEstimator(LocalCfg(), 0, 0, 0.0)
    e.predict(0.0, 0.05, imu_yaw=None, odom_dyaw=0.1)
    assert e.pose[2] == pytest.approx(0.1)


def test_sigma_growth_v4():
    e = LocalEstimator(LocalCfg(), 0, 0, 0)
    e.predict(0.1, 1.0, None, 0.0)
    assert e.sigma == pytest.approx(math.hypot(0.003, 0.1 * 0.025, 0.00025))
    s = e.sigma
    e.idle(0.5)
    assert e.sigma == pytest.approx(s + 0.002)


def test_visual_registration_converges_without_jumps(course):
    true = (5.0, 4.75, 0.0)
    kind, grown, lx, ly = S.render_grid(course, true)
    e = LocalEstimator(LocalCfg(), 5.0, 4.75 + 0.04, 0.0)
    prev = e.pose
    for _ in range(80):
        r = e.visual_update(kind, grown, lx, ly, e.pose, course)
        assert r.applied
        step = math.hypot(e.pose[0] - prev[0], e.pose[1] - prev[1])
        assert step <= 0.0015 + 1e-9                    # never jumps
        prev = e.pose
    assert abs(e.pose[1] - 4.75) < 0.008
    assert e.sigma <= 0.004 + 1e-9


def test_corner_gives_2d_rank(course):
    # at a junction / corner the edges constrain both directions -> landmark
    kind, grown, lx, ly = S.render_grid(course, (6.40, 4.60, 0.6))
    e = LocalEstimator(LocalCfg(), 6.40, 4.60, 0.6)
    straight_kind, straight_grown, _, _ = S.render_grid(course, (4.0, 4.75, 0.0))
    e2 = LocalEstimator(LocalCfg(), 4.0, 4.75, 0.0)
    r1 = e.visual_update(kind, grown, lx, ly, e.pose, course)
    r2 = e2.visual_update(straight_kind, straight_grown, lx, ly, e2.pose, course)
    assert r1.rank > r2.rank


def test_no_road_no_update(course):
    kind = np.full((100, 100), 3, np.uint8)
    e = LocalEstimator(LocalCfg(), 3.5, 2.0, 0.0)
    r = e.visual_update(kind, np.zeros_like(kind), *S.render_grid(course, (3.5, 2, 0))[2:], e.pose, course)
    assert not r.applied and e.pose == (3.5, 2.0, 0.0)


def test_odom_increment_signed():
    assert odom_increment((0, 0, 0), (0.1, 0, 0)) == pytest.approx((0.1, 0.0))
    assert odom_increment((0, 0, 0), (-0.1, 0, 0))[0] == pytest.approx(-0.1)   # reversing
    assert odom_increment(None, (5, 5, 1)) == (0.0, 0.0)


def test_pose_history_interpolation():
    h = PoseHistory(2.0)
    h.add(0.0, 0, 0, 3.1)
    h.add(1.0, 1, 0, -3.1)
    x, y, a = h.at(0.5, 0.1)
    assert x == pytest.approx(0.5) and abs(abs(a) - math.pi) < 0.01   # wraps through +-pi
    assert h.at(5.0, 0.1) is None


# ---------------------------------------------------------------- block 06
def _run06(start_err=0.0, spikes=0.08, missing=False, seed=1):
    rng = np.random.default_rng(seed)
    anc = AnchorSet.from_yaml(S.uwb_doc())
    t2v = TrackToVenue(0.1, -0.05, math.radians(2))
    g = GlobalEstimator(GlobalCfg())
    x, y = 7.18, 1.30
    drift = np.array([start_err, 0.0])
    errs, spk_acc = [], 0
    for k in range(1200):
        ds = 0.0075
        if x > 1.2:
            x -= ds
            drift += [0.0, 0.02 * ds]
        else:
            y += ds
            drift += [0.02 * ds, 0.0]
        local = (x + drift[0], y + drift[1])
        g.predict(0.05, ds)
        if k % 2 == 0:
            for aid in anc.ids:
                if missing and aid == '1783' and 400 < k < 700:
                    continue
                a = anc.anchors[aid]
                v = t2v.to_venue(x, y)
                r = math.hypot(v[0] - a.x, v[1] - a.y) + rng.normal(0, 0.04)
                spike = rng.random() < spikes
                if spike:
                    r += rng.uniform(0.3, 0.8)
                u = g.range_update(aid, local, (a.x, a.y), r, t2v)
                spk_acc += bool(spike and u and u.accepted)
        gp = g.pose((local[0], local[1], 0))
        errs.append(math.hypot(gp[0] - x, gp[1] - y))
    return np.array(errs), spk_acc, g


def test_uwb_corrects_drift_and_rejects_multipath():
    errs, spk_acc, g = _run06(missing=True)
    assert np.median(errs) < 0.03 and errs[200:].max() < 0.08
    assert spk_acc <= 2 and g.accepted > 1000


def test_reacquire_after_bad_start():
    errs, _, g = _run06(start_err=0.6)
    assert g.reacquires >= 1 and np.median(errs[-300:]) < 0.05


def test_landmark_pulls_offset_to_zero():
    g = GlobalEstimator(GlobalCfg())
    g.off = np.array([0.1, -0.1])
    g.P = np.eye(2) * 0.01
    assert not g.landmark_update(0.1)                   # below min rank: ignored
    assert g.landmark_update(0.9)
    assert np.linalg.norm(g.off) < 0.01


def test_whole_fix_mode_gates():
    g = GlobalEstimator(GlobalCfg())
    assert g.fix_update((1.0, 1.0), (1.02, 1.0))
    assert not g.fix_update((1.0, 1.0), (3.0, 1.0))


def test_track_venue_roundtrip():
    t = TrackToVenue(0.3, -0.2, 0.4)
    assert t.to_track(*t.to_venue(1.2, 3.4)) == pytest.approx((1.2, 3.4))


def test_ellipse():
    a, b, ang = covariance_ellipse(np.diag([0.04, 0.01]), k=1.0)
    assert (a, b) == pytest.approx((0.2, 0.1)) and abs(ang) < 1e-9


def test_fix_update_with_filter_covariance():
    """Haffiz position input: R comes from the CV Kalman covariance (+ floor)."""
    import numpy as np
    g = GlobalEstimator(GlobalCfg(initial_variance=0.04))
    tight = np.eye(2) * 0.02 ** 2
    assert g.fix_update((1.0, 1.0), (1.05, 1.0), tight)
    assert g.off[0] > 0.04                         # trusted: offset moves almost all the way
    g2 = GlobalEstimator(GlobalCfg(initial_variance=0.04))
    assert g2.fix_update((1.0, 1.0), (1.05, 1.0), np.eye(2) * 0.3 ** 2)
    assert g2.off[0] < 0.02 < g.off[0]             # loose filter: much smaller pull
    assert not g.fix_update((1.0, 1.0), (3.0, 1.0), tight)   # 2 m jump gated
