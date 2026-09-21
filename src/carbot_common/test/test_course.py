"""Prior map (course.py) against V4 core.js Course values."""
import math
import os

import numpy as np
import pytest
import yaml

from carbot_common.course import Course, js_round, sample_arc

TM = os.path.join(os.path.dirname(__file__), '..', '..', 'carbot_bringup', 'config', 'data', 'track_map.yaml')


@pytest.fixture(scope='module')
def course():
    with open(TM) as f:
        return Course(yaml.safe_load(f))


def test_raster_size_matches_v4(course):
    assert (course.nx, course.ny) == (766, 516)          # V4: ceil(7.65/.01)+1, ceil(5.15/.01)+1


def test_centreline_is_half_lane(course):
    assert course.clearance(1.0, 4.75) == pytest.approx(0.15, abs=1e-6)
    assert course.clearance(3.5, 4.75) == pytest.approx(0.15, abs=1e-6)
    assert course.clearance(3.5, 4.75 + 0.15) == pytest.approx(0.0, abs=2e-3)   # lane edge


def test_outside_is_negative(course):
    assert course.clearance(3.5, 2.0) < 0
    assert course.clearance(-1.0, 1.0) == -10.0
    assert course.clearance(100.0, 1.0) == -10.0


def test_rect_areas_drivable(course):
    x0, x1, y0, y1 = course.areas['lane_change']
    assert course.clearance((x0 + x1) / 2, (y0 + y1) / 2) > 0.3


def test_gradient_unit_norm_near_edge(course):
    gx, gy = course.gradient(3.5, 4.75 + 0.14)
    assert math.hypot(gx, gy) == pytest.approx(1.0, abs=0.05)
    assert gy < 0                                        # clearance falls toward the edge


def test_vectorised_matches_scalar(course):
    xs = np.array([1.0, 3.5, 7.18])
    ys = np.array([4.75, 4.75, 1.30])
    v = course.clearance(xs, ys)
    assert v == pytest.approx([course.clearance(x, y) for x, y in zip(xs, ys)])


def test_js_round_half_up():
    assert js_round(62.5) == 63 and js_round(-0.5) == 0 and js_round(2.4999) == 2


def test_arc_sampling_like_v4():
    p = sample_arc(1, 1.5, .75, math.pi, 1.5 * math.pi)
    assert len(p) == 49 and p[0] == pytest.approx([0.25, 1.5]) and p[-1] == pytest.approx([1.0, 0.75])


def test_features(course):
    assert course.start_pose() == pytest.approx((7.18, 1.30, math.pi), abs=1e-6)
    assert course.in_tunnel(1.0 - 0.75, 1.5)
    assert not course.in_tunnel(5.0, 4.75)
    assert course.surface(3.0, 4.75) == pytest.approx(0.155)
