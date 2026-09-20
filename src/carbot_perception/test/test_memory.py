import math

import numpy as np

from carbot_perception.memory_core import LocalMemory, OdomHistory, OdomPose
from carbot_perception.road_mask import GridSpec


def test_integrate_query_window():
    g = GridSpec(100, 0.018, -0.65, -0.90)
    gx, gy = g.centres()
    kind = np.zeros((100, 100), np.uint8)
    kind[np.abs(gy) < 0.3] = 1
    mem = LocalMemory()
    pose = OdomPose(2.0, 1.0, math.pi / 2, 5.0)
    mem.integrate(kind.ravel(), gx.ravel(), gy.ravel(), pose, 10.0)
    # a point 0.5 m ahead of the car in odom = (2.0, 1.5)
    found, k, age, dist = mem.query(np.array([2.0, 2.5]), np.array([1.5, 1.0]), 11.0, 3.0)
    assert found[0] and k[0] == 1 and age[0] == 1.0 and dist[0] == 5.0
    assert not found[1]                           # 0.5 m to the right: not road
    found, *_ = mem.query(np.array([2.0]), np.array([1.5]), 14.5, 3.0)
    assert not found[0]                           # too old for driving
    x0, y0, n, kind_w, age_w, _ = mem.window(2.0, 1.0, 1.5, 11.0, 25.0)
    assert n == 120 and (kind_w == 1).sum() > 1000 and age_w.max() == 1.0


def test_odom_history_interpolates():
    h = OdomHistory(2.0)
    h.add(0.0, 0.0, 0.0, 0.0)
    h.add(0.1, 0.1, 0.0, 0.2)
    p = h.at(0.05)
    assert abs(p.x - 0.05) < 1e-9 and abs(p.a - 0.1) < 1e-9 and abs(p.distance - 0.05) < 1e-9
    assert h.at(5.0) is None
