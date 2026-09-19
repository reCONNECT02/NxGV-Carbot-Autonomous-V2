import math

from carbot_common.geometry import Pose, bicycle, geometry, to_local, to_world, wrap

V = dict(car_length_m=0.30, car_width_m=0.192, wheelbase_m=0.216, rear_overhang_m=0.042,
         min_turning_radius_m=0.40, wheel_width_m=0.025)


def test_geometry_matches_v4():
    g = geometry(V)
    assert abs(g.front - 0.258) < 1e-9
    assert abs(g.max_steer - math.atan(0.216 / 0.40)) < 1e-12


def test_bicycle_straight_and_circle():
    p = bicycle(Pose(0, 0, 0), 1.0, 0.0)
    assert abs(p.x - 1.0) < 1e-12 and abs(p.y) < 1e-12
    r = 0.4
    q = bicycle(Pose(0, 0, 0), math.pi * r, 1 / r)   # half circle to the left
    assert abs(q.x) < 1e-9 and abs(q.y - 2 * r) < 1e-9 and abs(abs(wrap(q.a)) - math.pi) < 1e-9


def test_world_local_roundtrip():
    p = Pose(1.0, 2.0, 0.7)
    wx, wy = to_world(p, 0.3, -0.1)
    lx, ly = to_local(p, wx, wy)
    assert abs(lx - 0.3) < 1e-12 and abs(ly + 0.1) < 1e-12
