"""camera_model: V4 equivalence, frame conventions, lens models (no ROS)."""
import math

import numpy as np
import pytest
from synth import MOUNTS, fisheye, pinhole

from carbot_perception.camera_model import (Intrinsics, Mount, pixels_to_ground,
                                            project_ground)


def v4_project(s, px, py):
    """Verbatim V4 perception.js projectionMap maths (before rounding)."""
    cy, sy, cd, sd = math.cos(s['yaw']), math.sin(s['yaw']), math.cos(s['down']), math.sin(s['down'])
    fx = 96 / math.tan(100 * math.pi / 360)
    dx, dy, dz = px - s['x'], py - s['y'], -s['z']
    depth = dx * cy * cd + dy * sy * cd - dz * sd
    right = dx * sy - dy * cy
    up = dx * cy * sd + dy * sy * sd + dz * cd
    return 96 + fx * right / depth, 72 - fx * up / depth, depth


@pytest.mark.parametrize('spec', [
    dict(x=.225, y=0, z=.18, yaw=0, down=math.radians(25)),
    dict(x=.015, y=.071, z=.15, yaw=math.radians(135), down=math.radians(50)),
    dict(x=.015, y=-.071, z=.15, yaw=math.radians(-135), down=math.radians(50)),
])
def test_matches_v4_projection(spec):
    intr = Intrinsics.ideal(192, 144, 100.0)
    m = Mount(spec['x'], spec['y'], spec['z'], spec['yaw'], spec['down'])
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-0.6, 1.1, 300), rng.uniform(-0.9, 0.9, 300), np.zeros(300)])
    uv, ok, depth = project_ground(intr, m, pts, 0.025)
    for i, (x, y, _) in enumerate(pts):
        u4, v4, d4 = v4_project(spec, x, y)
        assert depth[i] == pytest.approx(d4, abs=1e-9)
        if ok[i]:
            assert uv[i, 0] == pytest.approx(u4, abs=1e-6)
            assert uv[i, 1] == pytest.approx(v4, abs=1e-6)


def test_mount_rotation_roundtrip():
    for m in MOUNTS.values():
        m2 = Mount.from_r_base_opt(m.r_base_opt(), m.position)
        dpos, dang = m.difference(m2)
        assert dpos < 1e-9 and dang < 1e-6
        assert m2.yaw == pytest.approx(m.yaw) and m2.pitch_down == pytest.approx(m.pitch_down)


def test_pitch_down_looks_at_ground():
    m = MOUNTS['right_rear']
    axis = m.r_base_body()[:, 0]
    assert axis[2] < 0                       # optical axis points down
    assert axis[1] < -0.9                    # right camera looks to -y
    assert axis[0] < 0                       # 5 deg towards the back


@pytest.mark.parametrize('make', [fisheye, pinhole])
def test_ground_roundtrip_with_distortion(make):
    intr = make()
    m = MOUNTS['left_rear']
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-0.3, 0.6, 400), rng.uniform(0.25, 0.9, 400), np.zeros(400)])
    uv, ok, _ = project_ground(intr, m, pts)
    inside = ok & (uv[:, 0] >= 0) & (uv[:, 0] < intr.width) & (uv[:, 1] >= 0) & (uv[:, 1] < intr.height)
    assert inside.sum() > 50
    g, gok = pixels_to_ground(intr, m, uv[inside])
    assert gok.all()
    assert np.max(np.linalg.norm(g - pts[inside, :2], axis=1)) < 1e-4


def test_scaled_intrinsics_consistent():
    big = pinhole(960, 544, 660.0)
    small = big.scaled(480, 272)
    m = MOUNTS['front']
    pts = np.array([[0.6, 0.05, 0.0], [0.9, -0.2, 0.0]])
    ub, okb, _ = project_ground(big, m, pts)
    us, oks, _ = project_ground(small, m, pts)
    assert okb.all() and oks.all()
    assert np.allclose((ub + 0.5) / 2 - 0.5, us, atol=1e-6)


def test_camera_info_roundtrip(tmp_path):
    from carbot_perception.camera_model import load_intrinsics, save_intrinsics
    for make in (fisheye, pinhole):
        a = make()
        p = tmp_path / f'{a.model}.yaml'
        save_intrinsics(str(p), a, 'test')
        b = load_intrinsics(str(p))
        assert b.model == a.model and np.allclose(a.K, b.K) and np.allclose(a.D, b.D)
