"""Synthetic cameras for the tests: render what a camera would see of a flat
floor texture, using the same lens + mount model as block 03."""
import numpy as np

from carbot_perception.camera_model import Intrinsics, Mount, pixels_to_ground

ROAD_BGR = (60, 62, 64)      # dark grey mat
PAINT_BGR = (235, 235, 235)  # white line
GRASS_BGR = (60, 140, 70)    # green outside
SKY_BGR = (230, 200, 150)


def lane_texture(x, y):
    """Straight road along +x: road |y| < 0.35, white lines 0.35..0.37, green outside."""
    out = np.empty(x.shape + (3,), np.uint8)
    out[:] = GRASS_BGR
    ay = np.abs(y)
    out[ay < 0.37] = PAINT_BGR
    out[ay < 0.35] = ROAD_BGR
    return out


def render(intr: Intrinsics, mount: Mount, texture=lane_texture) -> np.ndarray:
    u, v = np.meshgrid(np.arange(intr.width, dtype=float), np.arange(intr.height, dtype=float))
    g, ok = pixels_to_ground(intr, mount, np.column_stack([u.ravel(), v.ravel()]))
    img = np.empty((u.size, 3), np.uint8)
    img[:] = SKY_BGR
    ok &= np.hypot(g[:, 0], g[:, 1]) < 6.0
    img[ok] = texture(g[ok, 0], g[ok, 1])
    return img.reshape(intr.height, intr.width, 3)


def fisheye(width=480, height=272, f=150.0):
    K = np.array([[f, 0, width / 2 - 0.5 + 3.0], [0, f, height / 2 - 0.5 - 2.0], [0, 0, 1.0]])
    return Intrinsics(width, height, K, np.array([0.05, -0.02, 0.004, -0.001]), 'equidistant', 'calibrated')


def pinhole(width=480, height=272, f=330.0):
    K = np.array([[f, 0, width / 2 - 0.5], [0, f * 1.002, height / 2 - 0.5], [0, 0, 1.0]])
    return Intrinsics(width, height, K, np.array([-0.28, 0.09, 0.0008, -0.0005, -0.012]),
                      'plumb_bob', 'calibrated')


# The RISA Bot mounts measured in CAD (phase 2 chat), base_link = rear axle on ground.
MOUNTS = {
    'front': Mount(0.2170, 0.0114, 0.0935, 0.0, np.radians(2.5)),
    'left_rear': Mount(0.1145, 0.0654, 0.1406, np.radians(95.0), np.radians(10.0)),
    'right_rear': Mount(0.1145, -0.0654, 0.1406, np.radians(-95.0), np.radians(10.0)),
}
