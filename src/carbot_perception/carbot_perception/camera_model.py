"""Camera model shared by block 03, the calibration tools and (later) the wizard.

Pure numpy + OpenCV, no ROS imports, so it is unit tested on a laptop.

Frames
------
base_link      rear-axle centre ON THE GROUND, +x forward, +y left, +z up.
cam_<role>     body-style camera frame: +x along the optical axis, +y left, +z up.
               Mount = x, y, z, yaw, pitch_down, roll with
               R_base_cam = Rz(yaw) @ Ry(pitch_down) @ Rx(roll)
               (identical to the static TF stack.py publishes; pitch_down > 0
               tilts the optical axis towards the ground).
cam_<role>_optical   ROS optical frame: +z forward, +x right, +y down.

This reproduces V4 perception.js projectionMap exactly for an ideal pinhole
camera (test_camera_model.py checks it), and adds real lens distortion:

  plumb_bob            OpenCV standard model (k1 k2 p1 p2 k3), lenses up to ~100 deg
  rational_polynomial  OpenCV rational model (8 coefficients)
  equidistant          OpenCV fisheye model (k1..k4), wide lenses

Fisheye is not "undistorted" as a separate step: block 03 projects every grid
cell straight into the RAW image with the lens model, so undistortion and the
top-down warp happen in one cv2.remap.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

PINHOLE_MODELS = ('plumb_bob', 'rational_polynomial')
FISHEYE_MODELS = ('equidistant',)
MODELS = PINHOLE_MODELS + FISHEYE_MODELS

# columns = optical x, y, z axes expressed in the body camera frame
R_BODY_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# --------------------------------------------------------------------------- mount
@dataclass
class Mount:
    """Camera pose in base_link. Angles in radians."""
    x: float
    y: float
    z: float
    yaw: float
    pitch_down: float
    roll: float = 0.0

    @classmethod
    def from_yaml(cls, m: Dict) -> 'Mount':
        return cls(float(m['x_m']), float(m['y_m']), float(m['z_m']),
                   math.radians(float(m['yaw_deg'])), math.radians(float(m['pitch_down_deg'])),
                   math.radians(float(m.get('roll_deg', 0.0))))

    def to_yaml(self, extra: Optional[Dict] = None) -> Dict:
        out = {'x_m': round(self.x, 4), 'y_m': round(self.y, 4), 'z_m': round(self.z, 4),
               'yaw_deg': round(math.degrees(self.yaw), 2),
               'pitch_down_deg': round(math.degrees(self.pitch_down), 2),
               'roll_deg': round(math.degrees(self.roll), 2)}
        out.update(extra or {})
        return out

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def r_base_body(self) -> np.ndarray:
        return rot_z(self.yaw) @ rot_y(self.pitch_down) @ rot_x(self.roll)

    def r_base_opt(self) -> np.ndarray:
        return self.r_base_body() @ R_BODY_OPT

    def base_to_optical(self, pts: np.ndarray) -> np.ndarray:
        """(N,3) base_link points -> (N,3) optical-frame points."""
        return (np.asarray(pts, float) - self.position) @ self.r_base_opt()

    @classmethod
    def from_r_base_opt(cls, r_base_opt: np.ndarray, position: np.ndarray) -> 'Mount':
        r = r_base_opt @ R_BODY_OPT.T          # = Rz(yaw) Ry(pitch) Rx(roll)
        pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
        yaw = math.atan2(r[1, 0], r[0, 0])
        roll = math.atan2(r[2, 1], r[2, 2])
        p = np.asarray(position, float).ravel()
        return cls(float(p[0]), float(p[1]), float(p[2]), yaw, pitch, roll)

    def difference(self, other: 'Mount') -> Tuple[float, float]:
        """(position distance m, rotation angle deg) between two mounts."""
        d = float(np.linalg.norm(self.position - other.position))
        r = self.r_base_body().T @ other.r_base_body()
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))))
        return d, ang


# --------------------------------------------------------------------------- intrinsics
@dataclass
class Intrinsics:
    width: int
    height: int
    K: np.ndarray
    D: np.ndarray
    model: str = 'plumb_bob'
    source: str = 'assumed'          # 'calibrated' | 'assumed'
    meta: Dict = field(default_factory=dict)
    _theta_max: Optional[float] = field(default=None, repr=False)

    def __post_init__(self):
        if self.model not in MODELS:
            raise ValueError(f'unsupported distortion model {self.model!r} (use one of {MODELS})')
        self.K = np.asarray(self.K, float).reshape(3, 3)
        self.D = np.asarray(self.D, float).ravel()
        if self.model == 'equidistant' and self.D.size != 4:
            raise ValueError('equidistant needs exactly 4 coefficients')

    @property
    def fisheye(self) -> bool:
        return self.model in FISHEYE_MODELS

    # ---------------------------------------------------------------- builders
    @classmethod
    def ideal(cls, width: int, height: int, hfov_deg: float) -> 'Intrinsics':
        """Distortion-free pinhole from an assumed horizontal FOV (V4: fx = (w/2)/tan(hfov/2))."""
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        K = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
        return cls(int(width), int(height), K, np.zeros(5), 'plumb_bob', 'assumed',
                   {'hfov_deg': float(hfov_deg)})

    def scaled(self, width: int, height: int) -> 'Intrinsics':
        """Same lens at another resolution (pixel-centre convention)."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        K = self.K.copy()
        K[0, 0] *= sx
        K[1, 1] *= sy
        K[0, 1] *= sx
        K[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
        K[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
        return Intrinsics(int(width), int(height), K, self.D.copy(), self.model, self.source,
                          dict(self.meta, scaled_from=[self.width, self.height]))

    def aspect_matches(self, width: int, height: int, tol: float = 0.01) -> bool:
        return abs((width / height) / (self.width / self.height) - 1.0) <= tol

    # ---------------------------------------------------------------- projection
    def project(self, pts_opt: np.ndarray, min_depth: float = 1e-6) -> Tuple[np.ndarray, np.ndarray]:
        """Optical-frame points (N,3) -> pixels (N,2) and a validity mask.

        Invalid: behind the camera, or outside the field angle the lens model was
        fitted for (polynomial distortion models fold back beyond it)."""
        p = np.asarray(pts_opt, float).reshape(-1, 3)
        uv = np.full((p.shape[0], 2), np.nan)
        z = p[:, 2]
        theta = np.arctan2(np.hypot(p[:, 0], p[:, 1]), z)
        ok = (z > min_depth) & (theta <= self.theta_max() * 1.02)
        if not ok.any():
            return uv, ok
        q = p[ok].reshape(-1, 1, 3)
        zero = np.zeros((3, 1))
        if self.fisheye:
            out, _ = cv2.fisheye.projectPoints(q, zero, zero, self.K, self.D.reshape(4, 1))
        else:
            out, _ = cv2.projectPoints(q, zero, zero, self.K, self.D)
        uv[ok] = out.reshape(-1, 2)
        return uv, ok

    def normalize(self, uv: np.ndarray) -> np.ndarray:
        """Pixels (N,2) -> undistorted normalized coordinates (N,2) (x/z, y/z).
        For the fisheye model points beyond 90 deg are not representable (NaN)."""
        pts = np.asarray(uv, float).reshape(-1, 1, 2)
        if self.fisheye:
            out = cv2.fisheye.undistortPoints(pts, self.K, self.D.reshape(4, 1))
        else:
            crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-9)
            try:
                out = cv2.undistortPointsIter(pts, self.K, self.D, None, None, crit)
            except (AttributeError, cv2.error):
                out = cv2.undistortPoints(pts, self.K, self.D)
        return out.reshape(-1, 2)

    def rays(self, uv: np.ndarray) -> np.ndarray:
        """Pixels (N,2) -> unit rays (N,3) in the optical frame."""
        n = self.normalize(uv)
        r = np.column_stack([n, np.ones(len(n))])
        return r / np.linalg.norm(r, axis=1, keepdims=True)

    def theta_max(self) -> float:
        """Largest field angle (rad) seen anywhere on the image border."""
        if self._theta_max is None:
            w, h = self.width - 1.0, self.height - 1.0
            t = np.linspace(0.0, 1.0, 41)
            border = np.concatenate([np.column_stack([t * w, 0 * t]), np.column_stack([t * w, 0 * t + h]),
                                     np.column_stack([0 * t, t * h]), np.column_stack([0 * t + w, t * h])])
            n = self.normalize(border)
            th = np.arctan(np.hypot(n[:, 0], n[:, 1]))
            th = th[np.isfinite(th)]
            self._theta_max = float(th.max()) if th.size else math.radians(89.0)
        return self._theta_max

    def hfov_deg(self) -> float:
        n = self.normalize(np.array([[0.0, self.K[1, 2]], [self.width - 1.0, self.K[1, 2]]]))
        return math.degrees(abs(math.atan(n[0, 0])) + abs(math.atan(n[1, 0])))

    # ---------------------------------------------------------------- ROS camera_info yaml
    def to_camera_info(self, camera_name: str) -> Dict:
        P = np.zeros((3, 4))
        P[:3, :3] = self.K
        return {
            'image_width': int(self.width), 'image_height': int(self.height),
            'camera_name': camera_name,
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in self.K.ravel()]},
            'distortion_model': self.model,
            'distortion_coefficients': {'rows': 1, 'cols': int(self.D.size),
                                        'data': [float(v) for v in self.D]},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': [float(v) for v in np.eye(3).ravel()]},
            'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(v) for v in P.ravel()]},
            'carbot_meta': dict(self.meta),
        }

    @classmethod
    def from_camera_info(cls, d: Dict) -> 'Intrinsics':
        return cls(int(d['image_width']), int(d['image_height']),
                   np.array(d['camera_matrix']['data'], float).reshape(3, 3),
                   np.array(d['distortion_coefficients']['data'], float),
                   d.get('distortion_model', 'plumb_bob'), 'calibrated',
                   dict(d.get('carbot_meta') or {}))


def load_intrinsics(path: str) -> Intrinsics:
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        return Intrinsics.from_camera_info(yaml.safe_load(f))


def save_intrinsics(path: str, intr: Intrinsics, camera_name: str) -> None:
    import os
    import yaml
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(intr.to_camera_info(camera_name), f, sort_keys=False)


# --------------------------------------------------------------------------- ground geometry
def project_ground(intr: Intrinsics, mount: Mount, pts_base: np.ndarray,
                   min_depth: float = 1e-6) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """base_link points -> (pixels (N,2), valid (N,), depth along optical axis (N,))."""
    po = mount.base_to_optical(pts_base)
    uv, ok = intr.project(po, min_depth)
    return uv, ok, po[:, 2]


def pixels_to_ground(intr: Intrinsics, mount: Mount, uv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Pixels -> ground points (N,2) in base_link, valid where the ray hits z=0 ahead."""
    rays_opt = intr.rays(uv)
    rays_base = rays_opt @ mount.r_base_opt().T
    c = mount.position
    rz = rays_base[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = -c[2] / rz
    ok = np.isfinite(t) & (rz < -1e-6) & (t > 0)
    g = c[None, :2] + t[:, None] * rays_base[:, :2]
    g[~ok] = np.nan
    return g, ok


def camera_setup(cameras: Dict, role: str) -> Tuple[str, Dict, Mount, float]:
    """(sensor name, sensor dict, mount, assumed hfov) for one role of cameras.yaml."""
    sensor = cameras['roles'][role]
    m = cameras['mounts'][role]
    return sensor, cameras['sensors'][sensor], Mount.from_yaml(m), float(m.get('hfov_deg', 90.0))


def intrinsics_for(cameras: Dict, role: str, width: int, height: int) -> Intrinsics:
    """Calibrated intrinsics if the sensor has an intrinsics_file, else an ideal
    pinhole from the mount's assumed hfov_deg. Scaled to (width, height).
    Raises ValueError if a calibration exists for another aspect ratio."""
    import os
    _, s, _, hfov = camera_setup(cameras, role)
    path = (s.get('intrinsics_file') or '').strip()
    if path and os.path.isfile(os.path.expanduser(path)):
        intr = load_intrinsics(os.path.expanduser(path))
        if not intr.aspect_matches(width, height):
            raise ValueError(f'{role}: intrinsics {path} are for {intr.width}x{intr.height}, '
                             f'images are {width}x{height} (different aspect ratio: recalibrate)')
        return intr.scaled(width, height)
    return Intrinsics.ideal(width, height, hfov)
