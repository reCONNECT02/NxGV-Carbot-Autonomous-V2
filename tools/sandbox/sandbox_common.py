"""Shared helpers for the VS Code sandbox (no ROS needed).

Every runner imports the REAL node code from src/ (camera_model, road_mask,
memory_core, calib_*), so what you test here is what runs on the car. Only the
ROS plumbing (subscribers, publishers, timers) is replaced by plain loops.

World: the competition track from config/data/track_map.yaml rasterised at
5 mm per pixel (dark road, white lane edges and markings, green outside), and
three virtual cameras placed with the mounts in cameras.yaml.
"""
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
for p in (os.path.join(REPO, 'src', 'carbot_perception'), os.path.join(REPO, 'src', 'carbot_common')):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from carbot_perception.camera_model import (Intrinsics, Mount, camera_setup,  # noqa: E402
                                            intrinsics_for, pixels_to_ground)

CONFIG = os.path.join(REPO, 'src', 'carbot_bringup', 'config')
OUT = os.path.join(HERE, 'out')
ROLES = ('front', 'left_rear', 'right_rear')

# BGR colours of the virtual venue (tune to look like the real mat if you like)
ROAD = (58, 60, 62)
PAINT = (238, 238, 238)
OUTSIDE = (70, 128, 80)
FAR = (205, 195, 185)


# --------------------------------------------------------------------------- config
def load_yaml(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def params(node: str) -> Dict:
    """ros__parameters of `node` from config/params/*.yaml (same values the launch uses)."""
    for name in sorted(os.listdir(os.path.join(CONFIG, 'params'))):
        doc = load_yaml(os.path.join(CONFIG, 'params', name))
        if node in doc:
            return doc[node]['ros__parameters']
    raise KeyError(node)


def common() -> Dict:
    return load_yaml(os.path.join(CONFIG, 'params', 'common.yaml'))['/**']['ros__parameters']


def cameras(path: str = '') -> Dict:
    """cameras.yaml: a calibration session copy if given, else the repo default."""
    return load_yaml(path or os.path.join(CONFIG, 'data', 'cameras.yaml'))


def out_dir(*parts) -> str:
    d = os.path.join(OUT, *parts)
    os.makedirs(d, exist_ok=True)
    return d


# --------------------------------------------------------------------------- track world
class TrackWorld:
    """track_map.yaml rasterised to a floor texture."""

    def __init__(self, res: float = 0.005, edge_m: float = 0.02):
        t = load_yaml(os.path.join(CONFIG, 'data', 'track_map.yaml'))
        self.t = t
        self.res = res
        w, h = t['extent_m']
        self.nx, self.ny = int(math.ceil(w / res)) + 1, int(math.ceil(h / res)) + 1
        lane = t['lane_width_m']
        road = np.zeros((self.ny, self.nx), np.uint8)
        for c in t['centrelines']:
            pts = self._centreline(c)
            cv2.polylines(road, [self._px(pts)], False, 255, max(1, int(round(lane / res))), cv2.LINE_8)
        for name in t.get('drivable_areas', []):
            x0, x1, y0, y1 = t['areas'][name]
            cv2.rectangle(road, self._px([(x0, y0)])[0], self._px([(x1, y1)])[0], 255, -1)
        for key in ('east',):
            fl = t['flares'][key]
            polys = [fl, [(4.8 - x, y) for x, y in fl],
                     [(2.4 - (y - 0.8), 0.8 + (x - 2.4)) for x, y in fl]]
            for poly in polys:
                cv2.fillPoly(road, [self._px(poly)], 255)
        tex = np.empty((self.ny, self.nx, 3), np.uint8)
        tex[:] = OUTSIDE
        tex[road > 0] = ROAD
        # white edge line on the inside of every road boundary
        k = max(1, int(round(edge_m / res)))
        eroded = cv2.erode(road, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
        tex[(road > 0) & (eroded == 0)] = PAINT
        for m in t.get('markings', []):
            for (x0, x1, y0, y1) in self._marking_rects(m):
                cv2.rectangle(tex, self._px([(x0, y0)])[0], self._px([(x1, y1)])[0], PAINT, -1)
        self.road = road
        self.tex = tex

    def _px(self, pts) -> np.ndarray:
        a = np.asarray(pts, float).reshape(-1, 2)
        return np.column_stack([a[:, 0] / self.res, a[:, 1] / self.res]).round().astype(np.int32)

    @staticmethod
    def _centreline(c, step: float = 0.01) -> List[Tuple[float, float]]:
        if 'line' in c:
            x0, y0, x1, y1 = c['line']
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
            return [(x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]
        cx, cy, r, a0, a1 = c['arc']
        n = max(3, int(abs(a1 - a0) * r / step))
        return [(cx + r * math.cos(a0 + (a1 - a0) * i / (n - 1)),
                 cy + r * math.sin(a0 + (a1 - a0) * i / (n - 1))) for i in range(n)]

    @staticmethod
    def _marking_rects(m):
        if 'rect' in m:
            return [tuple(m['rect'])]
        d = m['dashed']
        out, s = [], d['from']
        while s < d['to']:
            e = min(s + d['dash'], d['to'])
            a0, a1 = d['across']
            out.append((s, e, a0, a1) if d['axis'] == 'x' else (a0, a1, s, e))
            s += d['period']
        return out

    def sample(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        ix = np.round(x / self.res).astype(np.int64)
        iy = np.round(y / self.res).astype(np.int64)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.nx) & (iy < self.ny)
        out = np.empty(x.shape + (3,), np.uint8)
        out[:] = OUTSIDE
        out[ok] = self.tex[iy[ok], ix[ok]]
        return out

    def top_view(self, scale: float = 0.25) -> np.ndarray:
        img = cv2.flip(self.tex, 0)                     # +y up on screen
        return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def start_pose(self) -> Tuple[float, float, float]:
        s = self.t['start_pose']
        return s['x'], s['y'], s['a']

    def demo_route(self, step: float = 0.01) -> List[Tuple[float, float, float]]:
        """Top straight -> top-right corner -> right straight -> bottom-right
        corner -> start lane westwards (about 9 m, two 90 deg bends)."""
        segs = [('line', (1.00, 4.75, 6.00, 4.75)),
                ('arc', (6.00, 4.00, 0.75, math.pi / 2, 0.0)),
                ('line', (6.75, 4.00, 6.75, 2.05)),
                ('arc', (6.00, 2.05, 0.75, 0.0, -math.pi / 2)),
                ('line', (6.00, 1.30, 4.40, 1.30))]
        pts = []
        for kind, v in segs:
            pts += self._centreline({kind: list(v)}, step)
        out = []
        for i in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            out.append((x0, y0, math.atan2(y1 - y0, x1 - x0)))
        return out


# --------------------------------------------------------------------------- virtual cameras
def synth_lens(kind: str, width: int, height: int, hfov_deg: float) -> Intrinsics:
    """pinhole (ideal) or fisheye (equidistant with the same horizontal FOV)."""
    if kind == 'pinhole':
        return Intrinsics.ideal(width, height, hfov_deg)
    f = (width / 2.0) / math.radians(hfov_deg / 2.0)       # equidistant: r = f * theta
    K = np.array([[f, 0, width / 2.0 - 0.5], [0, f, height / 2.0 - 0.5], [0, 0, 1.0]])
    return Intrinsics(width, height, K, np.array([0.02, -0.01, 0.002, 0.0]), 'equidistant', 'calibrated')


@dataclass
class VirtualCamera:
    role: str
    intr: Intrinsics
    mount: Mount
    ground: np.ndarray          # (H*W, 2) base_link ground point per pixel (NaN = sky)

    @classmethod
    def build(cls, role: str, intr: Intrinsics, mount: Mount) -> 'VirtualCamera':
        u, v = np.meshgrid(np.arange(intr.width, dtype=float), np.arange(intr.height, dtype=float))
        g, ok = pixels_to_ground(intr, mount, np.column_stack([u.ravel(), v.ravel()]))
        g[~ok] = np.nan
        far = np.hypot(g[:, 0], g[:, 1]) > 4.0
        g[far] = np.nan
        return cls(role, intr, mount, g)

    def render(self, world: TrackWorld, pose: Tuple[float, float, float]) -> np.ndarray:
        x, y, a = pose
        c, s = math.cos(a), math.sin(a)
        ok = np.isfinite(self.ground[:, 0])
        gx, gy = self.ground[ok, 0], self.ground[ok, 1]
        img = np.empty((self.ground.shape[0], 3), np.uint8)
        img[:] = FAR
        img[ok] = world.sample(x + gx * c - gy * s, y + gx * s + gy * c)
        return img.reshape(self.intr.height, self.intr.width, 3)


def camera_rig(cams: Dict, width: int, lens: str = 'auto') -> Dict[str, VirtualCamera]:
    """Virtual cameras at the cameras.yaml mounts, rendered `width` pixels wide.
    lens='auto' uses the calibrated intrinsics if cameras.yaml points to them
    (else ideal pinhole from hfov_deg); 'pinhole' / 'fisheye' force a lens."""
    rig = {}
    for role in ROLES:
        sensor, s, mount, hfov = camera_setup(cams, role)
        sw, sh = int(s.get('image_width', s.get('width', 640))), int(s.get('image_height', s.get('height', 480)))
        h = int(round(sh * width / sw))
        intr = intrinsics_for(cams, role, width, h) if lens == 'auto' else synth_lens(lens, width, h, hfov)
        rig[role] = VirtualCamera.build(role, intr, mount)
    return rig


# --------------------------------------------------------------------------- display
def label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 18), (0, 0, 0), -1)
    cv2.putText(out, text, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def tile(images: List[np.ndarray], cols: int, cell_w: int) -> np.ndarray:
    """Resize every image to cell_w wide and lay them out in a grid."""
    cells = []
    for im in images:
        if im is None:
            continue
        h = int(round(im.shape[0] * cell_w / im.shape[1]))
        cells.append(cv2.resize(im, (cell_w, h), interpolation=cv2.INTER_AREA))
    if not cells:
        return np.zeros((10, 10, 3), np.uint8)
    rows = []
    for i in range(0, len(cells), cols):
        row = cells[i:i + cols]
        hmax = max(c.shape[0] for c in row)
        row = [cv2.copyMakeBorder(c, 0, hmax - c.shape[0], 0, 0, cv2.BORDER_CONSTANT) for c in row]
        row += [np.zeros((hmax, cell_w, 3), np.uint8)] * (cols - len(row))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def show(name: str, img: np.ndarray, headless: bool, save_path: Optional[str] = None) -> int:
    """imshow + waitKey; in headless mode just save. Returns the key (or -1)."""
    if save_path:
        cv2.imwrite(save_path, img)
    if headless:
        return -1
    cv2.imshow(name, img)
    return cv2.waitKey(1) & 0xFF
