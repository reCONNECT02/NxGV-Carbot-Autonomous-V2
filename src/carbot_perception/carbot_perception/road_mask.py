"""BLOCK 03 algorithm (V4 perception.js), no ROS imports.

Per camera, once:   for every grid cell centre (x, y, 0) in base_link compute the
                    pixel it lands on in the RAW (distorted) image -> remap tables,
                    and weight 1 / (weight_eps + depth^2) (depth along the optical axis).
Per frame:          cv2.remap each image onto the grid (undistort + top-down warp
                    in one step), keep per cell the view with the largest weight
                    among the cameras that are fresh, classify the colour
                    (road / paint / other), grow the connected road region
                    (4-neighbour) from a seed box near the car.

Grid layout = LocalGrid: row index along +x, column index along +y, cell
(row, col) centre = (x0 + (row + .5) res, y0 + (col + .5) res).
"""
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from .camera_model import Intrinsics, Mount, pixels_to_ground, project_ground

UNSEEN, ROAD, PAINT, OTHER = 0, 1, 2, 3
ROLE_CODE = {'front': 1, 'left_rear': 2, 'right_rear': 3}

# V4 mask colours (RGB) -> stored BGR for OpenCV
_MASK_RGB = {'grown': (48, 205, 149), 'paint': (249, 240, 202), 'unseen': (24, 33, 48),
             'other': (69, 80, 95), 'road': (40, 110, 160)}
MASK_BGR = {k: v[::-1] for k, v in _MASK_RGB.items()}


@dataclass
class GridSpec:
    cells: int
    res: float
    x0: float
    y0: float

    def centres(self) -> Tuple[np.ndarray, np.ndarray]:
        idx = (np.arange(self.cells) + 0.5) * self.res
        xs = self.x0 + idx                       # per row
        ys = self.y0 + idx                       # per col
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        return gx, gy

    def cell_of(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Flat cell index for base_link points, -1 outside."""
        r = np.floor((x - self.x0) / self.res)
        c = np.floor((y - self.y0) / self.res)
        ok = np.isfinite(r) & np.isfinite(c) & (r >= 0) & (c >= 0) & (r < self.cells) & (c < self.cells)
        out = np.full(np.shape(x), -1, np.int32)
        out[ok] = (r[ok] * self.cells + c[ok]).astype(np.int32)
        return out


@dataclass
class BodyBox:
    """Car footprint in base_link (cells inside are never observed)."""
    rear: float
    front: float
    half_width: float


@dataclass
class CameraMap:
    role: str
    width: int
    height: int
    map_x: np.ndarray       # (n, n) float32 pixel x, -1 where invalid
    map_y: np.ndarray
    weight: np.ndarray      # (n, n) float32, 0 where the camera does not see the cell
    intr: Intrinsics
    mount: Mount

    @property
    def valid(self) -> np.ndarray:
        return self.weight > 0


def build_camera_map(role: str, grid: GridSpec, intr: Intrinsics, mount: Mount,
                     body: Optional[BodyBox], min_depth: float, weight_eps: float,
                     border_px: float) -> CameraMap:
    gx, gy = grid.centres()
    pts = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
    uv, ok, depth = project_ground(intr, mount, pts, min_depth)
    ok &= depth > min_depth
    with np.errstate(invalid='ignore'):
        ok &= (uv[:, 0] >= border_px) & (uv[:, 0] <= intr.width - 1 - border_px)
        ok &= (uv[:, 1] >= border_px) & (uv[:, 1] <= intr.height - 1 - border_px)
    if body is not None:
        inside = (pts[:, 0] > -body.rear) & (pts[:, 0] < body.front) & (np.abs(pts[:, 1]) < body.half_width)
        ok &= ~inside
    n = grid.cells
    mx = np.where(ok, uv[:, 0], -1.0).astype(np.float32).reshape(n, n)
    my = np.where(ok, uv[:, 1], -1.0).astype(np.float32).reshape(n, n)
    w = np.where(ok, 1.0 / (weight_eps + depth * depth), 0.0).astype(np.float32).reshape(n, n)
    return CameraMap(role, intr.width, intr.height, mx, my, w, intr, mount)


@dataclass
class Classify:
    road_max_luma: float = 105.0
    road_max_chroma: float = 50.0
    paint_min_luma: float = 190.0


@dataclass
class Seed:
    x_min: float = -0.25
    x_max: float = 0.45
    half_width: float = 0.19


@dataclass
class MaskResult:
    kind: np.ndarray        # (n, n) uint8 UNSEEN/ROAD/PAINT/OTHER
    grown: np.ndarray       # (n, n) uint8 0/1
    source: np.ndarray      # (n, n) uint8 ROLE_CODE of the winning camera, 0 none
    bgr: np.ndarray         # (n, n, 3) uint8 stitched colour
    warped: Dict[str, np.ndarray]   # role -> (n, n, 3) uint8 per-camera warp (black where unseen)
    coverage: float
    connected: int


class RoadMask:

    def __init__(self, grid: GridSpec, seed: Seed):
        self.grid = grid
        self.maps: Dict[str, CameraMap] = {}
        self.seed = None
        self.set_seed(seed)

    def set_seed(self, seed: Seed) -> None:
        if seed == self.seed:
            return
        self.seed = seed
        gx, gy = self.grid.centres()
        self._seed_box = (gx > seed.x_min) & (gx < seed.x_max) & (np.abs(gy) < seed.half_width)

    def process(self, images: Dict[str, np.ndarray], cls: Classify) -> MaskResult:
        """images: role -> BGR uint8 image at that role's map resolution
        (roles missing or None are treated as stale)."""
        n = self.grid.cells
        roles = [r for r in self.maps if images.get(r) is not None]
        warped: Dict[str, np.ndarray] = {}
        if roles:
            weights = np.stack([self.maps[r].weight for r in roles])       # (k, n, n)
            samples = []
            for r in roles:
                m = self.maps[r]
                img = images[r]
                if img.shape[1] != m.width or img.shape[0] != m.height:
                    raise ValueError(f'{r}: image {img.shape[1]}x{img.shape[0]} != map {m.width}x{m.height}')
                s = cv2.remap(img, m.map_x, m.map_y, cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
                s[~m.valid] = 0
                warped[r] = s
                samples.append(s)
            best = np.argmax(weights, axis=0)
            seen = np.take_along_axis(weights, best[None], 0)[0] > 0
            stack = np.stack(samples)                                       # (k, n, n, 3)
            bgr = np.take_along_axis(stack, best[None, :, :, None], 0)[0]
            codes = np.array([ROLE_CODE.get(r, 0) for r in roles], np.uint8)
            source = np.where(seen, codes[best], 0).astype(np.uint8)
        else:
            seen = np.zeros((n, n), bool)
            bgr = np.zeros((n, n, 3), np.uint8)
            source = np.zeros((n, n), np.uint8)
        bgr = bgr.copy()
        bgr[~seen] = _MASK_RGB['unseen'][::-1]

        f = bgr.astype(np.int16)
        luma = f.sum(axis=2) / 3.0
        chroma = f.max(axis=2) - f.min(axis=2)
        kind = np.full((n, n), OTHER, np.uint8)
        road = (luma < cls.road_max_luma) & (chroma < cls.road_max_chroma)
        kind[road] = ROAD
        kind[~road & (luma > cls.paint_min_luma)] = PAINT
        kind[~seen] = UNSEEN

        grown = self.grow(kind)
        return MaskResult(kind, grown, source, bgr, warped,
                          float(seen.mean()), int(grown.sum()))

    def grow(self, kind: np.ndarray) -> np.ndarray:
        road = (kind == ROAD).astype(np.uint8)
        count, labels = cv2.connectedComponents(road, connectivity=4)
        if count <= 1:
            return np.zeros_like(road)
        seeds = np.unique(labels[self._seed_box & (road > 0)])
        seeds = seeds[seeds > 0]
        return np.isin(labels, seeds).astype(np.uint8)


# --------------------------------------------------------------------------- debug renders
def bev_view(img: np.ndarray, scale: int = 3) -> np.ndarray:
    """Grid image -> display image: forward up, left on the left (V4 flip)."""
    out = img[::-1, ::-1]
    return cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)


def mask_colours(res: MaskResult) -> np.ndarray:
    n = res.kind.shape[0]
    out = np.empty((n, n, 3), np.uint8)
    out[:] = MASK_BGR['other']
    out[res.kind == UNSEEN] = MASK_BGR['unseen']
    out[res.kind == ROAD] = MASK_BGR['road']
    out[res.kind == PAINT] = MASK_BGR['paint']
    out[res.grown > 0] = MASK_BGR['grown']
    return out


def warped_strip(res: MaskResult, roles, scale: int = 2) -> np.ndarray:
    tiles = []
    n = res.kind.shape[0]
    for r in roles:
        t = res.warped.get(r)
        tiles.append(bev_view(t if t is not None else np.zeros((n, n, 3), np.uint8), scale))
    return np.hstack(tiles) if tiles else np.zeros((n, n, 3), np.uint8)


class OverlayLookup:
    """Image pixel (at overlay resolution) -> grid cell, for drawing the road
    mask back onto the original footage."""

    def __init__(self, grid: GridSpec, intr: Intrinsics, mount: Mount, width: int):
        height = int(round(width * intr.height / intr.width))
        self.width, self.height = width, height
        small = intr.scaled(width, height)
        u, v = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
        g, ok = pixels_to_ground(small, mount, np.column_stack([u.ravel(), v.ravel()]))
        cell = np.full(u.size, -1, np.int32)
        cell[ok] = grid.cell_of(g[ok, 0], g[ok, 1])
        self.cell = cell.reshape(height, width)

    def draw(self, img_bgr: np.ndarray, res: MaskResult, alpha: float = 0.45) -> np.ndarray:
        base = cv2.resize(img_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        colours = mask_colours(res).reshape(-1, 3)
        hit = self.cell >= 0
        state = np.zeros(self.cell.shape, np.uint8)
        flat_kind = res.kind.ravel()
        flat_grown = res.grown.ravel()
        idx = self.cell[hit]
        paint = flat_kind[idx] == PAINT
        grown = flat_grown[idx] > 0
        s = np.zeros(idx.shape, np.uint8)
        s[paint] = 1
        s[grown] = 2
        state[hit] = s
        tint = np.zeros_like(base)
        tint[hit] = colours[idx]
        m = state > 0
        out = base.copy()
        out[m] = (base[m] * (1 - alpha) + tint[m] * alpha).astype(np.uint8)
        return out
