"""road_perception output for wizard step 9 (venue thresholds).

Subscribes ONLY while asked (want()):
* the road grid (LocalGrid, 8 Hz) while step 9's page is open -> live coverage
  / false paint of the sample boxes and the page's mini map;
* the stitched debug image (JPEG, <= debug.max_rate_hz) only while step 9 is
  sampling: road_perception encodes it only while someone subscribes.

pair() returns the newest stitched image matched to the grid with the SAME
header stamp (road_perception stamps both with the grid stamp), decoded once:
(seq, grid dict, (rows, cols, 3) BGR per cell).
"""
import time
from collections import OrderedDict
from typing import Dict, Optional, Tuple

import numpy as np
from carbot_interfaces.msg import LocalGrid
from sensor_msgs.msg import CompressedImage

from .step_venue_thresholds import stitched_to_cells


def _stamp(h) -> Tuple[int, int]:
    return int(h.stamp.sec), int(h.stamp.nanosec)


class RoadTap:

    def __init__(self, node, grid_topic: str, stitched_topic: str, keep: int = 16):
        self.node, self.grid_topic, self.stitched_topic, self.keep = node, grid_topic, stitched_topic, keep
        self.sub_grid = self.sub_img = None
        self.grids: 'OrderedDict[Tuple[int, int], Dict]' = OrderedDict()
        self.newest: Optional[Dict] = None
        self.img = None                      # (seq, msg)
        self.img_seq = 0
        self.decoded: Optional[Tuple[int, Dict, np.ndarray]] = None

    def want(self, grid: bool, stitched: bool) -> None:
        grid = grid or stitched
        if grid and self.sub_grid is None:
            self.sub_grid = self.node.create_subscription(LocalGrid, self.grid_topic, self._on_grid, 5)
        elif not grid and self.sub_grid is not None:
            self.node.destroy_subscription(self.sub_grid)
            self.sub_grid, self.newest = None, None
            self.grids.clear()
        if stitched and self.sub_img is None:
            self.sub_img = self.node.create_subscription(CompressedImage, self.stitched_topic, self._on_img, 1)
            self.node.get_logger().info(f'road tap: sampling {self.stitched_topic}')
        elif not stitched and self.sub_img is not None:
            self.node.destroy_subscription(self.sub_img)
            self.sub_img, self.img, self.decoded = None, None, None

    def _on_grid(self, m) -> None:
        g = {'rows': int(m.rows), 'cols': int(m.cols), 'res': float(m.resolution_m), 'x0': float(m.origin_x_m),
             'y0': float(m.origin_y_m), 't': time.monotonic(),
             'kind': np.frombuffer(bytes(m.kind), np.uint8).reshape(int(m.rows), int(m.cols))}
        self.newest = g
        self.grids[_stamp(m.header)] = g
        while len(self.grids) > self.keep:
            self.grids.popitem(last=False)

    def _on_img(self, m) -> None:
        self.img_seq += 1
        self.img = (self.img_seq, m)

    def grid(self) -> Optional[Dict]:
        g = self.newest
        if g is None:
            return None
        return dict(g, age_s=round(time.monotonic() - g['t'], 2))

    def pair(self) -> Optional[Tuple[int, Dict, np.ndarray]]:
        if self.img is None:
            return None
        seq, msg = self.img
        if self.decoded is not None and self.decoded[0] == seq:
            return self.decoded
        g = self.grids.get(_stamp(msg.header))
        if g is None:
            return None                      # its grid has not arrived (yet): try on the next tick
        import cv2
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        self.decoded = (seq, g, stitched_to_cells(img, g['rows'], g['cols']))
        return self.decoded
