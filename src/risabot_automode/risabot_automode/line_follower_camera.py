#!/usr/bin/env python3
"""
Line Follower Camera Node  (MDPI-enhanced scanline detection)
Detects white lane lines using CLAHE + Otsu's adaptive thresholding and
computes a steering error from multi-scanline pixel scanning.

Enhanced with techniques from:
  MDPI Applied Sciences 2018 — "A Low Cost Vision-Based Road-Following System"
  - Inverse Perspective Mapping (Bird's Eye View warp)
  - 1D Kalman Filter for predictive lane center tracking

Algorithm:
  1. Resize + crop bottom portion of camera image (road surface)
  2. IPM warp: perspective → Bird's Eye View (parallel lane lines)
  3. CLAHE histogram equalization (adaptive lighting compensation)
  4. Gaussian blur + Otsu's auto-threshold
  5. Multiple horizontal scanlines scan for left/right white lane edges
  6. Kalman filter: predict + update lane center position & velocity
  7. Publish Float32 on /lane_error (range -1.0 to +1.0)

References:
  Cytron Technologies — Differential Line Following Algorithm
  https://my.cytron.io/tutorial/differential-line-following-algorithm
  MDPI — A Low Cost Vision-Based Road-Following System for Mobile Robots
  https://www.mdpi.com/2076-3417/8/9/1635
"""

import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from .topics import CAMERA_DEBUG_LINE_TOPIC, CAMERA_IMAGE_TOPIC, LANE_ERROR_TOPIC, LANE_LOST_TOPIC


# ──────────────────────────────────────────────────────────────────────────────
# 1D Kalman Filter for lane center tracking (MDPI-inspired)
# ──────────────────────────────────────────────────────────────────────────────

class LaneKalmanFilter:
    """Simple 1D Kalman filter tracking lane center position and velocity.

    State vector:  [position, velocity]
    Measurement:   position only

    The velocity component allows the filter to predict lane motion during
    curves and hold a reasonable estimate when the lane is temporarily lost.
    """

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1):
        self.x = np.array([0.0, 0.0])   # state: [position, velocity]
        self.P = np.eye(2) * 1.0         # covariance matrix
        self.Q_base = process_noise      # process noise scalar
        self.R = measurement_noise       # measurement noise scalar
        self.H = np.array([[1.0, 0.0]])  # measurement matrix (observe position only)

    def predict(self, dt: float) -> None:
        """Predict step: advance state by dt seconds."""
        F = np.array([[1.0, dt],
                       [0.0, 1.0]])
        Q = np.array([[self.Q_base * dt**2, self.Q_base * dt],
                       [self.Q_base * dt,    self.Q_base]])
        self.x = F @ self.x
        # Clamp position error to valid range so it doesn't explode when lost
        self.x[0] = np.clip(self.x[0], -1.0, 1.0)
        self.P = F @ self.P @ F.T + Q

    def decay_velocity(self, factor: float = 0.9) -> None:
        """Decay velocity when lane is lost to gently stop predicting."""
        self.x[1] *= factor

    def update(self, measurement: float) -> None:
        """Update step: correct state with a new measurement."""
        y = measurement - float(self.H @ self.x)  # innovation
        S = float(self.H @ self.P @ self.H.T) + self.R
        K = (self.P @ self.H.T) / S                # Kalman gain
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(2) - K @ self.H) @ self.P

    @property
    def position(self) -> float:
        """Current filtered lane center position (error)."""
        return float(self.x[0])

    @property
    def velocity(self) -> float:
        """Current rate of change of lane center (useful for curve anticipation)."""
        return float(self.x[1])

    def reset(self, position: float = 0.0) -> None:
        """Reset filter state."""
        self.x = np.array([position, 0.0])
        self.P = np.eye(2) * 1.0


# ──────────────────────────────────────────────────────────────────────────────
# Main Node
# ──────────────────────────────────────────────────────────────────────────────

class LineFollowerCamera(Node):
    """Lane detection and steering error estimation from camera frames."""

    def __init__(self):
        super().__init__('line_follower_camera')
        self.lane_error = 0.0
        self.filtered_error = 0.0    # output (Kalman or EMA)
        self.last_valid_error = 0.0  # last error from a confident scan

        # ── Tunable parameters ─────────────────────────────────────────────
        # Scanline detection
        self.declare_parameter('n_scanlines', 8)
        self.declare_parameter('min_valid_scanlines', 2)
        self.declare_parameter('min_line_width_px', 5)
        self.declare_parameter('crop_ratio_base', 0.55)
        self.declare_parameter('search_radius_px', 50)   # blob-to-expected match radius
        # Thresholding
        self.declare_parameter('white_threshold', 100)   # gray threshold (inverted: pixels BELOW this = lane)
        self.declare_parameter('use_otsu', False)         # True = Otsu auto-threshold
        self.declare_parameter('invert_binary', True)     # True = detect dark lane, False = detect white borders
        # Morphological cleanup
        self.declare_parameter('morph_open_size', 3)     # erosion→dilation kernel to remove noise (0=disable)
        self.declare_parameter('morph_close_size', 5)    # dilation→erosion kernel to fill gaps (0=disable)
        # CLAHE adaptive lighting
        self.declare_parameter('clahe_enabled', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        # IPM (Bird's Eye View) — MDPI-inspired
        # DISABLED by default: requires careful calibration for the specific camera
        # mount angle. Wrong IPM distorts lane lines and hurts detection.
        self.declare_parameter('ipm_enabled', False)
        self.declare_parameter('ipm_top_width_ratio', 0.35)   # narrow top of trapezoid
        self.declare_parameter('ipm_bottom_width_ratio', 1.0)  # wide bottom
        # Kalman filter — MDPI-inspired (replaces EMA when enabled)
        self.declare_parameter('kalman_enabled', True)
        self.declare_parameter('kalman_process_noise', 0.01)
        self.declare_parameter('kalman_measurement_noise', 0.1)
        # Legacy EMA smoothing (used when kalman_enabled=false)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('dead_zone', 0.05)
        # Steering persistence on lane loss
        self.declare_parameter('hold_error_frames', 15)
        self.declare_parameter('error_decay_rate', 0.92)
        # Display / debug
        self.declare_parameter('show_debug', False)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('print_debug', False)
        self.declare_parameter('debug_print_rate', 0.5)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)
        self._last_debug_print = 0.0

        # ── Internal state ──────────────────────────────────────────────────
        self.frames_lost = 0
        self.current_hold_frames = 0
        self.last_lane_widths: Dict[int, int] = {}
        self._expected_left: Optional[int] = None
        self._expected_right: Optional[int] = None
        self._last_frame_time = time.monotonic()

        # CLAHE object (reused across frames)
        self._clahe = cv2.createCLAHE(
            clipLimit=self._param_cache['clahe_clip_limit'],
            tileGridSize=(8, 8)
        )

        # IPM warp matrix (computed lazily on first frame)
        self._ipm_matrix = None
        self._ipm_inv_matrix = None
        self._ipm_cached_size = (0, 0)

        # Kalman filter for lane center tracking
        self._kalman = LaneKalmanFilter(
            process_noise=self._param_cache['kalman_process_noise'],
            measurement_noise=self._param_cache['kalman_measurement_noise'],
        )

        # ── ROS publishers / subscribers ────────────────────────────────────
        self.error_pub = self.create_publisher(Float32, LANE_ERROR_TOPIC, 10)
        self.lane_lost_pub = self.create_publisher(Bool, LANE_LOST_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, CAMERA_DEBUG_LINE_TOPIC, 10)
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_TOPIC,
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info(
            'Line Follower Camera: Ready (MDPI-enhanced — IPM + Kalman + Cytron scanline)'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Parameter helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-frame lookups."""
        self._param_cache = {
            'n_scanlines':             int(self.get_parameter('n_scanlines').value),
            'min_valid_scanlines':     int(self.get_parameter('min_valid_scanlines').value),
            'min_line_width_px':       int(self.get_parameter('min_line_width_px').value),
            'crop_ratio_base':         float(self.get_parameter('crop_ratio_base').value),
            'search_radius_px':        int(self.get_parameter('search_radius_px').value),
            'white_threshold':         int(self.get_parameter('white_threshold').value),
            'use_otsu':                bool(self.get_parameter('use_otsu').value),
            'invert_binary':           bool(self.get_parameter('invert_binary').value),
            'morph_open_size':         int(self.get_parameter('morph_open_size').value),
            'morph_close_size':        int(self.get_parameter('morph_close_size').value),
            'clahe_enabled':           bool(self.get_parameter('clahe_enabled').value),
            'clahe_clip_limit':        float(self.get_parameter('clahe_clip_limit').value),
            'ipm_enabled':             bool(self.get_parameter('ipm_enabled').value),
            'ipm_top_width_ratio':     float(self.get_parameter('ipm_top_width_ratio').value),
            'ipm_bottom_width_ratio':  float(self.get_parameter('ipm_bottom_width_ratio').value),
            'kalman_enabled':          bool(self.get_parameter('kalman_enabled').value),
            'kalman_process_noise':    float(self.get_parameter('kalman_process_noise').value),
            'kalman_measurement_noise': float(self.get_parameter('kalman_measurement_noise').value),
            'smoothing_alpha':         float(self.get_parameter('smoothing_alpha').value),
            'dead_zone':               float(self.get_parameter('dead_zone').value),
            'hold_error_frames':       int(self.get_parameter('hold_error_frames').value),
            'error_decay_rate':        float(self.get_parameter('error_decay_rate').value),
            'show_debug':              bool(self.get_parameter('show_debug').value),
            'resize_width':            int(self.get_parameter('resize_width').value),
            'print_debug':             bool(self.get_parameter('print_debug').value),
            'debug_print_rate':        float(self.get_parameter('debug_print_rate').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or dashboard."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                if p.name == 'clahe_clip_limit':
                    self._clahe = cv2.createCLAHE(
                        clipLimit=float(p.value), tileGridSize=(8, 8)
                    )
                # Invalidate IPM matrix if IPM params changed
                if p.name.startswith('ipm_'):
                    self._ipm_matrix = None
                # Update Kalman noise params
                if p.name == 'kalman_process_noise':
                    self._kalman.Q_base = float(p.value)
                if p.name == 'kalman_measurement_noise':
                    self._kalman.R = float(p.value)
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────────
    # IPM — Inverse Perspective Mapping (Bird's Eye View)
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_ipm_matrix(self, w: int, h: int) -> None:
        """Compute the perspective warp matrix for Bird's Eye View.

        The source trapezoid represents the perspective view of the road:
          - Bottom edge: close to robot, wide field of view
          - Top edge: further away, narrower due to perspective

        Camera specs: Orbbec Astra Mini at 8.5cm height, 0° tilt (horizontal).
        """
        top_ratio = self._param_cache['ipm_top_width_ratio']
        bot_ratio = self._param_cache['ipm_bottom_width_ratio']

        # Source trapezoid (perspective view of road)
        top_margin = int(w * (1.0 - top_ratio) / 2)
        bot_margin = int(w * (1.0 - bot_ratio) / 2)

        src = np.float32([
            [top_margin,     0],      # top-left
            [w - top_margin, 0],      # top-right
            [w - bot_margin, h - 1],  # bottom-right
            [bot_margin,     h - 1],  # bottom-left
        ])

        # Destination rectangle (bird's eye view — full image)
        dst = np.float32([
            [0,     0],
            [w - 1, 0],
            [w - 1, h - 1],
            [0,     h - 1],
        ])

        self._ipm_matrix = cv2.getPerspectiveTransform(src, dst)
        self._ipm_inv_matrix = cv2.getPerspectiveTransform(dst, src)
        self._ipm_cached_size = (w, h)

    def _apply_ipm(self, img: np.ndarray) -> np.ndarray:
        """Apply Bird's Eye View warp to road crop."""
        h, w = img.shape[:2]

        # Recompute matrix if image size changed or first time
        if self._ipm_matrix is None or self._ipm_cached_size != (w, h):
            self._compute_ipm_matrix(w, h)

        return cv2.warpPerspective(img, self._ipm_matrix, (w, h),
                                   flags=cv2.INTER_LINEAR)

    # ──────────────────────────────────────────────────────────────────────────
    # Scanline detection — Cytron-style pixel scanning
    # ──────────────────────────────────────────────────────────────────────────

    def _find_all_white_regions(self, row: np.ndarray, min_w: int, max_w: int) -> List[Tuple[int, int, int]]:
        """Find all white regions within a width range.
        Returns list of (center, start, end) tuples.
        """
        regions = []
        in_white = False
        white_start = 0
        for x in range(len(row)):
            if row[x] == 255:
                if not in_white:
                    white_start = x
                    in_white = True
            else:
                if in_white:
                    width = x - white_start
                    if min_w <= width <= max_w:
                        regions.append(((white_start + x) // 2, white_start, x))
                    in_white = False
        if in_white:
            width = len(row) - white_start
            if min_w <= width <= max_w:
                regions.append(((white_start + len(row)) // 2, white_start, len(row)))
        return regions

    def _detect_scanlines(
        self, binary: np.ndarray, crop_h: int, w: int
    ) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]], List[Tuple[int, int]], List[float], int]:
        """Run robust multi-scanline blob matching."""
        n_scanlines = self._param_cache['n_scanlines']
        min_width = self._param_cache['min_line_width_px']
        invert = self._param_cache.get('invert_binary', False)
        # In invert mode the lane is wider than border lines
        max_width = w - 10 if invert else w // 3
        search_radius = self._param_cache['search_radius_px']

        left_points = []
        right_points = []
        center_points = []
        scanline_weights = []  # weight per valid scanline (bottom = higher)
        valid_count = 0

        # Start from last known good position, or center if completely lost
        if self._expected_left is None or self._expected_right is None:
            expected_left = w // 4
            expected_right = 3 * w // 4
        else:
            expected_left = self._expected_left
            expected_right = self._expected_right

        for i in range(n_scanlines):
            y_frac = (i + 0.5) / n_scanlines
            y_in_crop = int(crop_h * (1.0 - y_frac))
            y_in_crop = max(0, min(crop_h - 1, y_in_crop))

            row = binary[y_in_crop, :]
            raw_regions = self._find_all_white_regions(row, min_width, max_width)

            left_x = None
            right_x = None

            if invert and len(raw_regions) > 0:
                # INVERT MODE: the white region IS the lane.
                # Pick the region whose center is closest to expected lane center.
                expected_center = (expected_left + expected_right) // 2
                best = min(raw_regions, key=lambda r: abs(r[0] - expected_center))
                # Only accept if within search radius of expected center
                if abs(best[0] - expected_center) < search_radius:
                    left_x = best[1]   # left edge of lane
                    right_x = best[2]  # right edge of lane

            elif len(raw_regions) > 0:
                # BORDER MODE (original): find left/right white border lines
                regions = [r[0] for r in raw_regions]  # extract centers only
                if i == 0 and (self._expected_left is None or self._expected_right is None):
                    if len(regions) >= 2:
                        target_w = self.last_lane_widths.get(0, w // 2)
                        best_pair = None
                        best_err = 9999
                        for a in range(len(regions)):
                            for b in range(a + 1, len(regions)):
                                err = abs((regions[b] - regions[a]) - target_w)
                                if err < best_err:
                                    best_err = err
                                    best_pair = (regions[a], regions[b])
                        if best_pair:
                            left_x, right_x = best_pair
                    elif len(regions) == 1:
                        if regions[0] < w // 2: left_x = regions[0]
                        else: right_x = regions[0]
                else:
                    best_left = min(regions, key=lambda x: abs(x - expected_left))
                    if abs(best_left - expected_left) < search_radius:
                        left_x = best_left
                    best_right = min(regions, key=lambda x: abs(x - expected_right))
                    if abs(best_right - expected_right) < search_radius:
                        right_x = best_right
                    if left_x == right_x and left_x is not None:
                        if abs(left_x - expected_left) < abs(right_x - expected_right):
                            right_x = None
                        else:
                            left_x = None

            # Determine lane center
            if left_x is not None and right_x is not None:
                valid_count += 1
                self.last_lane_widths[i] = right_x - left_x
                center_x = (left_x + right_x) // 2
                expected_left = left_x
                expected_right = right_x

            elif left_x is not None:
                valid_count += 1
                width = self.last_lane_widths.get(i, w // 2)
                right_x = left_x + width
                center_x = (left_x + right_x) // 2
                expected_left = left_x
                expected_right = right_x

            elif right_x is not None:
                valid_count += 1
                width = self.last_lane_widths.get(i, w // 2)
                left_x = right_x - width
                center_x = (left_x + right_x) // 2
                expected_left = left_x
                expected_right = right_x

            else:
                continue

            # Save the bottom-most valid row as the expectation for the NEXT frame
            # Use aggressive EMA smoothing to prevent frame-to-frame jumps
            if valid_count == 1:
                smooth = 0.15  # low = very stable (slower to react but no flickering)
                if self._expected_left is not None:
                    # Clamp jump: don't allow expected to shift more than 15px/frame
                    max_shift = 15
                    new_left = int(smooth * expected_left + (1 - smooth) * self._expected_left)
                    new_right = int(smooth * expected_right + (1 - smooth) * self._expected_right)
                    new_left = max(self._expected_left - max_shift, min(self._expected_left + max_shift, new_left))
                    new_right = max(self._expected_right - max_shift, min(self._expected_right + max_shift, new_right))
                    self._expected_left = new_left
                    self._expected_right = new_right
                else:
                    self._expected_left = expected_left
                    self._expected_right = expected_right

            left_points.append((int(left_x), y_in_crop))
            right_points.append((int(right_x), y_in_crop))
            center_points.append((int(center_x), y_in_crop))
            # Weight: bottom scanlines (close to robot) are more reliable
            # y_frac goes from 0.5/n (bottom) to ~1.0 (top), invert for weight
            scanline_weights.append(1.0 - y_frac + 0.5)

        # If completely lost, clear expectations so it resets next frame
        if valid_count == 0:
            self._expected_left = None
            self._expected_right = None

        return left_points, right_points, center_points, scanline_weights, valid_count

    # ──────────────────────────────────────────────────────────────────────────
    # Main camera callback
    # ──────────────────────────────────────────────────────────────────────────

    def color_callback(self, msg: Image) -> None:
        """Process a camera frame and publish lane error."""
        try:
            now = time.monotonic()
            dt = now - self._last_frame_time
            self._last_frame_time = now
            if dt <= 0.0 or dt > 0.5:
                dt = 0.033  # assume ~30 fps

            # ── 1. Resize — ALWAYS force to exactly 320x240 ──────────────
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            bgr = cv2.resize(bgr, (320, 240))
            h, w = 240, 320

            image_center = w / 2.0

            # ── 2. Crop bottom portion (road surface) ───────────────────────
            crop_ratio = self._param_cache['crop_ratio_base']
            crop_h = int(h * crop_ratio)
            road = bgr[h - crop_h:, :]

            # ── 3. IPM warp: perspective → Bird's Eye View ──────────────────
            if self._param_cache['ipm_enabled']:
                road = self._apply_ipm(road)

            # ── 4. CLAHE + threshold (fixed or Otsu) + morphology ───────────
            gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)

            if self._param_cache['clahe_enabled']:
                gray = self._clahe.apply(gray)

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            if self._param_cache['use_otsu']:
                _, binary = cv2.threshold(
                    blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
                )
            else:
                thresh_val = self._param_cache['white_threshold']
                if self._param_cache.get('invert_binary', False):
                    # INVERT: pixels BELOW threshold (dark lane) → white
                    _, binary = cv2.threshold(
                        blurred, thresh_val, 255, cv2.THRESH_BINARY_INV
                    )
                else:
                    # NORMAL: pixels ABOVE threshold (white borders) → white
                    _, binary = cv2.threshold(
                        blurred, thresh_val, 255, cv2.THRESH_BINARY
                    )

            # Morphological cleanup: remove noise then fill small gaps
            open_sz = self._param_cache['morph_open_size']
            close_sz = self._param_cache['morph_close_size']
            if open_sz > 0:
                kernel_open = cv2.getStructuringElement(
                    cv2.MORPH_RECT, (open_sz, open_sz)
                )
                binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
            if close_sz > 0:
                kernel_close = cv2.getStructuringElement(
                    cv2.MORPH_RECT, (close_sz, close_sz)
                )
                binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)

            # ── 5. Multi-scanline detection ─────────────────────────────────
            left_pts, right_pts, center_pts, scan_weights, valid_count = \
                self._detect_scanlines(binary, crop_h, w)

            # ── 6. Compute raw error ────────────────────────────────────────
            conf_min = self._param_cache['min_valid_scanlines']
            measurement_available = False

            if valid_count >= conf_min and len(center_pts) > 0:
                # Weighted average: bottom scanlines (closer to robot) count more
                total_weight = sum(scan_weights)
                if total_weight > 0:
                    avg_center_x = sum(
                        pt[0] * wt for pt, wt in zip(center_pts, scan_weights)
                    ) / total_weight
                else:
                    avg_center_x = sum(pt[0] for pt in center_pts) / len(center_pts)
                raw_error = float(
                    np.clip((avg_center_x - image_center) / image_center, -1.0, 1.0)
                )
                measurement_available = True
                self.frames_lost = 0
                self.current_hold_frames = self._param_cache['hold_error_frames']
                self.lane_lost_pub.publish(Bool(data=False))
            else:
                self.frames_lost += 1
                if self.frames_lost >= self._param_cache['hold_error_frames']:
                    self.lane_lost_pub.publish(Bool(data=True))
                    self._expected_left = None
                    self._expected_right = None
                raw_error = 0.0

            # ── 7. Filtering: Kalman or EMA ─────────────────────────────────
            if self._param_cache['kalman_enabled']:
                # Kalman predict step (always runs)
                self._kalman.predict(dt)

                if measurement_available:
                    # Deadband: only update if error exceeds threshold.
                    # When within dead zone, SKIP the update entirely so the
                    # Kalman filter coasts on its prediction. Feeding 0.0 is
                    # a false measurement that biases the filter toward center.
                    if abs(raw_error) >= self._param_cache['dead_zone']:
                        self._kalman.update(raw_error)
                    # else: let predict() carry the state forward (no update)
                # When lane is lost, Kalman continues predicting using velocity
                # This is much better than the old hold+decay approach

                self.filtered_error = self._kalman.position
                self.lane_error = self.filtered_error

                if measurement_available:
                    self.last_valid_error = self.lane_error

            else:
                # Legacy EMA path (backward compatible)
                if measurement_available:
                    if abs(raw_error) < self._param_cache['dead_zone']:
                        raw_error = 0.0
                    alpha = self._param_cache['smoothing_alpha']
                    self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
                    self.last_valid_error = self.filtered_error

                elif self.current_hold_frames > 0:
                    self.last_valid_error *= self._param_cache['error_decay_rate']
                    self.filtered_error = self.last_valid_error
                    self.current_hold_frames -= 1

                else:
                    self.filtered_error *= 0.95  # gentle fade to zero

                self.lane_error = self.filtered_error

            # ── 8. Publish ──────────────────────────────────────────────────
            self.error_pub.publish(Float32(data=self.lane_error))

            # ── 9. Debug visualisation ──────────────────────────────────────
            if self._param_cache['show_debug']:
                # Show the warped (IPM) view if enabled, otherwise raw
                debug = road.copy()
                crop_top = 0  # debug view is already cropped

                # Draw scanline detection points
                for lp, rp, cp in zip(left_pts, right_pts, center_pts):
                    ly = lp[1]
                    ry = rp[1]
                    cy = cp[1]

                    # Left line point (blue)
                    cv2.circle(debug, (lp[0], ly), 4, (255, 130, 130), -1)
                    # Right line point (pink)
                    cv2.circle(debug, (rp[0], ry), 4, (130, 130, 255), -1)
                    # Center point (green)
                    cv2.circle(debug, (cp[0], cy), 5, (0, 255, 0), -1)
                    # Scanline visualization
                    cv2.line(debug, (lp[0], ly), (rp[0], ry), (50, 50, 50), 1)

                # Draw center reference line
                cv2.line(debug, (w // 2, 0), (w // 2, crop_h), (0, 0, 255), 1)

                # Status text
                def put_text(img, text, pos, scale, color, thick=2):
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2)
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

                if abs(self.lane_error) < 0.05:
                    direction, t_color = 'CENTERED', (0, 255, 0)
                elif self.lane_error > 0:
                    # Positive error -> positive angular_z -> robot physically steers RIGHT
                    direction, t_color = 'STEER RIGHT', (0, 165, 255)
                else:
                    # Negative error -> negative angular_z -> robot physically steers LEFT
                    direction, t_color = 'STEER LEFT', (0, 165, 255)

                steer_deg = abs(self.lane_error) * 50.0
                put_text(debug, f'{direction} ({steer_deg:.0f} deg)', (10, 20), 0.55, t_color)

                status_str = (
                    f'LOCK({valid_count}/{self._param_cache["n_scanlines"]})'
                    if valid_count >= conf_min
                    else (f'HOLD({self.current_hold_frames})'
                          if self.current_hold_frames > 0
                          else f'LOST({self.frames_lost}f)')
                )
                ipm_str = 'IPM' if self._param_cache['ipm_enabled'] else 'RAW'
                kf_str = 'KF' if self._param_cache['kalman_enabled'] else 'EMA'
                put_text(debug, f'{status_str} [{ipm_str}|{kf_str}]', (10, 42), 0.45, (0, 255, 255))

                if len(self.last_lane_widths) > 0:
                    avg_w = sum(self.last_lane_widths.values()) / len(self.last_lane_widths)
                    lane_w_cm = avg_w * 40.0 / (w * 0.4)
                    put_text(debug, f'W={lane_w_cm:.0f}cm', (10, 60), 0.45, (0, 255, 255))

                # Kalman velocity indicator
                if self._param_cache['kalman_enabled']:
                    vel = self._kalman.velocity
                    put_text(debug, f'v={vel:.3f}', (10, 78), 0.4, (200, 200, 0))

                # Compose into full-frame debug image for dashboard
                debug_full = bgr.copy()
                debug_full[h - crop_h:, :] = debug
                # Draw crop boundary (fixed position)
                cv2.line(debug_full, (0, h - crop_h), (w, h - crop_h), (255, 0, 255), 1)

                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_full, encoding='bgr8'))

            if self._param_cache['print_debug']:
                now_mono = time.monotonic()
                if now_mono - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    status = ('CENTER' if abs(self.lane_error) < 0.05
                              else ('TURN RIGHT' if self.lane_error < 0 else 'TURN LEFT'))
                    kf_str = f'kv={self._kalman.velocity:.3f}' if self._param_cache['kalman_enabled'] else ''
                    print(
                        f'\r[LF] Err:{self.lane_error:.2f} | {status} | '
                        f'valid={valid_count}/{self._param_cache["n_scanlines"]} | {kf_str}',
                        end='', flush=True
                    )
                    self._last_debug_print = now_mono

        except Exception as e:
            self.get_logger().error(f'Line follower error: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowerCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
