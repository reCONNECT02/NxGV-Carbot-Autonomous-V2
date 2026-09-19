#!/usr/bin/env python3
"""
Traffic Light Detector Node — Two-Stage CV Detection (Pure OpenCV)

Stage 1: HSV colour thresholding + circularity gating on an upper ROI crop.
Stage 2: Confidence counter — N consecutive matching frames before publishing.

This eliminates false positives from single-frame noise (camera shake,
passing shadows, brief reflections) without requiring any AI model.

Publishes String on /traffic_light_state: 'red' | 'yellow' | 'green' | 'unknown'
"""

import time
from typing import Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .topics import CAMERA_DEBUG_TL_TOPIC, CAMERA_IMAGE_TOPIC, TRAFFIC_LIGHT_TOPIC


class TrafficLightDetector(Node):
    """Two-stage CV-only traffic light detector with confidence gating."""

    def __init__(self):
        super().__init__('traffic_light_detector')

        # ── Tunable parameters ─────────────────────────────────────────────
        self.declare_parameter('red_h_low1',  0)
        self.declare_parameter('red_h_high1', 10)
        self.declare_parameter('red_h_low2',  160)
        self.declare_parameter('red_h_high2', 180)
        self.declare_parameter('yellow_h_low',  20)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('green_h_low',   40)
        self.declare_parameter('green_h_high',  85)
        self.declare_parameter('sat_min', 80)
        self.declare_parameter('val_min', 80)
        self.declare_parameter('min_circle_radius', 8)
        self.declare_parameter('max_circle_radius', 80)
        self.declare_parameter('min_pixel_count',   50)
        self.declare_parameter('required_confidence', 3)
        self.declare_parameter('show_debug',       False)
        self.declare_parameter('resize_width',     320)
        self.declare_parameter('print_debug',      False)
        self.declare_parameter('debug_print_rate',  0.5)
        self.declare_parameter('heartbeat_sec',     0.5)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)
        self._last_debug_print = 0.0

        # ── ROS publishers / subscribers ────────────────────────────────────
        self.state_pub = self.create_publisher(String, TRAFFIC_LIGHT_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, CAMERA_DEBUG_TL_TOPIC, 10)
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_TOPIC,
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # ── Detection state ─────────────────────────────────────────────────
        self.cv_state         = 'unknown'
        self.confidence_count = 0
        self.current_state    = 'unknown'

        # Heartbeat timer — re-publish last state so downstream nodes stay fresh
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )
        self.get_logger().info('Traffic Light Detector started (Pure CV, 2-stage confidence)')

    # ──────────────────────────────────────────────────────────────────────────
    # Parameter helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'red_h_low1':          int(self.get_parameter('red_h_low1').value),
            'red_h_high1':         int(self.get_parameter('red_h_high1').value),
            'red_h_low2':          int(self.get_parameter('red_h_low2').value),
            'red_h_high2':         int(self.get_parameter('red_h_high2').value),
            'yellow_h_low':        int(self.get_parameter('yellow_h_low').value),
            'yellow_h_high':       int(self.get_parameter('yellow_h_high').value),
            'green_h_low':         int(self.get_parameter('green_h_low').value),
            'green_h_high':        int(self.get_parameter('green_h_high').value),
            'sat_min':             int(self.get_parameter('sat_min').value),
            'val_min':             int(self.get_parameter('val_min').value),
            'min_circle_radius':   int(self.get_parameter('min_circle_radius').value),
            'max_circle_radius':   int(self.get_parameter('max_circle_radius').value),
            'min_pixel_count':     int(self.get_parameter('min_pixel_count').value),
            'required_confidence': int(self.get_parameter('required_confidence').value),
            'show_debug':          bool(self.get_parameter('show_debug').value),
            'resize_width':        int(self.get_parameter('resize_width').value),
            'print_debug':         bool(self.get_parameter('print_debug').value),
            'debug_print_rate':    float(self.get_parameter('debug_print_rate').value),
            'heartbeat_sec':       float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _get_hsv_ranges(self) -> Dict[str, list]:
        s = self._param_cache['sat_min']
        v = self._param_cache['val_min']
        return {
            'red': [
                (np.array([self._param_cache['red_h_low1'],  s, v]),
                 np.array([self._param_cache['red_h_high1'], 255, 255])),
                (np.array([self._param_cache['red_h_low2'],  s, v]),
                 np.array([self._param_cache['red_h_high2'], 255, 255])),
            ],
            'yellow': [
                (np.array([self._param_cache['yellow_h_low'],  s, v]),
                 np.array([self._param_cache['yellow_h_high'], 255, 255])),
            ],
            'green': [
                (np.array([self._param_cache['green_h_low'],  s, v]),
                 np.array([self._param_cache['green_h_high'], 255, 255])),
            ],
        }

    def _heartbeat_publish(self) -> None:
        """Re-publish current state on a timer to keep downstream nodes fresh."""
        self.state_pub.publish(String(data=self.current_state))

    # ──────────────────────────────────────────────────────────────────────────
    # Main camera callback
    # ──────────────────────────────────────────────────────────────────────────

    def color_callback(self, msg: Image) -> None:
        """Detect traffic light colour using HSV masks + circularity gating."""
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]

            resize_w = self._param_cache['resize_width']
            if resize_w > 0 and w > resize_w:
                scale = resize_w / float(w)
                bgr   = cv2.resize(bgr, (resize_w, int(h * scale)))
                h, w  = bgr.shape[:2]

            # Focus on upper-centre region where a mounted traffic light appears
            roi = bgr[0:int(h * 0.7), int(w * 0.3):]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            ranges     = self._get_hsv_ranges()
            min_pixels = self._param_cache['min_pixel_count']
            kernel     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

            best_color = 'unknown'
            best_score = 0

            for color_name, hsv_ranges in ranges.items():
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                for (lower, upper) in hsv_ranges:
                    mask |= cv2.inRange(hsv, lower, upper)

                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < min_pixels:
                        continue
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                    circularity = 4.0 * np.pi * area / (perimeter * perimeter)
                    # Only accept circular blobs (traffic light bulbs are round)
                    if circularity > 0.6 and area > best_score:
                        best_score = area
                        best_color = color_name

            # ── Confidence gating ────────────────────────────────────────────
            # Require N consecutive frames of the same colour before publishing.
            # This filters out single-frame false positives from shadows/reflections.
            if best_color == self.cv_state:
                self.confidence_count += 1
            else:
                self.confidence_count = 1
                self.cv_state = best_color

            required = self._param_cache['required_confidence']
            if self.confidence_count >= required:
                self.current_state = self.cv_state
                self.state_pub.publish(String(data=self.current_state))

            # ── Debug visualisation ──────────────────────────────────────────
            if self._param_cache['show_debug']:
                self._draw_debug(roi, hsv, best_color, best_score, ranges, kernel, required)

            if self._param_cache['print_debug'] and self.current_state != 'unknown':
                now = time.monotonic()
                if now - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    print(
                        f'\r[TLD] CV:{best_color.upper()}'
                        f'({self.confidence_count}/{required})'
                        f' | OUT:{self.current_state.upper()}',
                        end='', flush=True
                    )
                    self._last_debug_print = now

        except Exception as e:
            self.get_logger().error(f'Traffic light detection error: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Debug visualisation helper
    # ──────────────────────────────────────────────────────────────────────────

    def _draw_debug(self, roi, hsv, best_color, best_score, ranges, kernel, required) -> None:
        if not self._param_cache['show_debug']:
            return

        debug = roi.copy()
        COLOR_MAP = {
            'red':    (0,   0,   255),
            'yellow': (0,   255, 255),
            'green':  (0,   255, 0),
        }

        if best_color in COLOR_MAP:
            color_rgb = COLOR_MAP[best_color]
            b_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in ranges[best_color]:
                b_mask |= cv2.inRange(hsv, lower, upper)
            b_mask = cv2.morphologyEx(b_mask, cv2.MORPH_OPEN,  kernel)
            b_mask = cv2.morphologyEx(b_mask, cv2.MORPH_CLOSE, kernel)
            b_cnts, _ = cv2.findContours(
                b_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if b_cnts:
                cnt = max(b_cnts, key=cv2.contourArea)
                ((cx, cy), radius) = cv2.minEnclosingCircle(cnt)
                cv2.circle(debug, (int(cx), int(cy)), int(radius), color_rgb, 2)
                cv2.putText(
                    debug,
                    f'{best_color.upper()} ({self.confidence_count}/{required})',
                    (int(cx) - 20, int(cy) - int(radius) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_rgb, 2
                )

        cv2.putText(
            debug,
            f'CV:{best_color.upper()} | OUT:{self.current_state.upper()}',
            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
        )
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
