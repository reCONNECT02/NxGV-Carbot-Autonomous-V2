#!/usr/bin/env python3
"""
Camera Obstacle Avoidance Node
==============================
Uses the Astra camera feed to detect obstacles by measuring edge density
in the center ROI. Objects (boxes, walls, bins) create significantly more
edges than a flat track surface (which only has smooth ground and lane lines).

Uses Canny edge detection + edge pixel ratio with hysteresis filtering.
Publishes a boolean flag to `/obstacle_detected_camera`.
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
from std_msgs.msg import Bool


class ObstacleAvoidanceCamera(Node):
    """
    Analyzes the center ROI of the camera feed for high edge density.
    Objects have sharp edges; a flat track does not.
    Used as an auxiliary detection method alongside LiDAR.
    """
    def __init__(self):
        super().__init__('obstacle_avoidance_camera')

        # Parameters (tunable at runtime)
        self.declare_parameter('edge_threshold', 0.02)   # edge pixel ratio to trigger (0.0–1.0)
        self.declare_parameter('canny_low', 50)          # Canny lower threshold
        self.declare_parameter('canny_high', 150)        # Canny upper threshold
        self.declare_parameter('blur_kernel', 5)         # GaussianBlur kernel size (odd number)
        self.declare_parameter('hysteresis_on', 3)       # consecutive frames to trigger
        self.declare_parameter('hysteresis_off', 5)      # consecutive frames to clear
        self.declare_parameter('show_debug', False)      # publish annotated debug frame
        self.declare_parameter('resize_width', 320)      # downscale width for faster processing
        self.declare_parameter('print_debug', False)     # print debug line to stdout
        self.declare_parameter('debug_print_rate', 0.5)  # seconds between prints
        self.declare_parameter('heartbeat_sec', 0.5)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)
        self._last_debug_print = 0.0

        # Publishers
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected_camera',
            10
        )
        self.debug_pub = self.create_publisher(Image, '/camera/debug/obstacle', 10)

        # Subscribers
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State — hysteresis filtering
        self.obstacle_active = False
        self.detect_count = 0
        self.clear_count = 0
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )

        self.get_logger().info('Obstacle Avoidance Camera Node Started (Edge Density Detection)')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-frame lookups."""
        self._param_cache = {
            'edge_threshold': float(self.get_parameter('edge_threshold').value),
            'canny_low': int(self.get_parameter('canny_low').value),
            'canny_high': int(self.get_parameter('canny_high').value),
            'blur_kernel': int(self.get_parameter('blur_kernel').value),
            'hysteresis_on': int(self.get_parameter('hysteresis_on').value),
            'hysteresis_off': int(self.get_parameter('hysteresis_off').value),
            'show_debug': bool(self.get_parameter('show_debug').value),
            'resize_width': int(self.get_parameter('resize_width').value),
            'print_debug': bool(self.get_parameter('print_debug').value),
            'debug_print_rate': float(self.get_parameter('debug_print_rate').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _heartbeat_publish(self) -> None:
        """Publish last obstacle state on a fixed heartbeat."""
        self.obstacle_pub.publish(Bool(data=self.obstacle_active))

    def color_callback(self, msg: Image) -> None:
        try:
            # Read parameters once per frame
            edge_threshold = self._param_cache['edge_threshold']
            canny_low = self._param_cache['canny_low']
            canny_high = self._param_cache['canny_high']
            blur_k = self._param_cache['blur_kernel']
            hysteresis_on = self._param_cache['hysteresis_on']
            hysteresis_off = self._param_cache['hysteresis_off']

            # Convert ROS Image to OpenCV
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = color_image.shape[:2]
            resize_w = self._param_cache['resize_width']
            if resize_w > 0 and w > resize_w:
                scale = resize_w / float(w)
                color_image = cv2.resize(color_image, (resize_w, int(h * scale)))

            # Get center region (thin horizontal band just above center to avoid track lines)
            h, w = color_image.shape[:2]
            cx, cy = w // 2, h // 2
            roi_width = w // 2
            roi_height = h // 8  # Thin strip
            
            # Shift the strip slightly up from true vertical center to look at the horizon
            roi_y_center = cy - (h // 16)
            
            roi = color_image[
                roi_y_center - roi_height: roi_y_center + roi_height,
                cx - roi_width: cx + roi_width
            ]

            # Convert to grayscale and blur to reduce noise
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)

            # Canny edge detection
            edges = cv2.Canny(blurred, canny_low, canny_high)

            # Calculate edge density (ratio of edge pixels to total pixels)
            total_pixels = edges.shape[0] * edges.shape[1]
            edge_pixels = np.count_nonzero(edges)
            edge_density = edge_pixels / total_pixels if total_pixels > 0 else 0.0

            is_obstacle = edge_density > edge_threshold

            if is_obstacle:
                self.detect_count += 1
                self.clear_count = 0
            else:
                self.clear_count += 1
                self.detect_count = 0

            if self.detect_count >= hysteresis_on and not self.obstacle_active:
                self.get_logger().warn(f'Obstacle detected. Edge density: {edge_density:.3f}')
                self.obstacle_active = True
            elif self.clear_count >= hysteresis_off and self.obstacle_active:
                self.get_logger().info(f'Path clear. Edge density: {edge_density:.3f}')
                self.obstacle_active = False

            # Publish obstacle status
            obstacle_msg = Bool()
            obstacle_msg.data = self.obstacle_active
            self.obstacle_pub.publish(obstacle_msg)

            # Debug Visualization
            if self._param_cache['show_debug']:
                debug = color_image.copy()
                color = (0, 0, 255) if self.obstacle_active else (0, 255, 0)

                # Draw ROI box
                cv2.rectangle(debug,
                              (cx - roi_width, roi_y_center - roi_height),
                              (cx + roi_width, roi_y_center + roi_height),
                              color, 2)

                # Overlay the edge map inside the ROI for visibility
                edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                # Tint edges with the status color
                edges_tinted = np.zeros_like(edges_bgr)
                edges_tinted[:, :] = color
                edge_mask = edges > 0
                edges_bgr[edge_mask] = edges_tinted[edge_mask]
                
                # Blend edges into the ROI area
                roi_slice = debug[roi_y_center - roi_height: roi_y_center + roi_height,
                                  cx - roi_width: cx + roi_width]
                blended = cv2.addWeighted(roi_slice, 0.6, edges_bgr, 0.4, 0)
                debug[roi_y_center - roi_height: roi_y_center + roi_height,
                      cx - roi_width: cx + roi_width] = blended

                # Add status text
                status_text = "STOP" if self.obstacle_active else "CLEAR"
                pct = edge_density * 100
                cv2.putText(debug, f"{status_text} | Edge: {pct:.1f}%",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                debug_msg_ros = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                self.debug_pub.publish(debug_msg_ros)

            if self._param_cache['print_debug']:
                now = time.monotonic()
                if now - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    status = 'STOP' if self.obstacle_active else 'CLEAR'
                    print(f"\r[OAC] Edge: {edge_density:.3f} | {status}", end='', flush=True)
                    self._last_debug_print = now

        except Exception as e:
            self.get_logger().error(f'Color processing error: {e}')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
