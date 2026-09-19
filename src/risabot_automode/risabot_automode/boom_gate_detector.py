#!/usr/bin/env python3
"""
Boom Gate Detector Node
Detects whether a boom gate barrier is blocking the lane ahead using LiDAR.
The gate appears as a narrow horizontal line of closely-spaced points in the
forward arc at a specific distance.

Publishes Bool on /boom_gate_open (True = open/clear, False = blocked).
"""

import math
from typing import Dict

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .topics import BOOM_GATE_TOPIC


class BoomGateDetector(Node):
    """Detects boom gate presence from LiDAR scans."""

    def __init__(self):
        super().__init__('boom_gate_detector')

        # --- Parameters ---
        # Detection zone: only check for gate within this distance range
        self.declare_parameter('min_detect_dist', 0.15)   # meters
        self.declare_parameter('max_detect_dist', 0.80)   # meters
        # Angular window: check ±angle_window radians from front (0°)
        self.declare_parameter('angle_window', 0.35)       # ~20° each side
        # Gate detection: if a dense cluster of points spans this angular width
        # at roughly the same distance, it's a gate
        self.declare_parameter('min_gate_points', 5)       # min points forming gate
        self.declare_parameter('distance_variance_max', 0.05)  # points must be at similar dist
        # Lidar mount offset (same 90° correction as obstacle_avoidance)
        self.declare_parameter('lidar_angle_offset', 1.5708)  # pi/2
        self.declare_parameter('hysteresis', 3)
        self.declare_parameter('heartbeat_sec', 0.5)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publisher
        self.gate_pub = self.create_publisher(Bool, BOOM_GATE_TOPIC, 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.gate_blocked = False
        self.blocked_count = 0
        self.clear_count = 0
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )

        self.get_logger().info('Boom Gate Detector started')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-scan lookups."""
        self._param_cache = {
            'min_detect_dist': float(self.get_parameter('min_detect_dist').value),
            'max_detect_dist': float(self.get_parameter('max_detect_dist').value),
            'angle_window': float(self.get_parameter('angle_window').value),
            'min_gate_points': int(self.get_parameter('min_gate_points').value),
            'distance_variance_max': float(self.get_parameter('distance_variance_max').value),
            'lidar_angle_offset': float(self.get_parameter('lidar_angle_offset').value),
            'hysteresis': int(self.get_parameter('hysteresis').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _heartbeat_publish(self) -> None:
        """Publish last gate state on a fixed heartbeat."""
        gate_msg = Bool()
        gate_msg.data = not self.gate_blocked
        self.gate_pub.publish(gate_msg)

    def scan_callback(self, msg: LaserScan) -> None:
        min_dist = self._param_cache['min_detect_dist']
        max_dist = self._param_cache['max_detect_dist']
        angle_win = self._param_cache['angle_window']
        min_pts = self._param_cache['min_gate_points']
        dist_var_max = self._param_cache['distance_variance_max']
        angle_offset = self._param_cache['lidar_angle_offset']

        # Collect points in the forward detection zone
        forward_distances = []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # Apply mount correction
            angle += angle_offset
            # Normalize to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Only look at the front arc
            if -angle_win <= angle <= angle_win:
                if min_dist <= r <= max_dist:
                    forward_distances.append(r)

        # Gate detection logic:
        # A boom gate creates a dense cluster of points at roughly the same distance
        is_blocked = False
        if len(forward_distances) >= min_pts:
            distances = np.array(forward_distances)
            # Check if points cluster tightly (low variance = solid barrier)
            dist_std = np.std(distances)
            dist_mean = np.mean(distances)
            # Require both: low spread AND reasonable distance (not noise at range_max)
            if dist_std < dist_var_max and dist_mean < max_dist * 0.9:
                is_blocked = True

        # Hysteresis to avoid flicker
        if is_blocked:
            self.blocked_count += 1
            self.clear_count = 0
        else:
            self.clear_count += 1
            self.blocked_count = 0

        hysteresis = self._param_cache['hysteresis']
        if self.blocked_count >= hysteresis and not self.gate_blocked:
            self.gate_blocked = True
            self.get_logger().warn('Boom gate CLOSED - barrier detected')
        elif self.clear_count >= hysteresis and self.gate_blocked:
            self.gate_blocked = False
            self.get_logger().info('Boom gate OPEN - path clear')

        # Publish
        gate_msg = Bool()
        gate_msg.data = not self.gate_blocked  # True = open
        self.gate_pub.publish(gate_msg)

        # Debug
        status = "CLOSED" if self.gate_blocked else "OPEN"
        pts = len(forward_distances)
        self.get_logger().debug(f"{status} | fwd points: {pts} | blocked_cnt: {self.blocked_count} | clear_cnt: {self.clear_count}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoomGateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
