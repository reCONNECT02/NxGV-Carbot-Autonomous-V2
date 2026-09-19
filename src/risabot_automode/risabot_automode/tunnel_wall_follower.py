#!/usr/bin/env python3
"""
Tunnel Wall Follower Node — Centerline Path Following
=============================================================================
LiDAR-based wall following for the tunnel section where camera lane detection
may not work due to poor lighting/visibility.

**Centerline approach** (similar to camera lane follower):
1. Classify LiDAR points into left and right wall sets
2. Bin wall points by forward distance (x-coordinate)
3. For each bin, compute the midpoint between left and right wall
4. These midpoints form a centerline path through the tunnel
5. The robot steers toward the centerline using a PD controller:
   - Lateral error = how far the centerline is from the robot's y=0 axis
   - Heading error = angle of the centerline path relative to forward

Publishes Twist on /tunnel_cmd_vel for auto_driver to use when in TUNNEL state.
"""

import math
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .topics import TUNNEL_CMD_TOPIC, TUNNEL_DETECTED_TOPIC


class TunnelWallFollower(Node):
    """LiDAR-based wall following using centerline path computation."""

    def __init__(self):
        super().__init__('tunnel_wall_follower')

        # --- Parameters ---
        self.declare_parameter('target_center_dist', 0.0)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('kp', 5.0)          # dramatically increased to match lane follower aggressiveness
        self.declare_parameter('kd', 0.5)          # increased to dampen the higher P gain
        self.declare_parameter('kp_heading', 1.0)  # stronger heading correction
        self.declare_parameter('kd_heading', 0.1)
        self.declare_parameter('max_angular', 2.0) # increased from 1.0 to 2.0 (same as lane follower)
        self.declare_parameter('output_alpha', 0.4)
        self.declare_parameter('left_angle_min', 0.26)        # ~15°
        self.declare_parameter('left_angle_max', 2.09)        # ~120°
        self.declare_parameter('right_angle_min', -2.09)      # ~-120°
        self.declare_parameter('right_angle_max', -0.26)      # ~-15°
        self.declare_parameter('lidar_angle_offset', 3.1416)  # 180° mount correction
        self.declare_parameter('min_wall_points', 5)
        self.declare_parameter('max_wall_dist', 0.80)
        self.declare_parameter('ransac_threshold', 0.03)      # inlier distance (m)
        self.declare_parameter('ransac_iterations', 50)
        self.declare_parameter('tunnel_hysteresis_frames', 3)
        self.declare_parameter('heartbeat_sec', 0.2)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, TUNNEL_CMD_TOPIC, 10)
        self.in_tunnel_pub = self.create_publisher(Bool, TUNNEL_DETECTED_TOPIC, 10)
        self.debug_pub = self.create_publisher(String, '/tunnel_debug', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.last_lateral_error = 0.0
        self.last_heading_error = 0.0
        self.smoothed_angular_z = 0.0   # exponential moving average output
        self.last_time = self.get_clock().now()
        self.last_cmd = Twist()
        self.last_in_tunnel = False
        self.last_centerline = []

        # Hysteresis counters
        self._tunnel_on_count = 0
        self._tunnel_off_count = 0

        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )

        self.get_logger().info('Tunnel Wall Follower started (Centerline Path)')

    # ── Parameter management ─────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'target_center_dist': float(self.get_parameter('target_center_dist').value),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'kp': float(self.get_parameter('kp').value),
            'kd': float(self.get_parameter('kd').value),
            'kp_heading': float(self.get_parameter('kp_heading').value),
            'kd_heading': float(self.get_parameter('kd_heading').value),
            'max_angular': float(self.get_parameter('max_angular').value),
            'output_alpha': float(self.get_parameter('output_alpha').value),
            'left_angle_min': float(self.get_parameter('left_angle_min').value),
            'left_angle_max': float(self.get_parameter('left_angle_max').value),
            'right_angle_min': float(self.get_parameter('right_angle_min').value),
            'right_angle_max': float(self.get_parameter('right_angle_max').value),
            'lidar_angle_offset': float(self.get_parameter('lidar_angle_offset').value),
            'min_wall_points': int(self.get_parameter('min_wall_points').value),
            'max_wall_dist': float(self.get_parameter('max_wall_dist').value),
            'ransac_threshold': float(self.get_parameter('ransac_threshold').value),
            'ransac_iterations': int(self.get_parameter('ransac_iterations').value),
            'tunnel_hysteresis_frames': int(self.get_parameter('tunnel_hysteresis_frames').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _heartbeat_publish(self) -> None:
        self.in_tunnel_pub.publish(Bool(data=self.last_in_tunnel))
        self.cmd_vel_pub.publish(self.last_cmd)

    # ── Centerline computation ───────────────────────────────────────────

    @staticmethod
    def _compute_centerline(
        left_xy: List[Tuple[float, float]],
        right_xy: List[Tuple[float, float]],
        bin_width: float = 0.05
    ) -> List[Tuple[float, float]]:
        """
        Compute centerline path by binning wall points by x-coordinate.

        For each x-bin where both left and right walls have points:
          center_y = (avg_left_y + avg_right_y) / 2

        Returns list of (x, center_y) points sorted by x (nearest first).
        """
        # Build bins: x_bin → {left_y_values, right_y_values}
        bins: Dict[int, Dict[str, List[float]]] = {}

        for x, y in left_xy:
            b = int(x / bin_width)
            if b not in bins:
                bins[b] = {'left': [], 'right': []}
            bins[b]['left'].append(y)

        for x, y in right_xy:
            b = int(x / bin_width)
            if b not in bins:
                bins[b] = {'left': [], 'right': []}
            bins[b]['right'].append(y)

        # Compute midpoints for bins that have BOTH left and right data
        centerline = []
        for b, data in bins.items():
            if data['left'] and data['right']:
                avg_left_y = sum(data['left']) / len(data['left'])
                avg_right_y = sum(data['right']) / len(data['right'])
                center_y = (avg_left_y + avg_right_y) / 2.0
                center_x = (b + 0.5) * bin_width
                centerline.append((center_x, center_y))

        # Sort by x (forward distance, nearest first)
        centerline.sort(key=lambda p: p[0])
        return centerline

    # ── Main scan processing ─────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan) -> None:
        """Process each LiDAR scan: classify walls, compute centerline, PD control."""
        offset = self._param_cache['lidar_angle_offset']
        l_min = self._param_cache['left_angle_min']
        l_max = self._param_cache['left_angle_max']
        r_min = self._param_cache['right_angle_min']
        r_max = self._param_cache['right_angle_max']
        max_wall = self._param_cache['max_wall_dist']
        min_pts = int(self._param_cache['min_wall_points'])

        left_xy = []
        right_xy = []

        # ── 1. Convert polar → Cartesian, classify left / right ──────────
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            if math.isnan(r) or math.isinf(r) or r > max_wall:
                continue

            angle = msg.angle_min + i * msg.angle_increment + offset
            angle = math.atan2(math.sin(angle), math.cos(angle))

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if l_min <= angle <= l_max:
                left_xy.append((x, y))
            elif r_min <= angle <= r_max:
                right_xy.append((x, y))

        has_left = len(left_xy) >= min_pts
        has_right = len(right_xy) >= min_pts
        walls_detected = has_left and has_right

        # ── 2. Hysteresis for tunnel detection ───────────────────────────
        hyst = int(self._param_cache['tunnel_hysteresis_frames'])

        if walls_detected:
            self._tunnel_on_count = min(self._tunnel_on_count + 1, hyst + 1)
            self._tunnel_off_count = 0
        else:
            self._tunnel_off_count = min(self._tunnel_off_count + 1, hyst + 1)
            self._tunnel_on_count = 0

        if not self.last_in_tunnel and self._tunnel_on_count >= hyst:
            self.last_in_tunnel = True
            self.get_logger().info(
                f'TUNNEL ENTERED (L={len(left_xy)} R={len(right_xy)} pts)')
        elif self.last_in_tunnel and self._tunnel_off_count >= hyst:
            self.last_in_tunnel = False
            self.get_logger().info('TUNNEL EXITED')

        self.in_tunnel_pub.publish(Bool(data=self.last_in_tunnel))

        # ── 3. Centerline computation + PD control ───────────────────────
        cmd = Twist()

        if self.last_in_tunnel and walls_detected:
            # Compute the centerline path
            centerline = self._compute_centerline(left_xy, right_xy)
            self.last_centerline = centerline

            if len(centerline) >= 2:
                # --- Lateral error ---
                # Use the centerline point nearest to the robot (smallest x)
                # center_y > 0 means center is to the LEFT → steer LEFT (positive ω)
                # center_y < 0 means center is to the RIGHT → steer RIGHT (negative ω)
                lateral_error = centerline[0][1]  # y-offset of nearest centerline point

                # Add target offset if desired (default 0 = stay centered)
                target = float(self._param_cache['target_center_dist'])
                lateral_error += target

                # --- Heading error ---
                # Fit a simple line through the centerline points to get heading
                # Use the first few points (closest to robot) for heading
                n_heading = min(len(centerline), 5)
                xs = [p[0] for p in centerline[:n_heading]]
                ys = [p[1] for p in centerline[:n_heading]]

                # Simple linear regression: y = mx + b
                x_mean = sum(xs) / len(xs)
                y_mean = sum(ys) / len(ys)
                num = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(xs, ys))
                den = sum((xi - x_mean) ** 2 for xi in xs)

                if abs(den) > 1e-8:
                    slope = num / den
                    heading_error = math.atan(slope)  # angle of centerline
                else:
                    heading_error = 0.0

                # --- Time delta ---
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                if dt <= 0 or dt > 0.5:
                    dt = 0.1

                # --- Derivatives ---
                d_lateral = (lateral_error - self.last_lateral_error) / dt
                d_heading = (heading_error - self.last_heading_error) / dt

                # --- PD gains ---
                kp = float(self._param_cache['kp'])
                kd = float(self._param_cache['kd'])
                kp_h = float(self._param_cache['kp_heading'])
                kd_h = float(self._param_cache['kd_heading'])
                max_ang = float(self._param_cache['max_angular'])

                # --- Combined steering ---
                angular_z = (kp * lateral_error + kd * d_lateral
                             + kp_h * heading_error + kd_h * d_heading)
                angular_z = max(-max_ang, min(max_ang, angular_z))

                # --- EMA smoothing to prevent sudden direction reversals ---
                alpha = float(self._param_cache['output_alpha'])
                self.smoothed_angular_z = (alpha * angular_z
                                           + (1.0 - alpha) * self.smoothed_angular_z)

                cmd.linear.x = float(self._param_cache['forward_speed'])
                cmd.angular.z = -self.smoothed_angular_z  # negate: servo uses inverted convention

                self.last_lateral_error = lateral_error
                self.last_heading_error = heading_error
                self.last_time = now

                # --- Compute L/R distances for debug display ---
                left_dists = [math.sqrt(x*x + y*y) for x, y in left_xy]
                right_dists = [math.sqrt(x*x + y*y) for x, y in right_xy]
                avg_l = sum(left_dists) / len(left_dists) if left_dists else 0
                avg_r = sum(right_dists) / len(right_dists) if right_dists else 0

                direction = 'LEFT' if angular_z > 0.01 else ('RIGHT' if angular_z < -0.01 else 'STRAIGHT')
                self.get_logger().info(
                    f'CL: lat:{lateral_error:.3f} head:{math.degrees(heading_error):.1f}° '
                    f'ω:{angular_z:.2f} ({direction}) '
                    f'L~{avg_l:.2f}m R~{avg_r:.2f}m pts:{len(centerline)}')

                # Publish debug: JSON with all info including centerline
                import json
                cl_pts = [{'x': round(p[0], 3), 'y': round(p[1], 3)} for p in centerline[:10]]
                dbg_obj = {
                    'l': round(avg_l, 3), 'r': round(avg_r, 3),
                    'lat': round(lateral_error, 3), 'w': round(angular_z, 3),
                    'cl': cl_pts
                }
                self.debug_pub.publish(String(data=json.dumps(dbg_obj)))

            else:
                # Not enough centerline points — drive straight slowly
                cmd.linear.x = float(self._param_cache['forward_speed']) * 0.5
                self.get_logger().info('CL: too few midpoints, creeping forward')

        else:
            # Not in tunnel — publish zero, reset errors
            self.last_lateral_error = 0.0
            self.last_heading_error = 0.0
            self.last_centerline = []

        self.cmd_vel_pub.publish(cmd)
        self.last_cmd = cmd


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TunnelWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
