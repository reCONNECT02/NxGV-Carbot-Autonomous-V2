#!/usr/bin/env python3
"""
Obstruction Avoidance Node (Lateral)
Unlike the existing obstacle_avoidance node which just publishes a stop signal,
this module actively steers AROUND an obstruction in the lane.

Uses LiDAR to detect which side the obstruction is on, then publishes
steering commands to navigate around it while maintaining forward progress.

Publishes: /obstruction_cmd_vel (Twist), /obstruction_active (Bool)
"""

import math
from enum import Enum
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .topics import OBSTRUCTION_ACTIVE_TOPIC, OBSTRUCTION_CMD_TOPIC


class AvoidPhase(Enum):
    CLEAR = 0           # No obstruction
    STEER_AWAY = 1      # Steering around obstruction
    PASS_ALONGSIDE = 2  # Moving past obstruction
    STEER_BACK = 3      # Returning to lane center


class ObstructionAvoidance(Node):
    """Steers around an obstruction detected via LiDAR."""

    def __init__(self):
        super().__init__('obstruction_avoidance')

        # --- Parameters ---
        self.declare_parameter('detect_dist', 0.50)          # m — start avoiding at this distance
        self.declare_parameter('clear_dist', 0.60)           # m — consider cleared beyond this
        self.declare_parameter('front_angle', 0.35)          # rad — front detection arc (±20°)
        self.declare_parameter('side_angle_min', 0.35)       # rad
        self.declare_parameter('side_angle_max', 1.20)       # rad (~70°)
        self.declare_parameter('steer_speed', 0.10)          # m/s forward while steering
        self.declare_parameter('steer_angular', 0.6)         # rad/s steering rate
        self.declare_parameter('pass_speed', 0.15)           # m/s while passing alongside
        self.declare_parameter('pass_duration', 2.0)         # seconds to drive alongside
        self.declare_parameter('steer_back_duration', 1.5)   # seconds to return to center
        self.declare_parameter('steer_away_duration', 1.0)   # seconds to steer away from obstacle
        self.declare_parameter('lidar_angle_offset', 1.5708) # 90° mount correction
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, OBSTRUCTION_CMD_TOPIC, 10)
        self.active_pub = self.create_publisher(Bool, OBSTRUCTION_ACTIVE_TOPIC, 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.phase = AvoidPhase.CLEAR
        self.avoid_direction = 0  # +1 = steer left, -1 = steer right
        self.phase_start_time = 0.0

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstruction Avoidance (lateral) started')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-scan lookups."""
        self._param_cache = {
            'detect_dist': float(self.get_parameter('detect_dist').value),
            'clear_dist': float(self.get_parameter('clear_dist').value),
            'front_angle': float(self.get_parameter('front_angle').value),
            'side_angle_min': float(self.get_parameter('side_angle_min').value),
            'side_angle_max': float(self.get_parameter('side_angle_max').value),
            'steer_speed': float(self.get_parameter('steer_speed').value),
            'steer_angular': float(self.get_parameter('steer_angular').value),
            'pass_speed': float(self.get_parameter('pass_speed').value),
            'pass_duration': float(self.get_parameter('pass_duration').value),
            'steer_back_duration': float(self.get_parameter('steer_back_duration').value),
            'steer_away_duration': float(self.get_parameter('steer_away_duration').value),
            'lidar_angle_offset': float(self.get_parameter('lidar_angle_offset').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan) -> None:
        """Analyze LiDAR to detect obstruction and determine avoid direction."""
        if self.phase != AvoidPhase.CLEAR:
            return  # Already in avoidance maneuver

        angle_offset = self._param_cache['lidar_angle_offset']
        detect_dist = self._param_cache['detect_dist']
        front_angle = self._param_cache['front_angle']

        front_left_dists = []
        front_right_dists = []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle += angle_offset
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Front arc detection
            if r <= detect_dist and -front_angle <= angle <= front_angle:
                if angle >= 0:
                    front_left_dists.append(r)
                else:
                    front_right_dists.append(r)

        # If we detect something close in front, start avoidance
        total_front = len(front_left_dists) + len(front_right_dists)
        if total_front >= 3:
            # Determine which side is more blocked → steer to the OTHER side
            left_blocked = len(front_left_dists)
            right_blocked = len(front_right_dists)

            if left_blocked > right_blocked:
                # More points on left → steer RIGHT to avoid
                self.avoid_direction = -1
                self.get_logger().info('Obstruction detected: steering RIGHT to avoid')
            else:
                # More points on right → steer LEFT to avoid
                self.avoid_direction = 1
                self.get_logger().info('Obstruction detected: steering LEFT to avoid')

            self._start_phase(AvoidPhase.STEER_AWAY)

    def _start_phase(self, phase: AvoidPhase) -> None:
        self.phase = phase
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'  -> Avoid phase: {phase.name}')

    def _time_in_phase(self) -> float:
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.phase_start_time

    def control_loop(self) -> None:
        """Execute avoidance maneuver phases."""
        cmd = Twist()
        is_active = self.phase != AvoidPhase.CLEAR

        if self.phase == AvoidPhase.STEER_AWAY:
            # Steer away from obstruction
            cmd.linear.x = self._param_cache['steer_speed']
            cmd.angular.z = self.avoid_direction * self._param_cache['steer_angular']
            if self._time_in_phase() >= self._param_cache['steer_away_duration']:
                self._start_phase(AvoidPhase.PASS_ALONGSIDE)

        elif self.phase == AvoidPhase.PASS_ALONGSIDE:
            # Drive straight past the obstruction
            cmd.linear.x = self._param_cache['pass_speed']
            if self._time_in_phase() >= self._param_cache['pass_duration']:
                self._start_phase(AvoidPhase.STEER_BACK)

        elif self.phase == AvoidPhase.STEER_BACK:
            # Steer back to lane center
            cmd.linear.x = self._param_cache['steer_speed']
            cmd.angular.z = -self.avoid_direction * self._param_cache['steer_angular'] * 0.7
            if self._time_in_phase() >= self._param_cache['steer_back_duration']:
                self.get_logger().info('Obstruction cleared, resuming lane following')
                self.phase = AvoidPhase.CLEAR
                self.avoid_direction = 0
                is_active = False

        # Publish
        self.cmd_vel_pub.publish(cmd)

        active_msg = Bool()
        active_msg.data = is_active
        self.active_pub.publish(active_msg)

        if is_active:
            direction = "LEFT" if self.avoid_direction > 0 else "RIGHT"
            self.get_logger().debug(f"{self.phase.name} -> steering {direction} | t: {self._time_in_phase():.1f}s")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstructionAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
