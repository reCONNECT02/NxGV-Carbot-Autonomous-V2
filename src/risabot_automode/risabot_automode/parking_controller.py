#!/usr/bin/env python3
"""
Parking Controller Node
Executes pre-programmed parallel and perpendicular parking maneuvers
using odometry-based dead reckoning. Triggered by the auto_driver state machine.

Signage detection is handled by the separate signage_detector node,
which publishes on /parking_signboard_detected.

Topics:
  Subscribes: /odom, /parking_command (String), /dashboard_state (String)
  Publishes:  /parking_cmd_vel (Twist), /parking_complete (Bool), /parking_status (String)
"""

from enum import Enum
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .topics import (
    DASH_STATE_TOPIC,
    ODOM_TOPIC,
    PARKING_CMD_TOPIC,
    PARKING_COMPLETE_TOPIC,
    PARKING_STATUS_TOPIC,
    PARKING_VEL_TOPIC,
)


class ParkingPhase(Enum):
    IDLE = 0
    # Parallel parking phases
    PARALLEL_FORWARD = 1      # Drive forward past the slot
    PARALLEL_STEER_REVERSE = 2  # Reverse while steering into slot
    PARALLEL_STRAIGHTEN = 3   # Straighten inside slot
    PARALLEL_WAIT = 4         # Wait inside slot
    PARALLEL_EXIT = 5         # Drive forward out of slot
    # Perpendicular parking phases
    PERP_TURN_IN = 6          # 90° turn into slot
    PERP_FORWARD = 7          # Drive into slot
    PERP_WAIT = 8             # Wait inside slot
    PERP_REVERSE_OUT = 9      # Reverse out of slot
    DONE = 10


class ParkingController(Node):
    """Odometry-based parking controller with signboard detection."""

    def __init__(self):
        super().__init__('parking_controller')

        # --- Parameters ---
        self.declare_parameter('parallel_forward_dist', 0.30)   # m to drive past slot
        self.declare_parameter('parallel_reverse_dist', 0.35)   # m to reverse into slot
        self.declare_parameter('parallel_steer_angle', 0.6)     # rad/s angular during reverse
        self.declare_parameter('perp_turn_angle', 1.57)         # ~90° turn
        self.declare_parameter('perp_forward_dist', 0.25)       # m into slot
        self.declare_parameter('park_wait_time', 3.0)           # seconds to wait in slot
        self.declare_parameter('drive_speed', 0.15)             # m/s linear speed
        self.declare_parameter('reverse_speed', -0.12)          # m/s reverse speed
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, PARKING_VEL_TOPIC, 10)
        self.complete_pub = self.create_publisher(Bool, PARKING_COMPLETE_TOPIC, 10)
        self.status_pub = self.create_publisher(String, PARKING_STATUS_TOPIC, 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, PARKING_CMD_TOPIC, self.command_callback, 10
        )
        self.dash_state_sub = self.create_subscription(
            String, DASH_STATE_TOPIC, self.dash_state_callback, 10
        )

        # State
        self.phase = ParkingPhase.IDLE
        self.current_lap = 1
        self.phase_start_time = self.get_clock().now()
        self.phase_start_dist = 0.0
        self.cumulative_yaw = 0.0
        self.current_speed = 0.0
        self.distance_traveled = 0.0
        self.last_odom_time = self.get_clock().now()
        self.current_command = 'none'

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Parking Controller started (IDLE)')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'parallel_forward_dist': float(self.get_parameter('parallel_forward_dist').value),
            'parallel_reverse_dist': float(self.get_parameter('parallel_reverse_dist').value),
            'parallel_steer_angle': float(self.get_parameter('parallel_steer_angle').value),
            'perp_turn_angle': float(self.get_parameter('perp_turn_angle').value),
            'perp_forward_dist': float(self.get_parameter('perp_forward_dist').value),
            'park_wait_time': float(self.get_parameter('park_wait_time').value),
            'drive_speed': float(self.get_parameter('drive_speed').value),
            'reverse_speed': float(self.get_parameter('reverse_speed').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry) -> None:
        """Track distance from odometry."""
        speed = msg.twist.twist.linear.x
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        self.current_speed = speed
        self.distance_traveled += abs(speed) * dt

    def dash_state_callback(self, msg: String) -> None:
        """Listen to auto_driver state to know which lap we are on."""
        try:
            parts = msg.data.split('|')
            if len(parts) > 1:
                self.current_lap = int(parts[1])
        except Exception:
            pass

    def command_callback(self, msg: String) -> None:
        """Receive parking command from auto_driver state machine."""
        cmd = msg.data.lower()
        if cmd == 'parallel' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('Starting PARALLEL parking maneuver')
            self.current_command = 'parallel'
            self.complete_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='parallel_start'))
            self._start_phase(ParkingPhase.PARALLEL_FORWARD)
        elif cmd == 'perpendicular' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('Starting PERPENDICULAR parking maneuver')
            self.current_command = 'perpendicular'
            self.complete_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='perpendicular_start'))
            self._start_phase(ParkingPhase.PERP_TURN_IN)
        elif cmd == 'stop':
            self.current_command = 'none'
            self._start_phase(ParkingPhase.IDLE)

    def _start_phase(self, phase: ParkingPhase) -> None:
        """Transition to a new parking phase."""
        self.phase = phase
        self.phase_start_time = self.get_clock().now()
        self.phase_start_dist = self.distance_traveled
        self.cumulative_yaw = 0.0
        self.get_logger().info(f'  -> Phase: {phase.name}')

    def _dist_since_phase(self) -> float:
        return self.distance_traveled - self.phase_start_dist

    def _time_since_phase(self) -> float:
        return (self.get_clock().now() - self.phase_start_time).nanoseconds / 1e9

    def control_loop(self) -> None:
        """Main parking control loop — executes maneuver phases."""
        cmd = Twist()
        drive_speed = self._param_cache['drive_speed']
        reverse_speed = self._param_cache['reverse_speed']

        if self.phase == ParkingPhase.IDLE:
            self.cmd_vel_pub.publish(cmd)  # zero velocity
            return

        # --- PARALLEL PARKING ---
        elif self.phase == ParkingPhase.PARALLEL_FORWARD:
            # Drive forward past the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self._param_cache['parallel_forward_dist']:
                self._start_phase(ParkingPhase.PARALLEL_STEER_REVERSE)

        elif self.phase == ParkingPhase.PARALLEL_STEER_REVERSE:
            # Reverse while steering into the slot
            cmd.linear.x = reverse_speed
            cmd.angular.z = -self._param_cache['parallel_steer_angle']  # steer right
            if self._dist_since_phase() >= self._param_cache['parallel_reverse_dist']:
                self._start_phase(ParkingPhase.PARALLEL_STRAIGHTEN)

        elif self.phase == ParkingPhase.PARALLEL_STRAIGHTEN:
            # Brief forward to straighten
            cmd.linear.x = drive_speed * 0.5
            cmd.angular.z = self._param_cache['parallel_steer_angle'] * 0.5  # counter-steer
            if self._dist_since_phase() >= 0.10:
                self._start_phase(ParkingPhase.PARALLEL_WAIT)

        elif self.phase == ParkingPhase.PARALLEL_WAIT:
            # Stop and wait
            wait_time = self._param_cache['park_wait_time']
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PARALLEL_EXIT)

        elif self.phase == ParkingPhase.PARALLEL_EXIT:
            # Drive forward out of slot
            cmd.linear.x = drive_speed
            cmd.angular.z = self._param_cache['parallel_steer_angle'] * 0.5  # steer left to exit
            if self._dist_since_phase() >= 0.30:
                self._finish()

        # --- PERPENDICULAR PARKING ---
        elif self.phase == ParkingPhase.PERP_TURN_IN:
            # Turn 90° into slot
            cmd.angular.z = 0.5  # turn left
            if self._time_since_phase() >= (self._param_cache['perp_turn_angle'] / 0.5):
                self._start_phase(ParkingPhase.PERP_FORWARD)

        elif self.phase == ParkingPhase.PERP_FORWARD:
            # Drive into the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self._param_cache['perp_forward_dist']:
                self._start_phase(ParkingPhase.PERP_WAIT)

        elif self.phase == ParkingPhase.PERP_WAIT:
            # Stop and wait
            wait_time = self._param_cache['park_wait_time']
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PERP_REVERSE_OUT)

        elif self.phase == ParkingPhase.PERP_REVERSE_OUT:
            # Reverse out
            cmd.linear.x = reverse_speed
            if self._dist_since_phase() >= self._param_cache['perp_forward_dist']:
                self._finish()

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Debug
        phase_name = self.phase.name
        dist = self._dist_since_phase()
        elapsed = self._time_since_phase()
        self.get_logger().debug(f"{phase_name} | dist: {dist:.3f}m | time: {elapsed:.1f}s")

    def _finish(self) -> None:
        """Complete the parking maneuver."""
        self.get_logger().info('Parking maneuver COMPLETE')
        self.phase = ParkingPhase.IDLE

        # Publish completion
        complete_msg = Bool()
        complete_msg.data = True
        self.complete_pub.publish(complete_msg)
        if self.current_command == 'parallel':
            self.status_pub.publish(String(data='parallel_done'))
        elif self.current_command == 'perpendicular':
            self.status_pub.publish(String(data='perpendicular_done'))
        self.current_command = 'none'

        # Stop robot
        self.cmd_vel_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
