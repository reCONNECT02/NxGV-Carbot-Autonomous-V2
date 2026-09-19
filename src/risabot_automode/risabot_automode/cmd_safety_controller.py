#!/usr/bin/env python3
"""Safety envelope node for autonomous velocity commands."""

import json
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .loop_monitor import LoopMonitor
from .topics import (
    AUTO_CMD_VEL_RAW_TOPIC,
    AUTO_CMD_VEL_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    E_STOP_TOPIC,
    LOOP_STATS_TOPIC,
)


class CmdSafetyController(Node):
    """Applies limits, timeout handling, and e-stop gating to autonomous cmd_vel."""

    def __init__(self) -> None:
        super().__init__('cmd_safety_controller')

        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.35)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('max_linear_accel', 0.8)
        self.declare_parameter('max_angular_accel', 4.0)
        self.declare_parameter('deadband_linear', 0.01)
        self.declare_parameter('deadband_angular', 0.01)
        self.declare_parameter('publish_loop_stats', True)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(Twist, AUTO_CMD_VEL_TOPIC, 10)
        self.status_pub = self.create_publisher(String, CMD_SAFETY_STATUS_TOPIC, 10)
        self.loop_stats_pub = self.create_publisher(String, LOOP_STATS_TOPIC, 10)

        self.create_subscription(Twist, AUTO_CMD_VEL_RAW_TOPIC, self._raw_cmd_cb, 10)
        self.create_subscription(Bool, E_STOP_TOPIC, self._estop_cb, 10)

        self.target_cmd = Twist()
        self.output_cmd = Twist()
        self.estop = False
        self.last_input_t = 0.0
        self.last_loop_t = time.monotonic()
        self.timeout_count = 0
        self.estop_count = 0
        self.limit_count = 0

        hz = float(self._param_cache['publish_hz'])
        self.loop_monitor = LoopMonitor('cmd_safety', hz)
        period = 1.0 / hz if hz > 0 else 0.02
        self.create_timer(period, self._control_loop)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Cmd safety controller started')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'publish_hz': float(self.get_parameter('publish_hz').value),
            'cmd_timeout': float(self.get_parameter('cmd_timeout').value),
            'max_linear_speed': float(self.get_parameter('max_linear_speed').value),
            'max_angular_speed': float(self.get_parameter('max_angular_speed').value),
            'max_linear_accel': float(self.get_parameter('max_linear_accel').value),
            'max_angular_accel': float(self.get_parameter('max_angular_accel').value),
            'deadband_linear': float(self.get_parameter('deadband_linear').value),
            'deadband_angular': float(self.get_parameter('deadband_angular').value),
            'publish_loop_stats': bool(self.get_parameter('publish_loop_stats').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cache for dynamic params."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                if p.name == 'publish_hz':
                    self.get_logger().warn('publish_hz change requires node restart to retime timer')
        return SetParametersResult(successful=True)

    def _raw_cmd_cb(self, msg: Twist) -> None:
        """Store incoming autonomous command."""
        self.target_cmd = msg
        self.last_input_t = time.monotonic()

    def _estop_cb(self, msg: Bool) -> None:
        """Update e-stop latch."""
        self.estop = bool(msg.data)
        if self.estop:
            self.estop_count += 1

    @staticmethod
    def _slew(current: float, target: float, max_rate: float, dt: float) -> float:
        """Rate-limit a value based on max units/second."""
        step = max_rate * dt
        if target > current + step:
            return current + step
        if target < current - step:
            return current - step
        return target

    def _control_loop(self) -> None:
        """Apply safety envelope and publish safe cmd_vel."""
        self.loop_monitor.tick()
        now = time.monotonic()
        dt = max(1e-3, now - self.last_loop_t)
        self.last_loop_t = now

        cmd_timeout = float(self._param_cache['cmd_timeout'])
        stale = (now - self.last_input_t) > cmd_timeout
        if stale:
            self.timeout_count += 1

        if self.estop or stale:
            target_lin = 0.0
            target_ang = 0.0
        else:
            target_lin = float(self.target_cmd.linear.x)
            target_ang = float(self.target_cmd.angular.z)

        max_lin = float(self._param_cache['max_linear_speed'])
        max_ang = float(self._param_cache['max_angular_speed'])
        limited_lin = max(-max_lin, min(max_lin, target_lin))
        limited_ang = max(-max_ang, min(max_ang, target_ang))
        if limited_lin != target_lin or limited_ang != target_ang:
            self.limit_count += 1

        max_lin_acc = float(self._param_cache['max_linear_accel'])
        max_ang_acc = float(self._param_cache['max_angular_accel'])
        out_lin = self._slew(self.output_cmd.linear.x, limited_lin, max_lin_acc, dt)
        out_ang = self._slew(self.output_cmd.angular.z, limited_ang, max_ang_acc, dt)

        db_lin = float(self._param_cache['deadband_linear'])
        db_ang = float(self._param_cache['deadband_angular'])
        if abs(out_lin) < db_lin:
            out_lin = 0.0
        if abs(out_ang) < db_ang:
            out_ang = 0.0

        self.output_cmd.linear.x = out_lin
        self.output_cmd.angular.z = out_ang
        self.cmd_pub.publish(self.output_cmd)

    def _publish_status(self) -> None:
        """Publish safety and loop diagnostics as JSON strings."""
        payload = {
            'estop': self.estop,
            'timeout_count': self.timeout_count,
            'estop_count': self.estop_count,
            'limit_count': self.limit_count,
            'stamp_sec': round(time.time(), 3),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

        if bool(self._param_cache['publish_loop_stats']):
            loop_payload = self.loop_monitor.snapshot()
            loop_payload['node'] = self.get_name()
            self.loop_stats_pub.publish(String(data=json.dumps(loop_payload, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
