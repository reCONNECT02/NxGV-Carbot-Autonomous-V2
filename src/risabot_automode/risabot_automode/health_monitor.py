#!/usr/bin/env python3
"""Health monitor node for freshness diagnostics across core competition topics."""

import json
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String

from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BOOM_GATE_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    DASH_STATE_TOPIC,
    HEALTH_STATUS_TOPIC,
    JOY_TOPIC,
    LANE_ERROR_TOPIC,
    OBSTACLE_CAMERA_TOPIC,
    OBSTACLE_LIDAR_TOPIC,
    ODOM_TOPIC,
    OBSTRUCTION_ACTIVE_TOPIC,
    PARKING_COMPLETE_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_DETECTED_TOPIC,
)


class HealthMonitor(Node):
    """Publishes a periodic freshness summary for mission-critical streams."""

    def __init__(self) -> None:
        super().__init__('health_monitor')

        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('timeout_perception', 1.5)
        self.declare_parameter('timeout_state', 2.0)
        self.declare_parameter('timeout_control', 1.0)
        self.declare_parameter('timeout_odom', 1.0)
        self.declare_parameter('timeout_joy', 2.0)

        self.health_pub = self.create_publisher(String, HEALTH_STATUS_TOPIC, 10)

        self.auto_mode = False
        self.last_seen: Dict[str, float] = {}
        self.watch_timeouts: Dict[str, float] = {}
        self._refresh_timeouts()
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(Bool, AUTO_MODE_TOPIC, self._auto_mode_cb, 10)
        self.create_subscription(Float32, LANE_ERROR_TOPIC, lambda _: self._touch('lane_error'), sensor_qos)
        self.create_subscription(Bool, OBSTACLE_LIDAR_TOPIC, lambda _: self._touch('obstacle_front'), sensor_qos)
        self.create_subscription(Bool, OBSTACLE_CAMERA_TOPIC, lambda _: self._touch('obstacle_camera'), sensor_qos)
        self.create_subscription(String, TRAFFIC_LIGHT_TOPIC, lambda _: self._touch('traffic_light'), 10)
        self.create_subscription(Bool, BOOM_GATE_TOPIC, lambda _: self._touch('boom_gate'), 10)
        self.create_subscription(Bool, TUNNEL_DETECTED_TOPIC, lambda _: self._touch('tunnel_detected'), 10)
        self.create_subscription(Bool, OBSTRUCTION_ACTIVE_TOPIC, lambda _: self._touch('obstruction_active'), 10)
        self.create_subscription(Bool, PARKING_COMPLETE_TOPIC, lambda _: self._touch('parking_complete'), 10)
        self.create_subscription(String, DASH_STATE_TOPIC, lambda _: self._touch('dashboard_state'), 10)
        self.create_subscription(Odometry, ODOM_TOPIC, lambda _: self._touch('odom'), 10)
        self.create_subscription(Joy, JOY_TOPIC, lambda _: self._touch('joy'), sensor_qos)
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, lambda _: self._touch('cmd_vel_auto'), 10)
        self.create_subscription(String, CMD_SAFETY_STATUS_TOPIC, lambda _: self._touch('cmd_safety_status'), 10)

        period = float(self.get_parameter('publish_period').value)
        self.create_timer(period, self._publish_health)

        self.get_logger().info('Health monitor started')

    def _refresh_timeouts(self) -> None:
        """Refresh timeout map from parameters."""
        perception = float(self.get_parameter('timeout_perception').value)
        state = float(self.get_parameter('timeout_state').value)
        control = float(self.get_parameter('timeout_control').value)
        odom = float(self.get_parameter('timeout_odom').value)
        joy = float(self.get_parameter('timeout_joy').value)
        self.watch_timeouts = {
            'lane_error': perception,
            'obstacle_front': perception,
            'obstacle_camera': perception,
            'traffic_light': perception,
            'boom_gate': perception,
            'tunnel_detected': perception,
            'obstruction_active': perception,
            'parking_complete': perception,
            'dashboard_state': state,
            'cmd_vel_auto': control,
            'cmd_safety_status': control,
            'odom': odom,
            'joy': joy,
            'auto_mode': state,
        }

    def _touch(self, key: str) -> None:
        """Mark a topic as freshly received."""
        self.last_seen[key] = time.monotonic()

    def _auto_mode_cb(self, msg: Bool) -> None:
        """Track current auto mode and freshness of auto_mode stream."""
        self.auto_mode = bool(msg.data)
        self._touch('auto_mode')

    def _publish_health(self) -> None:
        """Publish aggregate health payload as JSON string."""
        self._refresh_timeouts()
        now = time.monotonic()

        ages_sec = {}
        stale = []
        for key, timeout in self.watch_timeouts.items():
            last = self.last_seen.get(key, 0.0)
            age = None if last == 0.0 else round(now - last, 3)
            ages_sec[key] = age

            required = True
            if not self.auto_mode:
                required = key in ('auto_mode', 'joy', 'odom')
            if age is None and required:
                stale.append(key)
            elif age is not None and required and age > timeout:
                stale.append(key)

        ok = len(stale) == 0
        summary = 'ok' if ok else ('stale:' + ','.join(stale))
        payload = {
            'ok': ok,
            'auto_mode': self.auto_mode,
            'stale': stale,
            'ages_sec': ages_sec,
            'summary': summary,
            'stamp_sec': round(time.time(), 3),
        }
        self.health_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
