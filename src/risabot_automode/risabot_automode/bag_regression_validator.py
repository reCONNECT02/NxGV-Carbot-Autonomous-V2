#!/usr/bin/env python3
"""Runtime regression validator for bag replay or live competition runs."""

import json
import sys
import threading
import time
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .topics import (
    AUTO_CMD_VEL_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    HEALTH_STATUS_TOPIC,
    LOOP_STATS_TOPIC,
    ODOM_TOPIC,
)


class BagRegressionValidator(Node):
    """Collects runtime metrics and emits a pass/fail summary."""

    def __init__(self) -> None:
        super().__init__('bag_regression_validator')

        self.declare_parameter('window_sec', 45.0)
        self.declare_parameter('min_loop_hz_ratio', 0.80)
        self.declare_parameter('max_loop_overruns', 10)
        self.declare_parameter('max_cmd_linear', 0.35)
        self.declare_parameter('max_cmd_angular', 1.5)
        self.declare_parameter('max_odom_speed', 1.2)
        self.declare_parameter('require_health_ok', True)
        self.declare_parameter('fail_on_cmd_timeouts', True)
        self.declare_parameter('fail_on_estop', True)
        self.declare_parameter(
            'require_loop_topics',
            [
                'auto_driver:auto_driver_cmd',
                'cmd_safety_controller:cmd_safety',
                'servo_controller:servo_hw',
                'servo_controller:servo_encoder',
            ],
        )
        self.declare_parameter('output_file', '')

        self.window_sec = float(self.get_parameter('window_sec').value)
        self.start_t = time.monotonic()
        self.done_event = threading.Event()
        self.result_ok = False
        self.summary: Dict[str, object] = {}

        self.loop_latest: Dict[str, Dict[str, object]] = {}
        self.loop_seen_count: Dict[str, int] = {}
        self.max_cmd_linear_seen = 0.0
        self.max_cmd_angular_seen = 0.0
        self.max_odom_speed_seen = 0.0
        self.health_msgs = 0
        self.health_not_ok = 0
        self.last_health_stale: List[str] = []
        self.cmd_safety_timeout_count = 0
        self.cmd_safety_estop_count = 0

        self.create_subscription(String, LOOP_STATS_TOPIC, self._loop_cb, 50)
        self.create_subscription(String, HEALTH_STATUS_TOPIC, self._health_cb, 10)
        self.create_subscription(String, CMD_SAFETY_STATUS_TOPIC, self._cmd_safety_cb, 10)
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, self._cmd_cb, 20)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 20)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'Regression validator collecting metrics for {self.window_sec:.1f}s'
        )

    def _loop_cb(self, msg: String) -> None:
        """Store latest loop stats per node:loop key."""
        try:
            payload = json.loads(msg.data)
            node = str(payload.get('node', 'unknown'))
            loop = str(payload.get('loop', 'unknown'))
            key = f'{node}:{loop}'
            self.loop_latest[key] = payload
            self.loop_seen_count[key] = self.loop_seen_count.get(key, 0) + 1
        except Exception:
            return

    def _health_cb(self, msg: String) -> None:
        """Track health status failures."""
        self.health_msgs += 1
        try:
            payload = json.loads(msg.data)
            if not bool(payload.get('ok', False)):
                self.health_not_ok += 1
                self.last_health_stale = list(payload.get('stale', []))
        except Exception:
            self.health_not_ok += 1

    def _cmd_safety_cb(self, msg: String) -> None:
        """Track command safety counters."""
        try:
            payload = json.loads(msg.data)
            self.cmd_safety_timeout_count = max(
                self.cmd_safety_timeout_count,
                int(payload.get('timeout_count', 0)),
            )
            self.cmd_safety_estop_count = max(
                self.cmd_safety_estop_count,
                int(payload.get('estop_count', 0)),
            )
        except Exception:
            return

    def _cmd_cb(self, msg: Twist) -> None:
        """Track worst-case command magnitudes."""
        self.max_cmd_linear_seen = max(self.max_cmd_linear_seen, abs(float(msg.linear.x)))
        self.max_cmd_angular_seen = max(self.max_cmd_angular_seen, abs(float(msg.angular.z)))

    def _odom_cb(self, msg: Odometry) -> None:
        """Track worst-case odometry linear speed."""
        self.max_odom_speed_seen = max(
            self.max_odom_speed_seen,
            abs(float(msg.twist.twist.linear.x)),
        )

    def _tick(self) -> None:
        """Finalize once the collection window elapses."""
        if time.monotonic() - self.start_t < self.window_sec:
            return
        self._finalize()
        self.done_event.set()

    def _finalize(self) -> None:
        """Evaluate thresholds and produce pass/fail summary."""
        min_ratio = float(self.get_parameter('min_loop_hz_ratio').value)
        max_overruns = int(self.get_parameter('max_loop_overruns').value)
        max_cmd_linear = float(self.get_parameter('max_cmd_linear').value)
        max_cmd_angular = float(self.get_parameter('max_cmd_angular').value)
        max_odom_speed = float(self.get_parameter('max_odom_speed').value)
        require_health_ok = bool(self.get_parameter('require_health_ok').value)
        fail_on_timeouts = bool(self.get_parameter('fail_on_cmd_timeouts').value)
        fail_on_estop = bool(self.get_parameter('fail_on_estop').value)
        required_loops = [str(v) for v in list(self.get_parameter('require_loop_topics').value)]

        failures: List[str] = []
        loop_evaluation: Dict[str, Dict[str, object]] = {}

        for loop_key, payload in self.loop_latest.items():
            target = float(payload.get('target_hz', 0.0) or 0.0)
            avg = float(payload.get('avg_hz', 0.0) or 0.0)
            overruns = int(payload.get('overruns', 0) or 0)
            ratio = (avg / target) if target > 0.0 else 0.0
            loop_evaluation[loop_key] = {
                'avg_hz': round(avg, 3),
                'target_hz': round(target, 3),
                'ratio': round(ratio, 3),
                'overruns': overruns,
                'samples': int(payload.get('samples', 0) or 0),
            }
            if target > 0.0 and ratio < min_ratio:
                failures.append(
                    f'loop_ratio_low:{loop_key} avg={avg:.2f} target={target:.2f} ratio={ratio:.2f}'
                )
            if overruns > max_overruns:
                failures.append(
                    f'loop_overruns_high:{loop_key} overruns={overruns} max={max_overruns}'
                )

        for required_key in required_loops:
            if required_key not in self.loop_latest:
                failures.append(f'missing_loop_stats:{required_key}')

        if self.max_cmd_linear_seen > max_cmd_linear:
            failures.append(
                f'cmd_linear_high:max={self.max_cmd_linear_seen:.3f} limit={max_cmd_linear:.3f}'
            )
        if self.max_cmd_angular_seen > max_cmd_angular:
            failures.append(
                f'cmd_angular_high:max={self.max_cmd_angular_seen:.3f} limit={max_cmd_angular:.3f}'
            )
        if self.max_odom_speed_seen > max_odom_speed:
            failures.append(
                f'odom_speed_high:max={self.max_odom_speed_seen:.3f} limit={max_odom_speed:.3f}'
            )
        if require_health_ok and self.health_not_ok > 0:
            failures.append(
                f'health_not_ok:count={self.health_not_ok} last_stale={",".join(self.last_health_stale)}'
            )
        if fail_on_timeouts and self.cmd_safety_timeout_count > 0:
            failures.append(f'cmd_timeout_count:{self.cmd_safety_timeout_count}')
        if fail_on_estop and self.cmd_safety_estop_count > 0:
            failures.append(f'cmd_estop_count:{self.cmd_safety_estop_count}')

        self.result_ok = len(failures) == 0
        self.summary = {
            'ok': self.result_ok,
            'window_sec': self.window_sec,
            'metrics': {
                'max_cmd_linear': round(self.max_cmd_linear_seen, 3),
                'max_cmd_angular': round(self.max_cmd_angular_seen, 3),
                'max_odom_speed': round(self.max_odom_speed_seen, 3),
                'health_msgs': self.health_msgs,
                'health_not_ok': self.health_not_ok,
                'cmd_safety_timeout_count': self.cmd_safety_timeout_count,
                'cmd_safety_estop_count': self.cmd_safety_estop_count,
            },
            'loops': loop_evaluation,
            'failures': failures,
            'stamp_sec': round(time.time(), 3),
        }

        output = json.dumps(self.summary, separators=(',', ':'))
        if self.result_ok:
            self.get_logger().info(f'REGRESSION PASS {output}')
        else:
            self.get_logger().error(f'REGRESSION FAIL {output}')

        out_file = str(self.get_parameter('output_file').value).strip()
        if out_file:
            try:
                with open(out_file, 'w', encoding='utf-8') as f:
                    json.dump(self.summary, f, indent=2)
            except Exception as exc:
                self.get_logger().warn(f'Failed to write output file {out_file}: {exc}')


def main(args=None) -> None:
    """Run validator for one collection window and exit with pass/fail code."""
    rclpy.init(args=args)
    node = BagRegressionValidator()
    try:
        while rclpy.ok() and not node.done_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.result_ok
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
