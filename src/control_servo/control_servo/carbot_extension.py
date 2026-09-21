"""Carbot extension for the base servo_controller (phase 5). ADDITIVE ONLY:
no motor, servo, odometry or joystick code is changed.

servo_controller owns the Rosmaster serial port, so these two functions have
to live inside that process:

  battery   publishes Rosmaster.get_battery_voltage() on
            /carbot/vehicle/battery_v (Float32, volts) at carbot_battery_rate_hz
            (race preflight checks it). 0 V (no report from the board yet) is not
            published.
  arm       /carbot/vehicle/arm (Bool) from the command owner:
              True  -> AUTO mode (the base behaviour of the joystick Start/Y
                       toggle), so /cmd_vel_auto reaches the motors without a
                       joystick. Ignored during record/playback. After a base
                       'auto command stream stale' trip it re-arms only when
                       /cmd_vel_auto is fresh again.
              False -> MANUAL mode + stop (joystick driving again in calibrate).

The base auto_cmd_timeout watchdog, joy watchdog and hardware-failure trips
stay exactly as they are and still win.
"""
import time

from .topics import AUTO_MODE_TOPIC, VEHICLE_ARM_TOPIC, VEHICLE_BATTERY_TOPIC


def _param(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def arm_decision(manual_mode: bool, rp_state: str, want_auto: bool, last_auto_cmd_age: float,
                 auto_cmd_timeout: float):
    """Pure rule -> (new manual_mode, action) with action in
    ('none', 'to_auto', 'to_manual', 'ignored_playback', 'ignored_stale')."""
    if want_auto:
        if not manual_mode:
            return manual_mode, 'none'
        if rp_state != 'IDLE':
            return manual_mode, 'ignored_playback'
        if last_auto_cmd_age > auto_cmd_timeout:
            return manual_mode, 'ignored_stale'
        return False, 'to_auto'
    if manual_mode:
        return manual_mode, 'none'
    return True, 'to_manual'


class CarbotVehicleExtension:

    def __init__(self, node):
        from std_msgs.msg import Bool, Float32      # lazy: arm_decision() is testable without ROS
        self.Bool, self.Float32 = Bool, Float32
        self.node = node
        self.enabled_arm = bool(_param(node, 'carbot_arm_enabled', True))
        rate = float(_param(node, 'carbot_battery_rate_hz', 1.0))
        self.battery_pub = node.create_publisher(Float32, VEHICLE_BATTERY_TOPIC, 10)
        self._auto_pub = node.auto_mode_pub if hasattr(node, 'auto_mode_pub') else \
            node.create_publisher(Bool, AUTO_MODE_TOPIC, 10)
        if rate > 0:
            node.create_timer(1.0 / rate, self._battery)
        if self.enabled_arm:
            node.create_subscription(Bool, VEHICLE_ARM_TOPIC, self._on_arm, 10)
        self._last_action = ''
        node.get_logger().info(f'Carbot extension: battery {rate:.1f} Hz on {VEHICLE_BATTERY_TOPIC}, '
                               f'arm on {VEHICLE_ARM_TOPIC} ({"enabled" if self.enabled_arm else "disabled"})')

    def _battery(self) -> None:
        try:
            v = float(self.node.bot.get_battery_voltage())
        except Exception:  # noqa: BLE001  board not reporting
            return
        if v > 0.0:
            self.battery_pub.publish(self.Float32(data=v))

    def _on_arm(self, msg) -> None:
        n = self.node
        last = getattr(n, 'last_auto_cmd_time', 0.0)
        age = time.monotonic() - last if last > 0.0 else float('inf')
        timeout = float(n._param_cache.get('auto_cmd_timeout', 0.4))
        new_manual, action = arm_decision(n.manual_mode, getattr(n, 'rp_state', 'IDLE'), bool(msg.data), age, timeout)
        if action == 'to_auto':
            n.manual_mode = False
            n.last_auto_cmd_time = time.monotonic()
            n.auto_cmd_stale_reported = False
            self._auto_pub.publish(self.Bool(data=True))
            n.get_logger().info('Carbot ARM: AUTO (command owner)')
            n._update_dash()
        elif action == 'to_manual':
            n.manual_mode = True
            n.stop_robot()
            self._auto_pub.publish(self.Bool(data=False))
            n.get_logger().info('Carbot DISARM: MANUAL + stop')
            n._update_dash()
        elif action.startswith('ignored') and action != self._last_action:
            n.get_logger().warn(f'Carbot ARM ignored: {action}')
        self._last_action = action


def attach(node) -> CarbotVehicleExtension:
    node.carbot_extension = CarbotVehicleExtension(node)
    return node.carbot_extension
