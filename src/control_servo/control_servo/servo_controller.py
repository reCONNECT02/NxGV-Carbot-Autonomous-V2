#!/usr/bin/env python3
"""
Servo Controller V9 — Competition Ready
=============================================================================
Hardware interface between joystick, auto_driver, and Rosmaster motor board.
Handles mode toggling, manual driving, challenge cycling, and safety watchdog.

Features:
  - Joy watchdog: auto-stops robot if controller disconnects (no /joy for >0.5s)
  - Manual driving via set_motor(PWM) + set_pwm_servo(4, angle)
  - Auto mode: forwards /cmd_vel and /cmd_vel_auto to hardware
  - LB/RB challenge cycling via /set_challenge topic
  - D-Pad gear shifting with 4 speed levels

Controls:
  Left Stick Y (Axis 1):     Throttle (PWM ±255)
  Right Stick X (Axis 2):    Steering (Servo 4 Angle 40–140)
  D-Pad UP/DOWN (Axis 7):    Speed Limiter [25, 40, 60, 100]%
  Start (Btn 11) / Y (Btn 4): Toggle Auto/Manual
  LB (Btn 6):                Previous Challenge State
  RB (Btn 7):                Next Challenge State
"""

import json
import math
import os
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String, Float32

from Rosmaster_Lib import Rosmaster
from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BASE_FRAME,
    CMD_VEL_TOPIC,
    DASH_CTRL_TOPIC,
    JOY_TOPIC,
    LOOP_STATS_TOPIC,
    ODOM_FRAME,
    ODOM_TOPIC,
    SET_CHALLENGE_TOPIC,
    IMU_PITCH_TOPIC,
    IMU_DATA_TOPIC,
    IMU_CALIBRATE_TOPIC,
    RECORD_PLAYBACK_STATE_TOPIC,
    RECORD_PLAYBACK_CMD_TOPIC,
)

# --- Defaults ---
DEFAULT_SERVO_STEER_ID = 4
DEFAULT_SERVO_CENTER = 90
DEFAULT_SERVO_RANGE_LEFT = 50   # center - 50 = 40 (physical left)
DEFAULT_SERVO_RANGE_RIGHT = 50  # center + 50 = 140 (physical right)

CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]


class LoopMonitor:
    """Tracks loop timing statistics."""

    def __init__(self, loop_name: str, target_hz: float, overrun_ratio: float = 1.5) -> None:
        self.loop_name = loop_name
        self.target_hz = float(target_hz) if target_hz > 0 else 1.0
        self.expected_dt = 1.0 / self.target_hz
        self.overrun_dt = self.expected_dt * float(overrun_ratio)
        self.last_t = 0.0
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.sum_dt = 0.0
        self.max_dt = 0.0
        self.overruns = 0

    def tick(self) -> None:
        now = time.monotonic()
        if self.last_t > 0.0:
            dt = now - self.last_t
            self.count += 1
            self.sum_dt += dt
            if dt > self.max_dt:
                self.max_dt = dt
            if dt > self.overrun_dt:
                self.overruns += 1
        self.last_t = now

    def snapshot(self) -> Dict[str, float]:
        avg_dt = (self.sum_dt / self.count) if self.count > 0 else 0.0
        avg_hz = (1.0 / avg_dt) if avg_dt > 0 else 0.0
        data = {
            'loop': self.loop_name,
            'target_hz': round(self.target_hz, 3),
            'avg_hz': round(avg_hz, 3),
            'max_dt_ms': round(self.max_dt * 1000.0, 3),
            'overruns': int(self.overruns),
            'samples': int(self.count),
        }
        self.reset()
        return data

class ServoControllerV9(Node):
    """Interface between joystick, auto driver, and Rosmaster motor board."""

    def __init__(self):
        super().__init__('servo_controller')

        # --- Parameters ---
        self.declare_parameter('servo_steer_id', DEFAULT_SERVO_STEER_ID)
        self.declare_parameter('servo_center', DEFAULT_SERVO_CENTER)
        self.declare_parameter('servo_range_left', DEFAULT_SERVO_RANGE_LEFT)
        self.declare_parameter('servo_range_right', DEFAULT_SERVO_RANGE_RIGHT)
        self.declare_parameter('speed_levels', [15, 25, 40, 60, 100])
        self.declare_parameter('default_speed_index', 1)
        self.declare_parameter('joy_timeout', 0.8)
        self.declare_parameter('auto_cmd_timeout', 0.4)
        self.declare_parameter('unlock_requires_neutral', True)
        self.declare_parameter('unlock_neutral_threshold', 0.15)
        self.declare_parameter('hw_heartbeat_sec', 1.0)
        self.declare_parameter('hw_fail_limit', 5)
        self.declare_parameter('drive_motor_index', 0)  # which motor channel has the encoder (0=FL, 1=FR, 2=RL, 3=RR)
        self.declare_parameter('ticks_per_meter', 1050.0)
        self.declare_parameter('odom_distance_scale', 1.0)
        self.declare_parameter('odom_yaw_scale', 1.0)
        self.declare_parameter('encoder_jump_threshold', 800.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 6.0)
        self.declare_parameter('odom_reverse_polarity', False)
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('steering_max_deg', 50.0)
        self.declare_parameter('odom_vel_alpha', 0.3)
        self.declare_parameter('odom_velocity_deadband', 0.02)
        self.declare_parameter('publish_loop_stats', True)
        self.declare_parameter('odom_frame_id', ODOM_FRAME)
        self.declare_parameter('base_frame_id', BASE_FRAME)

        # IMU Software Calibration
        self.declare_parameter('imu_roll_offset', 0.0)
        self.declare_parameter('imu_pitch_offset', 0.0)
        self.declare_parameter('imu_yaw_offset', 0.0)
        self.declare_parameter('imu_roll_scale', 1.0)
        self.declare_parameter('imu_pitch_scale', 1.0)
        self.declare_parameter('imu_yaw_scale', 1.0)
        # Auto steering asymmetry correction
        # Multiplier applied ONLY to right-turn servo angle in auto mode.
        # Increase above 1.0 to make right turns sharper (compensates for Ackermann geometry).
        self.declare_parameter('auto_right_steer_boost', 1.3)
        
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.servo_steer_id = int(self._param_cache['servo_steer_id'])
        self.servo_center = int(self._param_cache['servo_center'])
        self.servo_range_left = int(self._param_cache['servo_range_left'])
        self.servo_range_right = int(self._param_cache['servo_range_right'])

        # Connect to Hardware
        self.bot = None
        try:
            self.bot = Rosmaster()
            self.bot.create_receive_threading()
            self.bot.set_auto_report_state(True)
            
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(self.servo_steer_id, self.servo_center)
            self.get_logger().info("✅ Rosmaster Connected (V9 Competition)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Rosmaster: {e}")
            exit(1)

        # Publishers
        self.dash_pub = self.create_publisher(String, DASH_CTRL_TOPIC, 10)
        self.auto_mode_pub = self.create_publisher(Bool, AUTO_MODE_TOPIC, 10)
        self.challenge_pub = self.create_publisher(String, SET_CHALLENGE_TOPIC, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.loop_stats_pub = self.create_publisher(String, LOOP_STATS_TOPIC, 10)
        self.pitch_pub = self.create_publisher(Float32, IMU_PITCH_TOPIC, 10)
        self.imu_data_pub = self.create_publisher(String, IMU_DATA_TOPIC, 10)
        self.rp_state_pub = self.create_publisher(String, RECORD_PLAYBACK_STATE_TOPIC, 10)

        # Subscribers
        self.create_subscription(Joy, JOY_TOPIC, self.joy_callback, 10)
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, self.cmd_vel_auto_callback, 10)
        self.create_subscription(String, RECORD_PLAYBACK_CMD_TOPIC, self._record_playback_cmd_cb, 10)
        self.create_subscription(String, IMU_CALIBRATE_TOPIC, self._imu_calibrate_cb, 10)

        # State
        self.manual_mode = True
        self.challenge_index = 0
        self.speed_levels = list(self._param_cache['speed_levels'])
        if not self.speed_levels:
            self.speed_levels = [25]
        default_idx = int(self._param_cache['default_speed_index'])
        self.speed_idx = max(0, min(default_idx, len(self.speed_levels) - 1))
        self.current_speed_limit = self.speed_levels[self.speed_idx]

        self.target_motor_val = 0
        self.target_servo_val = self.servo_center
        # Hardware continuous update loop (10 Hz)
        self.create_timer(0.1, self._hardware_update_loop)

        # Odometry Odometry properties
        # Ackermann kinematics parameters for RISA-Bot 
        # Ticks to meters needs calibration based on exact gear ratio and wheel diameter
        # Using a default scaling factor for now
        self.wheel_base = float(self._param_cache['wheel_base'])
        self.ticks_per_meter = float(self._param_cache['ticks_per_meter'])

        self.last_odom_time = time.monotonic()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self._filtered_linear = 0.0
        self._filtered_angular = 0.0
        self._encoder_glitch_count = 0
        self._last_glitch_warn_t = 0.0
        self.last_encoder_ticks = [0, 0, 0, 0] # FL, FR, RL, RR
        self.encoder_first_read = True
        self.encoder_loop_monitor = LoopMonitor('servo_encoder', 20.0)
        self.hw_loop_monitor = LoopMonitor('servo_hw', 10.0)

        # Fast encoder loop (20 Hz)
        self.create_timer(0.05, self._encoder_read_loop)

        # Debounce
        self.prev_buttons = [0] * 15
        self.prev_axes = [0.0] * 8

        # Safety: Joy watchdog & Ghost Input Protection
        self.joy_alive = False          # True once first /joy received
        self.joy_unlocked = False       # True once input changes from initial ghost state
        self.initial_joy_axes = None
        self.initial_joy_buttons = None
        
        self.last_joy_time = 0.0        # timestamp of last /joy message (monotonic)
        self.joy_timeout = float(self._param_cache['joy_timeout'])
        self.joy_lost_reported = False   # avoid spamming log
        self.create_timer(0.3, self._joy_watchdog)
        self.create_timer(1.0, self._publish_loop_stats)
        self.hw_error_count = 0
        self.hw_error_tripped = False
        self.awaiting_neutral = False
        self.last_auto_cmd_time = 0.0
        self.auto_cmd_stale_reported = False

        # --- IMU calibration ---
        self.imu_roll_offset = float(self._param_cache.get('imu_roll_offset', 0.0))
        self.imu_pitch_offset = float(self._param_cache.get('imu_pitch_offset', 0.0))
        self.imu_yaw_offset = float(self._param_cache.get('imu_yaw_offset', 0.0))
        self.imu_roll_scale = float(self._param_cache.get('imu_roll_scale', 1.0))
        self.imu_pitch_scale = float(self._param_cache.get('imu_pitch_scale', 1.0))
        self.imu_yaw_scale = float(self._param_cache.get('imu_yaw_scale', 1.0))
        
        self.raw_roll = 0.0
        self.raw_pitch = 0.0
        self.raw_yaw = 0.0
        self.imu_initialized = False
        # Stable (deadband-filtered) output values - suppresses micro-jitter
        self.stable_roll = 0.0
        self.stable_pitch = 0.0
        self.stable_yaw = 0.0
        self.IMU_DEADBAND_DEG = 0.5  # degrees - changes smaller than this are ignored

        # --- Record & Playback state ---
        self.rp_state = 'IDLE'       # 'IDLE', 'RECORDING', 'PLAYBACK'
        self.record_buffer = []       # list of {motor_pwm, servo_angle}
        self.record_last_sample_time = 0.0
        self.playback_index = 0
        self.playback_timer = None    # one-shot timer handle

        # Multi-slot recording management
        self.recordings_dir = os.path.expanduser('~/risabot_recordings')
        os.makedirs(self.recordings_dir, exist_ok=True)
        self.saved_recordings = {}     # {name: {name, sample_count, duration_sec, created_at}}
        self.current_recording_name = ''  # name of recording loaded in buffer
        self.active_parking_recording = ''  # recording used for parking sign trigger
        self._recording_names_sorted = []  # sorted list for joystick cycling
        self._cycling_index = -1  # current index when cycling with D-Pad

        # Migrate old single-file recording & scan saved recordings
        self._migrate_old_recording()
        self._scan_saved_recordings()
        self._load_active_parking_recording()

        self.get_logger().info("🎮 V9 Ready: Right Stick X = Steer | LB/RB = Challenges | A = Record/Stop | X = Play/Stop Playback")
        self._update_dash()

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'servo_steer_id': int(self.get_parameter('servo_steer_id').value),
            'servo_center': int(self.get_parameter('servo_center').value),
            'servo_range_left': int(self.get_parameter('servo_range_left').value),
            'servo_range_right': int(self.get_parameter('servo_range_right').value),
            'speed_levels': list(self.get_parameter('speed_levels').value),
            'default_speed_index': int(self.get_parameter('default_speed_index').value),
            'joy_timeout': float(self.get_parameter('joy_timeout').value),
            'auto_cmd_timeout': float(self.get_parameter('auto_cmd_timeout').value),
            'unlock_requires_neutral': bool(self.get_parameter('unlock_requires_neutral').value),
            'unlock_neutral_threshold': float(self.get_parameter('unlock_neutral_threshold').value),
            'hw_heartbeat_sec': float(self.get_parameter('hw_heartbeat_sec').value),
            'hw_fail_limit': int(self.get_parameter('hw_fail_limit').value),
            'drive_motor_index': int(self.get_parameter('drive_motor_index').value),
            'ticks_per_meter': float(self.get_parameter('ticks_per_meter').value),
            'odom_distance_scale': float(self.get_parameter('odom_distance_scale').value),
            'odom_yaw_scale': float(self.get_parameter('odom_yaw_scale').value),
            'encoder_jump_threshold': float(self.get_parameter('encoder_jump_threshold').value),
            'max_linear_velocity': float(self.get_parameter('max_linear_velocity').value),
            'max_angular_velocity': float(self.get_parameter('max_angular_velocity').value),
            'odom_reverse_polarity': bool(self.get_parameter('odom_reverse_polarity').value),
            'wheel_base': float(self.get_parameter('wheel_base').value),
            'steering_max_deg': float(self.get_parameter('steering_max_deg').value),
            'odom_vel_alpha': float(self.get_parameter('odom_vel_alpha').value),
            'odom_velocity_deadband': float(self.get_parameter('odom_velocity_deadband').value),
            'publish_loop_stats': bool(self.get_parameter('publish_loop_stats').value),
            'odom_frame_id': str(self.get_parameter('odom_frame_id').value),
            'base_frame_id': str(self.get_parameter('base_frame_id').value),
            'imu_roll_offset': float(self.get_parameter('imu_roll_offset').value),
            'imu_pitch_offset': float(self.get_parameter('imu_pitch_offset').value),
            'imu_yaw_offset': float(self.get_parameter('imu_yaw_offset').value),
            'imu_roll_scale': float(self.get_parameter('imu_roll_scale').value),
            'imu_pitch_scale': float(self.get_parameter('imu_pitch_scale').value),
            'imu_yaw_scale': float(self.get_parameter('imu_yaw_scale').value),
            'auto_right_steer_boost': float(self.get_parameter('auto_right_steer_boost').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                if p.name == 'joy_timeout':
                    self.joy_timeout = float(p.value)
                elif p.name == 'unlock_requires_neutral':
                    self.awaiting_neutral = bool(p.value)
                elif p.name == 'servo_steer_id':
                    self.servo_steer_id = int(p.value)
                elif p.name == 'servo_center':
                    self.servo_center = int(p.value)
                    self.target_servo_val = self.servo_center  # immediately apply new neutral
                elif p.name == 'servo_range_left':
                    self.servo_range_left = int(p.value)
                elif p.name == 'servo_range_right':
                    self.servo_range_right = int(p.value)
                elif p.name == 'speed_levels':
                    self.speed_levels = list(p.value)
                    if not self.speed_levels:
                        self.speed_levels = [25]
                    self.speed_idx = max(0, min(self.speed_idx, len(self.speed_levels) - 1))
                    self.current_speed_limit = self.speed_levels[self.speed_idx]
                elif p.name == 'ticks_per_meter':
                    self.ticks_per_meter = float(p.value)
                elif p.name == 'wheel_base':
                    self.wheel_base = float(p.value)
                elif p.name == 'imu_roll_offset':
                    self.imu_roll_offset = float(p.value)
                    self.get_logger().info(f'Parameter updated: imu_roll_offset = {self.imu_roll_offset}')
                elif p.name == 'imu_pitch_offset':
                    self.imu_pitch_offset = float(p.value)
                    self.get_logger().info(f'Parameter updated: imu_pitch_offset = {self.imu_pitch_offset}')
                elif p.name == 'imu_yaw_offset':
                    self.imu_yaw_offset = float(p.value)
                    self.get_logger().info(f'Parameter updated: imu_yaw_offset = {self.imu_yaw_offset}')
                elif p.name == 'imu_roll_scale': self.imu_roll_scale = float(p.value)
                elif p.name == 'imu_pitch_scale': self.imu_pitch_scale = float(p.value)
                elif p.name == 'imu_yaw_scale': self.imu_yaw_scale = float(p.value)
                # auto_right_steer_boost is read directly from _param_cache each call
        return SetParametersResult(successful=True)

    def _update_dash(self) -> None:
        """Publish controller state to dashboard (speed|index|state)."""
        state_str = CHALLENGE_STATES[self.challenge_index]
        msg = String()
        msg.data = f"{self.current_speed_limit}|{self.challenge_index}|{state_str}"
        self.dash_pub.publish(msg)

    def _publish_loop_stats(self) -> None:
        """Publish loop timing diagnostics."""
        if not bool(self._param_cache['publish_loop_stats']):
            return
        for monitor in (self.encoder_loop_monitor, self.hw_loop_monitor):
            payload = monitor.snapshot()
            payload['node'] = self.get_name()
            self.loop_stats_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def _joy_watchdog(self) -> None:
        """Stop robot if controller disconnects (no /joy for >joy_timeout)."""
        if not self.joy_alive:
            return  # Haven't received first joy yet, nothing to do
        elapsed = time.monotonic() - self.last_joy_time
        if elapsed > self.joy_timeout:
            if not self.joy_lost_reported:
                self.get_logger().warn(
                    f"⚠️ Controller lost (no /joy for {elapsed:.1f}s) — STOPPING ROBOT")
                self.joy_lost_reported = True
                self.joy_unlocked = False  # Lock to ignore phantom inputs on next receive
                # Force back to manual mode so auto_driver stops publishing
                if not self.manual_mode:
                    self.manual_mode = True
                    self.auto_mode_pub.publish(Bool(data=False))
                    self.get_logger().warn("⚠️ Forced MANUAL mode (controller lost)")
            # Always stop hardware (regardless of mode)
            self.stop_robot()
            # Publish zero cmd_vel to stop dashboard odometry
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

    def joy_callback(self, msg: Joy) -> None:
        """Handle joystick input for mode toggle, driving, and state cycling."""
        # Mark controller as alive
        self.last_joy_time = time.monotonic()
        
        if not self.joy_alive:
            self.joy_alive = True
            self.initial_joy_axes = list(msg.axes)
            self.initial_joy_buttons = list(msg.buttons)
            self.get_logger().info("🎮 Controller receiver connected (Awaiting sync...)")
            
        if self.joy_lost_reported:
            self.get_logger().info("🎮 Controller module synced. Press any BUTTON to unlock...")
            self.joy_lost_reported = False

        # Ghost state protection: require an explicit BUTTON PRESS to unlock.
        # Axis changes alone are not safe because controllers often have drift
        # or non-zero default axis values (e.g. axis 1 = 1.0) on power-on.
        if not self.joy_unlocked:
            any_button_pressed = any(b == 1 for b in msg.buttons)
            if any_button_pressed:
                self.joy_unlocked = True
                self.awaiting_neutral = bool(self._param_cache['unlock_requires_neutral'])
                self.get_logger().info("Controller unlocked (Button press detected)")
                # Consume unlock frame: do not process toggles/driving in same packet.
                self.prev_buttons = list(msg.buttons)
                self.prev_axes = list(msg.axes)
                self.stop_robot()
                return
            else:
                return  # Return early, ignore all input until a button is pressed

        if self.awaiting_neutral:
            neutral_thr = float(self._param_cache['unlock_neutral_threshold'])
            raw_throttle = msg.axes[1] if 1 < len(msg.axes) else 0.0
            raw_steer = msg.axes[2] if 2 < len(msg.axes) else 0.0
            if abs(raw_throttle) <= neutral_thr and abs(raw_steer) <= neutral_thr:
                self.awaiting_neutral = False
                self.get_logger().info('Controller neutral detected, manual drive enabled')
            else:
                self.stop_robot()
                self.prev_buttons = list(msg.buttons)
                self.prev_axes = list(msg.axes)
                return

        # Helpers
        def btn(idx): return msg.buttons[idx] if idx < len(msg.buttons) else 0
        def axis(idx): return msg.axes[idx] if idx < len(msg.axes) else 0.0
        def rose(idx): # Rising edge
            return btn(idx) == 1 and (idx >= len(self.prev_buttons) or self.prev_buttons[idx] == 0)

        # 0. Record / Stop Record toggle (Button A = index 0)
        if rose(0):
            if self.rp_state == 'IDLE':
                self._start_recording()
            elif self.rp_state == 'RECORDING':
                self._stop_recording()

        # Play / Stop Playback toggle (Button X = index 3)
        if rose(3):
            if self.rp_state == 'IDLE':
                if len(self.record_buffer) > 0 and self.manual_mode:
                    self._start_playback()
            elif self.rp_state == 'PLAYBACK':
                self._stop_playback()

        # Save movement with auto-generated name (Button B = index 1)
        if rose(1):
            if self.rp_state == 'IDLE' and len(self.record_buffer) > 0:
                import datetime
                auto_name = datetime.datetime.now().strftime('rec_%Y%m%d_%H%M%S')
                self._save_recording(auto_name)

        # D-Pad Left/Right to cycle through saved recordings (only in IDLE + manual)
        dpad_x = axis(6)
        prev_dpad_x = self.prev_axes[6] if 6 < len(self.prev_axes) else 0.0
        if self.rp_state == 'IDLE' and self.manual_mode and len(self._recording_names_sorted) > 0:
            if dpad_x > 0.5 and prev_dpad_x <= 0.5:  # D-Pad Right
                self._cycle_recording(1)
            elif dpad_x < -0.5 and prev_dpad_x >= -0.5:  # D-Pad Left
                self._cycle_recording(-1)

        # 1. Toggle Mode (Start=11, Y=4)
        if rose(11) or rose(4): 
            # Abort any active record/playback when switching modes
            if self.rp_state != 'IDLE':
                if self.rp_state == 'RECORDING':
                    self._stop_recording()
                elif self.rp_state == 'PLAYBACK':
                    self._stop_playback()
            self.manual_mode = not self.manual_mode
            self.auto_mode_pub.publish(Bool(data=not self.manual_mode))
            self.get_logger().info(f"Mode: {'MANUAL' if self.manual_mode else 'AUTO'}")
            if self.manual_mode:
                self.stop_robot()
            else:
                self.last_auto_cmd_time = time.monotonic()
                self.auto_cmd_stale_reported = False
            self._update_dash()

        # 2. Challenge Cycling (LB=6, RB=7)
        if rose(7): # RB -> Next
            self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
            self.challenge_pub.publish(String(data=CHALLENGE_STATES[self.challenge_index]))
            self.get_logger().info(f"RB Pressed -> Challenge: {CHALLENGE_STATES[self.challenge_index]}")
            self._update_dash()
        
        if rose(6): # LB -> Prev
            self.challenge_index = (self.challenge_index - 1) % len(CHALLENGE_STATES)
            self.challenge_pub.publish(String(data=CHALLENGE_STATES[self.challenge_index]))
            self.get_logger().info(f"LB Pressed -> Challenge: {CHALLENGE_STATES[self.challenge_index]}")
            self._update_dash()

        # 3. Speed Control (D-Pad UP/DOWN - axis 7)
        dpad = axis(7)
        prev_dpad = self.prev_axes[7] if 7 < len(self.prev_axes) else 0.0
        if dpad > 0.5 and prev_dpad <= 0.5:
            self.speed_idx = min(len(self.speed_levels)-1, self.speed_idx + 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()
        elif dpad < -0.5 and prev_dpad >= -0.5:
            self.speed_idx = max(0, self.speed_idx - 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()

        # 4. MANUAL DRIVING (skip during playback — robot is auto-replaying)
        if self.manual_mode and self.joy_alive and self.rp_state != 'PLAYBACK':
            # Throttle: Left Stick Y (Axis 1)
            throttle_raw = axis(1)
            
            # Steering: Right Stick X (Axis 2 — Left/Right)
            # Ghost state protection handles startup noise from triggers
            steer_raw = axis(2)
            
            # Deadzone
            if abs(throttle_raw) < 0.1: throttle_raw = 0.0
            if abs(steer_raw) < 0.1: steer_raw = 0.0

            # Drive (PWM)
            if self.rp_state == 'RECORDING':
                if throttle_raw > 0.0:
                    motor_pwm = int(self.current_speed_limit * 2.55)
                elif throttle_raw < 0.0:
                    motor_pwm = -int(self.current_speed_limit * 2.55)
                else:
                    motor_pwm = 0
            else:
                motor_pwm = int(throttle_raw * self.current_speed_limit * 2.55)
            
            # Steer (Servo 4) — asymmetric left/right ranges
            # steer_raw > 0 (joystick left) → servo angle decreases → uses range_left
            # steer_raw < 0 (joystick right) → servo angle increases → uses range_right
            if self.rp_state == 'RECORDING':
                if steer_raw > 0.0:
                    steer_angle = self.servo_center - self.servo_range_left
                elif steer_raw < 0.0:
                    steer_angle = self.servo_center + self.servo_range_right
                else:
                    steer_angle = self.servo_center
            else:
                if steer_raw >= 0:
                    steer_angle = int(self.servo_center - (steer_raw * self.servo_range_left))
                else:
                    steer_angle = int(self.servo_center - (steer_raw * self.servo_range_right))
                min_angle = self.servo_center - self.servo_range_left
                max_angle = self.servo_center + self.servo_range_right
                steer_angle = max(min_angle, min(max_angle, steer_angle))

            self.apply_hardware(motor_pwm, steer_angle)

            # Publish to /cmd_vel so dashboard can track manual velocity for odom
            cmd = Twist()
            cmd.linear.x = throttle_raw * self.current_speed_limit / 100.0  # approximate m/s
            cmd.angular.z = steer_raw  # normalized steering
            self.cmd_vel_pub.publish(cmd)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

    def cmd_vel_auto_callback(self, msg: Twist) -> None:
        """Forward auto commands to hardware when in auto mode."""
        self.last_auto_cmd_time = time.monotonic()
        self.auto_cmd_stale_reported = False
        if not self.manual_mode and self.rp_state != 'PLAYBACK':
            self.process_twist(msg)

    def process_twist(self, msg: Twist) -> None:
        """Convert Twist message to hardware PWM + servo angle."""
        # Auto Mode Driving
        pwm_val = int(msg.linear.x * 255.0)
        
        # Steering — asymmetric left/right ranges with Ackermann correction boost
        # angular_z > 0 → servo increases → physical right → uses range_right + right boost
        # angular_z < 0 → servo decreases → physical left  → uses range_left
        if msg.angular.z >= 0:
            right_boost = float(self._param_cache.get('auto_right_steer_boost', 1.0))
            angle_offset = msg.angular.z * float(self.servo_range_right) * right_boost
        else:
            angle_offset = msg.angular.z * float(self.servo_range_left)
        steer_angle = int(self.servo_center + angle_offset)
        
        # Clamp to asymmetric limits
        pwm_val = max(-255, min(255, pwm_val))
        min_angle = self.servo_center - self.servo_range_left
        max_angle = self.servo_center + self.servo_range_right
        steer_angle = max(min_angle, min(max_angle, steer_angle))
        
        self.apply_hardware(pwm_val, steer_angle)
        
        # Also publish to /cmd_vel so dashboard can track velocity/odometry
        self.cmd_vel_pub.publish(msg)

    def apply_hardware(self, motor_pwm: int, steer_angle: int) -> None:
        """Update target hardware state and write immediately to hardware on change."""
        self.target_motor_val = motor_pwm
        self.target_servo_val = steer_angle
        
        # Write immediately if changed to minimize input latency
        if (self.target_motor_val != getattr(self, 'sent_motor_val', None) or 
            self.target_servo_val != getattr(self, 'sent_servo_val', None)):
            try:
                self.bot.set_motor(self.target_motor_val, 0, 0, 0)
                self.bot.set_pwm_servo(self.servo_steer_id, self.target_servo_val)
                self.sent_motor_val = self.target_motor_val
                self.sent_servo_val = self.target_servo_val
                self.last_hw_send_time = time.monotonic()
                self.hw_error_count = 0
                self.hw_error_tripped = False
            except Exception as e:
                self.hw_error_count += 1
                if self.hw_error_count >= int(self._param_cache['hw_fail_limit']) and not self.hw_error_tripped:
                    self.hw_error_tripped = True
                    self.stop_robot()

    def _hardware_update_loop(self) -> None:
        """Send 10Hz heartbeat to Rosmaster, but only update on change or 1-second timeout to prevent serial spam."""
        self.hw_loop_monitor.tick()
        now = time.monotonic()
        if not self.manual_mode:
            timeout = float(self._param_cache['auto_cmd_timeout'])
            auto_age = now - self.last_auto_cmd_time if self.last_auto_cmd_time > 0.0 else float('inf')
            if auto_age > timeout:
                if not self.auto_cmd_stale_reported:
                    self.get_logger().warn(
                        f'Auto command stream stale for {auto_age:.2f}s (> {timeout:.2f}s), forcing MANUAL stop'
                    )
                    self.auto_cmd_stale_reported = True
                self.stop_robot()
                self.manual_mode = True
                self.auto_mode_pub.publish(Bool(data=False))
                self._update_dash()
                return

        heartbeat = float(self._param_cache['hw_heartbeat_sec'])
        # If values changed, OR every 1 second (safety heartbeat)
        if (self.target_motor_val != getattr(self, 'sent_motor_val', None) or 
            self.target_servo_val != getattr(self, 'sent_servo_val', None) or
            now - getattr(self, 'last_hw_send_time', 0.0) > heartbeat):
            
            try:
                self.bot.set_motor(self.target_motor_val, 0, 0, 0)
                self.bot.set_pwm_servo(self.servo_steer_id, self.target_servo_val)
                self.sent_motor_val = self.target_motor_val
                self.sent_servo_val = self.target_servo_val
                self.last_hw_send_time = now
                self.hw_error_count = 0
                self.hw_error_tripped = False
            except Exception as e:
                self.hw_error_count += 1
                self.get_logger().warn(f'Hardware send failed: {e}')
                if self.hw_error_count >= int(self._param_cache['hw_fail_limit']) and not self.hw_error_tripped:
                    self.hw_error_tripped = True
                    self.get_logger().error('Hardware send failure limit reached, stopping robot')
                    self.stop_robot()
                    if not self.manual_mode:
                        self.manual_mode = True
                        self.auto_mode_pub.publish(Bool(data=False))

    def stop_robot(self) -> None:
        """Helper to send zero velocity to motors."""
        try:
            self.bot.set_motor(0, 0, 0, 0)
        except Exception:
            pass

    def _normalize_angle(self, angle: float) -> float:
        """Normalize an angle to [-180, 180]."""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Compute shortest signed angular difference (a - b) in degrees, wrapped to [-180, 180]."""
        d = a - b
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return d

    def _ema_angle(self, current_ema: float, new_angle: float, alpha: float) -> float:
        """Calculate EMA for angles without breaking at 180/-180 wrap-arounds."""
        import math
        # Convert to radians
        rad_new = math.radians(new_angle)
        rad_cur = math.radians(current_ema)
        # Average vectors
        sin_avg = alpha * math.sin(rad_new) + (1.0 - alpha) * math.sin(rad_cur)
        cos_avg = alpha * math.cos(rad_new) + (1.0 - alpha) * math.cos(rad_cur)
        # Re-convert to degrees
        return math.degrees(math.atan2(sin_avg, cos_avg))

    def _encoder_read_loop(self) -> None:
        """Read hardware encoders and compute/publish /odom"""
        self.encoder_loop_monitor.tick()

        # Steady 20Hz recording loop
        if self.rp_state == 'RECORDING':
            self.record_buffer.append({
                'motor_pwm': self.target_motor_val,
                'servo_angle': self.target_servo_val
            })

        now = time.monotonic()
        dt = now - self.last_odom_time
        
        # Don't divide by zero if timer fires too fast
        if dt <= 0.001 or dt > 0.3:
            self.last_odom_time = now
            return

        # Read IMU Pitch + full RPY
        try:
            r, p, y = self.bot.get_imu_attitude_data()
            
            # Use raw values directly (hardware is already filtered)
            r = float(r)
            p = float(p)
            y = float(y)
            
            if not self.imu_initialized:
                self.raw_roll = r
                self.raw_pitch = p
                self.raw_yaw = y
                self.imu_initialized = True
            else:
                alpha = 0.15  # strong filter for micro-changes
                self.raw_roll = self._ema_angle(self.raw_roll, r, alpha)
                self.raw_pitch = self._ema_angle(self.raw_pitch, p, alpha)
                self.raw_yaw = self._ema_angle(self.raw_yaw, y, alpha)
            
            # Apply Software Calibration and normalize using angle-aware difference for all axes
            cal_roll = self._normalize_angle(self._angle_diff(self.raw_roll, self.imu_roll_offset) * self.imu_roll_scale)
            # Negate pitch so nose-up = positive (hardware reports inverted)
            cal_pitch = -self._normalize_angle(self._angle_diff(self.raw_pitch, self.imu_pitch_offset) * self.imu_pitch_scale)
            cal_yaw = self._normalize_angle(self._angle_diff(self.raw_yaw, self.imu_yaw_offset) * self.imu_yaw_scale)

            # Snap to nearest integer if within 0.25 degrees to hide small sensor noise (0.01 - 0.15)
            # This makes the UI feel completely stable when stationary at 0 or 90 degrees.
            def _snap(val, thresh=0.25):
                nearest = round(val)
                if abs(val - nearest) < thresh:
                    return float(nearest)
                return val

            self.stable_roll = _snap(cal_roll)
            self.stable_pitch = _snap(cal_pitch)
            self.stable_yaw = _snap(cal_yaw)

            pitch_msg = Float32()
            pitch_msg.data = self.stable_pitch
            self.pitch_pub.publish(pitch_msg)
            # Publish full RPY as JSON for dashboard
            rpy_payload = json.dumps(
                {'roll': round(self.stable_roll, 2),
                 'pitch': round(self.stable_pitch, 2),
                 'yaw': round(self.stable_yaw, 2)},
                separators=(',', ':')
            )
            self.imu_data_pub.publish(String(data=rpy_payload))
        except Exception as e:
            self.get_logger().error(f"Failed to read/publish IMU pitch: {e}")

        try:
            # Reads 4 motors: FL, FR, RL, RR 
            ticks = self.bot.get_motor_encoder()
            
            # First initialization
            if self.encoder_first_read or ticks is None or len(ticks) < 4:
                if ticks and len(ticks) == 4:
                    self.last_encoder_ticks = list(ticks)
                    self.encoder_first_read = False
                self.last_odom_time = now
                return
            
            # Calculate delta ticks
            d_ticks = [
                ticks[0] - self.last_encoder_ticks[0],
                ticks[1] - self.last_encoder_ticks[1],
                ticks[2] - self.last_encoder_ticks[2],
                ticks[3] - self.last_encoder_ticks[3]
            ]
            self.last_encoder_ticks = list(ticks)
            self.last_odom_time = now
            jump_thresh = float(self._param_cache['encoder_jump_threshold'])
            valid_d_ticks = []
            for d in d_ticks:
                if abs(d) > jump_thresh:
                    valid_d_ticks.append(0.0)
                    self._encoder_glitch_count += 1
                else:
                    valid_d_ticks.append(float(d))
            if self._encoder_glitch_count > 0 and now - self._last_glitch_warn_t > 1.0:
                self.get_logger().warn(f'Encoder jump filtered, count={self._encoder_glitch_count}')
                self._last_glitch_warn_t = now
                self._encoder_glitch_count = 0

            # RISA-Bot uses a single rear-drive motor (motor index 0).
            # Only use that motor's encoder — the other 3 channels are undriven
            # and return noise/garbage that corrupts the odometry.
            motor_idx = int(self._param_cache['drive_motor_index'])
            avg_ticks = valid_d_ticks[motor_idx]
            if bool(self._param_cache['odom_reverse_polarity']):
                avg_ticks = -avg_ticks

            # Convert to distance
            if self.ticks_per_meter <= 0:
                return
            distance = (avg_ticks / self.ticks_per_meter) * float(self._param_cache['odom_distance_scale'])
            linear_velocity = distance / dt
            max_lin = float(self._param_cache['max_linear_velocity'])
            if abs(linear_velocity) > max_lin:
                linear_velocity = max(-max_lin, min(max_lin, linear_velocity))
                distance = linear_velocity * dt
            vel_deadband = float(self._param_cache['odom_velocity_deadband'])
            if abs(linear_velocity) < vel_deadband:
                linear_velocity = 0.0
                distance = 0.0

            # Calculate steering angle from servo command (for Ackermann kinematics)
            # Use asymmetric left/right ranges based on which side of center
            servo_delta = self.servo_center - self.target_servo_val
            if servo_delta >= 0:
                effective_range = float(self.servo_range_left)
            else:
                effective_range = float(self.servo_range_right)
            if effective_range < 1.0:
                effective_range = 1.0
            steer_norm = servo_delta / effective_range
            steer_norm = max(-1.0, min(1.0, steer_norm))
            steering_angle_deg = steer_norm * float(self._param_cache['steering_max_deg'])
            steering_angle_rad = math.radians(steering_angle_deg)
            
            # Ackermann kinematics: w = v / R, where R = L / tan(delta)
            # L is wheel base, delta is steering angle
            if abs(steering_angle_rad) > 0.01:
                turning_radius = self.wheel_base / math.tan(steering_angle_rad)
                angular_velocity = linear_velocity / turning_radius
            else:
                angular_velocity = 0.0
            angular_velocity *= float(self._param_cache['odom_yaw_scale'])
            max_ang = float(self._param_cache['max_angular_velocity'])
            angular_velocity = max(-max_ang, min(max_ang, angular_velocity))

            alpha = float(self._param_cache['odom_vel_alpha'])
            self._filtered_linear = alpha * linear_velocity + (1 - alpha) * self._filtered_linear
            self._filtered_angular = alpha * angular_velocity + (1 - alpha) * self._filtered_angular

            yaw_delta = self._filtered_angular * dt

            # Update pose
            self.odom_yaw += yaw_delta
            self.odom_yaw = math.atan2(math.sin(self.odom_yaw), math.cos(self.odom_yaw))
            self.odom_x += self._filtered_linear * math.cos(self.odom_yaw) * dt
            self.odom_y += self._filtered_linear * math.sin(self.odom_yaw) * dt

            # Build and publish Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = str(self._param_cache['odom_frame_id'])
            odom.child_frame_id = str(self._param_cache['base_frame_id'])

            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            
            # Quaternion from yaw
            odom.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)

            odom.twist.twist.linear.x = self._filtered_linear
            odom.twist.twist.angular.z = self._filtered_angular

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Failed to read encoders: {e}")

    # ─────────── Record & Playback methods ───────────

    def _imu_calibrate_cb(self, msg: String) -> None:
        """Handle JSON calibration commands (zeroing, scaling, or hardware trigger)."""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')
            
            if action == 'zero':
                self.get_logger().info(f'IMU Zeroing: raw offsets R:{self.raw_roll:.2f}, P:{self.raw_pitch:.2f}, Y:{self.raw_yaw:.2f}')
                
                # Force update local variables immediately to bypass any param server async quirks
                self.imu_roll_offset = float(self.raw_roll)
                self.imu_pitch_offset = float(self.raw_pitch)
                self.imu_yaw_offset = float(self.raw_yaw)
                
                # Force reset stable output so it doesn't get stuck by the deadband
                self.stable_roll = 0.0
                self.stable_pitch = 0.0
                self.stable_yaw = 0.0
                
                # Reset the EMA accumulators to the new offsets so yaw zeroing
                # takes effect immediately (prevents stale EMA drift from
                # keeping the old angle alive after calibration)
                self.raw_roll = float(self.raw_roll)   # keep current
                self.raw_pitch = float(self.raw_pitch) # keep current
                self.raw_yaw = float(self.raw_yaw)     # keep current
                
                self.get_logger().info(
                    f'IMU Zeroed: offsets set to R:{self.imu_roll_offset:.2f}, '
                    f'P:{self.imu_pitch_offset:.2f}, Y:{self.imu_yaw_offset:.2f}')
                
                # Store raw values as new offsets in parameter server
                import rclpy
                results = self.set_parameters([
                    rclpy.Parameter('imu_roll_offset', rclpy.Parameter.Type.DOUBLE, self.imu_roll_offset),
                    rclpy.Parameter('imu_pitch_offset', rclpy.Parameter.Type.DOUBLE, self.imu_pitch_offset),
                    rclpy.Parameter('imu_yaw_offset', rclpy.Parameter.Type.DOUBLE, self.imu_yaw_offset),
                ])
                for r in results:
                    if not r.successful:
                        self.get_logger().error(f'Failed to set parameter: {r.reason}')
                
            elif action == 'set_scale':
                axis = cmd.get('axis', '')
                target = float(cmd.get('target', 90.0))
                
                param_name = ''
                new_scale = 1.0
                
                if axis == 'pitch':
                    param_name = 'imu_pitch_scale'
                    rel_val = self._angle_diff(self.raw_pitch, self.imu_pitch_offset)
                    if abs(rel_val) > 1.0: # avoid div by zero
                        # Pitch output is negated (cal_pitch = -(rel_val * scale)),
                        # so use -target to compensate for the negation
                        new_scale = -target / rel_val
                elif axis == 'roll':
                    param_name = 'imu_roll_scale'
                    rel_val = self._angle_diff(self.raw_roll, self.imu_roll_offset)
                    if abs(rel_val) > 1.0:
                        new_scale = target / rel_val
                elif axis == 'yaw':
                    # Take the current angle as the deadzone or 0 degree (offset = raw_yaw)
                    self.imu_yaw_offset = float(self.raw_yaw)
                    self.raw_yaw = float(self.raw_yaw)
                    self.get_logger().info(f'IMU Yaw Calibrated: offset set to current angle {self.imu_yaw_offset:.2f}°')
                    import rclpy
                    results = self.set_parameters([
                        rclpy.Parameter('imu_yaw_offset', rclpy.Parameter.Type.DOUBLE, self.imu_yaw_offset)
                    ])
                    for r in results:
                        if not r.successful:
                            self.get_logger().error(f'Failed to set imu_yaw_offset: {r.reason}')
                
                if param_name:
                    self.get_logger().info(f'IMU Scale Update: {axis} target={target}, raw-offset={rel_val:.2f}, new_scale={new_scale:.3f}')
                    import rclpy
                    results = self.set_parameters([
                        rclpy.Parameter(param_name, rclpy.Parameter.Type.DOUBLE, float(new_scale))
                    ])
                    for r in results:
                        if not r.successful:
                            self.get_logger().error(f'Failed to set parameter: {r.reason}')

        except Exception as e:
            self.get_logger().error(f'IMU calibration failed: {e}')

    def _migrate_old_recording(self) -> None:
        """Migrate legacy ~/recorded_movement.json to ~/risabot_recordings/default.json."""
        old_path = os.path.expanduser('~/recorded_movement.json')
        new_path = os.path.join(self.recordings_dir, 'default.json')
        if os.path.exists(old_path) and not os.path.exists(new_path):
            try:
                with open(old_path, 'r') as f:
                    samples = json.load(f)
                recording_data = {
                    'name': 'default',
                    'sample_count': len(samples),
                    'duration_sec': round(len(samples) * 0.05, 2),
                    'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'samples': samples,
                }
                with open(new_path, 'w') as f:
                    json.dump(recording_data, f)
                self.get_logger().info(f'Migrated old recording ({len(samples)} samples) to {new_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to migrate old recording: {e}')

    def _scan_saved_recordings(self) -> None:
        """Scan ~/risabot_recordings/ and build metadata index."""
        self.saved_recordings = {}
        try:
            for fname in os.listdir(self.recordings_dir):
                if not fname.endswith('.json'):
                    continue
                fpath = os.path.join(self.recordings_dir, fname)
                try:
                    with open(fpath, 'r') as f:
                        data = json.load(f)
                    name = data.get('name', fname.replace('.json', ''))
                    self.saved_recordings[name] = {
                        'name': name,
                        'sample_count': data.get('sample_count', len(data.get('samples', []))),
                        'duration_sec': data.get('duration_sec', 0),
                        'created_at': data.get('created_at', ''),
                    }
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'Failed to scan recordings: {e}')
        self._recording_names_sorted = sorted(self.saved_recordings.keys())
        self.get_logger().info(f'Found {len(self.saved_recordings)} saved recordings: {self._recording_names_sorted}')

    def _load_active_parking_recording(self) -> None:
        """Load the active parking recording into the playback buffer."""
        # Check for a saved preference
        pref_path = os.path.join(self.recordings_dir, '.active_parking')
        if os.path.exists(pref_path):
            try:
                with open(pref_path, 'r') as f:
                    name = f.read().strip()
                if name and name in self.saved_recordings:
                    self.active_parking_recording = name
                    self._load_recording_by_name(name)
                    return
            except Exception:
                pass
        # Fallback: load the first available recording
        if self._recording_names_sorted:
            name = self._recording_names_sorted[0]
            self.active_parking_recording = name
            self._load_recording_by_name(name)

    def _load_recording_by_name(self, name: str) -> bool:
        """Load a named recording from disk into the playback buffer."""
        fpath = os.path.join(self.recordings_dir, f'{name}.json')
        if not os.path.exists(fpath):
            self.get_logger().warn(f'Recording not found: {name}')
            return False
        try:
            with open(fpath, 'r') as f:
                data = json.load(f)
            self.record_buffer = data.get('samples', [])
            self.current_recording_name = name
            self._cycling_index = self._recording_names_sorted.index(name) if name in self._recording_names_sorted else -1
            self.get_logger().info(f'Loaded recording "{name}" ({len(self.record_buffer)} samples)')
            self._publish_rp_state()
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load recording "{name}": {e}')
            return False

    def _save_recording(self, name: str = '') -> None:
        """Save current record buffer as a named recording to disk."""
        if not name:
            name = time.strftime('rec_%Y%m%d_%H%M%S')
        # Sanitize name (alphanumeric, underscore, dash only)
        safe_name = ''.join(c for c in name if c.isalnum() or c in ('_', '-'))
        if not safe_name:
            safe_name = 'unnamed'
        fpath = os.path.join(self.recordings_dir, f'{safe_name}.json')
        try:
            recording_data = {
                'name': safe_name,
                'sample_count': len(self.record_buffer),
                'duration_sec': round(len(self.record_buffer) * 0.05, 2),
                'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'samples': self.record_buffer,
            }
            with open(fpath, 'w') as f:
                json.dump(recording_data, f)
            self.current_recording_name = safe_name
            self.get_logger().info(f'💾 Saved recording "{safe_name}" ({len(self.record_buffer)} samples) to {fpath}')
            self._scan_saved_recordings()  # refresh index
            self._publish_rp_state()
        except Exception as e:
            self.get_logger().error(f'Failed to save recording "{safe_name}": {e}')

    def _delete_recording(self, name: str) -> None:
        """Delete a named recording from disk."""
        fpath = os.path.join(self.recordings_dir, f'{name}.json')
        if os.path.exists(fpath):
            try:
                os.remove(fpath)
                self.get_logger().info(f'🗑 Deleted recording "{name}"')
                if self.current_recording_name == name:
                    self.record_buffer = []
                    self.current_recording_name = ''
                if self.active_parking_recording == name:
                    self.active_parking_recording = ''
                self._scan_saved_recordings()
                self._publish_rp_state()
            except Exception as e:
                self.get_logger().error(f'Failed to delete recording "{name}": {e}')
        else:
            self.get_logger().warn(f'Recording not found for delete: {name}')

    def _set_active_parking(self, name: str) -> None:
        """Set a recording as the active parking recording."""
        if name not in self.saved_recordings:
            self.get_logger().warn(f'Cannot set active parking: "{name}" not found')
            return
        self.active_parking_recording = name
        # Persist the preference
        pref_path = os.path.join(self.recordings_dir, '.active_parking')
        try:
            with open(pref_path, 'w') as f:
                f.write(name)
        except Exception:
            pass
        # Also load it into the buffer so it's ready for playback
        self._load_recording_by_name(name)
        self.get_logger().info(f'🅿 Active parking recording set to "{name}"')
        self._publish_rp_state()

    def _cycle_recording(self, direction: int) -> None:
        """Cycle through saved recordings with D-Pad Left/Right."""
        if not self._recording_names_sorted:
            return
        self._cycling_index = (self._cycling_index + direction) % len(self._recording_names_sorted)
        name = self._recording_names_sorted[self._cycling_index]
        self._load_recording_by_name(name)
        self.get_logger().info(f'📂 Cycling: [{self._cycling_index + 1}/{len(self._recording_names_sorted)}] "{name}"')

    def _publish_rp_state(self) -> None:
        """Publish current record/playback state with recording metadata."""
        payload = json.dumps({
            'state': self.rp_state,
            'buffer_size': len(self.record_buffer),
            'playback_index': self.playback_index,
            'recording_name': self.current_recording_name,
            'active_parking_recording': self.active_parking_recording,
            'saved_recordings': list(self.saved_recordings.values()),
        }, separators=(',', ':'))
        self.rp_state_pub.publish(String(data=payload))

    def _record_playback_cmd_cb(self, msg: String) -> None:
        """Handle record/playback commands from the dashboard."""
        cmd = msg.data.strip()
        cmd_lower = cmd.lower()

        if cmd_lower == 'record':
            if self.rp_state == 'IDLE':
                self._start_recording()
        elif cmd_lower == 'stop':
            if self.rp_state == 'RECORDING':
                self._stop_recording()
            elif self.rp_state == 'PLAYBACK':
                self._stop_playback()
        elif cmd_lower == 'playback':
            if self.rp_state == 'IDLE' and len(self.record_buffer) > 0:
                self._start_playback()
        elif cmd_lower == 'list':
            self._scan_saved_recordings()
        elif cmd_lower.startswith('save:'):
            name = cmd[5:].strip()
            if self.rp_state == 'IDLE' and len(self.record_buffer) > 0:
                self._save_recording(name)
        elif cmd_lower == 'save':
            if self.rp_state == 'IDLE' and len(self.record_buffer) > 0:
                self._save_recording()
        elif cmd_lower.startswith('load:'):
            name = cmd[5:].strip()
            if self.rp_state == 'IDLE':
                self._load_recording_by_name(name)
        elif cmd_lower.startswith('delete:'):
            name = cmd[7:].strip()
            if self.rp_state == 'IDLE':
                self._delete_recording(name)
        elif cmd_lower.startswith('set_active:'):
            name = cmd[11:].strip()
            self._set_active_parking(name)
        self._publish_rp_state()

    def _start_recording(self) -> None:
        """Enter RECORDING state: clear buffer, begin sampling."""
        self.record_buffer = []
        self.current_recording_name = ''
        self.rp_state = 'RECORDING'
        self.get_logger().info(f'🔴 RECORDING started (buffer cleared)')
        self._publish_rp_state()

    def _stop_recording(self) -> None:
        """Exit RECORDING state: keep buffer, return to IDLE."""
        self.rp_state = 'IDLE'
        self.get_logger().info(f'⏹ RECORDING stopped ({len(self.record_buffer)} samples)')
        self._publish_rp_state()

    def _start_playback(self) -> None:
        """Enter PLAYBACK state: begin replaying the recorded buffer."""
        if len(self.record_buffer) == 0:
            self.get_logger().warn('▶ Cannot start playback: buffer is empty')
            return
        self.rp_state = 'PLAYBACK'
        self.playback_index = 0
        self.get_logger().info(f'▶ PLAYBACK started "{self.current_recording_name}" ({len(self.record_buffer)} samples)')
        self._publish_rp_state()
        # Start the periodic playback timer at 20Hz (0.05 seconds)
        self.playback_timer = self.create_timer(0.05, self._playback_timer_cb)
        # Kick off the first step immediately
        self._playback_step()

    def _playback_step(self) -> None:
        """Apply the next recorded command."""
        # Safety: abort if state changed externally
        if self.rp_state != 'PLAYBACK':
            return

        if self.playback_index >= len(self.record_buffer):
            # Reached end of recording
            self.get_logger().info('🏁 PLAYBACK complete — stopping robot')
            self._stop_playback()
            return

        sample = self.record_buffer[self.playback_index]
        self.apply_hardware(sample['motor_pwm'], sample['servo_angle'])

        self.playback_index += 1
        self._publish_rp_state()

    def _playback_timer_cb(self) -> None:
        """Periodic timer callback for playback stepping."""
        self._playback_step()

    def _stop_playback(self) -> None:
        """Exit PLAYBACK state: stop robot, cancel timer, return to IDLE."""
        if self.playback_timer is not None:
            self.playback_timer.cancel()
            self.destroy_timer(self.playback_timer)
            self.playback_timer = None
        self.rp_state = 'IDLE'
        self.stop_robot()
        self.get_logger().info('⏹ PLAYBACK stopped')
        self._publish_rp_state()

    def stop_robot(self) -> None:
        self.target_motor_val = 0
        self.target_servo_val = self.servo_center
        try:
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(self.servo_steer_id, self.servo_center)
        except Exception:
            pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoControllerV9()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        try:
            del node.bot
            print("Rosmaster closed")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
