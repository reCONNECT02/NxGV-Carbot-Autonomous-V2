#!/usr/bin/env python3
"""
Robot Dashboard — Web-based status monitor
Subscribes to all key ROS topics and serves a live web dashboard
at http://<robot_ip>:8080

Run standalone:  python3 dashboard.py
Or via launch:   included in competition.launch.py
"""

import http.server
import json
import math
import socketserver
import threading
import time
import os
import yaml

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, Joy, LaserScan
from std_msgs.msg import Bool, Float32, String

from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BOOM_GATE_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    CAMERA_DEBUG_LINE_TOPIC,
    CAMERA_DEBUG_OBS_TOPIC,
    CAMERA_DEBUG_TL_TOPIC,
    CAMERA_IMAGE_TOPIC,
    CMD_VEL_TOPIC,
    DASH_CTRL_TOPIC,
    DASH_STATE_TOPIC,
    IMU_DATA_TOPIC,
    IMU_CALIBRATE_TOPIC,
    JOY_TOPIC,
    LANE_ERROR_TOPIC,
    OBSTACLE_CAMERA_TOPIC,
    OBSTACLE_FUSED_TOPIC,
    OBSTACLE_LIDAR_TOPIC,
    ODOM_TOPIC,
    OBSTRUCTION_ACTIVE_TOPIC,
    LOOP_STATS_TOPIC,
    PARKING_COMPLETE_TOPIC,
    HEALTH_STATUS_TOPIC,
    SET_CHALLENGE_TOPIC,
    SIGNAGE_DEBUG_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_DETECTED_TOPIC,
    PARKING_SIGN_TOPIC,
    RECORD_PLAYBACK_STATE_TOPIC,
    RECORD_PLAYBACK_CMD_TOPIC,
)

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

# ======================== HTML Dashboard ========================

from .dashboard_templates import DASHBOARD_HTML, TEACH_HTML

_DEFAULT_PARAMS = {}
_PARAMS_SOURCE_PATH = ''  # Path to the SOURCE params.yaml (for writing back)

def load_default_params():
    global _DEFAULT_PARAMS, _PARAMS_SOURCE_PATH
    try:
        from ament_index_python.packages import get_package_share_directory
        try:
            share_dir = get_package_share_directory('risabot_automode')
            params_file = os.path.join(share_dir, 'config', 'params.yaml')
        except Exception:
            params_file = ''
            
        if not os.path.exists(params_file):
            this_dir = os.path.dirname(os.path.abspath(__file__))
            params_file = os.path.abspath(os.path.join(this_dir, '..', '..', 'config', 'params.yaml'))

        if os.path.exists(params_file):
            # Resolve the SOURCE file path (inside src/) for writing back
            # The share/ copy is read-only after colcon build, so we find the src/ original
            this_dir = os.path.dirname(os.path.abspath(__file__))
            source_params = os.path.abspath(os.path.join(this_dir, '..', 'config', 'params.yaml'))
            if os.path.exists(source_params):
                _PARAMS_SOURCE_PATH = source_params
            else:
                _PARAMS_SOURCE_PATH = params_file  # fallback to whatever we found

            with open(params_file, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    for node_name, node_data in data.items():
                        if isinstance(node_data, dict) and 'ros__parameters' in node_data:
                            if node_name not in _DEFAULT_PARAMS:
                                _DEFAULT_PARAMS[node_name] = {}
                            _DEFAULT_PARAMS[node_name].update(node_data['ros__parameters'])
            print(f"Loaded default params from {params_file}")
            print(f"Source params path for saving: {_PARAMS_SOURCE_PATH}")
    except Exception as e:
        print(f"Failed to load default params: {e}")

# ======================== ROS2 Dashboard Node ========================

class DashboardNode(Node):
    """ROS 2 node providing data and camera streams to the web dashboard."""
    def __init__(self):
        super().__init__('dashboard')
        self.get_logger().info('Dashboard node starting on http://0.0.0.0:8080')

        self.declare_parameter('use_hw_odom', False)
        self.declare_parameter('freshness_stale_sec', 2.0)
        self.declare_parameter('sim_odom_scale', 1.55)
        self.declare_parameter('hw_odom_scale', 1.0)
        self.declare_parameter('hw_odom_yaw_scale', 1.0)

        # CV Bridge for camera
        self.bridge = CvBridge() if CvBridge else None
        self.latest_jpeg = None
        self.jpeg_condition = threading.Condition()
        self.frame_id = 0
        self.active_camera_view = 'raw'
        self.initial_joy_axes = None

        # LiDAR scan storage for 2D visualization
        self.lidar_points = []  # [{x, y}]
        self.lidar_lock = threading.Lock()
        self.lidar_angle_offset = 3.1416  # default, same as tunnel node

        # Tunnel debug info for LiDAR overlay
        self.tunnel_debug = ''  # "left,right,error,angular_z"
        self.tunnel_debug_lock = threading.Lock()
        
        # Client tracking for performance
        self.num_camera_clients = 0
        self.camera_clients_lock = threading.Lock()

        # Shared state (read by HTTP handler)
        self.data = {
            'state': 'LANE_FOLLOW',
            'lap': 1,
            'auto_mode': False,
            'state_time': 0,
            'state_dist': '0.00',
            'traffic_light': 'unknown',
            'lidar_obstacle': None,
            'camera_obstacle': None,
            'fused_obstacle': None,
            'boom_gate': None,
            'tunnel_detected': None,
            'obstruction_active': None,
            'parking_complete': None,
            'stop_reason': '',
            'lane_error': 0.0,
            'cmd_lin_x': 0.0,
            'cmd_ang_z': 0.0,
            'distance': 0.0,
            'speed': 0.0,
            'odom_x': 0.0,
            'odom_y': 0.0,
            'odom_yaw': 0.0,
            'buttons': [],
            'axes': [],
            'speed_pct': 25,
            'ctrl_state_index': 0,
            'ctrl_state_name': 'LANE_FOLLOW',
            'health_ok': None,
            'health_summary': '',
            'health_stale': [],
            'cmd_safety_estop': False,
            'cmd_safety_timeout_count': 0,
            'loop_stats': {},
            'rp_state': 'IDLE',
            'rp_buffer_size': 0,
            'rp_playback_index': 0,
            'rp_recording_name': '',
            'rp_active_parking': '',
            'rp_saved_recordings': [],
            'parking_sign_detected': None,
            # IMU
            'imu_roll':  0.0,
            'imu_pitch': 0.0,
            'imu_yaw':   0.0,
        }
        self.topic_last_update = {
            'auto_mode': 0.0,
            'obstacle_front': 0.0,
            'obstacle_camera': 0.0,
            'obstacle_fused': 0.0,
            'traffic_light': 0.0,
            'boom_gate': 0.0,
            'tunnel_detected': 0.0,
            'obstruction_active': 0.0,
            'parking_complete': 0.0,
            'lane_error': 0.0,
            'cmd_vel': 0.0,
            'odom': 0.0,
            'odom_sim': 0.0,
            'joy': 0.0,
            'dashboard_state': 0.0,
            'dashboard_ctrl': 0.0,
            'set_challenge': 0.0,
            'health_status': 0.0,
            'cmd_safety_status': 0.0,
            'loop_stats': 0.0,
            'record_playback_state': 0.0,
            'parking_sign': 0.0,
            'imu_rpy': 0.0,
        }
        self.data_lock = threading.Lock()

        # Record & Playback command publisher
        self.rp_cmd_pub = self.create_publisher(String, RECORD_PLAYBACK_CMD_TOPIC, 10)
        self.challenge_pub = self.create_publisher(String, SET_CHALLENGE_TOPIC, 10)
        self.imu_cal_pub = self.create_publisher(String, IMU_CALIBRATE_TOPIC, 10)

        # State tracking
        self._state_entry_time = time.time()

        # === Subscriptions ===
        qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(Bool, AUTO_MODE_TOPIC, self._auto_mode_cb, 10)
        self.create_subscription(Bool, OBSTACLE_LIDAR_TOPIC, self._lidar_cb, qos)
        self.create_subscription(Bool, OBSTACLE_CAMERA_TOPIC, self._cam_cb, qos)
        self.create_subscription(Bool, OBSTACLE_FUSED_TOPIC, self._fused_cb, 10)
        self.create_subscription(String, TRAFFIC_LIGHT_TOPIC, self._tl_cb, 10)
        self.create_subscription(Bool, BOOM_GATE_TOPIC, self._gate_cb, 10)
        self.create_subscription(Bool, TUNNEL_DETECTED_TOPIC, self._tunnel_cb, 10)
        self.create_subscription(Bool, OBSTRUCTION_ACTIVE_TOPIC, self._obst_cb, 10)
        self.create_subscription(Bool, PARKING_COMPLETE_TOPIC, self._park_cb, 10)
        self.create_subscription(String, HEALTH_STATUS_TOPIC, self._health_cb, 10)
        self.create_subscription(String, CMD_SAFETY_STATUS_TOPIC, self._cmd_safety_cb, 10)
        self.create_subscription(String, LOOP_STATS_TOPIC, self._loop_stats_cb, 10)
        self.create_subscription(Float32, LANE_ERROR_TOPIC, self._lane_cb, qos)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._cmd_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(Joy, JOY_TOPIC, self._joy_cb, 10)
        self.create_subscription(String, DASH_STATE_TOPIC, self._dash_state_cb, 10)
        self.create_subscription(String, DASH_CTRL_TOPIC, self._dash_ctrl_cb, 10)
        self.create_subscription(String, SET_CHALLENGE_TOPIC, self._set_challenge_cb, 10)
        # Also listen to auto commands for display
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, self._cmd_cb, 10)

        # Camera subscriptions (SENSOR_DATA QoS to match camera publisher)
        self.create_subscription(Image, CAMERA_IMAGE_TOPIC, lambda msg: self._image_cb(msg, 'raw'), qos)
        self.create_subscription(Image, CAMERA_DEBUG_LINE_TOPIC, lambda msg: self._image_cb(msg, 'line_follower'), qos)
        # Single subscription covers both 'signage' and 'traffic_light' dashboard views
        self.create_subscription(Image, SIGNAGE_DEBUG_TOPIC, lambda msg: self._image_cb(msg, 'signage'), qos)
        self.create_subscription(Image, CAMERA_DEBUG_OBS_TOPIC, lambda msg: self._image_cb(msg, 'obstacle'), qos)

        # Parking signboard detection flag
        self.create_subscription(Bool, PARKING_SIGN_TOPIC, self._parking_sign_cb, 10)

        # IMU full RPY data
        self.create_subscription(String, IMU_DATA_TOPIC, self._imu_cb, 10)

        # LiDAR scan for 2D visualization
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)

        # Tunnel debug for LiDAR overlay
        self.create_subscription(String, '/tunnel_debug', self._tunnel_debug_cb, 10)

        # Record & Playback state from servo_controller
        self.create_subscription(String, RECORD_PLAYBACK_STATE_TOPIC, self._rp_state_cb, 10)

        # Simulate odometry since hardware might not publish
        self.create_timer(0.05, self._simulate_odom_loop)

        self.get_logger().info('Dashboard subscriptions ready')

    def _simulate_odom_loop(self) -> None:
        """Simulates odometry position based on commanded velocities.
        Useful when the hardware driver fails to publish /odom data."""
        use_hw_odom = self.get_parameter('use_hw_odom').value
        if use_hw_odom:
            return # Let the hardware Odometry cb update the state completely

        now = time.time()
        if not hasattr(self, '_last_sim_t'):
            self._last_sim_t = now
            return
            
        dt = min(now - self._last_sim_t, 0.2)
        self._last_sim_t = now
        
        with self.data_lock:
            # If no cmd_vel received in last 0.5s, assume robot is stopped
            cmd_age = now - self.data.get('_cmd_time', 0)
            if cmd_age > 0.5:
                vel_x = 0.0
                vel_z = 0.0
            else:
                vel_x = self.data['cmd_lin_x']
                vel_z = self.data['cmd_ang_z']
            
            # Odometry calibration: measured 1m real → scale to match
            # Residual error is from wheel slip/coasting (robot moves after cmd_vel=0)
            odom_scale = float(self.get_parameter('sim_odom_scale').value)
            cal_vel_x = vel_x * odom_scale
            
            # Minimum velocity threshold: ignore tiny commanded speeds
            # (prevents odometer drift when motor can't actually move)
            if abs(cal_vel_x) < 0.03:
                cal_vel_x = 0.0
            
            # Distance integration
            self.data['speed'] = cal_vel_x
            self.data['distance'] += abs(cal_vel_x) * dt
            
            # Position integration (dead reckoning)
            self.data['odom_yaw'] += vel_z * dt
            
            # X and Y based on current heading
            self.data['odom_x'] += cal_vel_x * math.cos(self.data['odom_yaw']) * dt
            self.data['odom_y'] += cal_vel_x * math.sin(self.data['odom_yaw']) * dt
            self.topic_last_update['odom_sim'] = time.monotonic()

    def _set(self, key, value, source_key=None) -> None:
        with self.data_lock:
            self.data[key] = value
            if source_key:
                self.topic_last_update[source_key] = time.monotonic()

    def _auto_mode_cb(self, msg: Bool) -> None:
        self._set('auto_mode', msg.data, 'auto_mode')
        mode = "AUTO" if msg.data else "MANUAL"
        self.get_logger().info(f'Mode changed: {mode}')

    def _lidar_cb(self, msg: Bool) -> None:
        """Update LiDAR obstacle flag."""
        self._set('lidar_obstacle', msg.data, 'obstacle_front')

    def _cam_cb(self, msg: Bool) -> None:
        """Update camera obstacle flag."""
        self._set('camera_obstacle', msg.data, 'obstacle_camera')

    def _fused_cb(self, msg: Bool) -> None:
        """Update fused obstacle flag."""
        self._set('fused_obstacle', msg.data, 'obstacle_fused')

    def _tl_cb(self, msg: String) -> None:
        """Update traffic light state."""
        self._set('traffic_light', msg.data, 'traffic_light')

    def _gate_cb(self, msg: Bool) -> None:
        """Update boom gate state."""
        self._set('boom_gate', msg.data, 'boom_gate')

    def _tunnel_cb(self, msg: Bool) -> None:
        """Update tunnel detection flag."""
        self._set('tunnel_detected', msg.data, 'tunnel_detected')

    def _obst_cb(self, msg: Bool) -> None:
        """Update obstruction avoidance flag."""
        self._set('obstruction_active', msg.data, 'obstruction_active')

    def _park_cb(self, msg: Bool) -> None:
        """Update parking completion flag."""
        self._set('parking_complete', msg.data, 'parking_complete')

    def _lane_cb(self, msg: Float32) -> None:
        """Update lane error."""
        self._set('lane_error', msg.data, 'lane_error')

    def _health_cb(self, msg: String) -> None:
        """Update parsed health summary from /health_status."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['health_ok'] = bool(payload.get('ok', False))
                self.data['health_summary'] = str(payload.get('summary', ''))
                self.data['health_stale'] = list(payload.get('stale', []))
                self.topic_last_update['health_status'] = time.monotonic()
        except Exception:
            with self.data_lock:
                self.data['health_ok'] = False
                self.data['health_summary'] = msg.data
                self.topic_last_update['health_status'] = time.monotonic()

    def _cmd_safety_cb(self, msg: String) -> None:
        """Update command safety status fields."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['cmd_safety_estop'] = bool(payload.get('estop', False))
                self.data['cmd_safety_timeout_count'] = int(payload.get('timeout_count', 0))
                self.topic_last_update['cmd_safety_status'] = time.monotonic()
        except Exception:
            self._set('cmd_safety_estop', False, 'cmd_safety_status')

    def _loop_stats_cb(self, msg: String) -> None:
        """Track latest loop stat payloads by node:loop key."""
        try:
            payload = json.loads(msg.data)
            node = str(payload.get('node', 'unknown'))
            loop = str(payload.get('loop', 'unknown'))
            key = f'{node}:{loop}'
            with self.data_lock:
                stats = dict(self.data.get('loop_stats', {}))
                stats[key] = payload
                self.data['loop_stats'] = stats
                self.topic_last_update['loop_stats'] = time.monotonic()
        except Exception:
            pass

    def _cmd_cb(self, msg: Twist) -> None:
        with self.data_lock:
            self.data['cmd_lin_x'] = msg.linear.x
            self.data['cmd_ang_z'] = msg.angular.z
            self.data['_cmd_time'] = time.time()
            self.topic_last_update['cmd_vel'] = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        use_hw_odom = self.get_parameter('use_hw_odom').value
        if not use_hw_odom:
            return  # Ignore real hardware if simulation failsafe is active
            
        # We receive actual odometry! Use real data instead of dead reckoning.
        with self.data_lock:
            # Natively, many basic firmwares only populate Twist (speeds) and leave Pose (X, Y, Yaw) as exactly 0.0
            vel_x = msg.twist.twist.linear.x
            hw_scale = float(self.get_parameter('hw_odom_scale').value)
            yaw_scale = float(self.get_parameter('hw_odom_yaw_scale').value)
            vel_x_scaled = vel_x * hw_scale
            self.data['speed'] = vel_x_scaled
            
            now = time.time()
            if hasattr(self, '_last_real_odom_t'):
                dt = min(now - self._last_real_odom_t, 0.2)
                self.data['distance'] += abs(vel_x_scaled) * dt
            else:
                dt = 0.0
            self._last_real_odom_t = now

            # Check if Firmware is actually publishing Pose Quaternions
            q = msg.pose.pose.orientation
            if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
                # Dead reckon yaw from twist angular Z
                self.data['odom_yaw'] += msg.twist.twist.angular.z * yaw_scale * dt
            else:
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.data['odom_yaw'] = math.atan2(siny_cosp, cosy_cosp)
            
            # Check if Firmware is actually publishing Pose X/Y
            hw_x = msg.pose.pose.position.x
            hw_y = msg.pose.pose.position.y
            
            if abs(hw_x) < 0.0001 and abs(hw_y) < 0.0001 and self.data['distance'] > 0.0:
                # Firmware omitted Pose but shipped Speeds. Dead reckon the speeds using current Yaw!
                self.data['odom_x'] += vel_x_scaled * math.cos(self.data['odom_yaw']) * dt
                self.data['odom_y'] += vel_x_scaled * math.sin(self.data['odom_yaw']) * dt
            else:
                self.data['odom_x'] = hw_x
                self.data['odom_y'] = hw_y
            self.topic_last_update['odom'] = time.monotonic()

    def _joy_cb(self, msg: Joy) -> None:
        with self.data_lock:
            self.data['buttons'] = list(msg.buttons)
            if self.initial_joy_axes is None:
                self.initial_joy_axes = list(msg.axes)
            
            if self.initial_joy_axes and list(msg.axes) == self.initial_joy_axes:
                self.data['axes'] = [0.0] * len(msg.axes)
            else:
                self.initial_joy_axes = []  # Clear forever once moved
                self.data['axes'] = list(msg.axes)
            self.topic_last_update['joy'] = time.monotonic()

    def _dash_state_cb(self, msg: String) -> None:
        """Parse STATE|LAP|TOTAL_DIST|STOP_REASON format from auto_driver."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                if parts[0] != self.data.get('state'):
                    self._state_entry_time = time.time()  # Reset timer on state change
                self.data['state'] = parts[0]
                if len(parts) > 1:
                    self.data['lap'] = int(parts[1])
                if len(parts) > 2:
                    self.data['state_dist'] = parts[2]  # Now represents total distance
                if len(parts) > 3:
                    self.data['stop_reason'] = parts[3]
                else:
                    self.data['stop_reason'] = ''
                self.topic_last_update['dashboard_state'] = time.monotonic()
        except Exception:
            self._set('state', msg.data)

    def _dash_ctrl_cb(self, msg: String) -> None:
        """Parse SPD_PCT|STATE_INDEX|STATE_NAME format from servo_controller."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                self.data['speed_pct'] = int(parts[0])
                if len(parts) > 1:
                    self.data['ctrl_state_index'] = int(parts[1])
                if len(parts) > 2:
                    self.data['ctrl_state_name'] = parts[2]
                self.topic_last_update['dashboard_ctrl'] = time.monotonic()
        except Exception:
            pass

    def _set_challenge_cb(self, msg: String) -> None:
        """Also listen to /set_challenge as fallback for state cycle display."""
        name = msg.data.upper()
        self._set('ctrl_state_name', name, 'set_challenge')

    def _scan_cb(self, msg: LaserScan) -> None:
        """Convert LiDAR scan to Cartesian points for 2D visualization.
        Downsamples to every 4th point to keep bandwidth low."""
        import math
        pts = []
        offset = self.lidar_angle_offset
        step = 4  # downsample: take every 4th point
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max):
                continue
            if math.isnan(r) or math.isinf(r) or r > 2.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment + offset
            x = round(r * math.cos(angle), 3)
            y = round(r * math.sin(angle), 3)
            pts.append({'x': x, 'y': y})
        with self.lidar_lock:
            self.lidar_points = pts

    def _tunnel_debug_cb(self, msg: String) -> None:
        """Store tunnel debug info for dashboard overlay."""
        with self.tunnel_debug_lock:
            self.tunnel_debug = msg.data

    def _rp_state_cb(self, msg: String) -> None:
        """Update record/playback state from servo_controller."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['rp_state'] = str(payload.get('state', 'IDLE'))
                self.data['rp_buffer_size'] = int(payload.get('buffer_size', 0))
                self.data['rp_playback_index'] = int(payload.get('playback_index', 0))
                self.data['rp_recording_name'] = str(payload.get('recording_name', ''))
                self.data['rp_active_parking'] = str(payload.get('active_parking_recording', ''))
                self.data['rp_saved_recordings'] = list(payload.get('saved_recordings', []))
                self.topic_last_update['record_playback_state'] = time.monotonic()
        except Exception:
            pass

    def _parking_sign_cb(self, msg: Bool) -> None:
        """Update parking signboard detection flag."""
        self._set('parking_sign_detected', msg.data, 'parking_sign')

    def _imu_cb(self, msg: String) -> None:
        """Update IMU roll/pitch/yaw from /imu/rpy JSON payload."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['imu_roll']  = round(float(payload.get('roll',  0.0)), 2)
                self.data['imu_pitch'] = round(float(payload.get('pitch', 0.0)), 2)
                self.data['imu_yaw']   = round(float(payload.get('yaw',   0.0)), 2)
                self.topic_last_update['imu_rpy'] = time.monotonic()
        except Exception:
            pass

    def _image_cb(self, msg: Image, view_name: str) -> None:
        """Convert ROS Image to JPEG conditionally, tracking active view and clients."""
        active = self.active_camera_view
        if self.bridge is None:
            return
        # 'signage' topic covers both the 'signage' and 'traffic_light' dashboard views
        if view_name != active and not (view_name == 'signage' and active == 'traffic_light'):
            return
            
        with self.camera_clients_lock:
            if self.num_camera_clients == 0:
                return  # Skip processing entirely if nobody is watching
                
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Force to fixed 320x240 — prevents visual jumping when source sends varying sizes
            ih, iw = cv_image.shape[:2]
            if iw != 320 or ih != 240:
                cv_image = cv2.resize(cv_image, (320, 240))
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            
            with self.jpeg_condition:
                self.latest_jpeg = jpeg.tobytes()
                self.frame_id += 1
                self.jpeg_condition.notify_all()
        except Exception:
            pass

    def get_json(self) -> str:
        with self.data_lock:
            d = dict(self.data)
            updates = dict(self.topic_last_update)
        d['state_time'] = int(time.time() - self._state_entry_time)
        now_mono = time.monotonic()
        stale_sec = float(self.get_parameter('freshness_stale_sec').value)
        freshness = {}
        stale_streams = []
        for key, last_t in updates.items():
            if last_t <= 0.0:
                freshness[key] = None
                stale_streams.append(key)
                continue
            age = round(now_mono - last_t, 3)
            freshness[key] = age
            if age > stale_sec:
                stale_streams.append(key)
        d['freshness_sec'] = freshness
        d['stale_streams'] = stale_streams
        
        # Ensure odometry types are standard python floats for JSON serialization
        for k in ['distance', 'speed', 'odom_x', 'odom_y', 'odom_yaw']:
            if k in d and d[k] is not None:
                d[k] = float(d[k])
                
        return json.dumps(d)

    def get_jpeg(self) -> None:
        # We no longer use this standalone method because do_GET
        # manages the condition variable directly to block until new frame.
        pass


# ======================== HTTP Server ========================

# Cached service clients for parameter operations
_param_clients_lock = threading.Lock()
_param_clients = {}  # {'/node_name': {'get': client, 'set': client}}

# Dedicated lightweight node + executor for param Get/Set.
# Isolated from the main dashboard node so camera callbacks never block it.
_param_helper_node = None
_param_executor = None

def _get_client(node_name, svc_type):
    """Get or create a cached service client on the dedicated param helper node."""
    n = node_name if node_name.startswith('/') else '/' + node_name
    key = (n, svc_type)
    with _param_clients_lock:
        if key not in _param_clients:
            if _param_helper_node is None:
                return None
            if svc_type == 'get':
                _param_clients[key] = _param_helper_node.create_client(GetParameters, n + '/get_parameters')
            else:
                _param_clients[key] = _param_helper_node.create_client(SetParameters, n + '/set_parameters')
    return _param_clients[key]

def _ros_get_param(node_name, param_name):
    """Get a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'get')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.15):
                return None, f'Node /{node_name} not available'
        req = GetParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, 'Timeout'
        res = future.result()
        if res and res.values:
            v = res.values[0]
            if v.type == ParameterType.PARAMETER_BOOL:
                return str(v.bool_value).lower(), None
            elif v.type == ParameterType.PARAMETER_INTEGER:
                return str(v.integer_value), None
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                return str(v.double_value), None
            elif v.type == ParameterType.PARAMETER_STRING:
                return v.string_value, None
            elif v.type == ParameterType.PARAMETER_BYTE_ARRAY:
                return list(v.byte_array_value), None
            elif v.type == ParameterType.PARAMETER_BOOL_ARRAY:
                return list(v.bool_array_value), None
            elif v.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                return list(v.integer_array_value), None
            elif v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(v.double_array_value), None
            elif v.type == ParameterType.PARAMETER_STRING_ARRAY:
                return list(v.string_array_value), None
            elif v.type == ParameterType.PARAMETER_NOT_SET:
                return None, 'Not set'
            else:
                return '?', None
        return None, 'No result'
    except Exception as e:
        return None, str(e)

def _ros_set_param(node_name, param_name, value_str):
    """Set a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'set')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.15):
                return False, f'Node /{node_name} not available'
        param = RosParameter()
        param.name = param_name
        pv = ParameterValue()
        
        # Support setting array values
        if value_str.startswith('[') and value_str.endswith(']'):
            try:
                arr = json.loads(value_str)
                if isinstance(arr, list):
                    if all(isinstance(x, bool) for x in arr):
                        pv.type = ParameterType.PARAMETER_BOOL_ARRAY
                        pv.bool_array_value = arr
                    elif all(isinstance(x, int) for x in arr):
                        pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
                        pv.integer_array_value = arr
                    elif all(isinstance(x, (int, float)) for x in arr):
                        pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                        pv.double_array_value = [float(x) for x in arr]
                    elif all(isinstance(x, str) for x in arr):
                        pv.type = ParameterType.PARAMETER_STRING_ARRAY
                        pv.string_array_value = arr
                    else:
                        raise ValueError("Unsupported array element type")
                else:
                    raise ValueError("Not a list")
            except Exception:
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = value_str
        elif value_str.lower() in ('true', 'false'):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value_str.lower() == 'true'
        else:
            try:
                # Check if it's a pure integer (no decimal point)
                if '.' not in value_str:
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(value_str)
                else:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(value_str)
            except ValueError:
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = value_str
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'Timeout'
        res = future.result()
        if res and res.results:
            r = res.results[0]
            if r.successful:
                return True, 'OK'
            else:
                return False, r.reason or 'Failed'
        return False, 'No result'
    except Exception as e:
        return False, str(e)


def _save_params_to_yaml():
    """Read current runtime params from all nodes and write them to the source params.yaml."""
    global _PARAMS_SOURCE_PATH, _DEFAULT_PARAMS
    if not _PARAMS_SOURCE_PATH:
        return {'ok': False, 'error': 'No params.yaml path resolved'}
    if not os.path.exists(_PARAMS_SOURCE_PATH):
        return {'ok': False, 'error': f'File not found: {_PARAMS_SOURCE_PATH}'}

    try:
        # Read the existing file to preserve comments structure
        # We read it as raw text lines to do targeted value replacement
        with open(_PARAMS_SOURCE_PATH, 'r') as f:
            yaml_data = yaml.safe_load(f)
        if not yaml_data:
            return {'ok': False, 'error': 'Empty params.yaml'}

        # For each node section in the YAML, fetch current runtime values
        updated_count = 0
        errors = []
        for node_name, node_data in yaml_data.items():
            if not isinstance(node_data, dict) or 'ros__parameters' not in node_data:
                continue
            params = node_data['ros__parameters']
            for param_name in list(params.keys()):
                value, err = _ros_get_param(node_name, param_name)
                if err is not None:
                    # Node not running or param not found — keep existing default
                    continue
                # Convert string value back to the correct Python type
                old_val = params[param_name]
                try:
                    if isinstance(old_val, bool):
                        new_val = value.lower() == 'true'
                    elif isinstance(old_val, int):
                        # Handle float strings like "8.0" → int 8
                        new_val = int(float(value))
                    elif isinstance(old_val, float):
                        new_val = float(value)
                    elif isinstance(old_val, list):
                        new_val = value
                    else:
                        new_val = value
                except (ValueError, TypeError):
                    new_val = value

                if params[param_name] != new_val:
                    params[param_name] = new_val
                    updated_count += 1

        # Write updated YAML back
        # Use a custom representer to avoid YAML anchors and get clean output
        class CleanDumper(yaml.SafeDumper):
            pass

        def _repr_str(dumper, data):
            return dumper.represent_scalar('tag:yaml.org,2002:str', data)
        CleanDumper.add_representer(str, _repr_str)

        with open(_PARAMS_SOURCE_PATH, 'w') as f:
            yaml.dump(yaml_data, f, Dumper=CleanDumper, default_flow_style=False,
                      sort_keys=False, allow_unicode=True, width=120)

        # Also update the in-memory defaults
        for node_name, node_data in yaml_data.items():
            if isinstance(node_data, dict) and 'ros__parameters' in node_data:
                if node_name not in _DEFAULT_PARAMS:
                    _DEFAULT_PARAMS[node_name] = {}
                _DEFAULT_PARAMS[node_name].update(node_data['ros__parameters'])

        return {'ok': True, 'msg': f'Saved {updated_count} changed params to {_PARAMS_SOURCE_PATH}',
                'updated': updated_count, 'path': _PARAMS_SOURCE_PATH}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


_node_ref = None

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    """HTTP handler for dashboard HTML, JSON, and MJPEG streams."""
    def do_GET(self):
        """Serve dashboard HTML, JSON data, MJPEG camera stream, and LiDAR data."""
        if self.path == '/data':
            data = _node_ref.get_json() if _node_ref else '{}'
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data.encode())
        elif self.path == '/lidar_data':
            pts = []
            tunnel = False
            if _node_ref:
                with _node_ref.lidar_lock:
                    pts = list(_node_ref.lidar_points)
                with _node_ref.data_lock:
                    tunnel = bool(_node_ref.data.get('tunnel_detected', False))
            payload = {'points': pts, 'tunnel': tunnel}
            # Add tunnel debug info if available (JSON format)
            if _node_ref:
                with _node_ref.tunnel_debug_lock:
                    dbg = _node_ref.tunnel_debug
                if dbg:
                    try:
                        d = json.loads(dbg)
                        payload['left_dist'] = d.get('l', 0)
                        payload['right_dist'] = d.get('r', 0)
                        payload['dist_error'] = d.get('lat', 0)
                        payload['angular_z'] = d.get('w', 0)
                        payload['centerline'] = d.get('cl', [])
                    except Exception:
                        pass
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode())
        elif self.path.startswith('/camera_feed'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0')
            self.send_header('Connection', 'close')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()
            
            if not _node_ref:
                return
                
            with _node_ref.camera_clients_lock:
                _node_ref.num_camera_clients += 1
                
            try:
                last_frame_id = -1
                while True:
                    jpeg_bytes = None
                    with _node_ref.jpeg_condition:
                        # Wait until a new frame has been generated (up to 1s to keep conn alive)
                        if _node_ref.frame_id == last_frame_id:
                            _node_ref.jpeg_condition.wait(timeout=1.0)
                        
                        if _node_ref.frame_id != last_frame_id and _node_ref.latest_jpeg:
                            last_frame_id = _node_ref.frame_id
                            jpeg_bytes = _node_ref.latest_jpeg
                            
                    if jpeg_bytes:
                        frame = (b'--frame\r\n'
                                 b'Content-Type: image/jpeg\r\n'
                                 b'Content-Length: ' + str(len(jpeg_bytes)).encode() + b'\r\n'
                                 b'\r\n' + jpeg_bytes + b'\r\n')
                        self.wfile.write(frame)
                    else:
                        # Timeout fired but no new frame
                        pass
            except Exception:
                pass
            finally:
                with _node_ref.camera_clients_lock:
                    _node_ref.num_camera_clients = max(0, _node_ref.num_camera_clients - 1)
        elif self.path.startswith('/api/set_cam_view'):
            from urllib.parse import urlparse, parse_qs
            import threading
            qs = parse_qs(urlparse(self.path).query)
            view = qs.get('view', ['raw'])[0]
            if _node_ref:
                _node_ref.active_camera_view = view
                with _node_ref.jpeg_condition:
                    _node_ref.latest_jpeg = None
                    # Force the condition to wake any blocked clients
                    _node_ref.frame_id += 1
                    _node_ref.jpeg_condition.notify_all()
            
            # Auto-toggle show_debug for performance 
            def auto_toggle_debug(selected_view):
                nodes_to_enable = set()
                if selected_view == 'line_follower':
                    nodes_to_enable.add('line_follower_camera')
                elif selected_view == 'obstacle':
                    nodes_to_enable.add('obstacle_avoidance_camera')
                elif selected_view in ('traffic_light', 'signage'):
                    nodes_to_enable.add('signage_detector')
                
                all_nodes = {'line_follower_camera', 'obstacle_avoidance_camera', 'signage_detector'}
                for node_name in all_nodes:
                    val_str = 'true' if node_name in nodes_to_enable else 'false'
                    _ros_set_param(node_name, 'show_debug', val_str)
                    
            threading.Thread(target=auto_toggle_debug, args=(view,), daemon=True).start()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        elif self.path.startswith('/api/get_param'):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            node = qs.get('node', [''])[0]
            param = qs.get('param', [''])[0]
            result = {'ok': False}
            if node and param:
                value, err = _ros_get_param(node, param)
                if err is None:
                    result = {'ok': True, 'value': value}
                    if node in _DEFAULT_PARAMS and param in _DEFAULT_PARAMS[node]:
                        result['default'] = _DEFAULT_PARAMS[node][param]
                else:
                    result = {'ok': False, 'error': err}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif self.path.startswith('/api/recording_data'):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            name = qs.get('name', [''])[0]
            result = {'ok': False, 'error': 'No name specified'}
            if name:
                recordings_dir = os.path.expanduser('~/risabot_recordings')
                fpath = os.path.join(recordings_dir, f'{name}.json')
                if os.path.exists(fpath):
                    try:
                        with open(fpath, 'r') as f:
                            data = json.load(f)
                        result = {'ok': True, 'data': data}
                    except Exception as e:
                        result = {'ok': False, 'error': str(e)}
                else:
                    result = {'ok': False, 'error': 'Recording not found'}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif self.path.startswith('/teach'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(TEACH_HTML.encode())
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

    def do_POST(self):
        """Handle dashboard API POST requests."""
        if self.path == '/api/reset_odom':
            # Reset odometry counters
            global _node_ref
            if _node_ref:
                with _node_ref.data_lock:
                    _node_ref.data['distance'] = 0.0
                    _node_ref.data['odom_x'] = 0.0
                    _node_ref.data['odom_y'] = 0.0
                    _node_ref.data['odom_yaw'] = 0.0
                    _node_ref.data['speed'] = 0.0
            resp = {'ok': True, 'msg': 'Odometry reset'}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/set_param':
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len)
            try:
                data = json.loads(body)
                node = data['node']
                param = data['param']
                value = str(data['value'])
                ok, msg = _ros_set_param(node, param, value)
                if ok:
                    resp = {'ok': True, 'msg': msg}
                else:
                    resp = {'ok': False, 'error': msg}
            except Exception as e:
                resp = {'ok': False, 'error': str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/save_defaults':
            resp = _save_params_to_yaml()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/record_playback':
            # Handle Record/Playback commands from dashboard
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len)
            try:
                data = json.loads(body)
                action = data.get('action', '')
                name = data.get('name', '')
                # Build the command string for servo_controller
                valid_simple = ('record', 'stop', 'playback', 'save', 'list')
                valid_named = ('save', 'load', 'delete', 'set_active')
                if action in valid_simple and not name:
                    cmd_str = action
                elif action in valid_named and name:
                    cmd_str = f'{action}:{name}'
                else:
                    cmd_str = ''
                if cmd_str and _node_ref:
                    _node_ref.rp_cmd_pub.publish(String(data=cmd_str))
                    resp = {'ok': True, 'msg': f'Sent: {cmd_str}'}
                else:
                    resp = {'ok': False, 'error': f'Invalid action: {action}'}
            except Exception as e:
                resp = {'ok': False, 'error': str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/reset_competition':
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len) if content_len > 0 else b'{}'
            try:
                data = json.loads(body) if body else {}
                cmd = data.get('command', 'reset').upper()
                if cmd in ('RESET', 'LAP1', 'LAP2') and _node_ref:
                    msg = String()
                    msg.data = cmd
                    _node_ref.challenge_pub.publish(msg)
                    resp = {'ok': True, 'msg': f'Sent competition command: {cmd}'}
                else:
                    resp = {'ok': False, 'error': f'Invalid command: {cmd}'}
            except Exception as e:
                resp = {'ok': False, 'error': str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/calibrate_imu':
            # Forward JSON payload to hardware IMU calibration via servo_controller
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len) if content_len > 0 else b'{}'
            try:
                if _node_ref:
                    _node_ref.imu_cal_pub.publish(String(data=body.decode('utf-8')))
                    resp = {'ok': True, 'msg': 'Calibration command sent'}
                else:
                    resp = {'ok': False, 'error': 'Dashboard node not ready'}
            except Exception as e:
                resp = {'ok': False, 'error': str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        """Suppress default HTTP logging."""
        pass

    def handle_one_request(self):
        """Override to suppress BrokenPipeError from disconnecting clients."""
        try:
            super().handle_one_request()
        except BrokenPipeError:
            pass
        except ConnectionResetError:
            pass


def main(args=None) -> None:
    global _node_ref
    rclpy.init(args=args)
    load_default_params()
    node = DashboardNode()
    _node_ref = node

    # ── Dedicated param helper node (isolated from camera/subscription load) ──
    global _param_helper_node, _param_executor
    from rclpy.executors import SingleThreadedExecutor
    _param_helper_node = rclpy.create_node('dashboard_param_helper')
    _param_executor = SingleThreadedExecutor()
    _param_executor.add_node(_param_helper_node)

    def _spin_param_executor():
        while rclpy.ok():
            try:
                _param_executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

    param_spin_thread = threading.Thread(target=_spin_param_executor, daemon=True)
    param_spin_thread.start()

    # Start HTTP server in background thread
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True   # Prevent 'Address already in use' after crash
    server = ThreadedHTTPServer(('0.0.0.0', 8080), DashboardHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    # Print user-friendly access URLs
    import socket
    hostname = socket.gethostname()
    try:
        ip = socket.getsockname() if hasattr(socket, 'getsockname') else '?'
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = '?.?.?.?'
    node.get_logger().info(f'Dashboard live!')
    node.get_logger().info(f'  → http://{hostname}.local:8080')
    node.get_logger().info(f'  → http://{ip}:8080')

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        _param_executor.shutdown()
        _param_helper_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
