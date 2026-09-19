"""
Competition Launch File — RISA-bot
Brings up ALL nodes for the autonomous vehicle competition in one command:
  ros2 launch risabot_automode competition.launch.py

Launches:
  1. Astra Mini camera
  2. YDLiDAR Tmini Plus
  3. TF publisher (base_link → laser_frame)
  4. All risabot_automode nodes (auto_driver + modules)
  5. Obstacle avoidance nodes
  6. Servo controller (optional, for manual camera control)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Package paths ---
    astra_pkg = get_package_share_directory('astra_camera')
    risabot_pkg = get_package_share_directory('risabot_automode')

    # Centralized parameter file for all risabot nodes
    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')

    # --- Disable FastRTPS shared memory to prevent /dev/shm corruption ---
    shm_xml = os.path.join(risabot_pkg, 'config', 'disable_shm.xml')

    # --- Serial port mapping ---
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    return LaunchDescription([

        # Disable shared memory transport (prevents DDS communication failures)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),

        # ==================== SENSORS ====================

        # A. Astra Mini Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_pkg, 'launch', 'astra_mini.launch.py')
            )
        ),

        # B. YDLiDAR Tmini Plus (direct node — matches tested alias)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': lidar_port,
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 4,
                'support_motor_dtr': True,
                'intensity': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 16.0,
                'range_min': 0.02,
                'frequency': 10.0,
                'fixed_resolution': True,
                'reversion': True,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'invalid_range_is_inf': False,
                'abnormal_check_count': 4,
            }],
        ),

        # C. TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # ==================== PERCEPTION ====================

        # D. LiDAR obstacle detection (existing)
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[params_file]
        ),

        # E. Camera obstacle detection (existing)
        Node(
            package='obstacle_avoidance_camera',
            executable='obstacle_avoidance_camera',
            name='obstacle_avoidance_camera',
            output='screen',
            parameters=[params_file]
        ),

        # F. Line follower camera (existing)
        Node(
            package='risabot_automode',
            executable='line_follower_camera',
            name='line_follower_camera',
            output='screen',
            parameters=[params_file]
        ),


        # H. Boom gate detector (NEW)
        Node(
            package='risabot_automode',
            executable='boom_gate_detector',
            name='boom_gate_detector',
            output='screen',
            parameters=[params_file]
        ),

        # I2. Signage detector (YOLOv5 BPU model)
        Node(
            package='risabot_automode',
            executable='signage_detector',
            name='signage_detector',
            output='screen',
            parameters=[params_file]
        ),

        # I. Tunnel wall follower (NEW)
        Node(
            package='risabot_automode',
            executable='tunnel_wall_follower',
            name='tunnel_wall_follower',
            output='screen',
            parameters=[params_file]
        ),

        # J. Obstruction avoidance (NEW)
        Node(
            package='risabot_automode',
            executable='obstruction_avoidance',
            name='obstruction_avoidance',
            output='screen',
            parameters=[params_file]
        ),

        # K. Parking controller (NEW)
        Node(
            package='risabot_automode',
            executable='parking_controller',
            name='parking_controller',
            output='screen',
            parameters=[params_file]
        ),

        # ==================== CONTROL ====================

        # L. Auto Driver (brain — delayed start to let sensors initialize)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='risabot_automode',
                    executable='auto_driver',
                    name='auto_driver',
                    output='screen',
                    parameters=[params_file]
                )
            ]
        ),
        Node(
            package='risabot_automode',
            executable='cmd_safety_controller',
            name='cmd_safety_controller',
            output='screen',
            parameters=[params_file]
        ),

        # M. Joystick driver + Servo controller (mode toggle + manual driving)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }]
        ),
        Node(
            package='control_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[params_file]
        ),

        # N. Dashboard (web-based status monitor at http://<robot_ip>:8080)
        Node(
            package='risabot_automode',
            executable='dashboard',
            name='dashboard',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='risabot_automode',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[params_file]
        ),
    ])
