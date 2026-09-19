"""
Bringup Launch File — RISA-bot (refactor-test)
Launches ALL nodes in one command — no separate terminals needed.
  Usage: ros2 launch risabot_automode bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    astra_pkg = get_package_share_directory('astra_camera')
    risabot_pkg = get_package_share_directory('risabot_automode')
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

        # B. YDLiDAR Tmini Plus
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
        # Delayed 3s to give astra_camera time to start publishing

        # D. LiDAR obstacle detection
        TimerAction(period=3.0, actions=[
            Node(
                package='obstacle_avoidance',
                executable='obstacle_avoidance',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # E. Camera obstacle detection
        TimerAction(period=3.0, actions=[
            Node(
                package='obstacle_avoidance_camera',
                executable='obstacle_avoidance_camera',
                name='obstacle_avoidance_camera',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # F. Line follower camera (Cytron-style scanline detection)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='line_follower_camera',
                name='line_follower_camera',
                output='screen',
                parameters=[params_file]
            ),
        ]),


        # G2. Tunnel wall follower
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='tunnel_wall_follower',
                name='tunnel_wall_follower',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # G3. Signage detector (YOLOv5 BPU model)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='signage_detector',
                name='signage_detector',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # ==================== CONTROL ====================

        # H. Auto Driver (brain — delayed 5s to let sensors initialize)
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

        # I. Command safety controller
        Node(
            package='risabot_automode',
            executable='cmd_safety_controller',
            name='cmd_safety_controller',
            output='screen',
            parameters=[params_file]
        ),

        # J. Joystick driver
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

        # K. Servo controller (motor + steering hardware bridge)
        Node(
            package='control_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[params_file]
        ),

        # L. Health monitor
        Node(
            package='risabot_automode',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[params_file]
        ),

        # M. Dashboard (web UI at http://<robot_ip>:8080)
        Node(
            package='risabot_automode',
            executable='dashboard',
            name='dashboard',
            output='screen',
            parameters=[params_file]
        ),
    ])
