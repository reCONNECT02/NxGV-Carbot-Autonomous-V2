"""
Lane Test Launch File — RISA-bot
Minimal launch for lane follower testing ONLY.
Launches camera + lane detection + PID driver + safety + servo + joystick + dashboard.
No LiDAR, no obstacle avoidance, no traffic light, no health monitor.

  Usage: ros2 launch risabot_automode lane_test.launch.py
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

    return LaunchDescription([

        # Disable shared memory transport (prevents DDS communication failures)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),

        # ==================== SENSOR ====================

        # A. Astra Mini Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_pkg, 'launch', 'astra_mini.launch.py')
            )
        ),

        # ==================== PERCEPTION ====================
        # Delayed 3s to give camera time to start publishing

        # B. Line follower camera (lane detection only)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='line_follower_camera',
                name='line_follower_camera',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # ==================== CONTROL ====================

        # C. Auto Driver (PID brain — delayed 5s)
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

        # D. Command safety controller (rate limiting + timeout)
        Node(
            package='risabot_automode',
            executable='cmd_safety_controller',
            name='cmd_safety_controller',
            output='screen',
            parameters=[params_file]
        ),

        # E. Joystick driver (Start button = toggle auto/manual)
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

        # F. Servo controller (motor + steering hardware bridge)
        Node(
            package='control_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[params_file]
        ),

        # ==================== INTERFACE ====================

        # G. Dashboard (web UI at http://<robot_ip>:8080)
        Node(
            package='risabot_automode',
            executable='dashboard',
            name='dashboard',
            output='screen',
            parameters=[params_file]
        ),
    ])
