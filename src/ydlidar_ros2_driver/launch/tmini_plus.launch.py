from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        '../../config',
        'ydlidar_tmini_plus.yaml'
    )

    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_tmini_plus',
            output='screen',
            parameters=[config]
        )
    ])
