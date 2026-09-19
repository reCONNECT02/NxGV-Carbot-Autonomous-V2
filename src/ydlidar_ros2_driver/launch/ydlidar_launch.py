import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the parameter file
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = os.path.join(share_dir, 'params', 'ydlidar.yaml')

    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[parameter_file],
            # If your robot is upside down or different, 
            # you can add arguments here later
        )
    ])
