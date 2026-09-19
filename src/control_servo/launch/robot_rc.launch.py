from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to launch the joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        # Node to launch your controller script
        Node(
            package='control_servo',
            executable='control_servo',
            name='joy_robot_controller'
        ),
    ])
