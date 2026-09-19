"""CALIBRATE mode -- guided calibration wizard, one of the only two entry points.

    ros2 launch carbot_bringup calibrate.launch.py [session:=NAME] [start_joy:=true]

* Starts every sensor and every block so each step can show its live
  diagnostic view, plus calibration_wizard and (optionally) the joystick for
  the driving steps.
* The previous ACTIVE session is loaded as the starting point; the wizard
  writes a NEW timestamped session folder (older ones are never deleted, so
  you can roll back with race.launch.py session:=NAME).
* GUI (http://<robot_ip>:8080/) opens in calibrate mode with the numbered steps.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from carbot_bringup.stack import build, declare_arguments


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        DeclareLaunchArgument('start_joy', default_value='true',
                              description='joy_node for manual driving during calibration'),
        OpaqueFunction(function=lambda context: build(context, 'calibrate')),
    ])
