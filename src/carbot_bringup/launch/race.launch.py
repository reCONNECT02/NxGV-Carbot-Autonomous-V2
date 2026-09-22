"""RACE (start-line) mode -- one of the only two entry points.

    ros2 launch carbot_bringup race.launch.py [session:=20260925_081500]

* Loads the ACTIVE calibration session (or session:=NAME to roll back).
* race_supervisor runs the quick PREFLIGHT (not a recalibration) and refuses to
  arm while any required calibration is missing/failed, naming it.
* GUI (http://<robot_ip>:8080/) opens in race mode: Preflight -> READY -> one
  START button -> fully autonomous run. Diagnostic tabs read-only + throttled,
  tuning disabled. (phase 7: run recording and the scoreboard are not launched.)
* joy_node runs for the GUI "Manual control" button (after START = manual
  intervention, 0 marks). start_joy:=false removes it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from carbot_bringup.stack import build, declare_arguments


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        DeclareLaunchArgument('start_joy', default_value='true',
                              description='joy_node for the GUI Manual control button'),
        OpaqueFunction(function=lambda context: build(context, 'race')),
    ])
