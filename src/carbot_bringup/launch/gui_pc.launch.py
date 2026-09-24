"""The dashboard (gui_server) on a PC instead of the RDK. See tools/dashboard_pc/README.md.

    ros2 launch carbot_bringup gui_pc.launch.py [mode:=calibrate|race] [data_root:=~/carbot_data_pc]

Starts ONLY gui_server, with the same parameter files and data paths the car's launch gives it, in the ROS domain
of uwb.yaml (`agent.domain_id`). The robot's launch must then run with `start_gui:=false` (otherwise two nodes are
called gui_server). The PC has to see the robot's DDS traffic: same network, same domain, multicast allowed
(WSL2 needs `networkingMode=mirrored`).

Limits: the tuning tab's Save writes the session overlay on THIS machine (data_root), not on the robot: live
changes (Set) work, saved ones stay on the PC. The GUI needs no session on the PC; the wizard on the robot owns it.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node

from carbot_bringup.stack import PACKAGE, data_paths, load_yaml, param_file_list


def _build(context):
    from ament_index_python.packages import get_package_share_directory
    from carbot_common import calibration_store as cs
    arg = lambda n: context.launch_configurations.get(n, '')  # noqa: E731
    mode = arg('mode')
    if mode not in ('calibrate', 'race'):
        raise ValueError(f'mode must be calibrate or race, not {mode!r}')
    share = get_package_share_directory(PACKAGE)
    root = cs.data_root(arg('data_root'))
    os.makedirs(root, exist_ok=True)
    dpaths = data_paths(share, None)
    uwb = load_yaml(dpaths['data.uwb'])
    domain = str(int(uwb['agent']['domain_id']))
    extra = dict(dpaths)
    extra.update({'data_root': root, 'mode': mode, 'session': ''})
    shm_xml = os.path.join(share, 'config', 'fastdds', 'disable_shm.xml')
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),
        SetEnvironmentVariable('CARBOT_DATA', root),
        SetEnvironmentVariable('CARBOT_MODE', mode),
        Node(package='carbot_gui', executable='gui_server', name='gui_server', output='screen',
             parameters=list(param_file_list(share, None)) + [extra]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='calibrate', description='calibrate | race'),
        DeclareLaunchArgument('data_root', default_value='',
                              description='PC folder for GUI-side data (default $CARBOT_DATA or ~/carbot_data)'),
        OpaqueFunction(function=_build),
    ])
