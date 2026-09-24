"""Launch builder shared by the only two entry points:

    ros2 launch carbot_bringup calibrate.launch.py   (guided calibration wizard)
    ros2 launch carbot_bringup race.launch.py        (start-line mode)

What it does, in order:
  1. Environment: ROS_DOMAIN_ID from data/uwb.yaml (the UWB tag chooses it),
     ROS_LOCALHOST_ONLY=0 (UWB_Handoff.md section 8), FastDDS UDP-only profile
     (base repo disable_shm.xml, prevents /dev/shm corruption and root/user
     shared-memory permission clashes with any root-owned process).
  2. Calibration session: race loads the ACTIVE session (or session:=NAME for a
     rollback); calibrate loads it as the starting point for "keep previous".
     Its params_overlay.yaml is applied LAST and its data/*.yaml replace the
     repo defaults.
  3. Kills stale camera / codec / websocket / agent / node processes
     (Camera_Setup.md gotcha) and waits for that to finish.
  4. Drivers: Astra Pro colour camera (the ONLY camera; the two MIPI side cameras were
     removed 2026-09-24), T-mini Plus LiDAR, micro-ROS agent, static TFs.
  5. Base nodes reused unchanged: servo_controller, tunnel_wall_follower.
     The base auto_driver / cmd_safety_controller / dashboard are NOT started:
     command_owner is the single writer of /cmd_vel_auto and gui_server owns
     port 8080.
  6. Carbot nodes (16 blocks + ops), then the GUI in the launch's mode.

Every tunable is read from YAML; nothing here hard-codes a value that a
student might need to change on competition day.
"""
import os
import tempfile
from typing import Dict, List, Optional

import yaml

from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import DATA_KEYS, astra_launch

PACKAGE = 'carbot_bringup'

PARAM_FILES = ('common', 'base_nodes', 'drivers', 'perception', 'localization', 'planning',
               'control', 'detectors', 'gui', 'ops')

# (package, executable) of every Carbot node. Node name == executable.
CARBOT_NODES = {
    'common': [
        ('carbot_planning', 'track_map_server'),      # 01
        ('carbot_perception', 'road_perception'),     # 03
        ('carbot_perception', 'local_memory'),        # 04
        ('carbot_perception', 'camera_preview'),      # GUI / record streams
        ('uwb_localization', 'uwb_ranges'),           # 06 input
        ('carbot_localization', 'local_pose'),        # 05
        ('carbot_localization', 'global_pose'),       # 06
        ('carbot_planning', 'global_planner'),        # 07
        ('carbot_planning', 'mission_logic'),         # 08
        ('carbot_planning', 'corridor'),              # 09
        ('carbot_planning', 'local_planner'),         # 10
        ('carbot_planning', 'parking_planner'),       # 11
        ('carbot_planning', 'recovery_planner'),      # 12
        ('carbot_planning', 'path_tracker'),          # 13
        ('carbot_control', 'tunnel_bridge'),          # 13 (tunnel)
        ('carbot_control', 'safety_monitor'),         # 14
        ('carbot_control', 'command_owner'),          # 15 -> 16
        ('carbot_detectors', 'bpu_detector'),
        ('carbot_ops', 'system_monitor'),
        # phase 7: scoreboard and run_recorder are NOT launched (CPU; team decision
        # 2026-09-22). Code kept in carbot_ops: add the two lines back to re-enable.
    ],
    'calibrate': [('carbot_ops', 'calibration_wizard')],
    'race': [('carbot_ops', 'race_supervisor')],
}

# Base-repo nodes reused unchanged (block 16 hardware bridge + tunnel).
BASE_NODES = [
    ('control_servo', 'servo_controller'),
    ('risabot_automode', 'tunnel_wall_follower'),
]


# --------------------------------------------------------------------------- pure helpers
def load_yaml(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def ros_params(doc: Dict, node: str) -> Dict:
    """ros__parameters of `node` in a parameter-file document ({} if absent)."""
    return ((doc or {}).get(node) or {}).get('ros__parameters') or {}


def resolve_session(root: str, requested: str = '') -> Optional[str]:
    """Absolute session path: session:=NAME if given (must exist), else ACTIVE."""
    if requested:
        path = requested if os.path.isabs(requested) else \
            os.path.join(cs.calibration_dir(root), requested)
        if not os.path.isdir(path):
            raise FileNotFoundError(f'calibration session not found: {path} '
                                    f'(available: {cs.list_sessions(root)})')
        return path
    return cs.active_session(root)


def data_paths(share_dir: str, session: Optional[str]) -> Dict[str, str]:
    """data.<key> -> file path; a session copy overrides the repo default."""
    out = {}
    for key in DATA_KEYS:
        fname = f'{key}.yaml'
        out[f'data.{key}'] = cs.data_override(session, fname) or \
            os.path.join(share_dir, 'config', 'data', fname)
    return out


def param_file_list(share_dir: str, session: Optional[str]) -> List[str]:
    files = [os.path.join(share_dir, 'config', 'params', f'{n}.yaml') for n in PARAM_FILES]
    overlay = cs.overlay_params(session)
    if overlay:
        files.append(overlay)          # last wins
    return files


PROFILES = ('full', 'lite')
LITE_FILE = 'calibrate_lite'


def check_profile(profile: str, mode: str) -> str:
    """'' -> full. lite exists only for calibrate (race must always run the whole stack)."""
    profile = (profile or 'full').strip().lower()
    if profile not in PROFILES:
        raise ValueError(f'calibrate_profile must be one of {PROFILES}, not {profile!r}')
    if profile == 'lite' and mode != 'calibrate':
        raise ValueError('calibrate_profile:=lite is for calibrate.launch.py only')
    return profile


def lite_keep(cfg: Dict) -> List[str]:
    """carbot_launch.calibrate_lite_keep: executable names the lite profile still starts."""
    if 'calibrate_lite_keep' not in cfg:
        raise KeyError('missing YAML key carbot_launch.calibrate_lite_keep (drivers.yaml)')
    keep = [str(x) for x in cfg['calibrate_lite_keep']]
    known = {exe for _, exe in CARBOT_NODES['common'] + BASE_NODES}
    unknown = sorted(set(keep) - known)
    if unknown:
        raise KeyError(f'carbot_launch.calibrate_lite_keep names unknown nodes: {unknown} (known: {sorted(known)})')
    return keep


def select_nodes(nodes: List[tuple], keep: Optional[List[str]]) -> List[tuple]:
    """(package, executable) list filtered to `keep` (None = all), order preserved."""
    return list(nodes) if keep is None else [n for n in nodes if n[1] in keep]


def with_lite_overlay(files: List[str], share_dir: str) -> List[str]:
    """Insert calibrate_lite.yaml after the normal params files, before the session overlay."""
    path = os.path.join(share_dir, 'config', 'params', f'{LITE_FILE}.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'calibrate_profile:=lite needs {path}')
    out = list(files)
    out.insert(len(PARAM_FILES), path)
    return out


def launch_cfg(share_dir: str, session: Optional[str]) -> Dict:
    """carbot_launch + carbot_tf parameters, with the session overlay applied."""
    drivers = load_yaml(os.path.join(share_dir, 'config', 'params', 'drivers.yaml'))
    cfg = dict(ros_params(drivers, 'carbot_launch'))
    tf = dict(ros_params(drivers, 'carbot_tf'))
    overlay = cs.overlay_params(session)
    if overlay:
        ov = load_yaml(overlay)
        cfg.update(ros_params(ov, 'carbot_launch'))
        tf.update(ros_params(ov, 'carbot_tf'))
    cfg['carbot_tf'] = tf
    return cfg


def camera_static_tfs(cameras: Dict) -> List[Dict]:
    """base_link -> cam_<role> -> cam_<role>_optical transforms from cameras.yaml mounts.

    cam_<role> is a BODY-style frame (+x along the optical axis, +y left,
    +z up). pitch_down > 0 tilts +x towards the ground, which is a positive
    rotation about +y (R = Rz(yaw) Ry(pitch_down) Rx(roll), the same as
    carbot_perception.camera_model.Mount). cam_<role>_optical is the ROS
    optical frame (+z forward, +x right, +y down) the intrinsics refer to.
    Only roles in topics.CAMERA_ROLES (front) get frames: an older session's cameras.yaml may
    still carry side-camera mounts, which are ignored.
    """
    import math
    out = []
    for role, m in (cameras.get('mounts') or {}).items():
        if role not in T.CAMERA_ROLES:
            continue
        out.append({
            'parent': 'base_link', 'child': f'cam_{role}',
            'x': float(m['x_m']), 'y': float(m['y_m']), 'z': float(m['z_m']),
            'yaw': math.radians(float(m['yaw_deg'])),
            'pitch': math.radians(float(m['pitch_down_deg'])),
            'roll': math.radians(float(m.get('roll_deg', 0.0))),
        })
        out.append({'parent': f'cam_{role}', 'child': f'cam_{role}_optical',
                    'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'yaw': -math.pi / 2, 'pitch': 0.0, 'roll': -math.pi / 2})
    return out


def root_helper_cmd(cfg: Dict, share_dir: str, script: str, args: List[str],
                    is_root: bool) -> List[str]:
    """Command line for a helper that must run as root.

    Preferred: the root-owned copy in carbot_launch.root_helper_dir, which
    tools/setup/install_root_helpers.sh installs and allows in sudoers
    (NOPASSWD for exactly those files). Fallback: the package share copy,
    run as the current user (works when the whole launch runs as root, e.g.
    the systemd unit, or for testing without cameras).
    """
    root_dir = cfg.get('root_helper_dir', '') or ''
    installed = os.path.join(root_dir, script) if root_dir else ''
    if installed and os.path.isfile(installed):
        return ([] if is_root else ['sudo', '-n']) + [installed] + list(args)
    return ['bash', os.path.join(share_dir, 'scripts', script)] + list(args)


# --------------------------------------------------------------------------- launch
def build(context, mode: str):
    """OpaqueFunction body. mode = 'race' | 'calibrate'."""
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import (ExecuteProcess, IncludeLaunchDescription, LogInfo,
                                RegisterEventHandler, SetEnvironmentVariable, TimerAction)
    from launch.event_handlers import OnProcessExit
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    assert mode in ('race', 'calibrate')
    arg = lambda n: context.launch_configurations.get(n, '')  # noqa: E731
    truthy = lambda n: arg(n).lower() in ('1', 'true', 'yes', 'on')  # noqa: E731

    share = get_package_share_directory(PACKAGE)
    root = cs.data_root(arg('data_root'))
    session = resolve_session(root, arg('session'))
    cfg = launch_cfg(share, session)
    dpaths = data_paths(share, session)
    params = param_file_list(share, session)
    profile = check_profile(arg('calibrate_profile'), mode)
    keep = None
    if profile == 'lite':
        keep = lite_keep(cfg)
        params = with_lite_overlay(params, share)
    cameras = load_yaml(dpaths['data.cameras'])
    uwb = load_yaml(dpaths['data.uwb'])
    agent = uwb['agent']
    domain = str(int(agent['domain_id']))
    shm_xml = os.path.join(share, 'config', 'fastdds', 'disable_shm.xml')

    extra = dict(dpaths)
    extra['data_root'] = root
    extra['mode'] = mode
    extra['session'] = session or ''

    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', str(int(agent['localhost_only']))),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),
        SetEnvironmentVariable('CARBOT_DATA', root),
        SetEnvironmentVariable('CARBOT_MODE', mode),
        LogInfo(msg=f'[carbot] mode={mode} domain={domain} data_root={root}'
                    + (f' profile=LITE (only {keep}; step 13 needs profile:=full)' if keep is not None else '')),
        LogInfo(msg=f'[carbot] calibration session: {session or "NONE"}'
                    + ('' if session or mode == 'calibrate' else
                       '  -> race_supervisor will REFUSE TO ARM until a calibration is saved')),
    ]
    for k, v in sorted(dpaths.items()):
        actions.append(LogInfo(msg=f'[carbot]   {k} = {v}'))

    # ---------------------------------------------------------------- 3. stale cleanup
    pat_file = os.path.join(tempfile.gettempdir(), f'carbot_kill_patterns_{os.getpid()}.txt')
    with open(pat_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(cfg.get('kill_patterns', [])) + '\n')
    is_root = os.geteuid() == 0
    kill_cmd = root_helper_cmd(cfg, share, 'kill_stale.sh',
                               [pat_file, str(cfg.get('kill_grace_s', 2.0))], is_root)
    if not is_root and kill_cmd[0] == 'bash':
        actions.append(LogInfo(msg='[carbot] WARNING: root helpers not installed '
                                   '(tools/setup/install_root_helpers.sh): stale root-owned '
                                   'processes cannot be killed'))
    cleanup = ExecuteProcess(cmd=kill_cmd, name='kill_stale', output='screen')
    actions.append(cleanup)

    # ---------------------------------------------------------------- 4. drivers
    later = []

    def node(pkg, exe, extra_params=True, **kw):
        p = list(params) + ([extra] if extra_params else [])
        return Node(package=pkg, executable=exe, name=exe, output='screen', parameters=p, **kw)

    drivers = []
    if truthy('start_cameras'):
        a_pkg, a_file, a_args = astra_launch(cameras['sensors']['astra'])
        drivers.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(a_pkg), 'launch', a_file)),
            launch_arguments=[tuple(x.split(':=', 1)) for x in a_args]))

    if truthy('start_lidar'):
        drivers.append(node('ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', extra_params=False))

    if truthy('start_uwb_agent'):
        drivers.append(ExecuteProcess(
            cmd=['bash', os.path.join(share, 'scripts', 'run_uwb_agent.sh'),
                 agent['ros_setup'], agent['workspace'], str(agent['port']),
                 str(agent.get('verbosity', 4))],
            name='uwb_agent', output='screen'))

    # static TFs (TF is always started: no hardware involved)
    t = cfg['carbot_tf'].get('base_to_laser', [0.0, 0.0, 0.12, 0.0, 0.0, 0.0])
    tfs = [dict(parent='base_link', child='laser_frame', x=t[0], y=t[1], z=t[2],
                yaw=t[3], pitch=t[4], roll=t[5])]
    tfs += camera_static_tfs(cameras)
    for tf in tfs:
        drivers.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_base_to_{tf["child"]}', output='log',
            arguments=['--x', str(tf['x']), '--y', str(tf['y']), '--z', str(tf['z']),
                       '--yaw', str(tf['yaw']), '--pitch', str(tf['pitch']),
                       '--roll', str(tf['roll']),
                       '--frame-id', tf['parent'], '--child-frame-id', tf['child']]))
    later += drivers

    # joystick: manual driving during calibration, and the GUI "Manual control"
    # button in both modes (phase 7; race: after START it counts as intervention)
    if truthy('start_joy'):
        later.append(node('joy', 'joy_node', extra_params=False))

    # ---------------------------------------------------------------- 5. base nodes
    if truthy('start_base'):
        base = [node(pkg, exe, extra_params=False) for pkg, exe in select_nodes(BASE_NODES, keep)]
        later.append(TimerAction(period=float(cfg.get('delay_base_nodes_s', 2.0)), actions=base))

    # ---------------------------------------------------------------- 6. carbot stack + GUI
    stack = [node(pkg, exe) for pkg, exe in select_nodes(CARBOT_NODES['common'], keep) + CARBOT_NODES[mode]]
    later.append(TimerAction(period=float(cfg.get('delay_stack_s', 4.0)), actions=stack))
    if truthy('start_gui'):                    # start_gui:=false when the dashboard runs on a PC (tools/dashboard_pc)
        later.append(TimerAction(period=float(cfg.get('delay_gui_s', 5.0)),
                                 actions=[node('carbot_gui', 'gui_server')]))

    actions.append(RegisterEventHandler(OnProcessExit(target_action=cleanup, on_exit=later)))
    return actions


def declare_arguments():
    from launch.actions import DeclareLaunchArgument
    return [
        DeclareLaunchArgument('session', default_value='',
                              description='Calibration session folder name (default: ACTIVE). '
                                          'Use an older one to roll back.'),
        DeclareLaunchArgument('data_root', default_value='',
                              description='Data folder (default $CARBOT_DATA or '
                                          '/home/sunrise/carbot_data)'),
        DeclareLaunchArgument('start_cameras', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_uwb_agent', default_value='true'),
        DeclareLaunchArgument('calibrate_profile', default_value='full',
                              description="calibrate.launch.py only: 'lite' starts just the nodes in "
                                          "drivers.yaml carbot_launch.calibrate_lite_keep and lowers GUI/"
                                          "monitor rates (calibrate_lite.yaml). Steps 1-12 only; "
                                          "step 13 needs 'full'."),
        DeclareLaunchArgument('start_gui', default_value='true',
                              description='gui_server on this machine; false when the dashboard runs on a PC '
                                          '(tools/dashboard_pc), otherwise both would use the same node name'),
        DeclareLaunchArgument('start_base', default_value='true',
                              description='servo_controller + tunnel_wall_follower (base repo)'),
    ]
