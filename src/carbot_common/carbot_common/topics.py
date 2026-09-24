"""Single source of truth for every topic and service name.

Rules
-----
* Never hard-code a topic string in a node: import it from here.
* Base-repo topics keep their original names (they are owned by base nodes that
  we reuse unchanged). test_topics.py checks they still match
  risabot_automode/topics.py and control_servo/topics.py.
* New topics live under /carbot/.
* Renaming anything here is a contract change: announce it in the phase notes.
"""

# ---------------------------------------------------------------------------
# Base repo topics (unchanged names)
# ---------------------------------------------------------------------------
CMD_VEL_AUTO = '/cmd_vel_auto'          # command_owner -> servo_controller (Twist, base units)
CMD_VEL = '/cmd_vel'                    # servo_controller echo of what it applied
AUTO_MODE = '/auto_mode'                # servo_controller -> Bool (True = AUTO)
E_STOP = '/e_stop'                      # Bool; GUI e-stop. Counts as manual intervention.
ODOM = '/odom'                          # servo_controller wheel odometry (nav_msgs/Odometry)
IMU_RPY = '/imu/rpy'                    # servo_controller, String JSON {"roll","pitch","yaw"} (deg)
IMU_PITCH = '/imu/pitch'                # Float32
IMU_CALIBRATE = '/imu/calibrate'        # String (empty) -> hardware IMU calibration
SCAN = '/scan'                          # ydlidar LaserScan
JOY = '/joy'                            # calibrate mode only
TUNNEL_DETECTED = '/tunnel_detected'    # base tunnel_wall_follower (LiDAR trigger), Bool
TUNNEL_CMD = '/tunnel_cmd_vel'          # base tunnel_wall_follower (LiDAR lane tracking), Twist
TUNNEL_DEBUG = '/tunnel_debug'          # base tunnel_wall_follower, String JSON
TRAFFIC_LIGHT_STATE = '/traffic_light_state'  # String RED/GREEN/UNKNOWN (kept for base tools)
BOOM_GATE_OPEN = '/boom_gate_open'      # Bool (kept for base tools)
ASTRA_COLOR_IMAGE = '/camera/color/image_raw'

# UWB_Handoff.md
UWB_INPUT_JSON = '/uwb3/input_json'     # micro-ROS tag, std_msgs/String JSON, BEST_EFFORT
UWB_DEBUG_POSITION = '/uwb/position'    # tools/uwb/uwb_xy.py debug viewer only

# ---------------------------------------------------------------------------
# Carbot topics
# ---------------------------------------------------------------------------
STATUS = '/carbot/status'                               # NodeStatus, every node, 1 Hz

# Cameras: FRONT ONLY (the two MIPI side cameras were removed 2026-09-24). role -> sensor in cameras.yaml.
# Roles named in an older session's cameras.yaml that are not listed here are ignored.
CAMERA_ROLES = ('front',)


def cam_preview(role: str) -> str:
    """GUI preview stream (sensor_msgs/CompressedImage, lazy, downscaled)."""
    return f'/carbot/cam/{role}/preview/compressed'


def cam_record(role: str) -> str:
    """Throttled stream for rosbag (sensor_msgs/CompressedImage)."""
    return f'/carbot/cam/{role}/record/compressed'


# Block 03 perception
ROAD_GRID = '/carbot/perception/road_grid'              # LocalGrid (base_link)
PERCEPTION_DEBUG_WARPED = '/carbot/perception/debug/warped/compressed'
PERCEPTION_DEBUG_STITCHED = '/carbot/perception/debug/stitched/compressed'
PERCEPTION_DEBUG_MASK = '/carbot/perception/debug/mask/compressed'


def perception_overlay(role: str) -> str:
    """Road mask projected back onto original footage for one camera."""
    return f'/carbot/perception/debug/overlay/{role}/compressed'


# Block 04 memory
MEMORY_GRID = '/carbot/memory/grid'                     # LocalGrid (odom), with age_s

# Blocks 05/06 localization
LOCAL_POSE = '/carbot/localization/local_pose'          # nav_msgs/Odometry, frame track (no UWB jumps)
GLOBAL_POSE = '/carbot/localization/global_pose'        # PoseWithCovarianceStamped, frame track (UWB-aided)
LOCAL_STATUS = '/carbot/localization/local_status'     # LocalizationStatus (block 05 fields)
LOCALIZATION_STATUS = '/carbot/localization/status'     # LocalizationStatus (merged, block 06)
# phase 3: re-seed blocks 05+06 at a known track pose (preflight, calibration step 11, GUI).
# PoseWithCovarianceStamped, frame track. Ignored in race mode once the run has started.
LOCALIZATION_RESET = '/carbot/localization/reset'

# UWB (uwb_localization)
UWB_RANGES = '/carbot/uwb/ranges'                       # UwbRanges (parsed, offsets applied)
UWB_STATUS = '/carbot/uwb/status'                       # UwbStatus
UWB_RAW_FIX = '/carbot/uwb/raw_fix'                     # PointStamped venue, trilateration (GUI only)

# Blocks 01/07 map + global plan
TRACK_MAP_JSON = '/carbot/map/track_json'               # String JSON, transient local
GLOBAL_ROUTE = '/carbot/plan/global_route'              # nav_msgs/Path (all pieces, 1 cm), transient local
# Every planning Path (global route, active path, local path, parking path, corridor
# guide) is in the track frame with pose.position.z = direction (+1 forward, -1 reverse).
ROUTE_INFO_JSON = '/carbot/plan/route_info'             # String JSON: ok/reason, pieces, visits (exits), legs; latched

# Block 08 mission
MISSION_STATE = '/carbot/mission/state'                 # MissionState, transient local
MISSION_EVENTS = '/carbot/mission/events'               # MissionEvent
GATE_ROUTE_MISMATCH = '/carbot/mission/gate_route_mismatch'  # GateRouteMismatch
ACTIVE_PATH = '/carbot/mission/active_path'             # nav_msgs/Path: the active route piece, latched

# Blocks 09-13 planning
CORRIDOR = '/carbot/plan/corridor'                      # Corridor
LOCAL_CANDIDATES = '/carbot/plan/local_candidates'      # CandidateArray
LOCAL_PATH = '/carbot/plan/local_path'                  # nav_msgs/Path: guide + selected offset (empty = no feasible)
PARKING_CANDIDATES = '/carbot/parking/candidates'       # CandidateArray
PARKING_PATH = '/carbot/parking/path'                   # nav_msgs/Path
PARKING_BAY_JSON = '/carbot/parking/bay'                # String JSON (observed bay)
RECOVERY_CANDIDATES = '/carbot/recovery/candidates'     # CandidateArray
# phase 5: block 11 session state (String JSON, latched): state, stage, goal, done,
# replans, observation. mission_logic completes a manoeuvre piece on done.
PARKING_STATE = '/carbot/parking/state'
RECOVERY_STATE = '/carbot/recovery/state'              # String JSON: state, reason, attempts (GUI)

REQUEST_SOURCES = ('ROAD', 'TUNNEL', 'PARKING', 'RECOVERY')


def request_topic(source: str) -> str:
    """MotionRequest topic for one requester, e.g. /carbot/request/road."""
    return f'/carbot/request/{source.lower()}'


# Blocks 14-16 act
SAFETY_STATUS = '/carbot/safety/status'                 # SafetyStatus
OWNER_STATE = '/carbot/owner/state'                     # CommandOwnerState
VEHICLE_BATTERY = '/carbot/vehicle/battery_v'           # Float32 (servo_controller extension, phase 5)
VEHICLE_ARM = '/carbot/vehicle/arm'                     # Bool -> servo_controller AUTO (extension, phase 5)
# phase 5: calibration steps 7/8 drive the car THROUGH the command owner (one writer).
# MotionRequest, source CALIBRATION (m/s + rad through the speed PID and steering map)
# or CALIBRATION_RAW (speed_mps = base duty, steer_rad = base angular.z). Accepted
# only when command_owner.mode == calibrate and the race is not armed.
CALIBRATION_REQUEST = '/carbot/calibration/request'
# phase 7: GUI "Manual control" button. Bool, latched. True = command_owner releases
# base AUTO (winner MANUAL, zero /cmd_vel_auto) so the base servo_controller drives
# from /joy. In race mode after START this counts as manual intervention.
MANUAL_TAKEOVER = '/carbot/manual/takeover'

# Detectors
DETECTIONS = '/carbot/detections'                       # DetectionArray
DETECTIONS_DEBUG = '/carbot/detections/debug/compressed'

# Ops
RACE_PREFLIGHT = '/carbot/race/preflight'               # PreflightReport
RACE_ARMED = '/carbot/race/armed'                       # Bool, transient local
RACE_START_SRV = '/carbot/race/start'                   # std_srvs/Trigger (the one START button)
CALIBRATION_STATE = '/carbot/calibration/state'         # CalibrationState
CALIBRATION_ACTION_SRV = '/carbot/calibration/action'   # CalibrationAction
# phase 8: the open wizard step's live view, result, instructions and the session list.
# std_msgs/String JSON, latched, calibration_wizard.live_rate_hz. GUI calibration pages only.
CALIBRATION_LIVE = '/carbot/calibration/live'
RECORD_CONTROL_SRV = '/carbot/record/control'           # RecordControl
RECORD_STATE = '/carbot/record/state'                   # String JSON
SYSTEM_HEALTH = '/carbot/system/health'                 # SystemHealth
SCOREBOARD = '/carbot/scoreboard'                       # Scoreboard
