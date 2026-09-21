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

# Camera_Setup.md (verified 2026-09-18)
OV5647_IMAGE = '/cam_ov5647/image_raw'  # MIPI channel 2
IMX219_IMAGE = '/cam_imx219/image_raw'  # MIPI channel 0

# UWB_Handoff.md
UWB_INPUT_JSON = '/uwb3/input_json'     # micro-ROS tag, std_msgs/String JSON, BEST_EFFORT
UWB_DEBUG_POSITION = '/uwb/position'    # tools/uwb/uwb_xy.py debug viewer only

# ---------------------------------------------------------------------------
# Carbot topics
# ---------------------------------------------------------------------------
STATUS = '/carbot/status'                               # NodeStatus, every node, 1 Hz

# Cameras (role = front | left_rear | right_rear, mapped to sensors in cameras.yaml)
CAMERA_ROLES = ('front', 'left_rear', 'right_rear')


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
GLOBAL_ROUTE = '/carbot/plan/global_route'              # nav_msgs/Path (all legs), transient local
ROUTE_INFO_JSON = '/carbot/plan/route_info'             # String JSON: legs, checkpoints, exits

# Block 08 mission
MISSION_STATE = '/carbot/mission/state'                 # MissionState, transient local
MISSION_EVENTS = '/carbot/mission/events'               # MissionEvent
GATE_ROUTE_MISMATCH = '/carbot/mission/gate_route_mismatch'  # GateRouteMismatch
ACTIVE_PATH = '/carbot/mission/active_path'             # nav_msgs/Path the tracker follows

# Blocks 09-13 planning
CORRIDOR = '/carbot/plan/corridor'                      # Corridor
LOCAL_CANDIDATES = '/carbot/plan/local_candidates'      # CandidateArray
LOCAL_PATH = '/carbot/plan/local_path'                  # nav_msgs/Path (selected)
PARKING_CANDIDATES = '/carbot/parking/candidates'       # CandidateArray
PARKING_PATH = '/carbot/parking/path'                   # nav_msgs/Path
PARKING_BAY_JSON = '/carbot/parking/bay'                # String JSON (observed bay)
RECOVERY_CANDIDATES = '/carbot/recovery/candidates'     # CandidateArray

REQUEST_SOURCES = ('ROAD', 'TUNNEL', 'PARKING', 'RECOVERY')


def request_topic(source: str) -> str:
    """MotionRequest topic for one requester, e.g. /carbot/request/road."""
    return f'/carbot/request/{source.lower()}'


# Blocks 14-16 act
SAFETY_STATUS = '/carbot/safety/status'                 # SafetyStatus
OWNER_STATE = '/carbot/owner/state'                     # CommandOwnerState
VEHICLE_BATTERY = '/carbot/vehicle/battery_v'           # Float32 (servo_controller extension, phase 5)
VEHICLE_ARM = '/carbot/vehicle/arm'                     # Bool -> servo_controller AUTO (extension, phase 5)

# Detectors
DETECTIONS = '/carbot/detections'                       # DetectionArray
DETECTIONS_DEBUG = '/carbot/detections/debug/compressed'

# Ops
RACE_PREFLIGHT = '/carbot/race/preflight'               # PreflightReport
RACE_ARMED = '/carbot/race/armed'                       # Bool, transient local
RACE_START_SRV = '/carbot/race/start'                   # std_srvs/Trigger (the one START button)
CALIBRATION_STATE = '/carbot/calibration/state'         # CalibrationState
CALIBRATION_ACTION_SRV = '/carbot/calibration/action'   # CalibrationAction
RECORD_CONTROL_SRV = '/carbot/record/control'           # RecordControl
RECORD_STATE = '/carbot/record/state'                   # String JSON
SYSTEM_HEALTH = '/carbot/system/health'                 # SystemHealth
SCOREBOARD = '/carbot/scoreboard'                       # Scoreboard
