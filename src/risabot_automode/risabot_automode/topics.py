"""Shared topic and frame constants for risabot_automode nodes."""

# Command and control
AUTO_CMD_VEL_TOPIC = '/cmd_vel_auto'
AUTO_CMD_VEL_RAW_TOPIC = '/cmd_vel_auto_raw'
CMD_VEL_TOPIC = '/cmd_vel'
AUTO_MODE_TOPIC = '/auto_mode'
SET_CHALLENGE_TOPIC = '/set_challenge'
E_STOP_TOPIC = '/e_stop'
CMD_SAFETY_STATUS_TOPIC = '/cmd_safety_status'
LOOP_STATS_TOPIC = '/loop_stats'

# Dashboard
DASH_STATE_TOPIC = '/dashboard_state'
DASH_CTRL_TOPIC = '/dashboard_ctrl'

# Perception inputs
LANE_ERROR_TOPIC = '/lane_error'
LANE_LOST_TOPIC = '/lane_lost'
OBSTACLE_LIDAR_TOPIC = '/obstacle_front'
OBSTACLE_CAMERA_TOPIC = '/obstacle_detected_camera'
OBSTACLE_FUSED_TOPIC = '/obstacle_detected_fused'
TRAFFIC_LIGHT_TOPIC = '/traffic_light_state'
TRAFFIC_LIGHT_CONF_TOPIC = '/traffic_light_confidence'
BOOM_GATE_TOPIC = '/boom_gate_open'
BOOM_GATE_CONF_TOPIC = '/boom_gate_confidence'
TUNNEL_DETECTED_TOPIC = '/tunnel_detected'
TUNNEL_CMD_TOPIC = '/tunnel_cmd_vel'
TUNNEL_CONF_TOPIC = '/tunnel_confidence'
OBSTRUCTION_ACTIVE_TOPIC = '/obstruction_active'
OBSTRUCTION_CMD_TOPIC = '/obstruction_cmd_vel'
OBSTRUCTION_CONF_TOPIC = '/obstruction_confidence'

# Parking
PARKING_CMD_TOPIC = '/parking_command'
PARKING_VEL_TOPIC = '/parking_cmd_vel'
PARKING_COMPLETE_TOPIC = '/parking_complete'
PARKING_SIGN_TOPIC = '/parking_signboard_detected'
HILL_SIGN_TOPIC = '/hill_sign_detected'
PARKING_STATUS_TOPIC = '/parking_status'



# Diagnostics
HEALTH_STATUS_TOPIC = '/health_status'

# Sensors
ODOM_TOPIC = '/odom'
JOY_TOPIC = '/joy'
IMU_PITCH_TOPIC = '/imu/pitch'
IMU_DATA_TOPIC = '/imu/rpy'          # JSON: {"roll":0.0,"pitch":0.0,"yaw":0.0}
IMU_CALIBRATE_TOPIC = '/imu/calibrate'  # publish empty String to trigger hardware calibration
CAMERA_IMAGE_TOPIC = '/camera/color/image_raw'
CAMERA_DEBUG_LINE_TOPIC = '/camera/debug/line_follower'
CAMERA_DEBUG_TL_TOPIC = '/camera/debug/traffic_light'
CAMERA_DEBUG_OBS_TOPIC = '/camera/debug/obstacle'
SIGNAGE_DEBUG_TOPIC = '/camera/debug/signage'

# Record & Playback
RECORD_PLAYBACK_STATE_TOPIC = '/record_playback_state'
RECORD_PLAYBACK_CMD_TOPIC = '/record_playback_cmd'

# Frames
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
