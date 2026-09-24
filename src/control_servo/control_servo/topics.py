# Shared topic and frame constants for control_servo nodes.

AUTO_MODE_TOPIC = '/auto_mode'
SET_CHALLENGE_TOPIC = '/set_challenge'
CMD_VEL_TOPIC = '/cmd_vel'
AUTO_CMD_VEL_TOPIC = '/cmd_vel_auto'
ODOM_TOPIC = '/odom'
JOY_TOPIC = '/joy'
DASH_CTRL_TOPIC = '/dashboard_ctrl'
LOOP_STATS_TOPIC = '/loop_stats'
IMU_PITCH_TOPIC = '/imu/pitch'
IMU_DATA_TOPIC = '/imu/rpy'          # JSON: {"roll":0.0,"pitch":0.0,"yaw":0.0}
IMU_CALIBRATE_TOPIC = '/imu/calibrate'  # publish empty String to trigger hardware calibration

# Record & Playback
RECORD_PLAYBACK_STATE_TOPIC = '/record_playback_state'
RECORD_PLAYBACK_CMD_TOPIC = '/record_playback_cmd'

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'


# Carbot extension (carbot_extension.py, phase 5) - names must match carbot_common.topics
VEHICLE_BATTERY_TOPIC = '/carbot/vehicle/battery_v'   # Float32, volts
VEHICLE_ARM_TOPIC = '/carbot/vehicle/arm'             # Bool: True = AUTO, False = MANUAL + stop
VEHICLE_STEERING_TOPIC = '/carbot/vehicle/steering'   # String JSON, see carbot_extension.SteeringTracker
VEHICLE_STEERING_RESET_TOPIC = '/carbot/vehicle/steering_reset'   # Bool: reset min/max seen
