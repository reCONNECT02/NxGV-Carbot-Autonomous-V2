# Block 15 -> GUI / rosbag. Published every control cycle.
# Priority: SAFETY STOP -> WATCHDOG (stale request) -> active request.
std_msgs/Header header
string active_source    # source mission logic selected
string winner           # SAFETY_STOP | WATCHDOG | ROAD | TUNNEL | PARKING | RECOVERY | HOLD | DISARMED | MANUAL (phase 7)
string reason
bool armed
MotionRequest request   # the request that was considered (may be empty)
float32 request_age_s
float32 commanded_speed_mps
float32 commanded_steer_rad
float32 measured_speed_mps
float32 out_linear_x    # value written to /cmd_vel_auto linear.x (base duty units)
float32 out_angular_z   # value written to /cmd_vel_auto angular.z (base normalized steering)
bool watchdog_ok
