# Block 08 -> everyone. Latched (transient local) on /carbot/mission/state.
string MODE_ROAD="ROAD"
string MODE_TUNNEL="TUNNEL"
string MODE_PARKING="PARKING"
string MODE_RECOVERY="RECOVERY"
string MODE_SAFETY_STOP="SAFETY_STOP"
string MODE_HOLD="HOLD"
string MODE_IDLE="IDLE"
string MODE_COMPLETE="COMPLETE"
std_msgs/Header header
string mode
string active_source    # MotionRequest.source the command owner must follow
uint8 challenge_id      # 1..13 from challenges.yaml (0 = between challenges)
string challenge_name
uint8 route_leg         # index into mission.yaml legs
string hold_reason      # TRAFFIC HOLD / GATE HOLD / GEAR HOLD ... ("" if none)
string speed_zone       # active zone name from mission.yaml speed_zones
float32 zone_max_speed_mps
float32 set_max_speed_mps
string next_roundabout_exit   # planned exit label from global planner
string banner           # text for the main-tab banner ("" = none)
uint8 banner_level      # 0 info, 1 warn, 2 alarm
