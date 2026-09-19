# uwb_localization/uwb_ranges -> block 06 and GUI. /carbot/uwb/ranges
# Parsed from /uwb3/input_json. Empty ranges = no measurement (never (0,0)).
std_msgs/Header header  # stamp = arrival time, frame_id venue
string tag
string boot_id
uint32 seq
uint32 t_ms
string last_unknown_id
UwbRange[] ranges
