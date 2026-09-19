# uwb_localization -> GUI (Localization + System health tabs) and preflight.
std_msgs/Header header
bool agent_link_ok      # /uwb3/input_json received within link_timeout_s
float32 rate_hz
string boot_id
bool tag_rebooted       # boot_id changed since node start
string[] anchor_ids
bool[] anchor_seen      # seen within anchor_timeout_s
float32[] anchor_age_s
float32[] anchor_range_m
string last_unknown_id
