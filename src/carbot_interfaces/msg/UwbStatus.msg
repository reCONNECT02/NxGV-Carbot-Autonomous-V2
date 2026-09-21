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
# ---- added in phase 3 (extension, nothing renamed) ----
bool anchors_surveyed   # uwb.yaml anchors_surveyed (calibration step 10)
bool offsets_calibrated # uwb.yaml offsets_calibrated (step 10); false = ranges read ~1 m long
float32 latency_ms      # estimated extra WiFi delay of the last report (min-filter)
uint32 reboots          # tag boot_id changes seen since node start
uint32[] anchor_fresh_count  # new radio samples per anchor since node start
