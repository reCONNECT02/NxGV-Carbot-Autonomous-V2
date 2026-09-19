# carbot_detectors -> block 08, GUI. /carbot/detections
std_msgs/Header header
Detection[] detections
string traffic_light_state  # RED | GREEN | UNKNOWN (debounced)
string boom_gate_state      # OPEN | CLOSED | UNKNOWN (debounced)
bool speed_bump_sign        # debounced
float32 inference_ms
