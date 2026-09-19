# Block 08 -> GUI (Detections tab log + main-tab banner). Route is NOT changed.
std_msgs/Header header
uint8 roundabout_visit  # 1 or 2
string planned_exit     # from global planner / mission.yaml
string gate_state       # OPEN | CLOSED | UNKNOWN (detector)
string expected_exit_for_gate_state  # what the rulebook text would imply
float32 gate_confidence
string note
