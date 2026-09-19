# Heartbeat every Carbot node publishes on /carbot/status (1 Hz).
# Used by the System health tab, preflight and the calibration wizard.
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STUB=3            # node exists but its algorithm is not implemented yet
std_msgs/Header header
string node             # ROS node name
string block            # architecture block id "01".."16", or "" for support nodes
uint8 level
string state            # short machine state, e.g. "RUNNING", "WAITING_INPUT"
string detail           # human-readable one line
string[] input_topics   # topics this node waits for
float32[] input_age_s   # age of last message per input_topic (-1 = never received)
