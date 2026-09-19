# Block 14 -> block 15 and GUI (Control + safety tab). Published at safety_monitor.rate_hz.
std_msgs/Header header
bool motion_allowed
string veto_check       # name of the first failing check ("" when motion_allowed)
string veto_reason
SafetyCheck[] checks
