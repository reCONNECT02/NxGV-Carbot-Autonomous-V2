# carbot_ops/calibration_wizard -> GUI. /carbot/calibration/state
std_msgs/Header header
string session          # timestamped folder being written
uint8 current_index
CalibrationStepState[] steps
