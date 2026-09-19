# carbot_ops/race_supervisor -> GUI. /carbot/race/preflight
uint8 STATE_LOADING=0
uint8 STATE_CAL_MISSING=1   # refuses to arm: a required calibration step is missing/failed
uint8 STATE_PREFLIGHT=2
uint8 STATE_NOT_READY=3
uint8 STATE_READY=4
uint8 STATE_RUNNING=5
uint8 STATE_FINISHED=6
uint8 STATE_ESTOPPED=7
std_msgs/Header header
uint8 state
string calibration_session  # active calibration folder
string[] missing_calibrations
PreflightCheck[] checks
string summary
