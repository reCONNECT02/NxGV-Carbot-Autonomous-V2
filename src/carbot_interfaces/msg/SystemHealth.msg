# carbot_ops/system_monitor -> GUI System health tab. /carbot/system/health (1 Hz)
std_msgs/Header header
TopicHealth[] topics
float32 cpu_percent
float32[] cpu_core_percent
float32 bpu_percent
float32 ram_percent
float32 soc_temp_c
float32 battery_v
bool battery_ok
bool uwb_agent_running
string[] camera_process_names
int32[] camera_process_pids
