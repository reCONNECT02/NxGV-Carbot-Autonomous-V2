# Blocks 10/11/12 -> GUI Local planner tab. /carbot/plan/local_candidates etc.
std_msgs/Header header
string planner          # local | parking | recovery
Candidate[] candidates
int32 selected_id       # -1 = none valid
