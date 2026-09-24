#!/usr/bin/env bash
# Start the dashboard on this PC (inside WSL2). The robot's launch must run with start_gui:=false.
#   bash run_gui_pc.sh [calibrate|race]
set -eo pipefail
MODE="${1:-calibrate}"
WS="$HOME/NxGV-Carbot-Autonomous-V2"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0        # same as the robot (uwb.yaml agent.domain_id, localhost_only 0)
echo "Dashboard: http://localhost:8080/   (mode $MODE, ROS_DOMAIN_ID $ROS_DOMAIN_ID)"
exec ros2 launch carbot_bringup gui_pc.launch.py mode:="$MODE" data_root:="$HOME/carbot_data_pc"
