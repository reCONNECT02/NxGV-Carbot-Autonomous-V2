#!/bin/bash
cd ~/NxGV-Carbot-Autonomous-V2
source /opt/ros/humble/setup.bash
source install/setup.bash
export DISPLAY=:0
exec ros2 launch carbot_bringup calibrate.launch.py calibrate_profile:=lite
