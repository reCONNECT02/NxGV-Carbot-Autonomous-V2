#!/usr/bin/env bash
# Is the robot visible from this PC? Lists the robot's nodes; run it while the robot's launch is up.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0
echo "nodes seen (expect calibration_wizard, command_owner, servo_controller, ...):"
timeout 20 ros2 node list --no-daemon 2>/dev/null | sort | head -40
echo
echo "/odom rate:"
timeout 8 ros2 topic hz /odom --window 10 2>&1 | grep "average rate" | tail -1
