#!/usr/bin/env bash
# run_uwb_agent.sh <ros_setup> <uros_workspace> <port> <verbosity>
#
# micro-ROS agent for the UWB tag (UWB_Handoff.md). It was built in ~/uros_ws
# against /opt/ros/humble, so it is not on the stack's package path.
# ROS_LOCALHOST_ONLY must be 0: the agent ignores it and announces on the
# network interface, so localhost-only nodes never see /uwb3/input_json.
# The TAG chooses the DDS domain (TagConfig.h MICROROS_DOMAIN_ID = 1).
set -eu
ROS_SETUP="${1:?ros_setup}"; WS="$(eval echo "${2:?workspace}")"; PORT="${3:?port}"; VERB="${4:-4}"
export ROS_LOCALHOST_ONLY=0
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ ! -f "$WS/install/local_setup.bash" ]]; then
  echo "[run_uwb_agent] $WS/install/local_setup.bash not found: build the agent (docs/SETUP.md)" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/local_setup.bash"
set -u
echo "[run_uwb_agent] udp4 port ${PORT} -v${VERB}; tag must be set to domain ${ROS_DOMAIN_ID:-unset}"
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$PORT" "-v${VERB}"
