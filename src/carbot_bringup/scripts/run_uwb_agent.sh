#!/usr/bin/env bash
# run_uwb_agent.sh <ros_setup> <uros_workspace> <port> <verbosity>
#
# micro-ROS agent for the UWB tag (docs/UWB.md). It was built in ~/uros_ws (or ~/microros_ws)
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
# uwb.yaml agent.workspace first; then the two names in use on the team's machines:
# ~/uros_ws (UWB_Handoff.md) and ~/microros_ws (Haffiz's setup, tools/uwb/haffiz/).
if [[ ! -f "$WS/install/local_setup.bash" ]]; then
  for alt in "$HOME/uros_ws" "$HOME/microros_ws" /home/sunrise/uros_ws /home/sunrise/microros_ws; do
    if [[ -f "$alt/install/local_setup.bash" ]]; then
      echo "[run_uwb_agent] $WS has no agent build; using $alt (set uwb.yaml agent.workspace to silence this)" >&2
      WS="$alt"; break
    fi
  done
fi
if [[ ! -f "$WS/install/local_setup.bash" ]]; then
  echo "[run_uwb_agent] no micro-ROS agent build in $WS, ~/uros_ws or ~/microros_ws: build it (docs/UWB.md)" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/local_setup.bash"
set -u
# NOTE: no IP argument. Haffiz's notes show `udp4 <ip> --port 8888`; the agent listens on
# every interface and the IP belongs in the TAG (TagConfig.h MICROROS_AGENT_IP), not here.
echo "[run_uwb_agent] udp4 port ${PORT} -v${VERB}; tag must be set to domain ${ROS_DOMAIN_ID:-unset}"
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$PORT" "-v${VERB}"
