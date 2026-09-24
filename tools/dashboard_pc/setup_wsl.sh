#!/usr/bin/env bash
# One-time setup INSIDE WSL2 Ubuntu 22.04: ROS 2 Humble (ros-base) + the four packages the dashboard needs.
#   bash setup_wsl.sh            (about 10-20 min, ~1.5 GB download)
set -euo pipefail
REPO_URL="https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2"
BRANCH="${BRANCH:-phase8/complete-calibration}"
WS="$HOME/NxGV-Carbot-Autonomous-V2"

if ! grep -q "22.04" /etc/os-release; then
  echo "This needs Ubuntu 22.04 (ROS 2 Humble). Found: $(. /etc/os-release; echo "$PRETTY_NAME")"
  exit 1
fi
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release software-properties-common git python3-pip
sudo add-apt-repository -y universe
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-rmw-fastrtps-cpp ros-humble-tf2-ros ros-humble-joy \
  python3-colcon-common-extensions python3-numpy python3-yaml python3-opencv

if [ -d "$WS/.git" ]; then
  git -C "$WS" fetch origin "$BRANCH"
  git -C "$WS" checkout "$BRANCH"
  git -C "$WS" pull --ff-only origin "$BRANCH"
else
  git clone --branch "$BRANCH" "$REPO_URL" "$WS"
fi

cd "$WS"
source /opt/ros/humble/setup.bash
# only what gui_server needs: messages, shared lib, the GUI and the launch package (everything else stays on the robot)
colcon build --symlink-install --packages-select carbot_interfaces carbot_common carbot_gui carbot_bringup
echo
echo "Done. Start the dashboard with:  bash $WS/tools/dashboard_pc/run_gui_pc.sh"
