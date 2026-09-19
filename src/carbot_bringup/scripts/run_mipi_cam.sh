#!/usr/bin/env bash
# run_mipi_cam.sh <ns> <channel> <width> <height> <domain_id> <localhost_only> <fastdds_profile> [calib_yaml]
#
# Starts one D-Robotics mipi_cam exactly as verified in Camera_Setup.md:
#   ros2 run mipi_cam mipi_cam --ros-args -r __ns:=/cam_ov5647 -p channel:=2 \
#        -p image_width:=960 -p image_height:=544
# * mipi_cam MUST run as root (else "create_and_run_vflow failed"). If we are
#   not root we re-exec through `sudo -n` (allowed by
#   tools/setup/install_root_helpers.sh for the root-owned copy of this script).
# * width/height are ALWAYS passed (default 1088x1280 -> "creat_vse_node failed").
# * sudo drops the environment, so the DDS settings arrive as arguments and
#   are exported here; otherwise the camera would be invisible to the stack.
set -eu
NS="${1:?ns}"; CH="${2:?channel}"; W="${3:?width}"; H="${4:?height}"
DOMAIN="${5:?domain_id}"; LOCALHOST="${6:?localhost_only}"; PROFILE="${7:-}"; CALIB="${8:-}"
TROS_SETUP="/opt/tros/humble/setup.bash"

if [[ "$(id -u)" != "0" ]]; then
  exec sudo -n "$(readlink -f "$0")" "$@"
fi

export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY="$LOCALHOST"
if [[ -n "$PROFILE" && -f "$PROFILE" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
fi
export ROS_LOG_DIR=/tmp/carbot_root_logs

set +u
# shellcheck disable=SC1090
source "$TROS_SETUP"
set -u
ARGS=(--ros-args -r "__ns:=${NS}" -p "channel:=${CH}" -p "image_width:=${W}" -p "image_height:=${H}")
if [[ -n "$CALIB" && -f "$CALIB" ]]; then
  # Phase 2/8: confirm this parameter name against `ros2 param list` on the RDK.
  ARGS+=(-p "camera_calibration_file_path:=${CALIB}")
fi
echo "[run_mipi_cam] ns=${NS} channel=${CH} ${W}x${H} domain=${ROS_DOMAIN_ID} localhost_only=${ROS_LOCALHOST_ONLY}"
exec ros2 run mipi_cam mipi_cam "${ARGS[@]}"
