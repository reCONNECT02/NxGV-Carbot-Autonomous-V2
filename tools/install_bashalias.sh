#!/bin/bash
# ============================================================
# RISA-Bot Bash Aliases & Environment Setup Script
# This script sets up modular bash aliases and RISA-bot 
# functions in the user's ~/.bashrc and ~/.bash_aliases
#
# Usage:
#   cd ~/risabotcar_ws
#   bash tools/install_bashalias.sh
# ============================================================

set -e # Exit on error

echo "============================================================"
echo "🔧 RISA-Bot Bash Aliases Setup Script"
echo "============================================================"

echo "  -> Generating ~/.bash_aliases..."
cat << 'EOF' > ~/.bash_aliases
# Environment setup
S_CMD="source /opt/ros/humble/setup.bash && source ~/risabotcar_ws/install/setup.bash"

alias sos="source ~/risabotcar_ws/install/setup.bash"
# 1. Astra Camera
alias astra="$S_CMD && ros2 launch astra_camera astra_mini.launch.py"

# 2. YDLidar
alias ydlidar="$S_CMD && ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p baudrate:=230400 -p frame_id:=laser_frame -p lidar_type:=1 -p device_type:=0 -p sample_rate:=4 -p support_motor_dtr:=true -p intensity:=true"

# 3. Servo Controller
alias servoc="$S_CMD && ros2 run control_servo servo_controller"

# 4. Obstacle Avoidance
alias obstav="$S_CMD && ros2 run obstacle_avoidance obstacle_avoidance"

# 5. Auto Driver
alias autod="$S_CMD && ros2 run risabot_automode auto_driver"

# 6. Line Follower
alias linefollow="$S_CMD && ros2 run risabot_automode line_follower_camera"

alias s="source ~/.bashrc"

# Build the entire workspace (standard)
alias cb="cd ~/risabotcar_ws && colcon build --symlink-install"
alias cbc="cd ~/risabotcar_ws && rm -rf build/risabot_automode install/risabot_automode build/control_servo install/control_servo && colcon build --symlink-install && source install/setup.bash"

# Build ONLY the package you are currently working on (much faster)
# Usage: cbp risabot_automode
alias cbp="cd ~/risabotcar_ws && colcon build --symlink-install --packages-select"

# Clean build (deletes build and install folders first - use if you have weird errors)
alias cbd="rm -rf build/ install/ log/ && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

alias gs="git status"
alias gp="git pull"
# Quick Git Commit: adds all changes and commits with a message
alias gc='git add . && git commit -m'
alias gpu="git push"

alias risabot="ros2 launch risabot_automode competition.launch.py"
alias antigrav="rm -rf ~/.antigravity-server"
alias dashboard="python3 src/risabot_automode/risabot_automode/dashboard.py"

alias setup_wifi="sudo bash risabotcar_ws/tools/setup_wifi.sh"
alias kill_risa="pkill xfce4-terminal"
EOF
echo "✅ Generated ~/.bash_aliases"

# 2. Update ~/.bashrc
echo "  -> Updating ~/.bashrc with RISA-bot functions..."
if ! grep -q "run_risabot()" ~/.bashrc; then
    cat << 'EOF' >> ~/.bashrc

# ==========================================
# RISA-Bot Environment Setup & Functions
# ==========================================
if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

export PATH="$PATH:/home/sunrise/.local/bin"

# Disable FastRTPS Shared Memory to prevent /dev/shm corruption
export FASTRTPS_DEFAULT_PROFILES_FILE=~/risabotcar_ws/src/risabot_automode/config/disable_shm.xml

if [ -f ~/ydlidar_ros2_ws/install/setup.bash ]; then
    source ~/ydlidar_ros2_ws/install/setup.bash
fi

if [ -f ~/risabotcar_ws/install/setup.bash ]; then
    source ~/risabotcar_ws/install/setup.bash
fi

run_risabot() {
    SETUP_CMD="source /opt/ros/humble/setup.bash && source ~/risabotcar_ws/install/setup.bash"

    # 1. Astra Camera
    xfce4-terminal --title="Astra Camera" -e "bash -c '$SETUP_CMD && ros2 launch astra_camera astra_mini.launch.py; exec bash'" &

    sleep 2
    # 2. YDLidar
    xfce4-terminal --tab --title="YDLidar" -e "bash -c '$SETUP_CMD && ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p baudrate:=230400 -p frame_id:=laser_frame -p lidar_type:=1 -p device_type:=0 -p sample_rate:=4 -p support_motor_dtr:=true -p intensity:=true; exec bash'"

    # 3. Servo Controller
    xfce4-terminal --tab --title="Servo Ctrl" -e "bash -c '$SETUP_CMD && ros2 run control_servo servo_controller; exec bash'"

    # 4. Obstacle Avoidance
    xfce4-terminal --tab --title="Obstacle Avoid" -e "bash -c '$SETUP_CMD && ros2 run obstacle_avoidance obstacle_avoidance; exec bash'"

    # 5. Auto Driver
    xfce4-terminal --tab --title="Auto Driver" -e "bash -c '$SETUP_CMD && ros2 run risabot_automode auto_driver; exec bash'"

    # 6. Line Follower
    xfce4-terminal --tab --title="Line Follower" -e "bash -c '$SETUP_CMD && ros2 run risabot_automode line_follower_camera; exec bash'"
}

run_trisabot() {
    SETUP_CMD="source /opt/ros/humble/setup.bash && source ~/risabotcar_ws/install/setup.bash"

    tmux new-session -d -s risabot -n "astra"
    tmux send-keys -t risabot:astra "$SETUP_CMD && ros2 launch astra_camera astra_mini.launch.py" Enter

    sleep 2

    tmux new-window -t risabot -n "lidar"
    tmux send-keys -t risabot:lidar "$SETUP_CMD && ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p baudrate:=230400 -p frame_id:=laser_frame -p lidar_type:=1 -p device_type:=0 -p sample_rate:=4 -p support_motor_dtr:=true -p intensity:=true" Enter

    tmux new-window -t risabot -n "servo"
    tmux send-keys -t risabot:servo "$SETUP_CMD && ros2 run control_servo servo_controller" Enter

    tmux new-window -t risabot -n "obstacle"
    tmux send-keys -t risabot:obstacle "$SETUP_CMD && ros2 run obstacle_avoidance obstacle_avoidance" Enter

    tmux new-window -t risabot -n "autodrv"
    tmux send-keys -t risabot:autodrv "$SETUP_CMD && ros2 run risabot_automode auto_driver" Enter

    tmux new-window -t risabot -n "lane"
    tmux send-keys -t risabot:lane "$SETUP_CMD && ros2 run risabot_automode line_follower_camera" Enter

    tmux attach -t risabot
}

# Restore openni2_redist if missing
fix_astra() {
    if [ ! -d ~/risabotcar_ws/src/ros2_astra_camera/astra_camera/openni2_redist ]; then
        echo "Restoring openni2_redist..."
        cp -r ~/backups/openni2_redist ~/risabotcar_ws/src/ros2_astra_camera/astra_camera/
    fi
    echo "astra_camera is ready!"
}
EOF
    echo "✅ Workspace config added to ~/.bashrc"
else
    echo "✅ Workspace already configured in ~/.bashrc"
fi

echo "============================================================"
echo "🎉 ALIASES & SHELL PROFILE INSTALLATION COMPLETE! 🎉"
echo "   Please restart your terminal or run: source ~/.bashrc"
echo "============================================================"
