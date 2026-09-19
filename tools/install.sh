#!/bin/bash
# ============================================================
# RISA-Bot Fresh Installation Script
# This script automates the setup of a new robot, including
# udev rules, building 3rd-party dependencies, and colcon build.
#
# Usage:
#   cd ~/risabotcar_ws
#   bash tools/install.sh [path_to_backups_folder]
# ============================================================

set -e # Exit on error

echo "============================================================"
echo "🤖 RISA-Bot Fresh Installation Script"
echo "============================================================"

# Default backup path is ~/backups unless specified
BACKUP_DIR=${1:-"$HOME/backups"}
WS_DIR="$HOME/risabotcar_ws"
SRC_DIR="$WS_DIR/src"

echo "[1/8] Initial Checks..."
if [ ! -d "$BACKUP_DIR" ]; then
    echo "❌ ERROR: Backup directory not found at: $BACKUP_DIR"
    echo "Please copy the 4 modified dependency folders to this location first:"
    echo "  - ros2_astra_camera"
    echo "  - openni2_redist"
    echo "  - ydlidar_ros2_driver"
    echo "  - YDLidar-SDK"
    echo "Usage: bash tools/install.sh /path/to/backups"
    exit 1
fi

if [ ! -d "$SRC_DIR/RISA-bot" ] && [ ! -d "$SRC_DIR/risabot_automode" ]; then
    echo "❌ ERROR: Run this script from the workspace root (e.g. ~/risabotcar_ws)"
    exit 1
fi

echo "✅ Backup directory found at $BACKUP_DIR"
sleep 1

echo ""
echo "[2/8] Copying 3rd-party dependencies..."
# Copy the modified packages into the workspace src directory
cp -r "$BACKUP_DIR/ros2_astra_camera" "$SRC_DIR/" 2>/dev/null || echo "⚠️  ros2_astra_camera not found in backup (maybe already copied?)"
cp -r "$BACKUP_DIR/ydlidar_ros2_driver" "$SRC_DIR/" 2>/dev/null || echo "⚠️  ydlidar_ros2_driver not found in backup"
cp -r "$BACKUP_DIR/YDLidar-SDK" "$SRC_DIR/" 2>/dev/null || echo "⚠️  YDLidar-SDK not found in backup"

# Orbbec requires openni2_redist inside its folder
if [ -d "$BACKUP_DIR/openni2_redist" ]; then
    mkdir -p "$SRC_DIR/ros2_astra_camera/astra_camera/"
    cp -r "$BACKUP_DIR/openni2_redist" "$SRC_DIR/ros2_astra_camera/astra_camera/"
    echo "✅ Copied openni2_redist from backup"
elif [ -f "$SRC_DIR/ros2_astra_camera/astra_camera/scripts/install.sh" ]; then
    echo "⚠️  openni2_redist not found in backup. Running Astra Camera install script..."
    cd "$SRC_DIR/ros2_astra_camera/astra_camera/scripts"
    bash install.sh
    cd "$WS_DIR"
    echo "✅ Downloaded openni2_redist"
else
    echo "❌ ERROR: openni2_redist missing from $BACKUP_DIR and astra_camera install.sh not found!"
    exit 1
fi
echo "✅ Dependencies copied"
sleep 1

echo ""
echo "[3/8] Setting up udev rules..."
# Request sudo once
sudo -v

# YDLidar Rules
echo "  -> Installing YDLidar udev rules..."
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar.rules'
sudo sh -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar-V2.rules'
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"067b\", ATTRS{idProduct}==\"2303\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar-2303.rules'

# Astra Camera Rules
echo "  -> Installing Astra Camera udev rules..."
if [ -f "$SRC_DIR/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" ]; then
    sudo cp "$SRC_DIR/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" /etc/udev/rules.d/
    echo "✅ Copied 56-orbbec-usb.rules"
fi

sudo service udev reload
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
echo "✅ Udev rules applied. (Note: group changes require logout/login to fully apply)"
sleep 1

echo ""
echo "[4/8] Sourcing ROS 2 Environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✅ Found standard ROS 2 Humble"
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/tros/setup.bash" ]; then
    echo "✅ Found Horizon TROS"
    source /opt/tros/setup.bash
else
    echo "❌ ERROR: Neither /opt/ros/humble nor /opt/tros setups were found!"
    exit 1
fi
sleep 1

echo ""
echo "[5/8] Installing system dependencies and pulling LFS..."
echo "  -> Adding ROS 2 keys and repositories..."
sudo apt update
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "  -> Installing APT dependencies..."
sudo apt update
sudo apt install -y git-lfs libgflags-dev ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev \
    libeigen3-dev ros-$ROS_DISTRO-magic-enum python3-pip

echo "  -> Installing Python PIP dependencies (Rosmaster_Lib)..."
if [ -d "$WS_DIR/tools/rosmaster_lib" ]; then
    cd "$WS_DIR/tools/rosmaster_lib"
    sudo python3 setup.py install
    cd "$WS_DIR"
elif [ -d "$SRC_DIR/RISA-bot/tools/rosmaster_lib" ]; then
    cd "$SRC_DIR/RISA-bot/tools/rosmaster_lib"
    sudo python3 setup.py install
    cd "$WS_DIR"
else
    echo "⚠️  rosmaster_lib folder not found. Attempting pip install..."
    pip3 install Rosmaster_Lib || pip3 install --break-system-packages Rosmaster_Lib || true
fi

echo "  -> Setting up Git LFS and pulling..."
if [ -d "$WS_DIR/.git" ]; then
    cd "$WS_DIR"
    git lfs install
    git lfs pull
elif [ -d "$SRC_DIR/RISA-bot" ]; then
    cd "$SRC_DIR/RISA-bot"
    git lfs install
    git lfs pull
elif [ -d "$SRC_DIR/risabot_automode" ]; then
    cd "$SRC_DIR/risabot_automode"
    git lfs install
    git lfs pull
fi

cd "$WS_DIR"
echo "✅ System dependencies installed and Git LFS pulled."
sleep 1

echo ""
echo "[6/8] Building YDLidar-SDK..."
if [ -d "$SRC_DIR/YDLidar-SDK" ]; then
    cd "$SRC_DIR/YDLidar-SDK"
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    echo "✅ YDLidar-SDK built and installed"
else
    echo "❌ ERROR: YDLidar-SDK folder missing!"
    exit 1
fi
sleep 1

echo ""
echo "[7/8] Building ROS 2 Workspace..."
cd "$WS_DIR"

if [ -d "build" ] || [ -d "install" ]; then
    echo "⚠️  Existing build/install folders detected."
    echo "   If colcon fails, cancel this script (Ctrl+C), clean the workspace:"
    echo "   rm -rf ~/risabot_ws/build ~/risabot_ws/install ~/risabot_ws/log"
    echo "   and run this install script again."
    sleep 3
fi

echo "  -> Installing rosdep dependencies..."
sudo apt update
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || echo "⚠️  Some rosdep installations may have been skipped."

echo "  -> colcon build..."
# Use --parallel-workers 1 or 2 and set MAKEFLAGS to avoid OOM killer on low-RAM SBCs
MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential
echo "✅ Workspace built successfully!"

echo "  -> Adding COLCON_IGNORE to core C++ packages to speed up future builds..."
touch "$SRC_DIR/ros2_astra_camera/COLCON_IGNORE"
touch "$SRC_DIR/ydlidar_ros2_driver/COLCON_IGNORE"
echo "✅ Added COLCON_IGNORE to ros2_astra_camera and ydlidar_ros2_driver"
sleep 1

echo ""
echo "[8/8] Setting up ~/.bashrc and ~/.bash_aliases..."
if [ -f "$WS_DIR/tools/install_bashalias.sh" ]; then
    bash "$WS_DIR/tools/install_bashalias.sh"
else
    echo "⚠️  install_bashalias.sh not found. Skipping alias generation."
fi

echo ""
echo "============================================================"
echo "🎉 INSTALLATION COMPLETE! 🎉"
echo "============================================================"
echo "Next steps:"
echo "1. Run 'bash tools/setup_autostart.sh' if you want the robot to launch on boot."
echo "2. REBOOT YOUR ROBOT to apply udev rules and group permissions!"
echo "============================================================"
