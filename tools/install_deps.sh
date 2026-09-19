#!/bin/bash
# =============================================================================
# RISA-bot Deployment Script
# Run this ONCE on a fresh robot after cloning the repo.
# Usage: cd ~/risabotcar_ws && bash tools/install_deps.sh
# =============================================================================

set -e  # Exit on any error

WS_DIR="$HOME/risabotcar_ws"
echo "=============================================="
echo " RISA-bot Dependency Installer"
echo " Workspace: $WS_DIR"
echo "=============================================="

# ------------------------------------------------------------------------------
# 1. Source ROS 2 Humble
# ------------------------------------------------------------------------------
echo ""
echo "[1/7] Setting up ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    # Add to .bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        echo "  Added ROS 2 source to ~/.bashrc"
    fi
else
    echo "  ERROR: ROS 2 Humble not found at /opt/ros/humble"
    echo "  Please install ROS 2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# ------------------------------------------------------------------------------
# 2. Install build tools (rosdep, colcon)
# ------------------------------------------------------------------------------
echo ""
echo "[2/7] Installing build tools (rosdep, colcon)..."
sudo apt update -q
sudo apt install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    libusb-1.0-0-dev \
    libjpeg-dev \
    cmake \
    git \
    build-essential \
    git-lfs \
    ros-humble-joy \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    nlohmann-json3-dev \
    libgflags-dev \
    libgoogle-glog-dev

# Pull git-lfs files (like libOpenNI2 binaries)
cd "$WS_DIR"
git lfs install
git lfs pull

# Init rosdep (skip if already initialized)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ------------------------------------------------------------------------------
# 3. Install YDLidar SDK (required for LiDAR driver)
# ------------------------------------------------------------------------------
echo ""
echo "[3/7] Building and installing YDLidar SDK..."
if [ -d "$WS_DIR/src/YDLidar-SDK" ]; then
    cd "$WS_DIR/src/YDLidar-SDK"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    echo "  YDLidar SDK installed successfully."
else
    echo "  WARNING: YDLidar-SDK not found in src/. Skipping."
fi

# ------------------------------------------------------------------------------
# 4. Install libuvc (required for Astra camera driver)
# ------------------------------------------------------------------------------
echo ""
echo "[4/7] Building and installing libuvc..."
cd /tmp
if [ ! -d "libuvc" ]; then
    git clone https://github.com/libuvc/libuvc.git
fi
cd /tmp/libuvc
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
echo "  libuvc installed successfully."

# ------------------------------------------------------------------------------
# 4.5 Install magic_enum (required for Astra camera driver)
# ------------------------------------------------------------------------------
echo ""
echo "[4.5/7] Building and installing magic_enum..."
cd /tmp
if [ ! -d "magic_enum" ]; then
    git clone https://github.com/Neargye/magic_enum.git
fi
cd /tmp/magic_enum
mkdir -p build && cd build
cmake ..
sudo make install
sudo cp -r /usr/local/include/magic_enum/* /usr/local/include/
echo "  magic_enum installed successfully."

# ------------------------------------------------------------------------------
# 5. Install Rosmaster_Lib (required for motor/servo control)
# ------------------------------------------------------------------------------
echo ""
echo "[5/7] Installing Rosmaster_Lib..."
if [ -d "$WS_DIR/tools/rosmaster_lib" ]; then
    cd "$WS_DIR/tools/rosmaster_lib"
    pip3 install -e . --quiet
    echo "  Rosmaster_Lib installed successfully."
else
    echo "  WARNING: rosmaster_lib not found in tools/. Skipping."
fi

# ------------------------------------------------------------------------------
# 6. Install Astra camera USB rules + rosdep packages
# ------------------------------------------------------------------------------
echo ""
echo "[6/7] Installing Astra camera USB rules and rosdep packages..."
ASTRA_SCRIPTS="$WS_DIR/src/ros2_astra_camera/astra_camera/scripts"
if [ -f "$ASTRA_SCRIPTS/install.sh" ]; then
    sudo bash "$ASTRA_SCRIPTS/install.sh"
fi

# Install RISA-bot custom udev rules (written inline — avoids stale git file issues)
# Motor board: CH340 chip (1a86:7523) -> /dev/myserial
# LiDAR:       Silicon Labs CP2102 (10c4:ea60) -> /dev/ydlidar
echo "  Installing RISA-bot udev rules..."
sudo tee /etc/udev/rules.d/99-risabot.rules > /dev/null << 'UDEV_EOF'
# RISA-bot UDEV Rules
# ==============================================================================
# 1. Rosmaster Motor Driver Board (CH340 chip — Vendor 1a86, Product 7523)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="myserial"
# 2. YDLiDAR Tmini Plus (Silicon Labs CP2102 — Vendor 10c4, Product ea60)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
UDEV_EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  Udev rules installed. Replug USB devices to activate symlinks."


cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y

# ------------------------------------------------------------------------------
# 7. Build workspace
# ------------------------------------------------------------------------------
echo ""
echo "[7/7] Building workspace (this may take a few minutes)..."
cd "$WS_DIR"
colcon build --symlink-install

# Add workspace source to .bashrc
if ! grep -q "source $WS_DIR/install/setup.bash" ~/.bashrc; then
    echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
fi

# Disable FastRTPS shared memory (prevents DDS errors on Sunrise OS)
if ! grep -q "FASTRTPS_DEFAULT_PROFILES_FILE" ~/.bashrc; then
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=$WS_DIR/src/risabot_automode/config/disable_shm.xml" >> ~/.bashrc
fi

# ------------------------------------------------------------------------------
echo ""
echo "=============================================="
echo " RISA-bot setup complete!"
echo " Run: source ~/.bashrc"
echo " Then: ros2 launch risabot_automode bringup.launch.py"
echo "=============================================="
