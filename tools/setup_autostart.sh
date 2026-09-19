#!/bin/bash
# ============================================================
# RISA-Bot Autostart Setup (Tailscale Edition)
# Configures the robot to auto-launch all ROS nodes on boot.
#
# Usage (Run on the robot):
#   sudo bash tools/setup_autostart.sh
# ============================================================

set -e

echo "🤖 RISA-Bot Autostart Setup"
echo "============================"

# ---- 1. Create the startup script ----
echo "📝 Creating startup script..."

cat > /usr/local/bin/risabot-launch.sh << 'LAUNCH_EOF'
#!/bin/bash
# RISA-Bot startup script — called by systemd
set -e

export HOME=/home/sunrise
export USER=sunrise

# Source ROS2 environment (check common paths)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
WS=/home/sunrise/risabotcar_ws
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# Wait for hardware to be ready
sleep 5

# Launch all nodes
echo "[RISABOT] Starting bringup..."
ros2 launch risabot_automode bringup.launch.py &
BRINGUP_PID=$!

sleep 5

# Launch camera
echo "[RISABOT] Starting camera..."
ros2 launch astra_camera astra_mini.launch.py 2>/dev/null &

sleep 3

# Launch servo controller (joystick + motor)
echo "[RISABOT] Starting servo controller..."
ros2 run control_servo servo_controller &

sleep 2

# Launch line follower
echo "[RISABOT] Starting line follower..."
ros2 run risabot_automode line_follower_camera &

sleep 1

# Launch dashboard
echo "[RISABOT] Starting dashboard..."
ros2 run risabot_automode dashboard &

sleep 1

# Launch obstacle avoidance
echo "[RISABOT] Starting obstacle avoidance..."
ros2 run obstacle_avoidance_camera obstacle_avoidance_camera &

sleep 1

# Launch joy node
echo "[RISABOT] Starting joystick..."
ros2 run joy joy_node &

echo "[RISABOT] ✅ All nodes started!"

# Wait for any child to exit (keeps service alive)
wait $BRINGUP_PID
LAUNCH_EOF

chmod +x /usr/local/bin/risabot-launch.sh
echo "  ✅ Startup script created at /usr/local/bin/risabot-launch.sh"

# ---- 2. Create systemd service ----
echo "📝 Creating systemd service..."

cat > /etc/systemd/system/risabot.service << 'SERVICE_EOF'
[Unit]
Description=RISA-Bot ROS2 Autostart
After=network-online.target tailscaled.service
Wants=network-online.target tailscaled.service

[Service]
Type=simple
User=sunrise
Group=sunrise
Environment="HOME=/home/sunrise"
ExecStart=/usr/local/bin/risabot-launch.sh
ExecStop=/bin/bash -c "pkill -f 'ros2' || true"
Restart=on-failure
RestartSec=10
TimeoutStartSec=60

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# Reload systemd and enable
systemctl daemon-reload
systemctl disable risabot.service

echo "  ✅ Service 'risabot' created and enabled"

echo ""
echo "============================================"
echo "✅ AUTOLOAD SETUP COMPLETE!"
echo "============================================"
echo "The robot will now auto-start all ROS nodes whenever it boots."
echo ""
echo "Helpful Commands:"
echo "  Start manually (now):  sudo systemctl start risabot"
echo "  Stop the robot:        sudo systemctl stop risabot"
echo "  Check status:          sudo systemctl status risabot"
echo "  View live logs:        sudo journalctl -u risabot -f"
echo "============================================"
