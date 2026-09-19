#!/bin/bash
# ============================================================
# setup_mdns.sh — One-time setup for mDNS access to RISA-bot
# ============================================================
# After running this, the dashboard will be accessible at:
#   http://risabot.local:8080
# from any device on the same network (laptop, phone, tablet).
#
# Usage:  bash tools/setup_mdns.sh
# ============================================================

set -e

echo "🔧 Setting up mDNS (Avahi) for RISA-bot..."

# 1. Install avahi-daemon if not present
if ! dpkg -s avahi-daemon &> /dev/null; then
    echo "📦 Installing avahi-daemon..."
    sudo apt-get update -qq
    sudo apt-get install -y avahi-daemon avahi-utils
else
    echo "✅ avahi-daemon already installed"
fi

# 2. Set hostname (defaults to 'risabot' if no argument provided)
TARGET_HOSTNAME="${1:-risabot}"
CURRENT=$(hostname)
if [ "$CURRENT" != "$TARGET_HOSTNAME" ]; then
    echo "📝 Changing hostname from '$CURRENT' to '$TARGET_HOSTNAME'..."
    sudo hostnamectl set-hostname "$TARGET_HOSTNAME"

    # Update /etc/hosts
    if grep -q "$CURRENT" /etc/hosts; then
        sudo sed -i "s/$CURRENT/$TARGET_HOSTNAME/g" /etc/hosts
    fi
    # Add entry if not present
    if ! grep -q "$TARGET_HOSTNAME" /etc/hosts; then
        echo "127.0.1.1  $TARGET_HOSTNAME" | sudo tee -a /etc/hosts > /dev/null
    fi
else
    echo "✅ Hostname already set to '$TARGET_HOSTNAME'"
fi

# 3. Enable and start avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl restart avahi-daemon

echo ""
echo "=============================="
echo "✅ mDNS Setup Complete!"
echo ""
echo "Your DR can now open:"
echo "  http://$TARGET_HOSTNAME.local:8080"
echo ""
echo "From any device on the same WiFi network."
echo "=============================="
echo ""
echo "⚠️  If this is a fresh hostname change, you may need to reboot:"
echo "  sudo reboot"
