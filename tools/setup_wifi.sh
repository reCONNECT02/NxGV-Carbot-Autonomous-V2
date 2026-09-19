#!/bin/bash
# ============================================================
# RISA-Bot WiFi Setup — Connect to DR's Phone Hotspot
# Pre-configures the robot to auto-connect to a known WiFi.
#
# USAGE (run on the robot ONCE while you have a monitor):
#   sudo bash setup_wifi.sh "DR_HOTSPOT_NAME" "hotspot_password"
#
# After that, the robot is fully headless:
#   1. DR turns on phone hotspot with that exact SSID/password
#   2. Robot auto-connects on boot
#   3. DR opens browser to http://risabot.local:8080
# ============================================================

set -e

SSID="${1:?Usage: sudo bash setup_wifi.sh SSID PASSWORD}"
PASS="${2:?Usage: sudo bash setup_wifi.sh SSID PASSWORD}"

echo "🤖 RISA-Bot WiFi Setup"
echo "======================"
echo "  SSID:     $SSID"
echo "  Password: $PASS"
echo ""

# ---- Method 1: nmcli (Ubuntu / NetworkManager) ----
if command -v nmcli &> /dev/null; then
    echo "📡 Adding WiFi network via NetworkManager..."
    
    # Remove old connection with same name if exists
    nmcli connection delete "$SSID" 2>/dev/null || true
    
    # Add new connection with autoconnect
    nmcli connection add \
        type wifi \
        con-name "$SSID" \
        ifname wlan0 \
        ssid "$SSID" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$PASS" \
        connection.autoconnect yes \
        connection.autoconnect-priority 100
    
    echo ""
    echo "✅ WiFi configured! The robot will auto-connect to '$SSID' on every boot."

# ---- Method 2: wpa_supplicant (Raspberry Pi OS / Debian) ----
elif [ -f /etc/wpa_supplicant/wpa_supplicant.conf ]; then
    echo "📡 Adding WiFi network via wpa_supplicant..."
    
    # Append network block
    cat >> /etc/wpa_supplicant/wpa_supplicant.conf << EOF

network={
    ssid="$SSID"
    psk="$PASS"
    priority=100
}
EOF
    
    # Restart networking
    wpa_cli -i wlan0 reconfigure 2>/dev/null || systemctl restart networking
    
    echo ""
    echo "✅ WiFi configured via wpa_supplicant!"
    
else
    echo "❌ No supported WiFi manager found."
    echo "   Supported: NetworkManager (nmcli) or wpa_supplicant"
    exit 1
fi

# ---- Setup mDNS (risabot.local) ----
echo ""
TARGET_HOSTNAME="${3:-risabot}"
echo "📡 Setting up mDNS ($TARGET_HOSTNAME.local)..."
if ! dpkg -s avahi-daemon > /dev/null 2>&1; then
    apt-get update -qq && apt-get install -y -qq avahi-daemon > /dev/null 2>&1
fi
hostnamectl set-hostname "$TARGET_HOSTNAME" 2>/dev/null || echo "$TARGET_HOSTNAME" > /etc/hostname
systemctl enable avahi-daemon 2>/dev/null || true
systemctl restart avahi-daemon 2>/dev/null || true

echo ""
echo "============================================"
echo "✅ ALL DONE! Instructions for your DR:"
echo "============================================"
echo ""
echo "  1. Turn on phone hotspot:"
echo "     SSID: $SSID"
echo "     Password: $PASS"
echo ""
echo "  2. Connect DR's laptop to the SAME hotspot"
echo ""
echo "  3. Power on the robot (it auto-connects)"
echo ""
echo "  4. Open browser on DR's laptop:"
echo "     http://risabot.local:8080"
echo ""
echo "  If risabot.local doesn't work, find the"
echo "  robot's IP in your phone's hotspot settings"
echo "  and use http://<IP>:8080"
echo "============================================"
