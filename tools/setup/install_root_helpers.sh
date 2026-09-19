#!/usr/bin/env bash
# install_root_helpers.sh [user]   (run ONCE on the RDK X5, with sudo)
#
# mipi_cam must run as root (Camera_Setup.md), but the rest of the stack runs
# as the normal user. This installs ROOT-OWNED copies of the two helpers into
# /usr/local/lib/carbot and allows exactly those two files in sudoers without
# a password. Re-run after changing src/carbot_bringup/scripts/*.sh.
#
#   sudo bash tools/setup/install_root_helpers.sh sunrise
set -euo pipefail
USER_NAME="${1:-sunrise}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/../../src/carbot_bringup/scripts"
DEST=/usr/local/lib/carbot
SUDOERS=/etc/sudoers.d/carbot

if [[ "$(id -u)" != "0" ]]; then
  echo "run with sudo: sudo bash $0 $USER_NAME" >&2; exit 1
fi
id "$USER_NAME" >/dev/null

install -d -o root -g root -m 0755 "$DEST"
for f in kill_stale.sh run_mipi_cam.sh; do
  install -o root -g root -m 0755 "$SRC/$f" "$DEST/$f"
  echo "installed $DEST/$f"
done

TMP=$(mktemp)
cat > "$TMP" <<SUDO
# Carbot: mipi_cam must run as root (Camera_Setup.md). Only these root-owned
# helpers may be run without a password. Installed by install_root_helpers.sh.
$USER_NAME ALL=(root) NOPASSWD: $DEST/kill_stale.sh, $DEST/run_mipi_cam.sh
SUDO
visudo -cf "$TMP"
install -o root -g root -m 0440 "$TMP" "$SUDOERS"
rm -f "$TMP"
echo "installed $SUDOERS"
echo "test: sudo -n -u $USER_NAME sudo -n $DEST/kill_stale.sh /dev/null 0"
