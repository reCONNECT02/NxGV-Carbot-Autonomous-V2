#!/usr/bin/env bash
# install_root_helpers.sh [user]   (run ONCE on the RDK X5, with sudo)
#
# Stale camera / node processes can be root-owned, but the rest of the stack runs
# as the normal user. This installs a ROOT-OWNED copy of the kill_stale.sh helper into
# /usr/local/lib/carbot and allows exactly that file in sudoers without
# a password. Re-run after changing src/carbot_bringup/scripts/*.sh.
# (run_mipi_cam.sh was removed 2026-09-24 with the MIPI side cameras.)
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
for f in kill_stale.sh; do
  install -o root -g root -m 0755 "$SRC/$f" "$DEST/$f"
  echo "installed $DEST/$f"
done

TMP=$(mktemp)
cat > "$TMP" <<SUDO
# Carbot: only this root-owned helper may be run without a password.
# Installed by install_root_helpers.sh.
$USER_NAME ALL=(root) NOPASSWD: $DEST/kill_stale.sh
SUDO
visudo -cf "$TMP"
install -o root -g root -m 0440 "$TMP" "$SUDOERS"
rm -f "$TMP"
echo "installed $SUDOERS"
echo "test: sudo -n -u $USER_NAME sudo -n $DEST/kill_stale.sh /dev/null 0"
