#!/usr/bin/env bash
# kill_stale.sh <patterns_file> [grace_s]
#
# Kills processes left over from an earlier run before the Carbot stack starts
# (Camera_Setup.md: Ctrl+C does not always kill the camera / hobot_codec /
# websocket processes; a stale one makes the next one fail with
# "There are no available host"). One `pgrep -f` pattern per line in
# <patterns_file> ('#' comments allowed). SIGINT, wait grace_s, then SIGKILL.
#
# Runs as root through sudo (stale processes may be root-owned). Never kills itself or
# its ancestors (so the launch that called it survives even though an older
# "ros2 launch carbot_bringup ..." matches the same pattern).
set -u
PATTERNS_FILE="${1:?usage: kill_stale.sh <patterns_file> [grace_s]}"
GRACE="${2:-2.0}"

if [[ ! -r "$PATTERNS_FILE" ]]; then
  echo "[kill_stale] cannot read $PATTERNS_FILE" >&2
  exit 1
fi

declare -A PROTECT
pid=$$
while [[ -n "$pid" && "$pid" != "0" ]]; do
  PROTECT[$pid]=1
  pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
done

collect() {
  local out=() pat p
  while IFS= read -r pat || [[ -n "$pat" ]]; do
    pat="${pat%%#*}"
    pat="$(echo -n "$pat" | xargs)"
    [[ -z "$pat" ]] && continue
    for p in $(pgrep -f -- "$pat" 2>/dev/null); do
      [[ -n "${PROTECT[$p]:-}" ]] && continue
      out+=("$p")
    done
  done < "$PATTERNS_FILE"
  [[ ${#out[@]} -gt 0 ]] && printf '%s\n' "${out[@]}" | sort -u
  return 0
}

PIDS=$(collect)
if [[ -z "$PIDS" ]]; then
  echo "[kill_stale] no stale processes"
  exit 0
fi
echo "[kill_stale] stopping stale processes:"
for p in $PIDS; do
  echo "   $p  $(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-120)"
done
# shellcheck disable=SC2086
kill -INT $PIDS 2>/dev/null || true
sleep "$GRACE"
LEFT=$(collect)
if [[ -n "$LEFT" ]]; then
  echo "[kill_stale] still alive, SIGKILL: $(echo $LEFT)"
  # shellcheck disable=SC2086
  kill -KILL $LEFT 2>/dev/null || true
  sleep 0.5
fi
exit 0
