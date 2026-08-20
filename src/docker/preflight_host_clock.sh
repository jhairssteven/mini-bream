#!/usr/bin/env bash
# Fail if this host clock looks unset (near Unix epoch).
#
# Pi/Jetson clocks that boot at 1970 break ROS TF: GPS/odom on the Pi use wall
# time while LiDAR on the Jetson uses host time. Run sync_field_time.sh from the
# ground station before starting stacks.
#
# Usage:
#   ./preflight_host_clock.sh
#   source ./preflight_host_clock.sh && preflight_host_clock
#
# Environment:
#   SKIP_CLOCK_PREFLIGHT=1   Bypass the check (emergency only)
#   MIN_EPOCH_SECS=1704067200  Earliest accepted time (default: 2024-01-01 UTC)

preflight_host_clock() {
  if [[ "${SKIP_CLOCK_PREFLIGHT:-0}" == "1" ]]; then
    echo "[clock-preflight] SKIP_CLOCK_PREFLIGHT=1 — skipping host clock check" >&2
    return 0
  fi

  local min_epoch="${MIN_EPOCH_SECS:-1704067200}"
  local now
  now="$(date +%s)"
  local human
  human="$(date -u -d "@${now}" '+%Y-%m-%d %H:%M:%S UTC' 2>/dev/null \
    || date -u -r "${now}" '+%Y-%m-%d %H:%M:%S UTC' 2>/dev/null \
    || date -u)"

  if [[ "${now}" -lt "${min_epoch}" ]]; then
    echo "[clock-preflight] ERROR: host clock looks unset (${human}, epoch=${now})." >&2
    echo "[clock-preflight]   ROS TF/lidar stamps will desync across Pi and Jetson." >&2
    echo "[clock-preflight]   From the ground station (robot LAN up):" >&2
    echo "[clock-preflight]     cd src/docker && ./sync_field_time.sh" >&2
    echo "[clock-preflight]   Then re-run start. Override with SKIP_CLOCK_PREFLIGHT=1 only if intentional." >&2
    return 1
  fi

  echo "[clock-preflight] host clock OK (${human})"
  return 0
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  preflight_host_clock
fi
