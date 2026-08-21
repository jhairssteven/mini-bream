#!/usr/bin/env bash
# Push the ground-station clock to the Pi and Jetson for offline field work.
#
# Without internet, Pi/Jetson clocks drift or boot with the wrong time, which
# breaks ROS message ordering, bag timestamps, and TF (LiDAR vs map→base_link).
# Run this from the ground station after the robot LAN is up and before starting
# stacks. `mini_bream_env.sh start pi|jetson|autonomy` also runs
# preflight_host_clock.sh and refuses to start if the local host is still near
# epoch — sync here first, then start:
#
#   cd src/docker
#   SSHPASS_PI= SSHPASS_JETSON= ./sync_field_time.sh
#
# Options:
#   --pi-only       Sync only the Pi
#   --jetson-only   Sync only the Jetson
#   --dry-run       Show planned actions without changing remote clocks
#   -h, --help      Show help
#
# Environment (defaults match bandwidth_report.sh / mini_bream_sync.sh):
#   PI_IP=192.168.0.101              JETSON_IP=192.168.0.102
#   PI_SSH_USER=pi                   JETSON_SSH_USER=orin-nano
#   SSHPASS_PI=...                   SSHPASS_JETSON=...
#   SUDO_PASS_PI/JETSON=...          Optional; defaults to matching SSHPASS_*
#   SYNC_TIMEZONE=1                  Copy TZ from this host (default: 1)
#   SYNC_HWCLOCK=1                   Write system time to RTC on remotes (default: 1)
#   FAIL_ON_ERROR=0                  Exit 1 if any target fails (default: 0)
#
# SSH keys are preferred when configured. Password auth uses sshpass.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/.env"
fi

PI_IP="${PI_IP:-192.168.0.101}"
JETSON_IP="${JETSON_IP:-192.168.0.102}"
PI_SSH_USER="${PI_SSH_USER:-pi}"
JETSON_SSH_USER="${JETSON_SSH_USER:-orin-nano}"

PI_SSH="${PI_SSH_USER}@${PI_IP}"
JETSON_SSH="${JETSON_SSH_USER}@${JETSON_IP}"

SYNC_TIMEZONE="${SYNC_TIMEZONE:-1}"
SYNC_HWCLOCK="${SYNC_HWCLOCK:-1}"
FAIL_ON_ERROR="${FAIL_ON_ERROR:-0}"
DRY_RUN=0
SYNC_PI=1
SYNC_JETSON=1
FAILURES=0

usage() {
  sed -n '2,27p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

log() { echo "[sync-field-time] $*"; }
die() { echo "[sync-field-time] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pi-only) SYNC_JETSON=0 ;;
    --jetson-only) SYNC_PI=0 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

[[ "${SYNC_PI}" -eq 1 || "${SYNC_JETSON}" -eq 1 ]] || die "nothing to sync (use --pi-only or --jetson-only)"

ssh_with_pass() {
  local pass_var="$1"
  local target="$2"
  shift 2
  local pass="${!pass_var:-}"
  if [[ -n "${pass}" ]] && command -v sshpass >/dev/null 2>&1; then
    sshpass -p "${pass}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${target}" "$@"
  else
    ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${target}" "$@"
  fi
}

local_time_summary() {
  local epoch="$1"
  local tz="$2"
  date -d "@${epoch}" "+%Y-%m-%d %H:%M:%S %Z (%z)" 2>/dev/null \
    || date -r "${epoch}" "+%Y-%m-%d %H:%M:%S %Z" 2>/dev/null \
    || date -u -d "@${epoch}" "+%Y-%m-%d %H:%M:%S UTC"
  echo "  timezone=${tz}"
}

remote_status() {
  local pass_var="$1"
  local target="$2"
  ssh_with_pass "${pass_var}" "${target}" bash -s <<'EOF'
set -euo pipefail
if command -v timedatectl >/dev/null 2>&1; then
  timedatectl status | sed -n '1,4p'
else
  date
fi
EOF
}

sync_host_time() {
  local name="$1"
  local pass_var="$2"
  local sudo_pass_var="$3"
  local ssh_target="$4"
  local epoch="$5"
  local tz="$6"

  log "=== ${name} (${ssh_target}) ==="

  if ! ssh_with_pass "${pass_var}" "${ssh_target}" true 2>/dev/null; then
    log "  SKIP: SSH unreachable or auth failed"
    FAILURES=$((FAILURES + 1))
    return 1
  fi

  log "  Before:"
  remote_status "${pass_var}" "${ssh_target}" | sed 's/^/    /' || true

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would set time to $(local_time_summary "${epoch}" "${tz}" | tr '\n' ' ')"
    return 0
  fi

  local ssh_pass="${!pass_var:-}"
  local sudo_pass="${!sudo_pass_var:-${ssh_pass}}"

  if ! ssh_with_pass "${pass_var}" "${ssh_target}" \
      env \
      SYNC_EPOCH="${epoch}" \
      SYNC_TZ="${tz}" \
      SYNC_TIMEZONE="${SYNC_TIMEZONE}" \
      SYNC_HWCLOCK="${SYNC_HWCLOCK}" \
      SUDO_PASS="${sudo_pass}" \
      bash -s <<'EOF'
set -euo pipefail

run_sudo() {
  if [[ -n "${SUDO_PASS}" ]]; then
    echo "${SUDO_PASS}" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

# Stop NTP clients so they do not fight a manual clock set.
for svc in systemd-timesyncd chrony ntp ntpsec; do
  run_sudo systemctl stop "${svc}" >/dev/null 2>&1 || true
  run_sudo systemctl disable "${svc}" >/dev/null 2>&1 || true
done

# timedatectl is serialized over D-Bus; rapid calls return
# "Previous request is not finished, refusing." Wait between uses.
if command -v timedatectl >/dev/null 2>&1; then
  run_sudo timedatectl set-ntp false >/dev/null 2>&1 || true
  sleep 2
fi

if [[ "${SYNC_TIMEZONE}" == "1" && -f "/usr/share/zoneinfo/${SYNC_TZ}" ]]; then
  run_sudo ln -sf "/usr/share/zoneinfo/${SYNC_TZ}" /etc/localtime
  if [[ -f /etc/timezone ]]; then
    echo "${SYNC_TZ}" | run_sudo tee /etc/timezone >/dev/null
  fi
fi

# date(1) avoids the timedatectl D-Bus queue and is reliable offline.
run_sudo date -s "@${SYNC_EPOCH}"

if [[ "${SYNC_HWCLOCK}" == "1" ]] && command -v hwclock >/dev/null 2>&1; then
  run_sudo hwclock --systohc >/dev/null 2>&1 || true
fi
EOF
  then
    log "  FAIL: could not set clock (check sudo password / permissions)"
    FAILURES=$((FAILURES + 1))
    return 1
  fi

  log "  After:"
  remote_status "${pass_var}" "${ssh_target}" | sed 's/^/    /' || true
  return 0
}

SOURCE_EPOCH="$(date +%s)"
SOURCE_TZ="$(timedatectl show -p Timezone --value 2>/dev/null || date +%Z)"

log "Ground station source clock:"
local_time_summary "${SOURCE_EPOCH}" "${SOURCE_TZ}" | sed 's/^/  /'
echo

if [[ "${SYNC_PI}" -eq 1 ]]; then
  sync_host_time "Pi" SSHPASS_PI SUDO_PASS_PI "${PI_SSH}" "${SOURCE_EPOCH}" "${SOURCE_TZ}" || true
  echo
fi

if [[ "${SYNC_JETSON}" -eq 1 ]]; then
  sync_host_time "Jetson" SSHPASS_JETSON SUDO_PASS_JETSON "${JETSON_SSH}" "${SOURCE_EPOCH}" "${SOURCE_TZ}" || true
  echo
fi

if [[ "${FAILURES}" -gt 0 ]]; then
  log "Finished with ${FAILURES} failure(s)."
  [[ "${FAIL_ON_ERROR}" == "1" ]] && exit 1
  exit 0
fi

log "Done."
