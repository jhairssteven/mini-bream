#!/usr/bin/env bash
# Keep C2, the Pi, and the Jetson on one clock with chrony.
#
# A one-shot `date -s` over SSH is seconds late (the epoch is sampled, then
# SSH, sudo, and a sequential second host all run before the clock is set)
# and it used to disable NTP afterwards, so the error stuck. chrony measures
# the round trip and keeps correcting.
#
# Roles:
#   C2      NTP server. Uses public NTP when this host has internet, otherwise
#           its own clock (local stratum 8 in chrony.conf). Addresses:
#           192.168.0.104 bench Ethernet, 192.168.0.103 field Wi-Fi.
#   Jetson  Client of C2. If C2 leaves radio range, serves the boat clock so
#           the Pi does not free-run alone.
#   Pi      Client of the Jetson (always-on Ethernet). C2 is a fallback.
#
# Run from the ground station once the robot LAN is up, before starting stacks:
#
#   cd src/docker
#   ./sync_field_time.sh
#   ./sync_field_time.sh --check
#
# Options:
#   --pi-only       Skip the Jetson (C2 is still configured)
#   --jetson-only   Skip the Pi (C2 is still configured)
#   --check         Print chrony status; change nothing
#   --dry-run       Show planned actions
#   -h, --help      Show help
#
# Environment (defaults match bandwidth_report.sh / mini_bream_sync.sh):
#   PI_IP=192.168.0.101              JETSON_IP=192.168.0.102
#   PI_SSH_USER=pi                   JETSON_SSH_USER=orin-nano
#   SSHPASS_PI=...                   SSHPASS_JETSON=...
#   SUDO_PASS_PI/JETSON=...          Optional; defaults to matching SSHPASS_*
#   SUDO_PASS_LOCAL / SSHPASS_GROUND Local sudo for the C2 chrony drop-in
#   SYNC_TIMEZONE=1                  Copy TZ from this host (default: 1)
#   FAIL_ON_ERROR=0                  Exit 1 if any target fails (default: 0)
#
# Exits if chrony is not installed on a host. Install once while it has internet:
#   sudo apt-get install -y chrony
#
# SSH keys are preferred when configured. Password auth uses sshpass.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHRONY_DIR="${SCRIPT_DIR}/chrony"
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
FAIL_ON_ERROR="${FAIL_ON_ERROR:-0}"
DRY_RUN=0
CHECK_ONLY=0
SYNC_PI=1
SYNC_JETSON=1
DO_LOCAL=1
FAILURES=0

usage() {
  sed -n '2,42p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

log() { echo "[sync-field-time] $*"; }
die() { echo "[sync-field-time] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pi-only) SYNC_JETSON=0 ;;
    --jetson-only) SYNC_PI=0 ;;
    --check) CHECK_ONLY=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

[[ "${SYNC_PI}" -eq 1 || "${SYNC_JETSON}" -eq 1 || "${DO_LOCAL}" -eq 1 ]] || die "nothing to sync"

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

scp_with_pass() {
  local pass_var="$1"
  local src="$2"
  local target="$3"
  local dest="$4"
  local pass="${!pass_var:-}"
  if [[ -n "${pass}" ]] && command -v sshpass >/dev/null 2>&1; then
    sshpass -p "${pass}" scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${src}" "${target}:${dest}"
  else
    scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${src}" "${target}:${dest}"
  fi
}

run_local_sudo() {
  if sudo -n true 2>/dev/null; then
    sudo "$@"
  else
    local pass="${SUDO_PASS_LOCAL:-${SSHPASS_GROUND:-}}"
    if [[ -z "${pass}" ]]; then
      die "local sudo needs a password (set SUDO_PASS_LOCAL or SSHPASS_GROUND)"
    fi
    echo "${pass}" | sudo -S -p '' "$@"
  fi
}

remote_ok() {
  local pass_var="$1"
  local target="$2"
  ssh_with_pass "${pass_var}" "${target}" true 2>/dev/null
}

print_remote_status() {
  local pass_var="$1"
  local target="$2"
  ssh_with_pass "${pass_var}" "${target}" bash -s <<'EOF' || true
echo "  timedatectl:"
timedatectl 2>/dev/null | sed -n '1,6p' | sed 's/^/    /' || date | sed 's/^/    /'
if command -v chronyc >/dev/null 2>&1; then
  echo "  chronyc tracking:"
  chronyc -n tracking 2>/dev/null | sed 's/^/    /' || echo "    (chronyc tracking failed)"
  echo "  chronyc sources:"
  chronyc -n sources 2>/dev/null | sed 's/^/    /' || echo "    (chronyc sources failed)"
else
  echo "  chrony: not installed"
fi
EOF
}

require_chrony() {
  local name="$1"
  local pass_var="$2"
  local target="$3"
  if ssh_with_pass "${pass_var}" "${target}" command -v chronyd >/dev/null 2>&1; then
    return 0
  fi
  echo "[sync-field-time] WARNING: chrony is not installed on ${name}." >&2
  echo "[sync-field-time] WARNING: install it once while that host has internet, then re-run:" >&2
  echo "[sync-field-time] WARNING:   sudo apt-get install -y chrony" >&2
  exit 1
}

push_chrony_conf() {
  local pass_var="$1"
  local sudo_pass_var="$2"
  local target="$3"
  local conf="$4"
  local sudo_pass="${!sudo_pass_var:-}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would install ${conf} as /etc/chrony/chrony.conf"
    return 0
  fi

  scp_with_pass "${pass_var}" "${conf}" "${target}" "/tmp/mini-bream-chrony.conf"
  ssh_with_pass "${pass_var}" "${target}" \
    env SUDO_PASS="${sudo_pass}" SYNC_TIMEZONE="${SYNC_TIMEZONE}" SYNC_TZ="${SOURCE_TZ}" bash -s <<'EOF'
set -euo pipefail
run_sudo() {
  if [[ -n "${SUDO_PASS}" ]]; then
    echo "${SUDO_PASS}" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}
run_sudo cp /tmp/mini-bream-chrony.conf /etc/chrony/chrony.conf
run_sudo chmod 644 /etc/chrony/chrony.conf
rm -f /tmp/mini-bream-chrony.conf
if [[ "${SYNC_TIMEZONE}" == "1" && -n "${SYNC_TZ}" && -f "/usr/share/zoneinfo/${SYNC_TZ}" ]]; then
  run_sudo ln -sf "/usr/share/zoneinfo/${SYNC_TZ}" /etc/localtime
  if [[ -f /etc/timezone ]]; then
    echo "${SYNC_TZ}" | run_sudo tee /etc/timezone >/dev/null
  fi
fi
run_sudo systemctl enable chrony
run_sudo systemctl restart chrony
EOF
}

wait_and_step() {
  local pass_var="$1"
  local sudo_pass_var="$2"
  local target="$3"
  local sudo_pass="${!sudo_pass_var:-}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    return 0
  fi

  ssh_with_pass "${pass_var}" "${target}" \
    env SUDO_PASS="${sudo_pass}" bash -s <<'EOF'
set -euo pipefail
run_sudo() {
  if [[ -n "${SUDO_PASS}" ]]; then
    echo "${SUDO_PASS}" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}
selected=0
for _ in $(seq 1 40); do
  if chronyc -n sources 2>/dev/null | grep -q '^\^\*'; then
    selected=1
    break
  fi
  sleep 0.5
done
if [[ "${selected}" -ne 1 ]]; then
  echo "NO_SOURCE" >&2
  chronyc -n sources >&2 || true
  exit 1
fi
run_sudo chronyc makestep >/dev/null
if ! chronyc waitsync 20 0.02 0 0.5; then
  echo "WAITSYNC_TIMEOUT" >&2
  chronyc -n tracking >&2 || true
  exit 1
fi
EOF
}

configure_local_server() {
  local dropin="/etc/chrony/conf.d/mini-bream.conf"
  log "=== C2 (local NTP server) ==="
  if [[ "${CHECK_ONLY}" -eq 1 ]]; then
    chronyc -n tracking 2>/dev/null | sed 's/^/  /' || log "  chronyc tracking failed"
    if command -v chronyc >/dev/null 2>&1; then
      log "  access for ${PI_IP}: $(chronyc accheck "${PI_IP}" 2>/dev/null || echo 'unknown')"
    fi
    echo
    return 0
  fi
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would install ${CHRONY_DIR}/c2.conf as ${dropin} and allow 192.168.0.0/24"
    echo
    return 0
  fi
  if ! command -v chronyd >/dev/null 2>&1; then
    echo "[sync-field-time] WARNING: chrony is not installed on this ground station." >&2
    echo "[sync-field-time] WARNING: install it once, then re-run:" >&2
    echo "[sync-field-time] WARNING:   sudo apt-get install -y chrony" >&2
    exit 1
  fi
  if ! run_local_sudo cp "${CHRONY_DIR}/c2.conf" "${dropin}" \
      || ! run_local_sudo chmod 644 "${dropin}" \
      || ! { run_local_sudo systemctl reload chrony 2>/dev/null \
             || run_local_sudo systemctl restart chrony; } \
      || ! run_local_sudo chronyc allow 192.168.0.0/24 >/dev/null; then
    log "  FAIL: could not configure local chrony (sudo?)"
    FAILURES=$((FAILURES + 1))
    echo
    return 1
  fi
  local access
  access="$(run_local_sudo chronyc accheck "${PI_IP}" 2>/dev/null || true)"
  log "  serving 192.168.0.0/24 (${access:-accheck failed})"
  chronyc -n tracking 2>/dev/null | sed -n '1,4p' | sed 's/^/  /' || true
  echo
}

configure_remote() {
  local name="$1"
  local pass_var="$2"
  local sudo_pass_var="$3"
  local target="$4"
  local conf="$5"

  log "=== ${name} (${target}) ==="
  if ! remote_ok "${pass_var}" "${target}"; then
    log "  SKIP: SSH unreachable or auth failed"
    FAILURES=$((FAILURES + 1))
    echo
    return 1
  fi

  if [[ "${CHECK_ONLY}" -eq 1 ]]; then
    print_remote_status "${pass_var}" "${target}"
    echo
    return 0
  fi

  local sudo_pass="${!sudo_pass_var-}"
  if [[ -z "${sudo_pass}" ]]; then
    sudo_pass="${!pass_var-}"
  fi
  # shellcheck disable=SC2034
  printf -v "${sudo_pass_var}" '%s' "${sudo_pass}"

  require_chrony "${name}" "${pass_var}" "${target}"

  log "  installing $(basename "${conf}") and restarting chrony"
  if ! push_chrony_conf "${pass_var}" "${sudo_pass_var}" "${target}" "${conf}"; then
    log "  FAIL: could not install chrony config"
    FAILURES=$((FAILURES + 1))
    echo
    return 1
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo
    return 0
  fi

  log "  waiting for a selected source, then stepping"
  if ! wait_and_step "${pass_var}" "${sudo_pass_var}" "${target}"; then
    log "  FAIL: chrony did not lock within ~20s"
    print_remote_status "${pass_var}" "${target}"
    FAILURES=$((FAILURES + 1))
    echo
    return 1
  fi

  log "  locked:"
  print_remote_status "${pass_var}" "${target}"
  echo
}

SOURCE_TZ="$(timedatectl show -p Timezone --value 2>/dev/null || date +%Z)"

if [[ "${CHECK_ONLY}" -eq 1 ]]; then
  log "Status only."
else
  log "Timezone on this host: ${SOURCE_TZ}"
  log "Stepping robot clocks to the ground station. Do this before starting ROS stacks."
fi
echo

if [[ "${DO_LOCAL}" -eq 1 ]]; then
  configure_local_server || true
fi

# Jetson first. The Pi prefers it, so the Jetson must already be on C2 time
# or the Pi will step to a stale boat clock and then refuse another step.
if [[ "${SYNC_JETSON}" -eq 1 ]]; then
  configure_remote "Jetson" SSHPASS_JETSON SUDO_PASS_JETSON "${JETSON_SSH}" "${CHRONY_DIR}/jetson.conf" || true
fi

if [[ "${SYNC_PI}" -eq 1 ]]; then
  configure_remote "Pi" SSHPASS_PI SUDO_PASS_PI "${PI_SSH}" "${CHRONY_DIR}/pi.conf" || true
fi

if [[ "${FAILURES}" -gt 0 ]]; then
  log "Finished with ${FAILURES} failure(s)."
  [[ "${FAIL_ON_ERROR}" == "1" ]] && exit 1
  exit 0
fi

log "Done. Pi and Jetson stay locked if this host leaves Wi-Fi range; they pick C2 back up when it returns."
