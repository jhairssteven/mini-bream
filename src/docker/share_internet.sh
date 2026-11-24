#!/usr/bin/env bash
# Temporarily share ground-station Wi‑Fi internet to Pi/Jetson on the robot LAN.
#
# Use when the GS has internet on Wi‑Fi and Ethernet is on an offline LAN
# (192.168.0.0/24). Sets NAT on this host and points remotes at this host as
# gateway + public DNS. Also sets the Ethernet NM profile to
# ipv4.never-default=yes so this host keeps using Wi‑Fi for its own internet.
#
#   cd src/docker
#   ./share_internet.sh          # enable sharing
#   ./share_internet.sh --undo   # remove NAT + restore Ethernet default route
#
# Options:
#   --undo          Remove NAT rules; set Ethernet never-default=no (desk internet)
#   --pi-only       Only configure the Pi
#   --jetson-only   Only configure the Jetson
#   --nat-only      Only enable NAT on this host (skip remotes)
#   --dry-run       Show planned actions without changing anything
#   --verify-only   Only ping/curl remotes (no config changes)
#   -h, --help      Show help
#
# Environment (defaults match sync_field_time.sh / .env):
#   PI_IP=192.168.0.101              JETSON_IP=192.168.0.102
#   PI_SSH_USER=pi                   JETSON_SSH_USER=orin-nano
#   SSHPASS_PI=...                   SSHPASS_JETSON=...
#   SUDO_PASS_PI/JETSON=...          Optional; defaults to matching SSHPASS_*
#   SUDO_PASS_LOCAL / SSHPASS_GROUND Local sudo for iptables (optional)
#   WAN_IFACE=...                    Internet iface (default: default-route iface)
#   LAN_IFACE=...                    Robot LAN iface (default: iface with GROUND_LAN_IP)
#   NM_LAN_CONNECTION=...            NM profile for LAN (default: active on LAN_IFACE)
#   GROUND_LAN_IP=192.168.0.104      Gateway IP remotes should use
#   DNS_SERVERS="8.8.8.8 1.1.1.1"    Written to remote /etc/resolv.conf
#   FAIL_ON_ERROR=0                  Exit 1 if any target fails (default: 0)
#
# Requires: iptables, sudo on this host; ssh(+sshpass) to remotes; nmcli optional.

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
GROUND_LAN_IP="${GROUND_LAN_IP:-192.168.0.104}"
DNS_SERVERS="${DNS_SERVERS:-8.8.8.8 1.1.1.1}"
FAIL_ON_ERROR="${FAIL_ON_ERROR:-0}"

PI_SSH="${PI_SSH_USER}@${PI_IP}"
JETSON_SSH="${JETSON_SSH_USER}@${JETSON_IP}"

DRY_RUN=0
VERIFY_ONLY=0
UNDO=0
DO_NAT=1
DO_PI=1
DO_JETSON=1
FAILURES=0

usage() {
  sed -n '2,34p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

log() { echo "[share-internet] $*"; }
die() { echo "[share-internet] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --undo) UNDO=1; DO_NAT=0; DO_PI=0; DO_JETSON=0 ;;
    --pi-only) DO_JETSON=0 ;;
    --jetson-only) DO_PI=0 ;;
    --nat-only) DO_PI=0; DO_JETSON=0 ;;
    --dry-run) DRY_RUN=1 ;;
    --verify-only) VERIFY_ONLY=1; DO_NAT=0; DO_PI=0; DO_JETSON=0 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

detect_wan_iface() {
  ip route show default 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}'
}

detect_lan_iface() {
  local ip="$1"
  ip -o -4 addr show 2>/dev/null | awk -v ip="${ip}/" '$0 ~ ip {print $2; exit}'
}

detect_ethernet_iface() {
  nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null \
    | awk -F: '$2=="ethernet" && $3 ~ /connected/ {print $1; exit}'
}

detect_nm_connection_for_iface() {
  local iface="$1"
  nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
    | awk -F: -v d="${iface}" '$2==d {print $1; exit}'
}

WAN_IFACE="${WAN_IFACE:-${GROUND_IFACE:-}}"
if [[ -z "${WAN_IFACE}" ]]; then
  WAN_IFACE="$(detect_wan_iface || true)"
fi
LAN_IFACE="${LAN_IFACE:-}"
if [[ -z "${LAN_IFACE}" ]]; then
  LAN_IFACE="$(detect_lan_iface "${GROUND_LAN_IP}" || true)"
fi
if [[ -z "${LAN_IFACE}" ]]; then
  LAN_IFACE="$(detect_ethernet_iface || true)"
fi

NM_LAN_CONNECTION="${NM_LAN_CONNECTION:-}"
if [[ -z "${NM_LAN_CONNECTION}" && -n "${LAN_IFACE}" ]]; then
  NM_LAN_CONNECTION="$(detect_nm_connection_for_iface "${LAN_IFACE}" || true)"
fi
NM_LAN_CONNECTION="${NM_LAN_CONNECTION:-Wired connection 1}"

if [[ "${UNDO}" -eq 0 && "${VERIFY_ONLY}" -eq 0 ]]; then
  [[ -n "${WAN_IFACE}" ]] || die "could not detect WAN_IFACE (set WAN_IFACE= or GROUND_IFACE=)"
  [[ -n "${LAN_IFACE}" ]] || die "could not detect LAN_IFACE for ${GROUND_LAN_IP} (set LAN_IFACE=)"
fi

local_sudo() {
  local pass="${SUDO_PASS_LOCAL:-${SSHPASS_GROUND:-}}"
  if [[ -n "${pass}" ]]; then
    echo "${pass}" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

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

iptables_has() {
  local table="$1"
  shift
  if [[ "${table}" == "filter" ]]; then
    local_sudo iptables -C "$@" >/dev/null 2>&1
  else
    local_sudo iptables -t "${table}" -C "$@" >/dev/null 2>&1
  fi
}

iptables_delete_all() {
  local table="$1"
  shift
  while iptables_has "${table}" "$@"; do
    if [[ "${table}" == "filter" ]]; then
      local_sudo iptables -D "$@" || break
    else
      local_sudo iptables -t "${table}" -D "$@" || break
    fi
  done
}

set_lan_never_default() {
  local value="$1" # yes|no
  if ! command -v nmcli >/dev/null 2>&1; then
    log "  WARN: nmcli missing; skip ipv4.never-default=${value}"
    return 0
  fi
  if [[ -z "${NM_LAN_CONNECTION}" ]]; then
    log "  WARN: no NM LAN connection; skip ipv4.never-default=${value}"
    return 0
  fi

  local current
  current="$(nmcli -g ipv4.never-default connection show "${NM_LAN_CONNECTION}" 2>/dev/null || true)"
  if [[ "${current}" == "${value}" ]]; then
    log "  ${NM_LAN_CONNECTION}: ipv4.never-default already ${value}"
    return 0
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would set '${NM_LAN_CONNECTION}' ipv4.never-default=${value}"
    return 0
  fi

  nmcli connection modify "${NM_LAN_CONNECTION}" ipv4.never-default "${value}"
  nmcli connection up "${NM_LAN_CONNECTION}" >/dev/null
  log "  ${NM_LAN_CONNECTION}: ipv4.never-default=${value} (persisted)"
}

enable_nat() {
  log "=== NAT on this host (${LAN_IFACE} → ${WAN_IFACE}) ==="

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would enable ip_forward and MASQUERADE ${LAN_IFACE}→${WAN_IFACE}"
    set_lan_never_default yes
    return 0
  fi

  # Keep this host on Wi‑Fi for its own internet while Ethernet is robot-LAN only.
  set_lan_never_default yes

  local_sudo sysctl -w net.ipv4.ip_forward=1 >/dev/null

  if ! iptables_has nat POSTROUTING -o "${WAN_IFACE}" -j MASQUERADE; then
    local_sudo iptables -t nat -A POSTROUTING -o "${WAN_IFACE}" -j MASQUERADE
    log "  added MASQUERADE -o ${WAN_IFACE}"
  else
    log "  MASQUERADE already present"
  fi

  if ! iptables_has filter FORWARD -i "${LAN_IFACE}" -o "${WAN_IFACE}" -j ACCEPT; then
    local_sudo iptables -A FORWARD -i "${LAN_IFACE}" -o "${WAN_IFACE}" -j ACCEPT
    log "  added FORWARD ${LAN_IFACE}→${WAN_IFACE}"
  else
    log "  FORWARD ${LAN_IFACE}→${WAN_IFACE} already present"
  fi

  if ! iptables_has filter FORWARD -i "${WAN_IFACE}" -o "${LAN_IFACE}" -m state --state RELATED,ESTABLISHED -j ACCEPT; then
    local_sudo iptables -A FORWARD -i "${WAN_IFACE}" -o "${LAN_IFACE}" -m state --state RELATED,ESTABLISHED -j ACCEPT
    log "  added FORWARD ${WAN_IFACE}→${LAN_IFACE} (established)"
  else
    log "  FORWARD ${WAN_IFACE}→${LAN_IFACE} already present"
  fi
}

undo_share() {
  log "=== undo share on this host ==="

  WAN_IFACE="${WAN_IFACE:-${GROUND_IFACE:-}}"
  if [[ -z "${WAN_IFACE}" ]]; then
    WAN_IFACE="$(detect_wan_iface || true)"
  fi
  LAN_IFACE="${LAN_IFACE:-$(detect_ethernet_iface || true)}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would remove NAT ${LAN_IFACE:-?}→${WAN_IFACE:-?} and set never-default=no"
    set_lan_never_default no
    return 0
  fi

  if [[ -n "${WAN_IFACE}" ]]; then
    iptables_delete_all nat POSTROUTING -o "${WAN_IFACE}" -j MASQUERADE
    log "  removed MASQUERADE -o ${WAN_IFACE} (if any)"
  fi
  if [[ -n "${WAN_IFACE}" && -n "${LAN_IFACE}" ]]; then
    iptables_delete_all filter FORWARD -i "${LAN_IFACE}" -o "${WAN_IFACE}" -j ACCEPT
    iptables_delete_all filter FORWARD -i "${WAN_IFACE}" -o "${LAN_IFACE}" -m state --state RELATED,ESTABLISHED -j ACCEPT
    log "  removed FORWARD ${LAN_IFACE}↔${WAN_IFACE} (if any)"
  fi

  # Desk / normal Ethernet: allow DHCP default gateway again (persists in NM).
  set_lan_never_default no
  log "  Ethernet may use DHCP default route again. Remotes keep old gateway until reboot/DHCP."
}

configure_remote() {
  local name="$1"
  local pass_var="$2"
  local sudo_pass_var="$3"
  local ssh_target="$4"

  log "=== ${name} (${ssh_target}) ==="

  if ! ssh_with_pass "${pass_var}" "${ssh_target}" true 2>/dev/null; then
    log "  SKIP: SSH unreachable or auth failed"
    FAILURES=$((FAILURES + 1))
    return 1
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "  DRY-RUN: would set default via ${GROUND_LAN_IP} and DNS ${DNS_SERVERS}"
    return 0
  fi

  local ssh_pass="${!pass_var:-}"
  local sudo_pass="${!sudo_pass_var:-${ssh_pass}}"
  # Comma-separated so a single argv survives ssh/env word-splitting.
  local dns_csv="${DNS_SERVERS// /,}"

  if ! ssh_with_pass "${pass_var}" "${ssh_target}" \
      bash -s -- "${GROUND_LAN_IP}" "${dns_csv}" "${sudo_pass}" <<'EOF'
set -euo pipefail
GATEWAY="$1"
DNS_CSV="$2"
SUDO_PASS="$3"

# Authenticate once; never pipe file contents into `sudo tee` — on some hosts
# (Jetson) a non-tty stdin makes sudo re-prompt and fail without a terminal.
if [[ -n "${SUDO_PASS}" ]]; then
  if ! echo "${SUDO_PASS}" | sudo -S -v >/dev/null 2>&1; then
    echo "sudo authentication failed" >&2
    exit 1
  fi
  run_sudo() { sudo -n "$@"; }
else
  run_sudo() { sudo "$@"; }
fi

run_sudo ip route replace default via "${GATEWAY}"

resolv=""
IFS=',' read -r -a dns_arr <<< "${DNS_CSV}"
for ns in "${dns_arr[@]}"; do
  [[ -n "${ns}" ]] || continue
  resolv+="nameserver ${ns}"$'\n'
done
tmp="$(mktemp)"
printf '%s' "${resolv}" > "${tmp}"
run_sudo cp "${tmp}" /etc/resolv.conf
rm -f "${tmp}"
EOF
  then
    log "  FAIL: could not set gateway/DNS"
    FAILURES=$((FAILURES + 1))
    return 1
  fi

  log "  gateway → ${GROUND_LAN_IP}, DNS → ${DNS_SERVERS}"
  return 0
}

verify_remote() {
  local name="$1"
  local pass_var="$2"
  local ssh_target="$3"

  log "=== verify ${name} (${ssh_target}) ==="

  if ! ssh_with_pass "${pass_var}" "${ssh_target}" true 2>/dev/null; then
    log "  FAIL: SSH unreachable"
    FAILURES=$((FAILURES + 1))
    return 1
  fi

  if ssh_with_pass "${pass_var}" "${ssh_target}" bash -s <<'EOF'
set -euo pipefail
ping -c 2 -W 3 8.8.8.8 >/dev/null
curl -sI --max-time 8 https://example.com | head -1 | grep -q 'HTTP'
EOF
  then
    log "  OK: ping 8.8.8.8 + HTTPS"
    return 0
  fi

  log "  FAIL: no internet from ${name}"
  FAILURES=$((FAILURES + 1))
  return 1
}

if [[ "${UNDO}" -eq 1 ]]; then
  log "NM_LAN_CONNECTION=${NM_LAN_CONNECTION}  WAN=${WAN_IFACE:-?}  LAN=${LAN_IFACE:-?}"
  echo
  undo_share
  log "Done. Ethernet internet restored (never-default=no persisted)."
  exit 0
fi

log "WAN=${WAN_IFACE}  LAN=${LAN_IFACE}  NM=${NM_LAN_CONNECTION}  gateway_for_remotes=${GROUND_LAN_IP}"
echo

if [[ "${DO_NAT}" -eq 1 ]]; then
  enable_nat
  echo
fi

if [[ "${DO_PI}" -eq 1 ]]; then
  configure_remote "Pi" SSHPASS_PI SUDO_PASS_PI "${PI_SSH}" || true
  echo
fi

if [[ "${DO_JETSON}" -eq 1 ]]; then
  configure_remote "Jetson" SSHPASS_JETSON SUDO_PASS_JETSON "${JETSON_SSH}" || true
  echo
fi

# Verify remotes after configure, or when --verify-only.
VERIFY_PI=0
VERIFY_JETSON=0
if [[ "${VERIFY_ONLY}" -eq 1 ]]; then
  VERIFY_PI=1
  VERIFY_JETSON=1
else
  VERIFY_PI="${DO_PI}"
  VERIFY_JETSON="${DO_JETSON}"
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "DRY-RUN: skipping verify"
elif [[ "${VERIFY_PI}" -eq 1 || "${VERIFY_JETSON}" -eq 1 ]]; then
  [[ "${VERIFY_PI}" -eq 1 ]] && { verify_remote "Pi" SSHPASS_PI "${PI_SSH}" || true; echo; }
  [[ "${VERIFY_JETSON}" -eq 1 ]] && { verify_remote "Jetson" SSHPASS_JETSON "${JETSON_SSH}" || true; echo; }
fi

if [[ "${FAILURES}" -gt 0 ]]; then
  log "Finished with ${FAILURES} failure(s)."
  [[ "${FAIL_ON_ERROR}" == "1" ]] && exit 1
  exit 0
fi

log "Done. When finished sharing, run: ./share_internet.sh --undo"
