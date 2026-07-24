#!/usr/bin/env bash
# Rsync mini-bream from this host (Pi) to Jetson or ground station.
#
# Usage:
#   ./mini_bream_sync.sh jetson [--restart] [--build]
#   ./mini_bream_sync.sh gs [--restart]
#   ./mini_bream_sync.sh ground [--restart]
#
# Options:
#   --restart   Restart the role stack on the target after sync
#   --build     Pass --build to mini_bream_env.sh start (with --restart)
#   -n, --dry-run  Show rsync changes without copying
#   -h, --help  Show help
#
# Environment (optional):
#   JETSON_IP=192.168.0.102          JETSON_SSH_USER=orin-nano
#   JETSON_REPO=/home/orin-nano/mini-bream
#   GROUND_IP=192.168.0.103          GROUND_SSH_USER=steven
#   GROUND_REPO=/path/on/ground/to/mini-bream
#   SSHPASS_JETSON=...  SSHPASS_GROUND=...  (password auth via sshpass)
#
# SSH keys are preferred when configured.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_REPO="$(cd "${SCRIPT_DIR}/../.." && pwd)"

JETSON_IP="${JETSON_IP:-192.168.0.102}"
GROUND_IP="${GROUND_IP:-192.168.0.103}"
JETSON_SSH_USER="${JETSON_SSH_USER:-orin-nano}"
GROUND_SSH_USER="${GROUND_SSH_USER:-steven}"
JETSON_REPO="${JETSON_REPO:-/home/orin-nano/mini-bream}"
GROUND_REPO="${GROUND_REPO:-/home/steven/Documents/phd_while_alienware/naslab/LINC project/source code/reflexAI_directory_structure/codebase/mini-bream}"

TARGET=""
RESTART=0
BUILD=0
DRY_RUN=0

RSYNC_EXCLUDES=(
  --exclude '.git/'
  --exclude 'build/'
  --exclude 'install/'
  --exclude 'log/'
  --exclude '__pycache__/'
  --exclude '*.pyc'
  --exclude '.cursor/'
  --exclude 'src/field_tests/rosbags/*.db3'
)

usage() {
  sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

log() { echo "[mini-bream-sync] $*"; }
die() { echo "[mini-bream-sync] ERROR: $*" >&2; exit 1; }

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

rsync_with_pass() {
  local pass_var="$1"
  local target="$2"
  shift 2
  local pass="${!pass_var:-}"
  local -a ssh_cmd=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10)
  if [[ -n "${pass}" ]] && command -v sshpass >/dev/null 2>&1; then
    ssh_cmd=(sshpass -p "${pass}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10)
  fi
  rsync -az --info=stats2 "${RSYNC_EXCLUDES[@]}" -e "${ssh_cmd[*]}" "$@"
}

sync_repo() {
  local pass_var="$1"
  local ssh_target="$2"
  local remote_repo="$3"

  log "Syncing ${LOCAL_REPO}/ -> ${ssh_target}:${remote_repo}/"
  local -a rsync_flags=()
  [[ "${DRY_RUN}" -eq 1 ]] && rsync_flags+=(--dry-run -v)

  rsync_with_pass "${pass_var}" "${ssh_target}" \
    "${rsync_flags[@]}" \
    "${LOCAL_REPO}/" "${ssh_target}:${remote_repo}/"
}

restart_jetson() {
  local pass_var="$1"
  local ssh_target="$2"
  local remote_repo="$3"
  local -a build_args=()
  [[ "${BUILD}" -eq 1 ]] && build_args=(--build)

  log "Restarting Jetson perception stack..."
  ssh_with_pass "${pass_var}" "${ssh_target}" bash -s <<EOF
set -euo pipefail
cd '${remote_repo}/src/docker'
chmod +x mini_bream_env.sh
./mini_bream_env.sh stop jetson
# LiDAR secondary IP is usually already configured on Jetson; avoid sudo over SSH.
ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} docker compose -f docker-compose.frontseat.yml up -d ${build_args[*]} perception
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep mini_bream_perception || true
EOF
}

restart_ground() {
  local pass_var="$1"
  local ssh_target="$2"
  local remote_repo="$3"

  log "Restarting ground station stack (detached)..."
  ssh_with_pass "${pass_var}" "${ssh_target}" bash -s <<EOF
set -euo pipefail
cd '${remote_repo}/src/docker'
chmod +x mini_bream_env.sh
./mini_bream_env.sh stop gs || true
export DISPLAY=\${DISPLAY:-:0}
xhost +local:docker >/dev/null 2>&1 || true
ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} docker compose -f docker-compose.ground.yml up -d --force-recreate ground_station
ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} docker compose -f docker-compose.ground.telemetry.yaml up -d telemetry_tx || true
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep mini_bream || true
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    jetson|gs|ground|ground-station)
      [[ -z "${TARGET}" ]] || die "target already set: ${TARGET}"
      case "$1" in
        ground|ground-station) TARGET=gs ;;
        *) TARGET="$1" ;;
      esac
      ;;
    --restart) RESTART=1 ;;
    --build) BUILD=1 ;;
    -n|--dry-run) DRY_RUN=1 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

[[ -n "${TARGET}" ]] || usage 1
command -v rsync >/dev/null 2>&1 || die "rsync not found"

case "${TARGET}" in
  jetson)
    sync_repo SSHPASS_JETSON "${JETSON_SSH_USER}@${JETSON_IP}" "${JETSON_REPO}"
    [[ "${RESTART}" -eq 1 ]] && restart_jetson SSHPASS_JETSON "${JETSON_SSH_USER}@${JETSON_IP}" "${JETSON_REPO}"
    ;;
  gs)
    sync_repo SSHPASS_GROUND "${GROUND_SSH_USER}@${GROUND_IP}" "${GROUND_REPO}"
    [[ "${RESTART}" -eq 1 ]] && restart_ground SSHPASS_GROUND "${GROUND_SSH_USER}@${GROUND_IP}" "${GROUND_REPO}"
    ;;
  *)
    die "unknown target: ${TARGET}"
    ;;
esac

log "Done (${TARGET})"
