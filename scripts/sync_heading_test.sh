#!/usr/bin/env bash
# Sync mini-bream workspace to Pi, Jetson, and c2 ground station.
set -euo pipefail

LOCAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${LOCAL_ROOT}/src"

sync_host() {
  local host="$1" user="$2" pass="$3" remote="$4"
  echo "=== Syncing to ${user}@${host} ==="
  python3 - "$host" "$user" "$pass" "$remote" "$SRC" <<'PY'
import sys, pexpect, os
host, user, pw, remote, src = sys.argv[1:6]
pairs = [
    (os.path.join(src, "ros2_ws/src/frontseat"), f"{remote}/src/ros2_ws/src/frontseat/"),
    (os.path.join(src, "docker"), f"{remote}/src/docker/"),
]
for local, dest in pairs:
    cmd = (
        f"rsync -az --exclude '__pycache__' -e 'ssh -o StrictHostKeyChecking=accept-new' "
        f"'{local}/' {user}@{host}:{dest}"
    )
    c = pexpect.spawn(cmd, timeout=300, encoding='utf-8')
    i = c.expect(['password:', pexpect.EOF, pexpect.TIMEOUT])
    if i == 0:
        c.sendline(pw)
        c.expect(pexpect.EOF, timeout=300)
    rc = c.wait()
    print(f"  {os.path.basename(local)} -> exit {rc}")
PY
}

sync_host 192.168.0.100 pi blue-bream /home/pi/mini-bream
sync_host 192.168.0.102 orin-nano 1 /home/orin-nano/mini-bream
sync_host 192.168.0.104 c2 1234 /home/c2/linc_ws/mini-bream

echo "Sync complete."
