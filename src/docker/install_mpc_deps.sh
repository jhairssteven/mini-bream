#!/usr/bin/env bash
# MPC Python packages for molo_wpt_follower (H0 boat experiments).
# Baked into images via install_ros2_ws_deps.sh; this file can also be run standalone.
set -euo pipefail

install_mpc_python_deps() {
  pip3 install --no-cache-dir utm 'transforms3d>=0.4.2' osqp scipy matplotlib \
    'git+https://github.com/rdesc/pydubins.git@58c4b68d9ddf972dfd32afea295c3195da648cf2'
  python3 -c 'import dubins, osqp, scipy, numpy, matplotlib, utm, transforms3d; print("mpc deps ok")'
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  apt-get update
  apt-get install -y --no-install-recommends gcc g++ git python3-pip
  rm -rf /var/lib/apt/lists/*
  install_mpc_python_deps
fi
