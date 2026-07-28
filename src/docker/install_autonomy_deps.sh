#!/usr/bin/env bash
# Extra Python deps for autonomy / planning experiments (BO tuning, offline-safe).
# Installed at image build time — do not pip install during field tests.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=install_mpc_deps.sh
source "${SCRIPT_DIR}/install_mpc_deps.sh"

install_mpc_python_deps

# scikit-optimize for tune_h0.py / tune_ilos.py; pin NumPy 1.x for tf_transformations + skopt.
pip3 install --no-cache-dir 'numpy<2' scikit-optimize

python3 -c '
import dubins, osqp, scipy, matplotlib, utm, transforms3d, skopt, numpy
assert numpy.__version__.startswith("1."), numpy.__version__
print("autonomy deps ok (numpy", numpy.__version__ + ")")
'
