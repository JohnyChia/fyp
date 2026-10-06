#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python="$workspace/.venv/bin/python"

if [[ ! -x "$python" ]]; then
  echo "missing workspace interpreter: $python" >&2
  exit 1
fi

export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
unset PYTEST_ADDOPTS

cd "$workspace"
exec "$python" -m pytest -q -p no:cacheprovider \
  src/durian_inspection_pkg/test/test_association_calibration.py \
  src/durian_inspection_pkg/test/test_navigation_stability.py \
  src/durian_inspection_pkg/test/test_runtime_resilience.py \
  src/durian_inspection_pkg/test/test_same_input_benchmark.py
