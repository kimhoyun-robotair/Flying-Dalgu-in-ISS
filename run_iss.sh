#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
export PYTHONDONTWRITEBYTECODE=1
exec "$ISAAC_SIM_DIR/python.sh" "$PROJECT_DIR/scripts/run_sim.py" "$@"
