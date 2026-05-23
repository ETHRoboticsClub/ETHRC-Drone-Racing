#!/usr/bin/env bash

# -------------------------------------------------------------
# PX4 SITL + Gazebo Launcher
#
# Usage:
#   ./run_sim.sh [model] [world]
#
# Examples:
#   ./run_sim.sh                              # zephyros + default (1x)
#   ./run_sim.sh gz_zephyros default_slow     # zephyros + 0.5x speed
#   ./run_sim.sh gz_x500_mono_cam default     # x500 + default (1x)
#
# Available worlds:
#   default       → real_time_factor 1.0
#   default_slow  → real_time_factor 0.5
# -------------------------------------------------------------

MODEL=${1:-gz_zephyros}
WORLD=${2:-default}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="$REPO_ROOT/px4/PX4-Autopilot"

cd "$PX4_DIR"

echo ""
echo "====================================================="
echo "Launching PX4 SITL"
echo "  Model : $MODEL"
echo "  World : $WORLD"
echo "====================================================="
echo ""

BUILD_NINJA="$PX4_DIR/build/px4_sitl_default/build.ninja"
if [[ -f "$BUILD_NINJA" ]] && ! grep -q "build $MODEL:" "$BUILD_NINJA"; then
    echo "Refreshing PX4 SITL build targets..."
    make px4_sitl
fi

PX4_GZ_WORLD=$WORLD make px4_sitl $MODEL
