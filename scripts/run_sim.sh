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

cd /home/drone_dev/ros_ws/px4/PX4-Autopilot

echo ""
echo "====================================================="
echo "Launching PX4 SITL"
echo "  Model : $MODEL"
echo "  World : $WORLD"
echo "====================================================="
echo ""

PX4_GZ_WORLD=$WORLD make px4_sitl $MODEL
