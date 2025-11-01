#!/usr/bin/env bash

# -------------------------------------------------------------
# PX4 SITL + Gazebo Launcher for Custom Drone Model
# -------------------------------------------------------------

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_DIR="$WS_ROOT/sim/gazebo/models"
PX4_DIR="$WS_ROOT/px4"
MODEL_NAME="drone" 


# Add model path to Gazebo resources
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$MODEL_DIR"

export PX4_SIM_MODEL=$MODEL_NAME

echo ""
echo "====================================================="
echo "Launching PX4 SITL with Model: $MODEL_NAME"
echo "Model Search Path: $MODEL_DIR"
echo "====================================================="
echo ""

# Launch PX4 SITL
cd ../..
cd px4/PX4-Autopilot
make px4_sitl gz
