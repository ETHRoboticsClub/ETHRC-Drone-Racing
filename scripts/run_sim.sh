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

XRCE_AGENT_PID=""
cleanup() {
    if [[ -n "$XRCE_AGENT_PID" ]] && kill -0 "$XRCE_AGENT_PID" 2>/dev/null; then
        kill "$XRCE_AGENT_PID" 2>/dev/null || true
        wait "$XRCE_AGENT_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo ""
echo "====================================================="
echo "Launching PX4 SITL"
echo "  Model : $MODEL"
echo "  World : $WORLD"
echo "====================================================="
echo ""

if command -v MicroXRCEAgent >/dev/null 2>&1; then
    if ! pgrep -f "MicroXRCEAgent.*udp4.*-p 8888" >/dev/null 2>&1; then
        echo "Starting Micro XRCE-DDS Agent on UDP port 8888..."
        MicroXRCEAgent udp4 -p 8888 >/tmp/micro_xrce_agent_8888.log 2>&1 &
        XRCE_AGENT_PID=$!
        sleep 1
        if ! kill -0 "$XRCE_AGENT_PID" 2>/dev/null; then
            echo "ERROR: MicroXRCEAgent exited immediately. See /tmp/micro_xrce_agent_8888.log"
            exit 1
        fi
    else
        echo "Micro XRCE-DDS Agent already running on UDP port 8888."
    fi
else
    cat <<'EOF'
ERROR: MicroXRCEAgent is not installed or not in PATH.

PX4 SITL starts uxrce_dds_client on UDP port 8888, but ROS 2 will not see
/fmu/out/... topics unless the Micro XRCE-DDS Agent is running.

Install eProsima Micro XRCE-DDS Agent, then run again:
  MicroXRCEAgent udp4 -p 8888

EOF
    exit 1
fi

BUILD_NINJA="$PX4_DIR/build/px4_sitl_default/build.ninja"
if [[ -f "$BUILD_NINJA" ]] && ! grep -q "build $MODEL:" "$BUILD_NINJA"; then
    echo "Refreshing PX4 SITL build targets..."
    make px4_sitl
fi

PX4_GZ_WORLD=$WORLD make px4_sitl $MODEL
