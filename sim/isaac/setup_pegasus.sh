#!/usr/bin/env bash
# =============================================================================
# Setup script for Pegasus Simulator + Isaac Sim
#
# Prerequisites:
#   - Ubuntu 22.04
#   - NVIDIA GPU with driver >= 550
#   - Isaac Sim 5.1.0 installed (standalone)
#   - PX4-Autopilot built for SITL
#
# Usage:
#   ./setup_pegasus.sh [--isaacsim-path PATH] [--px4-dir PATH]
# =============================================================================
set -euo pipefail

# --- Defaults ---
ISAACSIM_PATH="${ISAACSIM_PATH:-${HOME}/isaacsim}"
PX4_DIR="${PX4_DIR:-$(cd "$(dirname "$0")/../../px4/PX4-Autopilot" 2>/dev/null && pwd || echo "${HOME}/ros_ws/px4/PX4-Autopilot")}"
PEGASUS_DIR="${PEGASUS_DIR:-$(cd "$(dirname "$0")/../../" && pwd)/PegasusSimulator}"

# --- Parse args ---
while [[ $# -gt 0 ]]; do
    case $1 in
        --isaacsim-path) ISAACSIM_PATH="$2"; shift 2 ;;
        --px4-dir) PX4_DIR="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

echo "============================================"
echo "  Pegasus Simulator Setup"
echo "============================================"
echo "  Isaac Sim:  ${ISAACSIM_PATH}"
echo "  PX4 dir:    ${PX4_DIR}"
echo "  Pegasus:    ${PEGASUS_DIR}"
echo "============================================"

# --- 1. Verify Isaac Sim ---
if [[ ! -f "${ISAACSIM_PYTHON}" ]]; then
    echo ""
    echo "ERROR: Isaac Sim not found at ${ISAACSIM_PATH}"
    echo ""
    echo "Install Isaac Sim 5.1.0 (standalone):"
    echo "  cd ~ && mkdir -p isaacsim && cd isaacsim"
    echo "  wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip"
    echo "  unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip"
    echo "  ./post_install.sh"
    echo ""
    exit 1
fi
echo "[OK] Isaac Sim found"

# --- 2. Verify PX4 SITL build ---
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
if [[ ! -f "${PX4_BIN}" ]]; then
    echo ""
    echo "PX4 SITL not built. Building now..."
    echo "  cd ${PX4_DIR} && make px4_sitl_default none"
    (cd "${PX4_DIR}" && make px4_sitl_default none)
fi
echo "[OK] PX4 SITL build found"

# --- 3. Clone / update Pegasus ---
if [[ ! -d "${PEGASUS_DIR}" ]]; then
    echo "Cloning PegasusSimulator..."
    git clone https://github.com/PegasusSimulator/PegasusSimulator.git "${PEGASUS_DIR}"
else
    echo "Updating PegasusSimulator..."
    (cd "${PEGASUS_DIR}" && git pull --ff-only || true)
fi
echo "[OK] PegasusSimulator at ${PEGASUS_DIR}"

# --- 4. Install Pegasus as editable package in Isaac Sim's Python ---
echo "Installing Pegasus into Isaac Sim Python environment..."
"${ISAACSIM_PYTHON}" -m pip install --editable "${PEGASUS_DIR}/extensions/pegasus.simulator"
echo "[OK] Pegasus installed"

# --- 5. Configure PX4 path in Pegasus ---
PEGASUS_CONFIG="${PEGASUS_DIR}/extensions/pegasus.simulator/config/configs.yaml"
if [[ -f "${PEGASUS_CONFIG}" ]]; then
    # Update px4_dir in configs.yaml
    sed -i "s|px4_dir:.*|px4_dir: \"${PX4_DIR}\"|" "${PEGASUS_CONFIG}"
    echo "[OK] Pegasus config updated with PX4 path"
fi

# --- 6. Add env vars to .bashrc if not present ---
BASHRC="${HOME}/.bashrc"
if ! grep -q "ISAACSIM_PATH" "${BASHRC}" 2>/dev/null; then
    cat >> "${BASHRC}" << EOF

# === Isaac Sim + Pegasus ===
export ISAACSIM_PATH="${ISAACSIM_PATH}"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"
export PX4_DIR="${PX4_DIR}"
EOF
    echo "[OK] Environment variables added to ~/.bashrc"
else
    echo "[SKIP] Environment variables already in ~/.bashrc"
fi

echo ""
echo "============================================"
echo "  Setup complete!"
echo ""
echo "  To launch PX4 in Isaac Sim:"
echo "    ${ISAACSIM_PYTHON} sim/isaac/launch_px4_isaacsim.py"
echo ""
echo "  Then connect your ROS 2 / MAVSDK nodes as usual."
echo "============================================"
