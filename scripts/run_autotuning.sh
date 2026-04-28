#!/usr/bin/env bash
# Run MPCC++ Bayesian optimisation autotuning.
# Execute this script INSIDE the Docker container (docker compose exec mpc bash).
#
# Usage:
#   ./scripts/run_autotuning.sh [extra optuna args]
#
# Environment overrides (export before calling the script):
#   N_TRIALS   number of Optuna trials        (default: 30)
#   LAPS       laps required per trial        (default: 3)
#   TIMEOUT    max wall-clock seconds/trial   (default: 150)
#   SAMPLER    tpe | gp | random              (default: tpe)
#   OUTPUT_DIR output directory               (default: ~/mpcc_pp_optuna_runs)
#   STUDY_NAME Optuna study name              (default: mpcc_pp_<timestamp>)
#   PX4_MODEL  Gazebo model target            (default: gz_zephyros)
#   STREAM     set to 1 to mirror sim stdout  (default: 0)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Source ROS + colcon ────────────────────────────────────────────────────────
if [[ -f /opt/ros/humble/setup.bash ]]; then
    # ROS setup scripts use unbound variables internally; disable -u around them.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[run_autotuning] WARNING: /opt/ros/humble/setup.bash not found. Continuing anyway."
fi

INSTALL_SETUP="${WS_DIR}/install/setup.bash"
if [[ -f "${INSTALL_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${INSTALL_SETUP}"
    set -u
else
    echo "[run_autotuning] ERROR: ${INSTALL_SETUP} not found. Build the workspace first."
    exit 1
fi

# ── Check optuna is available ──────────────────────────────────────────────────
if ! python3 -c "import optuna" 2>/dev/null; then
    echo "[run_autotuning] optuna not found. Installing..."
    pip3 install optuna
fi

# ── Parameters ────────────────────────────────────────────────────────────────
N_TRIALS="${N_TRIALS:-30}"
LAPS="${LAPS:-3}"
TIMEOUT="${TIMEOUT:-150}"
SAMPLER="${SAMPLER:-tpe}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/mpcc_pp_optuna_runs}"
STUDY_NAME="${STUDY_NAME:-mpcc_pp_$(date +%Y%m%d_%H%M%S)}"
PX4_MODEL="${PX4_MODEL:-gz_zephyros}"
STREAM="${STREAM:-0}"

# ── Build the trial runner command ────────────────────────────────────────────
TRIAL_CMD="python3 -m mpcc_pp.tuning.gazebo_trial_runner --px4-model ${PX4_MODEL}"

# ── Run ───────────────────────────────────────────────────────────────────────
echo ""
echo "=== MPCC++ Optuna Autotuning ==="
echo "  Study:    ${STUDY_NAME}"
echo "  Trials:   ${N_TRIALS}"
echo "  Laps:     ${LAPS} per trial"
echo "  Timeout:  ${TIMEOUT}s per trial"
echo "  Sampler:  ${SAMPLER}"
echo "  Output:   ${OUTPUT_DIR}"
echo "  PX4 model: ${PX4_MODEL}"
echo "================================"
echo ""

STREAM_FLAG=""
[[ "${STREAM}" == "1" ]] && STREAM_FLAG="--stream-logs"

python3 -m mpcc_pp.tuning.optuna_tuner \
    --study-name        "${STUDY_NAME}" \
    --n-trials          "${N_TRIALS}" \
    --laps-per-episode  "${LAPS}" \
    --timeout-s         "${TIMEOUT}" \
    --sampler           "${SAMPLER}" \
    --output-dir        "${OUTPUT_DIR}" \
    --command           "${TRIAL_CMD}" \
    ${STREAM_FLAG} \
    "$@"
