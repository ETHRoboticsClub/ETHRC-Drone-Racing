#!/usr/bin/env bash
# Run MPCC++ Bayesian optimisation autotuning.
# Execute this script INSIDE the Docker container (docker compose exec mpc bash).
#
# Usage:
#   ./src/mpcc_pp/scripts/run_autotuning.sh [extra optuna args]
#
# Environment overrides (export before calling the script):
#   N_TRIALS   number of Optuna trials        (default: 30)
#   LAPS       laps required per trial        (default: 3)
#   TIMEOUT    max wall-clock seconds/trial   (default: 150)
#   SAMPLER    tpe | gp | random              (default: tpe)
#   OUTPUT_DIR output directory               (default: src/mpcc_pp/mpcc_pp_optuna_runs)
#   STUDY_NAME Optuna study name              (default: mpcc_pp_<timestamp>)
#   PX4_MODEL  Gazebo model target            (default: gz_zephyros)
#   STARTUP_DELAY extra seconds after PX4 ready (default: 1.0)
#   GUI        set to 1 to launch Gazebo GUI    (default: 0)
#   SIM_SPEED_FACTOR faster-than-real-time sim  (default: unset)
#   KILL_EXISTING terminate stale sim processes (default: 1)
#   STREAM     set to 1 to mirror sim stdout  (default: 0)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MPCC_PP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${MPCC_PP_DIR}/../.." && pwd)"

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
    echo "[run_autotuning] ERROR: optuna not found. Rebuild the Docker image or install the mpcc_pp dependencies."
    exit 1
fi

# ── Parameters ────────────────────────────────────────────────────────────────
N_TRIALS="${N_TRIALS:-30}"
LAPS="${LAPS:-3}"
TIMEOUT="${TIMEOUT:-150}"
SAMPLER="${SAMPLER:-tpe}"
OUTPUT_DIR="${OUTPUT_DIR:-${MPCC_PP_DIR}/mpcc_pp_optuna_runs}"
STUDY_NAME="${STUDY_NAME:-mpcc_pp_$(date +%Y%m%d_%H%M%S)}"
PX4_MODEL="${PX4_MODEL:-gz_zephyros}"
STARTUP_DELAY="${STARTUP_DELAY:-1.0}"
GUI="${GUI:-0}"
SIM_SPEED_FACTOR="${SIM_SPEED_FACTOR:-}"
KILL_EXISTING="${KILL_EXISTING:-1}"
STREAM="${STREAM:-0}"

# ── Build the trial runner command ────────────────────────────────────────────
TRIAL_CMD="ros2 run mpcc_pp mpcc_pp_gazebo_trial --px4-model ${PX4_MODEL} --startup-delay-s ${STARTUP_DELAY}"
if [[ "${GUI}" == "1" ]]; then
    TRIAL_CMD="${TRIAL_CMD} --gui"
fi
if [[ -n "${SIM_SPEED_FACTOR}" ]]; then
    TRIAL_CMD="${TRIAL_CMD} --sim-speed-factor ${SIM_SPEED_FACTOR}"
fi
if [[ "${KILL_EXISTING}" != "1" ]]; then
    TRIAL_CMD="${TRIAL_CMD} --keep-existing"
fi
if [[ "${STREAM}" == "1" ]]; then
    TRIAL_CMD="${TRIAL_CMD} --stream-logs"
fi

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
echo "  Gazebo GUI: ${GUI}"
echo "  Startup delay: ${STARTUP_DELAY}s"
echo "  Sim speed factor: ${SIM_SPEED_FACTOR:-default}"
echo "  Kill existing sim: ${KILL_EXISTING}"
echo "  Stream child logs: ${STREAM}"
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
