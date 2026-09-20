#!/usr/bin/env bash
# A/B helper: rebuild robot_localization at the pre-refactor commit, run
# perception_replay.launch.py with the legacy 15-state EKF config, then
# restore the 16-state quaternion build.
#
# Use AFTER running the replay against the new (16-state) EKF and recording
# its outputs to e.g. out/replay_175153_new/. Drives the script as a
# foreground job — `Ctrl-C` to stop the launch and the script will restore.
#
# This must run INSIDE the dev container (or any environment where colcon
# build works for this workspace). It does NOT touch git remotes.
# `set -u` is omitted on purpose: ROS install/setup.bash references env vars
# (COLCON_TRACE, AMENT_*) that may not be set in a clean shell, and `set -u`
# would abort sourcing before the launch ever runs.
set -eo pipefail

# ---- configurable ---------------------------------------------------------
PRE_REFACTOR_COMMIT="${PRE_REFACTOR_COMMIT:-663436c}"  # parent of a1a6d25
LAUNCH_FILE="${LAUNCH_FILE:-launch/perception_replay.launch.py}"
EKF_CONFIG_FILE="${EKF_CONFIG_FILE:-rl_ekf_legacy.yaml}"
WS_ROOT="${WS_ROOT:-$(pwd)}"
SUBMODULE="src/perception/ekf/robot_localization"
# ---------------------------------------------------------------------------

if [ ! -f "$WS_ROOT/$LAUNCH_FILE" ]; then
    echo "[ERROR] $WS_ROOT/$LAUNCH_FILE not found. Run from workspace root or set WS_ROOT." >&2
    exit 1
fi
if [ ! -d "$WS_ROOT/$SUBMODULE/.git" ] && [ ! -f "$WS_ROOT/$SUBMODULE/.git" ]; then
    echo "[ERROR] $WS_ROOT/$SUBMODULE is not a git submodule." >&2
    exit 1
fi

cd "$WS_ROOT/$SUBMODULE"
ORIGINAL_HEAD="$(git rev-parse HEAD)"
ORIGINAL_REF="$(git rev-parse --abbrev-ref HEAD)"
echo "[INFO] robot_localization was at $ORIGINAL_HEAD (ref: $ORIGINAL_REF)"

# Stash any uncommitted submodule changes so the checkout doesn't refuse;
# pop them on exit. Required after the filter_base.cpp identity-quaternion
# fix until that fix is committed upstream.
STASH_TAG="run_legacy_ekf_replay_autostash_$$"
DID_STASH=0
if ! git diff-index --quiet HEAD -- || \
   [ -n "$(git ls-files --others --exclude-standard)" ]; then
    echo "[INFO] stashing uncommitted submodule changes (tag: $STASH_TAG)"
    git stash push -u -m "$STASH_TAG" >/dev/null
    DID_STASH=1
fi

restore() {
    echo
    echo "[INFO] restoring robot_localization to $ORIGINAL_HEAD ..."
    cd "$WS_ROOT/$SUBMODULE"
    git checkout -q "$ORIGINAL_HEAD"
    if [ "$DID_STASH" -eq 1 ]; then
        local stash_ref
        stash_ref=$(git stash list 2>/dev/null | \
                    awk -F: -v tag="$STASH_TAG" '$0 ~ tag {print $1; exit}')
        if [ -n "$stash_ref" ]; then
            echo "[INFO] popping stashed changes ($stash_ref)"
            git stash pop -q "$stash_ref" || \
                echo "[WARN] git stash pop had conflicts. " \
                     "Stash kept; resolve manually with 'git stash list' inside $SUBMODULE."
        else
            echo "[WARN] could not locate stash tagged $STASH_TAG. " \
                 "Check 'git stash list' inside $SUBMODULE."
        fi
    fi
    cd "$WS_ROOT"
    echo "[INFO] rebuilding 16-state robot_localization ..."
    colcon build --symlink-install --packages-select robot_localization
    echo "[INFO] restored. Re-source install/setup.bash before using normal EKF."
}
trap restore EXIT

cd "$WS_ROOT/$SUBMODULE"
echo "[INFO] checking out pre-refactor $PRE_REFACTOR_COMMIT ..."
git checkout -q "$PRE_REFACTOR_COMMIT"
cd "$WS_ROOT"
echo "[INFO] building 15-state legacy robot_localization ..."
colcon build --symlink-install --packages-select robot_localization
echo "[INFO] sourcing install/setup.bash ..."
# shellcheck disable=SC1091
source install/setup.bash

echo
echo "[OK] legacy EKF build ready. Launching replay pipeline ..."
echo "    config: $EKF_CONFIG_FILE"
echo "    launch: $LAUNCH_FILE"
echo
echo "Now in another terminal:"
echo "    ros2 bag play --clock <bag_dir> \\"
echo "        --remap /cam0/camera_info:=/cam0/_camera_info_bag_unused"
echo "    ros2 bag record -o out/replay_legacy /ekf/odometry"
echo

ros2 launch "$WS_ROOT/$LAUNCH_FILE" ekf_config_file:="$EKF_CONFIG_FILE"
