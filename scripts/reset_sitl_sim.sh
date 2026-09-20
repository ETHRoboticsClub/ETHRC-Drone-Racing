#!/usr/bin/env bash
set -euo pipefail

world_name="${1:-sitl_racing_custom}"
model_name="${2:-drone}"
pose_x="${3:-0}"
pose_y="${4:-0}"
pose_z="${5:-0.5}"
pose_qz="${6:-0}"
pose_qw="${7:-1}"
timeout_ms="${SITL_RESET_TIMEOUT_MS:-2000}"
blocking_service="/world/${world_name}/set_pose/blocking"
request="name: \"${model_name}\" position {x: ${pose_x} y: ${pose_y} z: ${pose_z}} orientation {z: ${pose_qz} w: ${pose_qw}}"

if ! pgrep -af 'gz-sim-main|gz sim .* -s' >/dev/null; then
  echo "[reset_sitl_sim.sh] Gazebo server is not running. Restart racing bringup." >&2
  exit 1
fi

if output="$(
  gz service \
    -s "$blocking_service" \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout "${timeout_ms}" \
    --req "$request" 2>&1
)"; then
  echo "$output"
  if grep -q 'data: true' <<<"$output"; then
    exit 0
  fi

  echo "[reset_sitl_sim.sh] Blocking reset service did not confirm success." >&2
else
  status=$?
  echo "$output" >&2
  echo "[reset_sitl_sim.sh] Blocking reset service failed with exit code ${status}." >&2
fi

echo "[reset_sitl_sim.sh] Gazebo reset failed. Matching services currently visible:" >&2
timeout "$((timeout_ms / 1000 + 1))" gz service -l | grep -E "/world/${world_name}/.*set_pose" >&2 || true
exit 1
