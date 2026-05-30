#!/usr/bin/env bash
# Pipeline: raw bag -> filtered bag -> preprocessed pickle -> trajectory JSON.
#
# Layout produced (relative to bags/):
#   filtered/<name>/        ros2 bag with only /vicon/zephyrus/zephyrus + /imu0
#   processed/<name>_traj.pkl
#   processed/<name>_traj.json
#
# Usage:  bag_to_json.sh <bag_dir> [output.json]
set -eo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <bag_dir> [output.json]" >&2
    exit 1
fi

BAG="$(realpath "$1")"
# If a .db3 file was passed, use its parent directory (ros2 bag expects the dir).
if [[ -f "$BAG" && "$BAG" == *.db3 ]]; then
    BAG="$(dirname "$BAG")"
fi
NAME="$(basename "$BAG")"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

FILTERED="${BAGS_DIR}/filtered/${NAME}"
PKL="${BAGS_DIR}/processed/${NAME}_traj.pkl"
JSON="${2:-${BAGS_DIR}/processed/${NAME}_traj.json}"

mkdir -p "${BAGS_DIR}/filtered" "${BAGS_DIR}/processed"

set +u
source /opt/ros/jazzy/setup.bash
set -u

python3 "${SCRIPT_DIR}/preprocess_bag.py" "$BAG" \
    --out-bag "$FILTERED" \
    --out-pkl "$PKL"
python3 "${SCRIPT_DIR}/to_json.py" "$PKL" "$JSON"

echo "done: $JSON"
