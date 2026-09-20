#!/usr/bin/env bash
#
# Record an IMX219 calibration bag on zephyrus (Jetson Orin NX).
#
# Launches the imx219_argus_camera node at 1280x720 @ 60 fps (stock driver,
# no 90Hz device-tree mod required) and records /cam0/image_raw +
# /cam0/camera_info to a rosbag2 directory. CTRL+C to stop.
#
# Usage (on zephyrus, after sourcing the workspace):
#     ./record_bag.sh                    # default: bags/imx219_calib_<timestamp>
#     ./record_bag.sh my_session_name    # bags/my_session_name
#
# Recording guidance for fisheye intrinsics:
#   - Hold the printed checkerboard and slowly move it through the FOV.
#   - Cover ALL regions, including corners (where fisheye distortion is
#     strongest).  Tilt the board to multiple angles (~20-45 deg).
#   - Multiple distances: arm's length down to ~30 cm.
#   - Aim for 60-120 seconds of recording. cv2 will subsample frames.
#   - Avoid motion blur: move slowly. The 1-5ms exposure helps.

set -euo pipefail

SESSION="${1:-imx219_calib_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="bags/${SESSION}"
TOPICS=(/cam0/image_raw /cam0/camera_info)

if [[ -e "$BAG_DIR" ]]; then
    echo "[ERROR] bag dir already exists: $BAG_DIR" >&2
    exit 1
fi
mkdir -p bags

if ! command -v ros2 >/dev/null; then
    echo "[ERROR] ros2 not found. Source your workspace first:" >&2
    echo "    source /opt/ros/humble/setup.bash && source install/setup.bash" >&2
    exit 1
fi

echo "[INFO] launching imx219_argus_camera at 1280x720 @ 60 fps (stock driver)"
# setsid puts the launch in its own session/process group so we can SIGINT
# every child (argus_camera_node, gscam2, ...) cleanly via the negative pid.
setsid ros2 launch imx219_argus_camera isaac_camera.launch.py \
    sensor_mode:=4 framerate:=60 width:=1280 height:=720 \
    > /tmp/imx219_camera.log 2>&1 &
CAMERA_PID=$!
trap '
    echo "[INFO] stopping camera (pgid=$CAMERA_PID)"
    kill -INT -- "-$CAMERA_PID" 2>/dev/null || kill "$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
' EXIT

# Give the camera a moment to come up; bail early if the launch crashed.
sleep 4
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo "[ERROR] camera launch died. Last 20 lines of /tmp/imx219_camera.log:" >&2
    tail -20 /tmp/imx219_camera.log >&2
    exit 1
fi

# Verify the topic is actually publishing — `topic echo --once` exits 0 on
# the first message, non-zero on timeout. (`topic hz` runs forever, so
# `timeout` always returns 124 and the check is meaningless.)
if ! timeout 5 ros2 topic echo --once /cam0/image_raw >/dev/null 2>&1; then
    echo "[ERROR] /cam0/image_raw not publishing 5s after launch. Aborting." >&2
    echo "[ERROR] last 20 lines of /tmp/imx219_camera.log:" >&2
    tail -20 /tmp/imx219_camera.log >&2
    exit 1
fi
echo "[INFO] /cam0/image_raw is live"

echo "[INFO] recording to $BAG_DIR  (CTRL+C to stop)"
echo "[INFO] topics: ${TOPICS[*]}"
ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}"
