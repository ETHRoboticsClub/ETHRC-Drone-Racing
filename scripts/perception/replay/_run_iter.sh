#!/usr/bin/env bash
# Helper for replay-iteration runs. Pass tag as $1 (e.g., v5, v6).
set -eo pipefail
TAG="${1:?Usage: $0 <tag>}"
cd /home/drone_dev/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

pkill -KILL -f "vicon_to_odometry|ekf_filter_node|static_transform_publisher" 2>/dev/null || true
sleep 2

OUT_BAG="out/replay_175153_subst_new_fixed_${TAG}"
INPUT_BAG=/home/drone_dev/data_20260416/perception/20260416_175153
rm -rf "$OUT_BAG"

echo "[1/4] launching ..."
ros2 launch launch/perception_replay_vio_subst.launch.py > "out/_launch_${TAG}.log" 2>&1 &
LP=$!
sleep 6
echo "[2/4] recording ..."
ros2 bag record -o "$OUT_BAG" /ekf/odometry /tf /tf_static /ov_msckf/odomimu > "out/_record_${TAG}.log" 2>&1 &
RP=$!
sleep 3
echo "[3/4] playing ..."
ros2 bag play "$INPUT_BAG" --clock --remap /cam0/camera_info:=/cam0/_camera_info_bag_unused > "out/_play_${TAG}.log" 2>&1
sleep 3
echo "[4/4] stopping ..."
kill -INT $RP 2>/dev/null || true
sleep 2
kill -INT $LP 2>/dev/null || true
sleep 3
pkill -KILL -f "vicon_to_odometry|ekf_filter_node|static_transform_publisher" 2>/dev/null || true
kill -KILL $RP $LP 2>/dev/null || true
echo "DONE ${TAG}"
ls -la "$OUT_BAG" | head -3
