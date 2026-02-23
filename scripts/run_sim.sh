#!/usr/bin/env bash

# -------------------------------------------------------------
# PX4 SITL + Gazebo Launcher for Custom Drone Model
# -------------------------------------------------------------

# Navigate to PX4 directory (correct path inside the container)
cd /home/drone_dev/ros_ws/px4/PX4-Autopilot

echo ""
echo "====================================================="
echo "Launching PX4 SITL with x500_cam model"
echo "====================================================="
echo ""

# Launch PX4 SITL with camera-equipped drone
make px4_sitl gz_x500_cam
