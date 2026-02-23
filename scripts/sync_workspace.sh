#!/bin/bash

# This script helps sync the container's internal build/install directories
# to the host's volume-mapped directories. This resolves the "Package not found"
# issue when a host's empty .colcon directory masks the container's built-in build.

WORKSPACE_DIR="/home/drone_dev/ros_ws"

echo "Checking workspace synchronization..."

if [ ! -d "$WORKSPACE_DIR/install" ] || [ "$(ls -A $WORKSPACE_DIR/install 2>/dev/null)" == "" ]; then
    echo "WARNING: $WORKSPACE_DIR/install is empty!"
    echo "This is expected on first startup with volume mounts."
    echo "Please build the workspace:"
    echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    echo "  source install/setup.bash"
    exit 0
fi

echo "Workspace appears healthy. install/ contains packages."
echo "Run 'source install/setup.bash' to activate the workspace."
