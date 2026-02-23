#!/bin/bash

# This script helps sync the container's internal build/install directories
# to the host's volume-mapped directories. This resolves the "Package not found"
# issue when a host's empty .colcon directory masks the container's built-in build.

WORKSPACE_DIR="/home/drone_dev/ros_ws"
HOST_VOL_DIR="$WORKSPACE_DIR/.colcon/ros2_ws"

echo "Checking workspace synchronization..."

if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo "ERROR: Internal /install directory not found!"
    exit 1
fi

# Check if we are currently masked by a volume
if [ -d "$HOST_VOL_DIR/install" ] && [ "$(ls -A $HOST_VOL_DIR/install 2>/dev/null)" == "" ]; then
    echo "WARNING: Host volume $HOST_VOL_DIR/install is empty and masking the container build!"
    echo "Syncing container build to volume..."
    
    mkdir -p $HOST_VOL_DIR/build $HOST_VOL_DIR/install $HOST_VOL_DIR/log
    
    # We use cp -a to preserve permissions and symlinks (crucial for --symlink-install)
    cp -rp $WORKSPACE_DIR/build/* $HOST_VOL_DIR/build/ 2>/dev/null
    cp -rp $WORKSPACE_DIR/install/* $HOST_VOL_DIR/install/ 2>/dev/null
    
    echo "Sync complete. You may need to 'source install/setup.bash' again."
else
    echo "Workspace appears healthy or already synced."
fi
