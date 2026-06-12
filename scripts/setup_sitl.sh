#!/bin/bash
set -e

# Setup script for Betaflight SITL + Gazebo Harmonic simulation verification

echo "=== Starting Betaflight SITL + Gazebo Setup ==="

# Get workspace root directory (parent of scripts/)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Workspace directory: $WORKSPACE_DIR"

# 1. Install dependencies (Gazebo Harmonic + ROS 2 bridges)
if ! dpkg -l | grep -q "gz-harmonic"; then
    echo "Installing Gazebo Harmonic..."
    sudo apt-get update
    sudo apt-get install -y lsb-release wget gnupg
    
    # Add OSRF package repository keys
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    sudo apt-get update
    sudo apt-get install -y gz-harmonic
else
    echo "Gazebo Harmonic is already installed."
fi

if ! dpkg -l | grep -q "ros-humble-ros-gzharmonic-bridge"; then
    echo "Installing ros-humble-ros-gzharmonic-bridge and ros-humble-ros-gzharmonic-sim..."
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-sim
else
    echo "ros-humble-ros-gzharmonic-bridge is already installed."
fi

# 2. Clone and build aeroloop_gazebo plugins
EXT_DIR="$WORKSPACE_DIR/ext"
mkdir -p "$EXT_DIR"

AEROLOOP_DIR="$EXT_DIR/aeroloop_gazebo"
if [ ! -d "$AEROLOOP_DIR" ]; then
    echo "Cloning aeroloop_gazebo..."
    git clone -b gz https://github.com/betaflight/aeroloop_gazebo.git "$AEROLOOP_DIR"
else
    echo "aeroloop_gazebo repository already exists."
fi

echo "Building aeroloop_gazebo plugin..."
cd "$AEROLOOP_DIR"
./build_plugin.sh

# 3. Build Betaflight SITL binary
BF_DIR="$EXT_DIR/anemos-betaflight"
if [ ! -d "$BF_DIR" ]; then
    echo "Error: ext/anemos-betaflight not found. Please ensure custom betaflight source is cloned."
    exit 1
fi

echo "Building Betaflight SITL binary..."
cd "$BF_DIR"
# Clean and compile SITL target
make clean TARGET=SITL || true
make TARGET=SITL -j$(nproc)

# 4. Verify compilation output
SITL_BIN=$(find "$BF_DIR/obj" -name "betaflight_*_SITL*" -type f | head -n 1)
if [ -z "$SITL_BIN" ] || [ ! -x "$SITL_BIN" ]; then
    # Try alternate location or binary name
    SITL_BIN="$BF_DIR/obj/main/betaflight_SITL"
    if [ ! -x "$SITL_BIN" ]; then
         echo "Error: Betaflight SITL binary not found/executable after build."
         exit 1
    fi
fi
echo "Betaflight SITL binary compiled successfully at: $SITL_BIN"

# 5. Load Betaflight SITL CLI configuration to eeprom.bin in the workspace root
echo "Loading Betaflight SITL configuration..."
CONFIG_FILE="$WORKSPACE_DIR/docs/docs/files/betaflight_conf/sitl_config.txt"
if [ -f "$CONFIG_FILE" ]; then
    # Run the binary with --config and wait for it to save and exit.
    # We execute it from the workspace root so that the eeprom.bin is created there.
    cd "$WORKSPACE_DIR"
    "$SITL_BIN" --config "$CONFIG_FILE"
    echo "Betaflight SITL configuration loaded and saved to eeprom.bin."
else
    echo "Warning: Betaflight config file not found at $CONFIG_FILE"
fi

# 6. Build ROS 2 workspace packages
echo "Building ROS 2 workspace packages..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-up-to drone_control betaflight_msp_controller gazebo_ros_bridges

echo "=== Setup Completed Successfully! ==="
echo "Betaflight SITL binary path: $SITL_BIN"
echo "To run SITL: ros2 launch drone_control racing_bringup.launch.py sitl:=true"
