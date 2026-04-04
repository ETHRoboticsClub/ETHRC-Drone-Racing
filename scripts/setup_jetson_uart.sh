#!/bin/bash
# setup_jetson_uart.sh

# 1. Jetson High-Speed UART Configuration (BPMP clock)
echo "Configuring Jetson UART for 1.5Mbps..."
echo 80000000 | sudo -S tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate <<CLKEF
strongpass
CLKEF

# 2. Performance Mode
echo "Setting Jetson to MAXN performance mode..."
sudo -S nvpmodel -m 0 <<NPEF
strongpass
NPEF
sudo -S jetson_clocks <<CLEF
strongpass
CLEF

# 3. Kill existing driver
echo "Stopping existing blackbox driver..."
pkill -f blackbox_driver_node || true

# 4. Build the workspace
PROJECT_DIR="/home/aero/proj/ETHRC-Drone-Racing"
echo "Building the workspace at $PROJECT_DIR..."
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
colcon build --packages-select betaflight_blackbox_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Provide instructions
echo "------------------------------------------------"
echo "Setup Complete."
echo "To start the driver, run:"
echo "source install/setup.bash"
echo "ros2 run betaflight_blackbox_driver blackbox_driver_node --ros-args -p port:=/dev/ttyTHS1 -p baud:=1500000"
echo "------------------------------------------------"
