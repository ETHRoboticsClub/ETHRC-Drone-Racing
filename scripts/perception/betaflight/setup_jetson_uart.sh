#!/bin/bash
# Script to configure Jetson UART for Betaflight Blackbox streaming

# 1. Set Parent Clock to 80MHz for high speed (up to 5Mbps)
if [ -f /sys/kernel/debug/bpmp/debug/clk/uarti/rate ]; then
    echo "Setting UART clock to 80MHz..."
    echo 80000000 | sudo tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate
else
    echo "Could not find uarti clock rate file."
fi

# 2. Set low latency flag for reduced jitter
if [ -e /dev/ttyTHS1 ]; then
    echo "Configuring /dev/ttyTHS1 to low_latency..."
    sudo setserial /dev/ttyTHS1 low_latency
else
    echo "Warning: /dev/ttyTHS1 not found."
fi

# 3. Configure group and permissions for ttyTCU0 (Tegra Combined UART)
if [ -e /dev/ttyTCU0 ]; then
    echo "Configuring /dev/ttyTCU0 permissions..."
    sudo chown root:dialout /dev/ttyTCU0
    sudo chmod 660 /dev/ttyTCU0
else
    echo "Warning: /dev/ttyTCU0 not found."
fi

echo "Jetson UART tuning complete."
