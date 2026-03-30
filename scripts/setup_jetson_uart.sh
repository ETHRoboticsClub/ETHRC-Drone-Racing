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

echo "Jetson UART tuning complete."
