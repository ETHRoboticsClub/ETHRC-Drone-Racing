#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo ""
echo "====================================================="
echo "  ETHRC Drone Groundstation"
echo "====================================================="
echo ""

echo "[1/3] Building groundstation package..."
cd "$REPO_ROOT"
colcon build --packages-select groundstation --symlink-install

echo ""
echo "[2/3] Sourcing install/setup.bash..."
source "$REPO_ROOT/install/setup.bash"

echo ""
echo "[3/3] Launching web server..."
echo "      http://localhost:8080"
echo ""
ros2 run groundstation web_server
