#!/usr/bin/env bash
# Ensure host-side colcon bind-mount directories exist with correct ownership.
# Run on the host before docker compose up so the container can write logs/builds.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# Default to UID/GID 1000 (standard for the 'drone_dev' container user)
# to avoid permission denied errors inside the container.
TARGET_UID="${CONTAINER_UID:-1000}"
TARGET_GID="${CONTAINER_GID:-1000}"
TARGET_DIR=".colcon"
DIRS=(build install log)

echo "Preparing host bind directories under $TARGET_DIR for container user ($TARGET_UID:$TARGET_GID)"

# Bail out if any of the targets already exist as non-directories to avoid surprises.
for d in "" "${DIRS[@]}"; do
  path="$TARGET_DIR/${d}"
  if [ -e "$path" ] && [ ! -d "$path" ]; then
    echo "ERROR: $path exists but is not a directory. Please remove or fix it first." >&2
    exit 1
  fi
done

# Create the directories.
mkdir -p "$TARGET_DIR"
for d in "${DIRS[@]}"; do
  mkdir -p "$TARGET_DIR/$d"
done

# Set ownership and reasonable permissions (container user read/write/execute).
chown -R "$TARGET_UID:$TARGET_GID" "$TARGET_DIR"
chmod -R u+rwX,g+rwX,o+rwX "$TARGET_DIR"

echo "Done. You can now run 'docker compose up -d' safely."