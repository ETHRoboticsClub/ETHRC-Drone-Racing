#!/usr/bin/env bash
# Ensure host-side colcon bind-mount directories exist with correct ownership.
# Run on the host before docker compose up so the container can write logs/builds.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

USER_NAME="${SUDO_USER:-$USER}"
USER_ID="$(id -u "$USER_NAME")"
GROUP_ID="$(id -g "$USER_NAME")"

TARGET_DIR=".colcon"
DIRS=(build install log)

echo "Preparing host bind directories under $TARGET_DIR for user $USER_NAME ($USER_ID:$GROUP_ID)"

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

# Set ownership and reasonable permissions (user+group read/write/execute; no world write).
chown -R "$USER_ID:$GROUP_ID" "$TARGET_DIR"
chmod -R u+rwX,g+rwX,o-rwx "$TARGET_DIR"

echo "Done. You can now run 'docker compose up -d' safely."