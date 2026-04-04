#!/bin/bash

# sync_time.sh: Syncs the Jetson (Boreas) clock with the host to avoid certificate errors.
# Usage: ./sync_time.sh [hostname]

HOST=${1:-boreas}
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
SSH_EXEC="$SCRIPT_DIR/ssh_exec.py"

if [ ! -f "$SSH_EXEC" ]; then
    echo "Error: $SSH_EXEC not found."
    exit 1
fi

# Get current UTC time from host
CURRENT_TIME=$(date -u +"%Y-%m-%d %H:%M:%S")

echo "--------------------------------------------------------"
echo "Syncing time on $HOST..."
echo "Host UTC Time: $CURRENT_TIME"

# Execute sudo date command on remote via ssh_exec.py
# ssh_exec.py handles the 'strongpass' password automatically
python3 "$SSH_EXEC" "sudo -S date -u -s '$CURRENT_TIME'" "$HOST"

echo "--------------------------------------------------------"
echo "Verification:"
python3 "$SSH_EXEC" "date -u" "$HOST"
echo "--------------------------------------------------------"
