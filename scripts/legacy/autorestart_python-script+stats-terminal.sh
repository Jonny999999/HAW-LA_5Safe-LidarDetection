#!/bin/bash

# === Auto-Restart Loop for Dual Sensor Merge GUI ===
# This script repeatedly starts the main Python GUI and a status-monitoring terminal.
# Its a workaround for long-term stability issues by restarting everything every 10 minutes.

# --- Configuration ---
REPO_ROOT="$HOME/git/HAW-LA_5Safe-LidarDetection"

PYTHON_SCRIPT_FOLDER="$REPO_ROOT/lidar_system"
PYTHON_SCRIPT_NAME="main.py"

TERMINAL_FOLDER="$REPO_ROOT"
TERMINAL_SCRIPT="$REPO_ROOT/scripts/watch_status_json_file.sh"

PYTHON_EXECUTABLE="python"
    #/usr/bin/python3.10 $PYTHON_SCRIPT_NAME &

RESTART_INTERVAL=10m

# configure display the windows should be opened
export DISPLAY=:0


# initially kill python processes 
# in case an old instance is still partially running
killall python3.10

while true; do

    # --- Launch Python GUI ---
    cd $PYTHON_SCRIPT_FOLDER
    source ~/python-envs/py3.10-5Safe/bin/activate
    "$PYTHON_EXECUTABLE" "$PYTHON_SCRIPT_NAME" &

    # --- Wait for GUI to launch ---
    sleep 5  # Ensure the Python GUI window has time to open

    # --- Launch status monitor terminal ---
    cd $TERMINAL_FOLDER
    xterm -geometry 51x20 \
          -fa 'Monospace' -fs 30 \
          -bg black -fg white \
          -e $TERMINAL_SCRIPT &

    # --- Runtime Duration ---
    sleep $RESTART_INTERVAL  # Run duration before restarting components

    # --- Cleanup ---
    killall python3.10
    killall xterm

    echo "Restarting main.py in 1s..."
    sleep 1

done
