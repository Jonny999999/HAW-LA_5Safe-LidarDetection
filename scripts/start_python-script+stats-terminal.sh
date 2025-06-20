#!/bin/bash

# === Auto-Restart for Dual Sensor Merge GUI ===
# Starts the Python GUI and launches a status terminal after 5s.
# Restarts everything only when the Python script exits.

# --- Configuration ---
PYTHON_SCRIPT_FOLDER="/home/mablee/HAW-LA_5Safe-LidarDetection/custom-detection/dual_sensor_merge"
PYTHON_SCRIPT_NAME="main.py"

TERMINAL_FOLDER="/home/mablee/HAW-LA_5Safe-LidarDetection/"
TERMINAL_SCRIPT="./scripts/watch_status_json_file.sh"

export DISPLAY=:0  # Needed for GUI and xterm

# Clean up old processes
killall python3.10 2>/dev/null
killall xterm 2>/dev/null

while true; do
    echo "[Auto-Restart] Starting Python GUI..."

    cd "$PYTHON_SCRIPT_FOLDER"

    # Launch Python GUI in background, so we can start xterm after delay
    /usr/bin/python3.10 "$PYTHON_SCRIPT_NAME" &
    PY_PID=$!

    # Wait a bit to ensure the GUI starts
    sleep 5

    # Launch terminal with monitoring script
    cd "$TERMINAL_FOLDER"
    xterm -geometry 51x20 \
          -fa 'Monospace' -fs 30 \
          -bg black -fg white \
          -e "$TERMINAL_SCRIPT" &
    XTERM_PID=$!

    # Wait for Python process to finish (foreground wait)
    wait $PY_PID
    EXIT_CODE=$?

    echo "================================================================"
    echo "[Auto-Restart] Python GUI exited with code $EXIT_CODE, cleaning up..."
    echo "================================================================"

    # Clean up python process if still any running
    killall python3.10 2>/dev/null
    # Clean up terminal
    kill "$XTERM_PID" 2>/dev/null

    echo "========================================="
    echo "[Auto-Restart] Restarting in 2 seconds..."
    echo "========================================="
    sleep 2
done
