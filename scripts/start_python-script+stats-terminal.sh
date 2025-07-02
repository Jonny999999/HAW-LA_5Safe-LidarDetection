#!/bin/bash

# === Auto-Restart for Dual Sensor Merge GUI ===
# Starts the Python GUI and launches a status terminal after 5s.
# Restarts everything only when the Python script exits.

# --- Configuration ---
REPO_ROOT="$HOME/git/HAW-LA_5Safe-LidarDetection"

PYTHON_SCRIPT_FOLDER="$REPO_ROOT/lidar_system"
PYTHON_SCRIPT_NAME="main.py"

TERMINAL_SCRIPT="$REPO_ROOT/scripts/watch_status_json_file.sh"

PYTHON_EXECUTABLE="/usr/bin/python3.10"
#PYTHON_EXECUTABLE="python"
#PYTHON_VENV_CMD="source ~/python-envs/py3.10-5Safe/bin/activate" # command run before starting the python script

export DISPLAY=:0  # Needed for GUI and xterm

TERMINAL_STARTUP_DELAY_SECONDS=10


# === Cleanup function on exit (e.g. CTRL+C) ===
cleanup_on_exit() {
    echo ""
    echo "========================================================="
    echo "[Auto-Restart] Caught termination signal, cleaning up..."
    echo "========================================================="
    killall "$PYTHON_EXECUTABLE" 2>/dev/null
    killall xterm 2>/dev/null
    echo "killed python and xterm..."
    exit 0
}
# Trap CTRL+C (SIGINT), SIGTERM, and EXIT
trap cleanup_on_exit SIGINT SIGTERM



# === Identify and kill previous running instances of this script ===
CURRENT_PID=$$
CURRENT_NAME=$(basename "$0")
#echo "[DEBUG] Current PID: $CURRENT_PID"
#echo "[DEBUG] Current script name: $CURRENT_NAME"
#echo "[DEBUG] Searching for other matching bash scripts..."

# List all bash processes that include this script name (excluding this one)
MATCHES=$(ps -eo pid,cmd | grep "[b]ash .*${CURRENT_NAME}" | grep -v "$CURRENT_PID" || true)
#echo "[DEBUG] Matching processes:"
#echo "$MATCHES"

# Extract PIDs to kill
PIDS_TO_KILL=$(echo "$MATCHES" | awk '{print $1}')

for pid in $PIDS_TO_KILL; do
    echo "================================================================="
    echo "[Auto-Restart] Killing previous bash script instance (PID=$pid)"
    echo "================================================================="
    kill "$pid" 2>/dev/null
done



# Clean up old processes
killall "$PYTHON_EXECUTABLE"
killall xterm 2>/dev/null

# Start python script + terminal, kill and restart both on crash
while true; do
    echo "======================================"
    echo "[Auto-Restart] Starting Python GUI..."
    echo "======================================"

    cd "$PYTHON_SCRIPT_FOLDER"

    # start venv if configured
    if [[ -n "$PYTHON_VENV_CMD" ]]; then
        echo "Starting python venv using cmd: $PYTHON_VENV_CMD"
        eval "$PYTHON_VENV_CMD"
    fi
    # Launch Python GUI in background, so we can start xterm after delay
    "$PYTHON_EXECUTABLE" "$PYTHON_SCRIPT_NAME" &
    PY_PID=$!

    # Wait a bit to ensure the GUI starts
    sleep $TERMINAL_STARTUP_DELAY_SECONDS

    echo "===================================================="
    echo "[Auto-Restart] Starting xterm showing status.json..."
    echo "===================================================="
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
