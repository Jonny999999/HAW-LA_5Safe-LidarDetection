#!/bin/bash
# Use first argument as status file if provided, otherwise fallback to default
STATUS_FILE="${1:-custom-detection/dual_sensor_merge/output/status.json}"
REFRESH_DELAY=0.05  # minimum delay between updates in seconds
# note: too low delay causes flickering

tput smcup
tput civis

cleanup() {
  tput cnorm
  tput rmcup
  clear
  exit
}

trap cleanup INT TERM

echo -e "waiting for file to update initially:\n$STATUS_FILE\n..."

while true; do
  # Wait until the file is written
  inotifywait -qq -e close_write "$STATUS_FILE"
  sleep $REFRESH_DELAY

  tput cup 0 0

  # Read output from jq line by line
  line_num=0
  while IFS= read -r line; do
    tput cup $line_num 0
    printf "\r\033[K%s" "$line"
    ((line_num++))
  done < <(jq --color-output . "$STATUS_FILE" 2>/dev/null)

  # Clear any remaining lines if the new content is shorter than previous
  tput ed  # Clear to end of screen

done
