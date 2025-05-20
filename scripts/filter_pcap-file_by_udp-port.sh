#!/bin/bash

# Removes all packets that do not have specified target port
# Usage: ./filter_udp_port.sh input.pcap output.pcap [port]
# Default port is 5004 if not specified

INPUT="$1"
OUTPUT="$2"
PORT="${3:-5004}"

if [ -z "$INPUT" ] || [ -z "$OUTPUT" ]; then
    echo "Removes all packets that do not have specified target port"
    echo "Usage: $0 input.pcap output.pcap [udp_port]"
    exit 1
fi

tshark -r "$INPUT" -Y "udp.dstport == $PORT" -w "$OUTPUT"

echo "Filtered packets written to $OUTPUT (UDP dst port $PORT)"