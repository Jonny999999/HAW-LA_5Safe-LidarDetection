import socket

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5001
BUFFER_SIZE = 4096  # Adjust if you expect bigger packets

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}")

try:
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)  # Receive data
        src_ip, src_port = addr
        print(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
        if data:
            print(f"Data (hex): {data.hex()}")
            # Uncomment for ASCII (if expected to be readable):
            # print(f"Data (ascii): {data.decode(errors='replace')}")
        print("-" * 60)
except KeyboardInterrupt:
    print("Exiting.")
finally:
    sock.close()
