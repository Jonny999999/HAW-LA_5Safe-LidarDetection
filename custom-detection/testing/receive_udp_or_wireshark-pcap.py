# Testing on how to receive sensor packets in python (especially on how to simulate it by using pcap file):
# 1. Receive and log UDP packets from a Wireshark save (pcap file)
# 2. Or alternatively receive and log data received directly via UDP socket

import socket
import time
from scapy.all import rdpcap, UDP, IP
from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether


# --- Configuration ---
USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM = True  # Set to True to read from pcap file instead of live socket
PCAP_FILE = "../../data/testdata/2025.05.16_wireshark-dump_sensor-on-desk.pcap"

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5001
BUFFER_SIZE = 4096  # Adjust based on expected packet size


# --- Common processing function ---
def process_packet(data, src_ip, src_port):
    print(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
    if data:
        print(f"Data (hex): {data.hex()}")
        # Uncomment if data is ASCII-readable
        # print(f"Data (ascii): {data.decode(errors='replace')}")
    print("-" * 60)



# --- Live UDP mode ---
def listen_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}")
    try:
        while True:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            src_ip, src_port = addr
            process_packet(data, src_ip, src_port)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        sock.close()



## --- PCAP reading mode ---
## load file at once, then process (long loading)
#def read_pcap(filename):
#    print(f"Reading packets from pcap file: {filename}")
#    packets = rdpcap(filename)
#    for pkt in packets:
#        if UDP in pkt and IP in pkt:
#            ip = pkt[IP]
#            udp = pkt[UDP]
#            data = bytes(udp.payload)
#            process_packet(data, ip.src, udp.sport)
#            time.sleep(0.01)  # optional: simulate real-time stream


# --- PCAP reading mode ---
# stream file content in single packets (prevent long loading)
def read_pcap(filename):
    print(f"Streaming packets from pcap file: {filename}")
    try:
        for pkt_data, _ in RawPcapReader(filename):
            try:
                eth = Ether(pkt_data)
                if IP in eth and UDP in eth:
                    ip_layer = eth[IP]
                    udp_layer = eth[UDP]
                    data = bytes(udp_layer.payload)
                    process_packet(data, ip_layer.src, udp_layer.sport)
                    time.sleep(0.01)  # simulate real-time
            except Exception as pkt_err:
                print(f"[!] Skipping malformed packet: {pkt_err}")
                continue
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as e:
        print(f"Error while reading pcap: {e}")





# --- Main ---
if __name__ == "__main__":
    if USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM:
        read_pcap(PCAP_FILE)
    else:
        listen_udp()
