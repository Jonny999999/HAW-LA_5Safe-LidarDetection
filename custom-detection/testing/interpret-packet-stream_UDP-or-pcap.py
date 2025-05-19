# 1. Receive and log UDP packets from a Wireshark save (pcap file)
# 2. Or alternatively receive and log data received directly via UDP socket
# 3. manually interpret lidar data  (Accumulate multiple UDP frames and decode until full frame is received)

import socket
import time
from scapy.all import rdpcap, UDP, IP
from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether
from colorama import Fore, Style
import velodyne_decoder as vd # for decoder


# --- Configuration ---
USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM = True  # Set to True to read from pcap file instead of live socket
PCAP_FILE = "../../data/testdata/2025.05.16_wireshark-dump_sensor-on-desk.pcap"
#PCAP_FILE = "../../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5001
BUFFER_SIZE = 4096  # Adjust based on expected packet size



# Global or class-level
config = vd.Config()
config.model = vd.Model.VLP32C  # or autodetect if needed
decoder = vd.StreamDecoder(config)




# helper functions for logging
def log_warn(msg):
    print(f"{Fore.YELLOW}[WARN] {msg}{Style.RESET_ALL}")

def log_info(msg):
    print(f"{Fore.GREEN}[INFO]{Style.RESET_ALL} {msg}")

def log_error(msg):
    print(f"{Fore.RED}[ERROR] {msg}{Style.RESET_ALL}")

def log_debug(msg):
    print(f"{Fore.WHITE}[DEBUG] {msg}{Style.RESET_ALL}")





# Custom function for decoding and accumulating multiple UDP packets for full decoded pointcloud (one scan frame)
from collections import namedtuple
ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))

def decode_packet(data, src_ip, src_port):
    log_debug(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
    global decoder  # if decoder is defined globally
    # You may optionally timestamp here
    host_stamp = time.time()

    if len(data) != vd.PACKET_SIZE:
        log_warn("Skipping malformed packet of unexpected size.")
        return None

    result = decoder.decode(host_stamp, data, as_pcl_structs=False)
    if result is not None:
        log_info("Frame decoded completely")
        return ResultTuple(*result)
    return None




# process one received UDP packet
# -> decode it, try to assemble fully pointcloud frame using custom decoder
# -> run action with the data (log)
def process_packet(data, src_ip, src_port):
    decoded_point_cloud = decode_packet(data, src_ip, src_port)
    if decoded_point_cloud:
        stamp, pointcloud = decoded_point_cloud
        # → use exactly like from read_pcap
        # example:
        xyz = pointcloud[:, :3]
        log_info(xyz)
        #frame_buffer.append(xyz)
    else:
        log_debug("No complete frame received yet")
        pass
        





# --- Live UDP mode ---
# Listen on udp socket and run process_packet() for each packet received
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





# --- PCAP reading mode ---
# stream file content in single packets (prevent long loading)
# run process_packet() for each frame stored in the file
def read_pcap(filename):
    log_info(f"Streaming packets from pcap file: {filename}")
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
                log_warn(f"Skipping malformed packet: {pkt_err}")
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
