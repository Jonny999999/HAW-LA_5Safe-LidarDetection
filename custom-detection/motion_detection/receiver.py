import socket
import threading
from queue import Queue
from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether
from scapy.layers.inet import IP, UDP
import time

from config import UDP_IP, UDP_PORT, PCAP_FILE, POINTCLOUD_HISTORY_BUFFER_SIZE, USE_PCAP, FRAME_DELAY
from utils import log_info, log_warn, log_error

# Thread-safe queue for decoded UDP packets
udp_packet_queue = Queue(maxsize=1000)



# --- Live UDP mode ---
# Listen on udp socket and run process_packet() for each packet received
def _udp_listener():
    """
    Live mode: Listens for incoming UDP packets and pushes them into the queue.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    log_info(f"[receiver] Listening for UDP packets on {UDP_IP}:{UDP_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            udp_packet_queue.put_nowait(data)
        except queue.Full:
            log_warn("[receiver] Packet queue full. (receiving packets faster than decoding) Dropping UDP packet.")
        except Exception as e:
            log_error(f"[receiver] UDP receive error: {e}")
    # socket never closes (daemon thread), no finally needed




# --- PCAP reading mode ---
# stream file content in single packets (prevent long loading)
# run process_packet() for each frame stored in the file
def _pcap_stream_reader():
    """
    PCAP mode: Streams raw packets from a PCAP file and feeds the queue.
    """
    log_info(f"[receiver] Streaming packets from PCAP file: {PCAP_FILE}")
    try:
        for pkt_data, _ in RawPcapReader(PCAP_FILE):
            try:
                eth = Ether(pkt_data)
                if IP in eth and UDP in eth:
                    udp_layer = eth[UDP]
                    data = bytes(udp_layer.payload)
                    udp_packet_queue.put_nowait(data)
                    # Optional: simulate real-time stream
                    time.sleep(FRAME_DELAY)
            except Exception as pkt_err:
                log_warn(f"[receiver] Malformed packet skipped: {pkt_err}")
            except queue.Full:
                log_warn("[receiver] Packet queue full. (reading from PCAP file faster than decoding) Dropping UDP packet.")
    except Exception as e:
        log_error(f"[receiver] Failed to read PCAP: {e}")




def start_receiver():
    """
    Starts the appropriate reader thread based on config (UDP or PCAP).
    """
    mode = "PCAP" if USE_PCAP else "UDP"
    log_info(f"[receiver] Starting receiver thread in {mode} mode...")

    target_func = _pcap_stream_reader if USE_PCAP else _udp_listener
    t = threading.Thread(target=target_func, daemon=True)
    t.start()
