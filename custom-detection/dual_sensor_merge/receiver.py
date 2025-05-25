import socket
import threading
import gzip
from queue import Queue
import queue
from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether
from scapy.layers.inet import IP, UDP
import time

from config import UDP_IP, UDP_PORT, PCAP_FILE_1, POINTCLOUD_HISTORY_BUFFER_SIZE, USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM, PCAP_FILE_PACKET_DELAY, PCAP_FILE_FILTER_UDP_PORT, PCAP_FILE_REALTIME_PLAYBACK
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




## --- PCAP reading mode ---
def _pcap_stream_reader(pcap_path, packet_queue, sensor_id):
    """
    PCAP mode: Streams raw packets from a PCAP file (supports .pcap and .pcap.gz).
    Replays packets using original capture timing, with optional additional delay.
    """
    NO_MATCH_WARNING_THRESHOLD = 500
    MAX_PACKET_DELAY_MS = 1
    no_match_counter = 0
    replay_start_time = time.time()
    pcap_start_time = None
    log_info(f"[receiver {sensor_id}] Streaming packets from PCAP file: {PCAP_FILE_1}")
    
    try:
        # Detect .gz and open accordingly
        open_func = gzip.open if PCAP_FILE_1.endswith(".gz") else open
        with open_func(PCAP_FILE_1, 'rb') as f:
            reader = RawPcapReader(f)
            for pkt_data, pkt_metadata in reader:
                try:
                    eth = Ether(pkt_data)
                    if IP in eth and UDP in eth:
                        udp_layer = eth[UDP]
                        if PCAP_FILE_FILTER_UDP_PORT is None or udp_layer.dport == PCAP_FILE_FILTER_UDP_PORT:
                            # Delay to simulate real-time playback
                            if PCAP_FILE_REALTIME_PLAYBACK:
                                ts = pkt_metadata.sec + pkt_metadata.usec / 1e6
                                if pcap_start_time is None:
                                    pcap_start_time = ts
                                # Compute target wall-clock time
                                target_time = replay_start_time + (ts - pcap_start_time)
                                now = time.time()
                                sleep_time = target_time - now
                                if sleep_time > MAX_PACKET_DELAY_MS:
                                    log_warn(f"[receiver {sensor_id}] pcap file contains packets with a {sleep_time} ms pause. Limiting delay to threshold of {MAX_PACKET_DELAY_MS} ms")
                                    time.sleep(MAX_PACKET_DELAY_MS)
                                elif sleep_time > 0:
                                    time.sleep(sleep_time)
                            # Optional: add additional custom PCAP_FILE_PACKET_DELAY
                            if PCAP_FILE_PACKET_DELAY > 0:
                                time.sleep(PCAP_FILE_PACKET_DELAY)

                            data = bytes(udp_layer.payload)
                            packet_queue.put_nowait(data)
                            no_match_counter = 0 # reset error count at valid packet
                        else:
                            no_match_counter += 1
                            if no_match_counter >= NO_MATCH_WARNING_THRESHOLD:
                                log_warn(f"[receiver {sensor_id}] No packets matched UDP port {PCAP_FILE_FILTER_UDP_PORT} for {NO_MATCH_WARNING_THRESHOLD} packets!")
                                log_warn("-> hint: verify sensor port, or set 'PCAP_FILE_FILTER_UDP_PORT' to 'None' to skip this filter")
                except queue.Full:
                    pass
                    #log_warn(f"[receiver {sensor_id}] Packet queue full. Dropping UDP packet.")
                except Exception as pkt_err:
                    log_warn(f"[receiver {sensor_id}] Malformed packet skipped: {pkt_err}")
    except Exception as e:
        log_error(f"[receiver {sensor_id}] Failed to read PCAP: {e}")






def start_receiver_thread(pcap_path, packet_queue, sensor_id):
    """
    Starts the appropriate reader thread based on config (UDP or PCAP).
    """
    mode = "PCAP" if USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM else "UDP"
    log_info(f"[receiver {sensor_id}] Starting receiver thread in {mode} mode...")

    # TODO: add support for UDP Stream
    # target_func = _pcap_stream_reader if USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM else _udp_listener
    # t = threading.Thread(target=target_func, daemon=True)
    t = threading.Thread(target=_pcap_stream_reader, args=(pcap_path, packet_queue, sensor_id), daemon=True)

    t.start()
