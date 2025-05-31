import socket
import threading
import gzip
from multiprocessing import Process
from multiprocessing import Queue
import queue
from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether
from scapy.layers.inet import IP, UDP
import time

from config import PCAP_FILE_PACKET_DELAY, PCAP_FILE_FILTER_UDP_PORT, PCAP_FILE_REALTIME_PLAYBACK
from utils import log_info, log_warn, log_error



# --- Live UDP mode ---
# Listen on udp socket and run process_packet() for each packet received
def _udp_listener(udp_ip_addr, udp_port, packet_queue, sensor_id):
    """
    Live mode: Listens for incoming UDP packets and pushes them into the queue.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip_addr, udp_port))
    log_info(f"[receiver-{sensor_id}] Listening for UDP packets on {udp_ip_addr}:{udp_port} for sensor {sensor_id}")

    while True:
        try:
            data, _ = sock.recvfrom(2048) # TODO: adjust to actual packet length?
            timestamp = time.time() # store time packet was received (used for pointcloud synchronization)
            packet_queue.put_nowait((timestamp, data))
        except queue.Full:
            log_warn(f"[receiver-{sensor_id}] Packet queue full. (receiving packets faster than decoding) Dropping UDP packet.")
        except Exception as e:
            log_error(f"[receiver-{sensor_id}] UDP receive error: {e}")
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
    log_info(f"[receiver-{sensor_id}] Streaming packets from PCAP file: {pcap_path}")
    
    try:
        # Detect .gz and open accordingly
        open_func = gzip.open if pcap_path.endswith(".gz") else open
        with open_func(pcap_path, 'rb') as f:
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
                                    log_warn(f"[receiver-{sensor_id}] pcap file contains packets with a {sleep_time} ms pause. Limiting delay to threshold of {MAX_PACKET_DELAY_MS} ms")
                                    time.sleep(MAX_PACKET_DELAY_MS)
                                elif sleep_time > 0:
                                    time.sleep(sleep_time)
                            # Optional: add additional custom PCAP_FILE_PACKET_DELAY
                            if PCAP_FILE_PACKET_DELAY > 0:
                                time.sleep(PCAP_FILE_PACKET_DELAY)

                            data = bytes(udp_layer.payload) # acutal packet data
                            timestamp = pkt_metadata.sec + pkt_metadata.usec / 1e6  # Absolute timestamp from PCAP
                            packet_queue.put_nowait((timestamp, data))
                            no_match_counter = 0 # reset error count at valid packet
                        else:
                            no_match_counter += 1
                            if no_match_counter >= NO_MATCH_WARNING_THRESHOLD:
                                log_warn(f"[receiver-{sensor_id}] No packets matched UDP port {PCAP_FILE_FILTER_UDP_PORT} for {NO_MATCH_WARNING_THRESHOLD} packets!")
                                log_warn("-> hint: verify sensor port, or set 'PCAP_FILE_FILTER_UDP_PORT' to 'None' to skip this filter")
                except queue.Full:
                    pass
                    #log_warn(f"[receiver {sensor_id}] Packet queue full. Dropping UDP packet.")
                except Exception as pkt_err:
                    log_warn(f"[receiver-{sensor_id}] Malformed packet skipped: {pkt_err}")
    except Exception as e:
        log_error(f"[receiver-{sensor_id}] Failed to read PCAP: {e}")







def start_receiver_thread(mode, packet_queue, sensor_id, *, pcap_path=None, udp_listen_ip=None, udp_port=None):
    """
    Starts a receiver thread based on the selected mode (PCAP or UDP).

    Args:
        mode (str): "PCAP" or "UDP"
        packet_queue (Queue): Queue to push packets to
        sensor_id (int): ID of the sensor
        pcap_path (str): Path to pcap file (if in PCAP mode)
        udp_ip (str): IP to listen to (if in UDP mode)
        udp_port (int): Port to listen on (if in UDP mode)
    """
    log_info(f"[receiver-{sensor_id}] Starting receiver Process in {mode} mode...")

    if mode == "PCAP":
        p = Process(
            target=_pcap_stream_reader,
            args=(pcap_path, packet_queue, sensor_id),
            daemon=True
        )
    elif mode == "UDP":
        p = Process(
            target=_udp_listener,
            args=(udp_listen_ip, udp_port, packet_queue, sensor_id),
            daemon=True
        )
    else:
        raise ValueError(f"Unknown mode '{mode}' for receiver {sensor_id}")

    p.start()