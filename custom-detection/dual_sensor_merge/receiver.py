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

from config import PCAP_FILE_PACKET_DELAY, PCAP_FILE_REALTIME_PLAYBACK, PCAP_LOOP_WHEN_FILE_COMPLETED
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
def _pcap_stream_reader(pcap_path, packet_queue, sensor_id, filtered_udp_port=None):
    NO_MATCH_WARNING_THRESHOLD = 500
    MAX_PACKET_DELAY_MS = 1
    no_match_counter = 0
    loop_count = 0
    cumulative_time_offset = 0.0  # Offset applied to timestamps with each loop

    log_info(f"[receiver-{sensor_id}] Streaming packets from PCAP file: {pcap_path}")

    open_func = gzip.open if pcap_path.endswith(".gz") else open

    while True:
        try:
            with open_func(pcap_path, 'rb') as f:
                reader = RawPcapReader(f)
                replay_start_time = time.time()
                pcap_start_time = None

                for pkt_data, pkt_metadata in reader:
                    try:
                        eth = Ether(pkt_data)
                        if IP in eth and UDP in eth:
                            udp_layer = eth[UDP]
                            if filtered_udp_port is None or udp_layer.dport == filtered_udp_port:
                                # Real-time playback simulation
                                ts = pkt_metadata.sec + pkt_metadata.usec / 1e6
                                if pcap_start_time is None:
                                    pcap_start_time = ts
                                adjusted_ts = ts - pcap_start_time + cumulative_time_offset

                                # Wall-clock replay delay
                                if PCAP_FILE_REALTIME_PLAYBACK:
                                    target_time = replay_start_time + (ts - pcap_start_time)
                                    sleep_time = target_time - time.time()
                                    if sleep_time > MAX_PACKET_DELAY_MS:
                                        log_warn(f"[receiver-{sensor_id}] pcap file contains packets with a {sleep_time} ms pause. Limiting delay to {MAX_PACKET_DELAY_MS} ms")
                                        time.sleep(MAX_PACKET_DELAY_MS)
                                    elif sleep_time > 0:
                                        time.sleep(sleep_time)
                                if PCAP_FILE_PACKET_DELAY > 0:
                                    time.sleep(PCAP_FILE_PACKET_DELAY)

                                # Push to queue with adjusted timestamp
                                data = bytes(udp_layer.payload)
                                packet_queue.put_nowait((adjusted_ts, data))
                                no_match_counter = 0
                            else:
                                no_match_counter += 1
                                if no_match_counter >= NO_MATCH_WARNING_THRESHOLD:
                                    log_warn(f"[receiver-{sensor_id}] No packets matched UDP port {filtered_udp_port} for {NO_MATCH_WARNING_THRESHOLD} packets!")
                    except queue.Full:
                        pass
                    except Exception as pkt_err:
                        log_warn(f"[receiver-{sensor_id}] Malformed packet skipped: {pkt_err}")

            # End of file reached
            print("===================================")
            print("===== END OF PCAP FILE REACHED ====")
            print(f"File: '{pcap_path}'")
            print("===================================")
            if PCAP_LOOP_WHEN_FILE_COMPLETED:
                print(f"PCAP_LOOP_WHEN_FILE_COMPLETED enabled -> starting over (count={loop_count}), adding offset to timestamps")
                loop_count += 1
                cumulative_time_offset = time.time() - (pcap_start_time or time.time())
                log_info(f"[receiver-{sensor_id}] PCAP file completed, restarting from beginning (loop {loop_count})")
                continue
            else:
                break  # exit if not looping
        except Exception as e:
            log_error(f"[receiver-{sensor_id}] Failed to read PCAP: {e}")
            break






def start_receiver_thread(mode, packet_queue, sensor_id, *, pcap_path=None, udp_listen_ip=None, udp_port=None, filtered_udp_port=None):
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
            args=(pcap_path, packet_queue, sensor_id, filtered_udp_port),
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