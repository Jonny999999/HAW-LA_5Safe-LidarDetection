import velodyne_decoder as vd
from collections import namedtuple
import time
from config import get_decoder_config
from receiver import udp_packet_queue
from utils import log_info, log_warn, log_debug



# Custom function for decoding and accumulating multiple UDP packets for full decoded pointcloud (one scan frame)

ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
decoder = vd.StreamDecoder(get_decoder_config())

def decode_loop(frame_queue):
    #log_debug(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
    udp_packet_count = 0
    while True:
        # get packet from receiver queue
        packet = udp_packet_queue.get()

        # check for corrent size
        if len(packet) != vd.PACKET_SIZE:
            log_warn("[decoder] Skipping malformed packet of unexpected size.")
            continue
        else:
            udp_packet_count += 1
        
        # decode with part of official velodyne library
        result = decoder.decode(time.time(), packet)

        # has pointcloud data when full scan accumulated over several packets
        if result:
            log_debug(f"[decoder] Frame decoded completely ({udp_packet_count} packets)")
            udp_packet_count = 0
            frame_queue.put(ResultTuple(*result))
            # TODO: handle full queue (processing cant keep up with sensor data)
        else:
            #log_debug("[decoder] decoded packet, full scan frame not complete yet")
            pass
