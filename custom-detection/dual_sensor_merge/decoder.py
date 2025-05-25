import velodyne_decoder as vd
from collections import namedtuple
import time
from config import get_decoder_config
from utils import log_info, log_warn, log_debug
from queue import Full

from shared_types import StampCloudTuple


# Custom function for decoding and accumulating multiple UDP packets for full decoded pointcloud (one scan frame)

#ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
ResultTuple = StampCloudTuple
decoder = vd.StreamDecoder(get_decoder_config())

def decode_loop(frame_queue, udp_packet_queue, sensor_id):
    #log_debug(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
    log_warn(f"Starting decoder loop for sensr {sensor_id}")
    udp_packet_count = 0
    while True:
        # get packet from receiver queue
        packet = udp_packet_queue.get()

        # check for corrent size
        if len(packet) != vd.PACKET_SIZE:
            log_warn(f"[decoder {sensor_id}] Skipping malformed packet of unexpected size. (expected {vd.PACKET_SIZE} but received {len(packet)})")
            continue
        else:
            udp_packet_count += 1
        
        # decode with part of official velodyne library
        result = decoder.decode(time.time(), packet)

        # has pointcloud data when full scan accumulated over several packets
        if result:
            log_debug(f"[decoder {sensor_id}] Frame decoded completely ({udp_packet_count} packets)")
            udp_packet_count = 0
            # Add pointcloud to queue
            try:
                #log_debug(f"[decoder {sensor_id}] Result type: {type(result)}, content: {result}")
                #frame_queue.put(ResultTuple(*result), block=False)

                # flatten result structure (drop timepair object only keep timestamp)
                timepair, points = result
                stamp = timepair.host  # or float(timepair), if it supports __float__
                frame_queue.put(ResultTuple(stamp, points), block=False)
            except Full:
                # Drop oldest to make space
                try: # catch race condition when other task already modified it by then
                    dropped = frame_queue.get_nowait()
                    log_warn(f"[decoder {sensor_id}] Frame queue full! Dropped oldest frame.")
                    # Retry putting, now it should work
                    timepair, points = result
                    stamp = timepair.host  # or float(timepair), if it supports __float__
                    frame_queue.put(ResultTuple(stamp, points), block=False)
                except queue.Empty:
                    log_warn(f"[decoder {sensor_id}] Tried to drop frame but queue was already empty!")
                except Full:
                    log_error(f"[decoder {sensor_id}] Queue still full after dropping — giving up on frame.")

        else:
            #log_debug("[decoder] decoded packet, full scan frame not complete yet")
            pass









from collections import deque

def frame_synchronizer(queue_1, queue_2, synced_queue, tolerance=0.01):
    """
    Synchronizes frames from two sources by timestamp.
    """
    buffer_1 = deque()
    buffer_2 = deque()

    while True:
        # Blocking reads from both queues
        if len(buffer_1) == 0:
            buffer_1.append(queue_1.get())
        if len(buffer_2) == 0:
            buffer_2.append(queue_2.get())

        # Compare front elements
        stamp1, pc1 = buffer_1[0]
        stamp2, pc2 = buffer_2[0]

        dt = abs(stamp1 - stamp2)
        if dt <= tolerance:
            synced_queue.put(((stamp1 + stamp2) / 2, pc1, pc2))
            buffer_1.popleft()
            buffer_2.popleft()
        elif stamp1 < stamp2:
            # Frame 1 too early, discard
            log_warn("[sync] Sensor 1 frame too early — discarding")
            buffer_1.popleft()
        else:
            # Frame 2 too early, discard
            log_warn("[sync] Sensor 2 frame too early — discarding")
            buffer_2.popleft()


