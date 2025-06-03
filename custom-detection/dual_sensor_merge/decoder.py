import velodyne_decoder as vd
from collections import namedtuple
from collections import deque
import time
from config import get_decoder_config
from utils import log_info, log_warn, log_debug, log_error
from queue import Full

from shared_types import StampCloudTuple
from status_file import update_status_single_key

from multiprocessing import Queue  # For the Queue class
from queue import Full, Empty      # For the exceptions

# Custom function for decoding and accumulating multiple UDP packets for full decoded pointcloud (one scan frame)

#ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
ResultTuple = StampCloudTuple
decoder = vd.StreamDecoder(get_decoder_config())

def decode_loop(frame_queue, udp_packet_queue, sensor_id):
    #log_debug(f"Received {len(data)} bytes from {src_ip}:{src_port} → {UDP_IP}:{UDP_PORT}")
    log_warn(f"Starting decoder loop for sensr {sensor_id}")
    udp_packet_count = 0
    current_stamp = None  # Track last packets timestamp in the frame
    stats_last_frame_decoded_time = time.time()

    while True:
        # get timestamp packet received + packet from receiver queue
        current_stamp, packet = udp_packet_queue.get()

        # check for corrent size
        if len(packet) != vd.PACKET_SIZE:
            log_warn(f"[decoder {sensor_id}] Skipping malformed packet of unexpected size. (expected {vd.PACKET_SIZE} but received {len(packet)})")
            continue
        else:
            udp_packet_count += 1
        
        # decode with part of official velodyne library
        result = decoder.decode(current_stamp, packet)

        # has pointcloud data when full scan accumulated over several packets
        if result:
            stats_framerate = 1/(time.time() - stats_last_frame_decoded_time)
            update_status_single_key(
                f"TIMING_FRAMERATE_DECODER_{sensor_id}",
                f"{stats_framerate:.1f} fps",
                trigger_file_update=False
            )
            stats_last_frame_decoded_time = time.time()
            
            log_debug(f"[decoder {sensor_id}] Frame decoded completely ({udp_packet_count} packets)")
            udp_packet_count = 0
            # Add pointcloud to queue
            try:
                #log_debug(f"[decoder {sensor_id}] Result type: {type(result)}, content: {result}")
                #frame_queue.put(ResultTuple(*result), block=False)

                # flatten result structure (drop timepair object only keep timestamp)
                timepair, points = result
                # TimePair has host and device time (host=1748687909, device=1748688660)
                # device sounds good but both sensors have different time set, so useless for syncing...
                # host is also no good, since pcap file can be anything, in UDP there is queue time also
                # use timestamp extracted from the udp-packet queue above:
                stamp = current_stamp
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
                except Empty:
                    log_error(f"[decoder {sensor_id}] Tried to drop frame but queue was already empty! (by other thread)")
                except Full:
                    log_error(f"[decoder {sensor_id}] Queue still full after dropping — giving up on frame.")

        else:
            #log_debug("[decoder] decoded packet, full scan frame not complete yet")
            pass










def frame_synchronizer(queue_1, queue_2, synced_queue, tolerance=0.05, max_buffer_size=50):
    """
    Synchronizes frames from two sources by timestamp.
    Uses a small buffer and finds best match instead of aggressively discarding.
    """
    buffer_1 = deque()
    buffer_2 = deque()
    failed_match_counter = 0
    max_failed_match_warn = 5 # max failed sync attempts in a row for warning to be printed (tolerance too tight)
    stats_last_frame_synced = time.time()

    def get_from_queue(q, buffer):
        """Blocking get with timeout, returns True if new item added"""
        try:
            item = q.get(timeout=0.01)
            buffer.append(item)
            return True
        except:
            return False

    while True:
        # Wait for at least one new item
        received1 = get_from_queue(queue_1, buffer_1)
        received2 = get_from_queue(queue_2, buffer_2)
        if not received1 and not received2:
            continue  # No new data, skip processing

        # Skip if any buffer is still empty
        if not buffer_1 or not buffer_2:
            time.sleep(0.01)
            continue

        # Search for the best match between buffer_1 and buffer_2
        best_pair = None
        best_dt = float('inf')


        for i, (t1, pc1) in enumerate(buffer_1):
            for j, (t2, pc2) in enumerate(buffer_2):
                dt = abs(t1 - t2)
                if dt < best_dt:
                    best_dt = dt
                    if dt <= tolerance:
                        best_pair = (i, j, (t1, t2, pc1, pc2))
        
        if best_pair:
            i, j, (t1, t2, pc1, pc2) = best_pair
            # Remove all earlier frames (up to and including matched ones)
            buffer_1 = deque(list(buffer_1)[i+1:])
            buffer_2 = deque(list(buffer_2)[j+1:])
            synced_stamp = (t1 + t2) / 2
            log_debug(f"[sync] Synced frames (Δt={best_dt:.3f}s) at {synced_stamp}")
            failed_match_counter = 0  # reset on success
            # Insert the successfully synced frames into queue
            stats_synced_frame_interval = int((time.time() - stats_last_frame_synced) * 1000)
            update_status_single_key(
                "TIMING_SYNCED_FRAMERATE",
                f"{1000/stats_synced_frame_interval:.1f} fps ({stats_synced_frame_interval} ms)",
                trigger_file_update=False
            )
            try:
                synced_queue.put((synced_stamp, pc1, pc2), timeout=0.01)
            except Full:
                try:
                    log_warn("[sync] Synced queue full, processing cant keept up? → dropping oldest synced frame.")
                    # Remove one old item to make space
                    _ = synced_queue.get_nowait()
                    # Try again to insert
                    synced_queue.put((synced_stamp, pc1, pc2), timeout=0.01)
                except:
                    log_error("[sync] Synced queue full and could not recover by dropping. -> losing newly synced frame")
            stats_last_frame_synced = time.time()
        else:
            failed_match_counter += 1
            if failed_match_counter >= max_failed_match_warn:
                log_warn(f"[sync] Consecutive failed frame syncs: {failed_match_counter}. (dropped frames)\n"
                         f"     min-dt={best_dt:.3f}s, tolerance={tolerance}. \n"
                         f"     -> Consider adjusting tolerance or buffer size")
            # No match found, but prevent buffer overflow
            if len(buffer_1) > max_buffer_size:
                dropped = buffer_1.popleft()
                log_warn(f"[sync] Dropped old Sensor 1 frame: {dropped[0]}")
                log_warn(f"[sync] increase max-buffer or decrease threshold? min-dt={best_dt}, threshold={tolerance}")
            if len(buffer_2) > max_buffer_size:
                dropped = buffer_2.popleft()
                log_warn(f"[sync] Dropped old Sensor 2 frame: {dropped[0]}")
                log_warn(f"[sync] increase max-buffer or decrease threshold? min-dt={best_dt}, threshold={tolerance}")