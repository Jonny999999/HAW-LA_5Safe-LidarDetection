import queue
from queue import Queue
from collections import deque
import time

from config import USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM, PCAP_FILE, POINTCLOUD_HISTORY_BUFFER_SIZE
from receiver import start_receiver
from decoder import decode_loop
from processor import process_and_visualize_latest_frame
from visualizer import initialize_visualizer
from utils import log_info, log_warn, log_debug



def main():

    # === Queue 1: Transfer of raw UDP/PCAP packets ===
    # Filled by: receiver.py → start_receiver() thread
    # Read by:  decoder.py → decode_loop()
    # NOTE: This is imported directly from receiver.py
    # receiver.py: udp_packet_queue = Queue(maxsize=1000)


    # === Queue 2: Completed 360° scan point clouds ===
    # Filled by: decode.py -> decode_loop() 
    # Read by: main thread (here)
    decoded_pointcloud_frames_queue = Queue(maxsize=10)


    # === Rolling buffer for motion filtering ===
    # Used for highpass, temporal denoise, clustering
    # This stores the *XYZ arrays* (after [:, :3])
    pointcloud_history_buffer = deque(maxlen=POINTCLOUD_HISTORY_BUFFER_SIZE)


    # === Initialize Open3D window ===
    visualizer = initialize_visualizer()


    # === Start packet receiver thread ===
    # This will feed udp_packet_queue (live UDP or PCAP mode)
    start_receiver()


    # === Start decoding thread ===
    # Pulls packets from udp_packet_queue, assembles full scans, pushes to decoded_pointcloud_frames_queue
    import threading
    threading.Thread(target=decode_loop, args=(decoded_pointcloud_frames_queue,), daemon=True).start()

    # === Main loop: process and visualize each complete scan ===
    while True:
        try:
            # Block until a full scan is available, with timeout to keep visualizer responsive
            stamp, pointcloud = decoded_pointcloud_frames_queue.get(timeout=0.1)
        except queue.Empty:
            #log_debug("waiting for full frame timed out, updating visualizer (responsive)")
            # Timeout expired, no new frame — keep the visualizer responsive (camera control)
            visualizer.poll_events()
            visualizer.update_renderer()
            continue

        # Strip to XYZ only (drop intensity/ring/time if present)
        xyz_points = pointcloud[:, :3]

        # Add to rolling history buffer
        pointcloud_history_buffer.append(xyz_points)

        # Wait until enough frames for filters
        if len(pointcloud_history_buffer) >= POINTCLOUD_HISTORY_BUFFER_SIZE:
            # Process next frame for the gui update
            # Note: This / everything that uses the visualizer has to be in the main thread (where visualizer was initialized)
            # TODO: use open3d gui API (also has control elements etc) or custom lamda cmd queue to have separate thread for the gui
            process_and_visualize_latest_frame(pointcloud_history_buffer, visualizer)
        else:
            log_warn(f"too few frames in buffer for processing, waiting for buffer to fill up...({len(pointcloud_history_buffer)}/{POINTCLOUD_HISTORY_BUFFER_SIZE})")




# call main() when file called directly
if __name__ == "__main__":
    main()
