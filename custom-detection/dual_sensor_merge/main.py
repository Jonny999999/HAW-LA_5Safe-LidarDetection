from collections import deque
import time
import numpy as np
import open3d as o3d

from config import USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM, PCAP_FILE_1, PCAP_FILE_2, POINTCLOUD_HISTORY_BUFFER_SIZE
from receiver import start_receiver_thread
from decoder import decode_loop, frame_synchronizer
from processor import process_and_visualize_latest_frame
from visualizer import initialize_visualizer, visualize_single_frame, visualize_dual_frame
from utils import log_info, log_warn, log_debug
from shared_types import StampCloudTuple

import queue
#from queue import Queue
#import threading

from multiprocessing import Process
from multiprocessing import Queue


def main():

    # === Queue 1: Transfer of raw UDP/PCAP packets ===
    # Filled by: receiver.py → start_receiver() thread
    # Read by:  decoder.py → decode_loop()
    # 2 packet queues for 2 sensors
    udp_packet_queue_1 = Queue(maxsize=1000)
    udp_packet_queue_2 = Queue(maxsize=1000)


    # === Queue 2: Completed 360° scan point clouds ===
    # Filled by: decode.py -> decode_loop() 
    # Read by: frame_synchroniser thread
    decoded_pointcloud_frames_queue_1 = Queue(maxsize=10)
    decoded_pointcloud_frames_queue_2 = Queue(maxsize=10)


    ### # === Rolling buffer for motion filtering ===
    ### # Used for highpass, temporal denoise, clustering
    ### # This stores the *XYZ arrays* (after [:, :3])
    ### pointcloud_history_buffer = deque(maxlen=POINTCLOUD_HISTORY_BUFFER_SIZE)




    # === Start packet receiver thread ===
    # This will feed udp_packet_queue (live UDP or PCAP mode)
    start_receiver_thread(PCAP_FILE_1, udp_packet_queue_1, sensor_id=1)
    start_receiver_thread(PCAP_FILE_2, udp_packet_queue_2, sensor_id=2)
    #TODO: handle case only one file defined


    # === Start decoding thread ===
    # Pulls packets from udp_packet_queue, assembles full scans, pushes to decoded_pointcloud_frames_queue
    # using multiprocessing instead of threading to run on actual cpu cores
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_1, udp_packet_queue_1, 1)).start()
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_2, udp_packet_queue_2, 2)).start()



    # === Start process for synchronising the decoded frames ===
    # queue for synchronized pointclouds (frames)
    synced_frame_queue = Queue(maxsize=10)

    Process(target=frame_synchronizer, args=(
        decoded_pointcloud_frames_queue_1,
        decoded_pointcloud_frames_queue_2,
        synced_frame_queue,
    )).start()



    # === Initialize Open3D window ===
    ### #visualizer = initialize_visualizer()
    # create 3 visualizer windows
    vis1 = initialize_visualizer("Sensor 1")
    vis2 = initialize_visualizer("Sensor 2")
    vis_merged = initialize_visualizer("Merged View")

    # variables for drawing the pointclouds
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    pcd_merged = o3d.geometry.PointCloud()

    # repeatedly update visualizers with synced pointcloud
    while True:
        # wait for synced pointclouds
        stamp, pc1, pc2 = synced_frame_queue.get()
        log_info(f"[main] Synced frame at {stamp:.3f}s")

        # show each individually
        visualize_single_frame(pc1, vis1, pcd1, color=[0.0, 0.5, 1.0])  # blue
        visualize_single_frame(pc2, vis2, pcd2, color=[1.0, 0.5, 0.0])  # orange

        # Merge
        # TODO add transformation here
        #merged_points = np.vstack((pc1, pc2))
        #visualize_single_frame(merged_points, vis_merged, pcd_merged, color=[0.7, 0.7, 0.7])  # gray

        # draw both clouds (different colors)
        visualize_dual_frame(pc1, pc2, vis_merged)



    # old code for detecting motion
    ### # === Main loop: process and visualize each complete scan ===
    ### while True:
    ###     try:
    ###         # Block until a full scan is available, with timeout to keep visualizer responsive
    ###         stamp, pointcloud = decoded_pointcloud_frames_queue_1.get(timeout=0.1)
    ###     except queue.Empty:
    ###         #log_debug("waiting for full frame timed out, updating visualizer (responsive)")
    ###         # Timeout expired, no new frame — keep the visualizer responsive (camera control)
    ###         visualizer.poll_events()
    ###         visualizer.update_renderer()
    ###         continue

    ###     # Strip to XYZ only (drop intensity/ring/time if present)
    ###     xyz_points = pointcloud[:, :3]

    ###     # Add to rolling history buffer
    ###     pointcloud_history_buffer.append(xyz_points)

    ###     # Wait until enough frames for filters
    ###     if len(pointcloud_history_buffer) >= POINTCLOUD_HISTORY_BUFFER_SIZE:
    ###         # Process next frame for the gui update
    ###         # Note: This / everything that uses the visualizer has to be in the main thread (where visualizer was initialized)
    ###         # TODO: use open3d gui API (also has control elements etc) or custom lamda cmd queue to have separate thread for the gui
    ###         process_and_visualize_latest_frame(pointcloud_history_buffer, visualizer)
    ###     else:
    ###         log_warn(f"too few frames in buffer for processing, waiting for buffer to fill up...({len(pointcloud_history_buffer)}/{POINTCLOUD_HISTORY_BUFFER_SIZE})")




# call main() when file called directly
if __name__ == "__main__":
    main()
