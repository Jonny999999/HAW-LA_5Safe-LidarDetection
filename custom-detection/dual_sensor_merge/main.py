from collections import deque
import time
import numpy as np
import open3d as o3d
import os
import laspy

import config as config # import entire config (use with prefix)
from receiver import start_receiver_thread
from decoder import decode_loop, frame_synchronizer
from processor import process_and_visualize_latest_frame
from visualizer import initialize_visualizer, visualize_single_frame, visualize_dual_frame, pick_point_from_cloud
from utils import log_info, log_warn, log_debug
from shared_types import StampCloudTuple
from playback_control import start_playback_input_thread, should_advance_frame, get_pick_request, clear_pick_request
import filters as filters

import queue
#from queue import Queue
#import threading

from multiprocessing import Process, Queue
import multiprocessing



def main():

    # === Queue 1: Transfer of raw UDP/PCAP packets ===
    # Filled by: receiver.py → start_receiver() thread
    # Read by:  decoder.py → decode_loop()
    # 2 packet queues for 2 sensors
    # TODO: Reduce latency, temporary reduced queue size from 1000 to 100
    udp_packet_queue_1 = Queue(maxsize=100)
    udp_packet_queue_2 = Queue(maxsize=100)


    # === Queue 2: Completed 360° scan point clouds ===
    # Filled by: decode.py -> decode_loop() 
    # Read by: frame_synchroniser thread
    # TODO: Reduce latency, temporary reduced queue size from 200 to 5
    decoded_pointcloud_frames_queue_1 = Queue(maxsize=500)
    decoded_pointcloud_frames_queue_2 = Queue(maxsize=500)






    # === Start packet receiver processes ===
    # This will feed udp_packet_queue (live UDP or PCAP mode)
    #TODO: handle case only one file defined
    start_receiver_thread(
        mode =           config.DATA_RECEIVE_MODE, # either 'UDP' or 'PCAP'
        packet_queue =   udp_packet_queue_1,
        sensor_id =      1,
        pcap_path =      config.PCAP_FILE_1,
        udp_listen_ip =  config.UDP_LISTEN_IP_SENSOR1,
        udp_port =       config.UDP_PORT_SENSOR1)

    start_receiver_thread(
        mode =           config.DATA_RECEIVE_MODE, # either 'UDP' or 'PCAP'
        packet_queue =   udp_packet_queue_2,
        sensor_id =      2,
        pcap_path =      config.PCAP_FILE_2,
        udp_listen_ip =  config.UDP_LISTEN_IP_SENSOR2,
        udp_port =       config.UDP_PORT_SENSOR2)



    # === Start decoding processes ===
    # Pulls packets from udp_packet_queue, assembles full scans, pushes to decoded_pointcloud_frames_queue
    # using multiprocessing instead of threading to run on actual cpu cores
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_1, udp_packet_queue_1, 1)).start()
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_2, udp_packet_queue_2, 2)).start()



    # === Start process for synchronising the decoded frames ===
    # queue for synchronized pointclouds (frames)
    # TODO: Reduce latency, temporary reduced queue size from 10 to 2
    synced_frame_queue = Queue(maxsize=2)

    Process(target=frame_synchronizer, args=(
        decoded_pointcloud_frames_queue_1,
        decoded_pointcloud_frames_queue_2,
        synced_frame_queue,
    )).start()



    # === Start Thread for terminal input ===
    # create thread for parsing terminal user input (payback_control)
    start_playback_input_thread()




    # === Initialize Open3D windows ===
    ### #visualizer = initialize_visualizer()
    # create 3 visualizer windows
    vis1 = initialize_visualizer("Sensor 1")
    vis2 = initialize_visualizer("Sensor 2")
    vis_merged = initialize_visualizer("Merged View")

    # variables for drawing the pointclouds
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    pcd_merged = o3d.geometry.PointCloud()



    # Transformation (Translation and Rotation) Matrix. calculated in calculate_transformation_maxtrix.py based on 3 Points

    # 2025.05.26: works for `2025-05-20_dual-sensor-test_sensor-xxx.pcap.gz`
    # Determined 3 reference points using recorded data
    # TODO: outsource transformation matrix to CONFIG, or even separate .json file for automatic transfer
    T_static = np.array([
                    [ 0.62288801, -0.78223369,  0.01099957, -9.16811678],
                    [ 0.77832635,  0.6210714,   0.09207824, -1.12348435],
                    [-0.07885822, -0.04879318,  0.99569102, -0.61174572],
                    [ 0.0,         0.0,         0.0,         1.0]
    ])


    # 2025.05.28: works for `data/testdata/2025-05-28_dual-sensor-test_sensor-xxx.pcap.gz`
    # Determined 3 reference points during live sensor setup
    # TODO: outsource transformation matrix to CONFIG, or even separate .json file for automatic transfer
    T_static = np.array([
                 [ 0.66810762, 0.7309481, -0.13909377, 6.27345587],
                 [-0.74305003, 0.66519418,-0.07343942,-6.48224065],
                 [ 0.03884396, 0.15241907, 0.98755232, 1.1114492 ],
                 [ 0,          0,          0,          1        ],
    ])


    # -------------- Initialisiere Liste für das Ablegen der merged Pointcloud:
    all_merged_points = []

    save = 0

    # === Main loop ===
    # update visualizer windows
    # handle play/pause/launch-point-picker
    while True:
        # Check if a pick was requested by terminal input
        pick_req = get_pick_request() # by entering e.g. "pick1" in console and pressing enter
        if pick_req:
            log_info(f"[main] Executing pick request for {pick_req}")
            if pick_req == "sensor1":
                idxs = pick_point_from_cloud(pc1, "Sensor 1")
                log_warn(f"[main] Picked indices of Sensor1: {idxs}, Resuming main loop")
            elif pick_req == "sensor2":
                idxs = pick_point_from_cloud(pc2, "Sensor 2")
                log_warn(f"[main] Picked indices of Sensor2: {idxs}, Resuming main loop")
            clear_pick_request()
            continue  # wait until resumed

        # check if paused
        if not should_advance_frame():
            # when paused, no data update, only run handlers (camera control responsive)
            vis1.poll_events(); vis1.update_renderer()
            vis2.poll_events(); vis2.update_renderer()
            vis_merged.poll_events(); vis_merged.update_renderer()
            time.sleep(0.05)
            continue # wait until resumed


        # pull next synced pointclouds from queue
        # stamp, pc1, pc2 = synced_frame_queue.get()


        # --------------- TESTING Ende der Datei erkennen. durch warten von einigen Sekunden und dann Abbruch
        try:
            stamp, pc1, pc2 = synced_frame_queue.get(timeout=5.0)  # 5 Sekunden warten
        except queue.Empty:
            log_info("[main] No more frames in synced_frame_queue. Exiting loop.")
            break

        # splice down pc1 and pc2 to XYZ Coordinates
        pc1 = pc1[:, :3]	
        pc2 = pc2[:, :3]	

        # Create Pointcloud from 3 dimensional array
        pc2_03dpc  = o3d.geometry.PointCloud()
        pc2_03dpc.points = o3d.utility.Vector3dVector(pc2.astype(np.float64))


        # === Update single sensor visualization ===
        # update visualizer windows with new pointclouds
        log_info(f"[main] Updating views with synced frame from {stamp:.3f}s")
        visualize_single_frame(pc1, vis1, pcd1, color=[0.0, 0.5, 1.0])
        visualize_single_frame(pc2, vis2, pcd2, color=[1.0, 0.5, 0.0])

        # Merge
        # TODO add transformation here
        #merged_points = np.vstack((pc1, pc2))
        #visualize_single_frame(merged_points, vis_merged, pcd_merged, color=[0.7, 0.7, 0.7])  # gray
        

        # === Apply transformation ===
        pc2_03dpc.transform(T_static)
        # Punktwolken zusammenfügen im Koordinatensystem von Sensor A


        # === Merge pointclouds ===
        merged_points = np.vstack((pc1, np.asarray(pc2_03dpc.points)))
        # visualize_single_frame(merged_points, vis_merged, pcd_merged, color=[0.7, 0.7, 0.7])  # gray


        # === Filter relevant points ===
        # filter out points outside of the configured polygon (config.py)
        # also drop points that are above certain z coordinate (1m)
        cropped_points = filters.crop_points_within_xy_polygon(merged_points, polygon_xy=config.CROP_POINTCLOUD_POLYGON, visualizer=vis_merged, draw_box=True, z_max_height_threshold=1)


        MOTION_DETECTION_ENABLED = True
        if MOTION_DETECTION_ENABLED:
            process_and_visualize_latest_frame(cropped_points, vis_merged)
        else:
            # === Visualize transformed,merged,cropped points ===
            # draw both clouds (different colors)
            if False:
                # visualize sensor merge (points sensor1 + transformed points sensor2 in red)
                visualize_dual_frame(pc1, np.asarray(pc2_03dpc.points), vis_merged)
            else:
                # visualize applied point filtering (all merged-points + points after crop in red)
                visualize_dual_frame(merged_points, cropped_points, vis_merged)


        # === Track finished frames ===
        # TODO: remove this, instead call e.g. create_laz_file_from_pointcloud() when FILE_EXPORT_ENABLE is set
        # add Frame to all_merged_points list
        if config.FILE_EXPORT_ENABLE:
            if save >= 6:
                #all_merged_points.append(merged_points)
                all_merged_points.append(cropped_points)
                save = 0
            else:
                save += 1



    # ------------- Erstellung der Dateien aus merged point cloud 
    
    # TODO: outsource this, also run per frame instead of at the end
    output_dir = "output/laz"
    os.makedirs(output_dir, exist_ok=True)
    if config.FILE_EXPORT_ENABLE:
        if all_merged_points:  # nur wenn überhaupt etwas gesammelt wurde
            num_frames = len(all_merged_points)
            log_info(f"[main] Saving {num_frames} individual frame pointclouds...")

            for i, merged_points in enumerate(all_merged_points):
                points = merged_points.astype(np.float32)

                header = laspy.LasHeader(point_format=3, version="1.4")
                las = laspy.LasData(header)

                # LAS expects scaled integers internally, so set scale to 0.001 for mm precision
                header.scales = [0.001, 0.001, 0.001]
                header.offsets = [0.0, 0.0, 0.0]

                las.x = points[:, 0]
                las.y = points[:, 1]
                las.z = points[:, 2]

                filename = os.path.join(output_dir, f"frame_{i:04d}.laz")
                las.write(filename)  # auto compress to .laz if laszip is installed


            log_info("Frames saved")

        else:
            log_warn("[main] No pointclouds merged. Nothing to save.")


    for p in multiprocessing.active_children():
        print(f"[EXIT] Killing process {p.pid}")
        p.terminate()
        p.join()
        print("[EXIT] force-killing script...")
        os._exit(0)



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

# call main() when file called directly
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted from keyboard.")
        # TODO kill all started processes here
        for p in multiprocessing.active_children():
            print(f"[EXIT] Killing process {p.pid}")
            p.terminate()
            p.join()
        print("[EXIT] force-killing script...")
        os._exit(0)
