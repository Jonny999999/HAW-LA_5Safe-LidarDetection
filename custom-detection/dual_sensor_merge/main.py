# Imported libraries
from collections import deque
import time
import numpy as np
import open3d as o3d
import os
import laspy
import sys
import queue
import multiprocessing
from multiprocessing import Process, Queue

# Imports from custom files
import config as config # import entire config (use with prefix)
from receiver import start_receiver_thread
from decoder import decode_loop, frame_synchronizer
from processor import process_and_visualize_latest_frame
from visualizer import pick_point_from_cloud, update_visualizer_by_mode, initialize_all_used_visualizer_windows, get_visualizer_by_mode, handle_inputs_of_active_visualizers
from utils import log_info, log_warn, log_debug, serialize_numpy_array
from shared_types import StampCloudTuple
from playback_control import start_playback_input_thread, should_advance_frame, get_pick_request, clear_pick_request
import filters as filters
import exporter
from status_file import update_status_single_key, update_dashboard_key
from tcp_senderthread import run_sender_thread






# TODO 2025.06.03:
#   - fix: exit correctly, finish UDP socket due to error OSError: [Errno 98] Address already in use after restarting in UDP mode
#   - fix: random gui crash and sync fail (same as sensor disconnected), not happening when debug output on
#   - remove timeout no data
#   - fallback to 1 sensor if second one not sending
#   - visualization interfacae
#   - improved people tracking
#   - loglevels?
# 





def main():

    ############################
    ####### Declarations #######
    ############################

    # === Queue 1: Transfer of raw UDP/PCAP packets ===
    # Filled by: receiver.py → start_receiver() thread
    # Read by:  decoder.py → decode_loop()
    # 2 packet queues for 2 sensors
    # TODO: Reduce latency, temporary reduced queue size from 1000 to 100
    udp_packet_queue_1 = Queue(maxsize=1000)
    udp_packet_queue_2 = Queue(maxsize=1000)


    # === Queue 2: Completed 360° scan point clouds ===
    # Filled by: decode.py -> decode_loop() 
    # Read by: frame_synchroniser thread
    # TODO: Reduce latency, temporary reduced queue size from 200 to 5
    decoded_pointcloud_frames_queue_1 = Queue(maxsize=500)
    decoded_pointcloud_frames_queue_2 = Queue(maxsize=500)


    # === Queue 3: synchronized pointclouds of both sensors ===
    synced_frame_queue = Queue(maxsize=2)


    # Exporter class can export Frames in LAZ Format. Saving is enabled through Config File.
    # Enabling the Config throgh FILE_EXPORT_ENABLE = True will clear the output directory on initializing an exporter object
    LazFileSave = exporter.LazFrameExporter(output_dir="output/laz", skip_n_frames = 2)


    # variable for logging processing duration
    stats_processing_start_time = time.time()


    # === Initialize Open3D windows ===
    ### #visualizer = initialize_visualizer()
    # create 3 visualizer windows
    initialize_all_used_visualizer_windows()

    # variables for drawing the pointclouds
    pointcloud_1_o3d = o3d.geometry.PointCloud()
    pointcloud_2_o3d = o3d.geometry.PointCloud()
    pointcloud_merged_o3d = o3d.geometry.PointCloud()



    #######################################
    ###### Start threads / processes ######
    #######################################

    # === Start packet receiver processes ===
    # This will feed udp_packet_queue (live UDP or PCAP mode)
    #TODO: handle case only one file defined
    start_receiver_thread(
        mode =           config.DATA_RECEIVE_MODE, # either 'UDP' or 'PCAP'
        packet_queue =   udp_packet_queue_1,
        sensor_id =      1,
        pcap_path =      config.PCAP_FILE_1,
        udp_listen_ip =  config.UDP_LISTEN_IP_SENSOR1,
        udp_port =       config.UDP_PORT_SENSOR1,
        filtered_udp_port= config.PCAP_FILE_FILTER_UDP_PORT_SENSOR_1
        )

    start_receiver_thread(
        mode =           config.DATA_RECEIVE_MODE, # either 'UDP' or 'PCAP'
        packet_queue =   udp_packet_queue_2,
        sensor_id =      2,
        pcap_path =      config.PCAP_FILE_2,
        udp_listen_ip =  config.UDP_LISTEN_IP_SENSOR2,
        udp_port =       config.UDP_PORT_SENSOR2,
        filtered_udp_port= config.PCAP_FILE_FILTER_UDP_PORT_SENSOR_2
        )
    

    # === Start Sender Thread for Dashboard ===
    # Starts thread to send status file in json format to client that connects to the Server
    # Currently only one Client can connect to the Server
    run_sender_thread()


    # === Start decoding processes ===
    # Pulls packets from udp_packet_queue, assembles full scans, pushes to decoded_pointcloud_frames_queue
    # using multiprocessing instead of threading to run on actual cpu cores
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_1, udp_packet_queue_1, 1)).start()
    Process(target=decode_loop, args=(decoded_pointcloud_frames_queue_2, udp_packet_queue_2, 2)).start()


    # === Start process for synchronising the decoded frames ===
    # TODO: Reduce latency, temporary reduced queue size from 10 to 2
    Process(target=frame_synchronizer, args=(
        decoded_pointcloud_frames_queue_1, # reads from decoded pointclouds queues
        decoded_pointcloud_frames_queue_2,
        synced_frame_queue, # writes synced frame pair in synced_frame queue
    )).start()


    # === Start Thread for terminal input ===
    # create thread for parsing terminal user input (payback_control)
    start_playback_input_thread()








    #######################
    ###### Main Loop ######
    #######################
    # receive synced frames
    # update visualizer windows
    # run motion detection
    # handle play/pause/launch-point-picker
    while True:
        
        #=== get synced pointcloud from queue ===
        stamp, pointcloud_1_array, pointcloud_2_array = synced_frame_queue.get() # TODO: add timeout here to stay responsive when no data received?
        stats_processing_start_time = time.time()

        # splice down pc1 and pc2 to XYZ Coordinates
        pointcloud_1_array = pointcloud_1_array[:, :3]	
        pointcloud_2_array = pointcloud_2_array[:, :3]	


        # === Apply transformation ===
        # create Pointcloud from 3 dimensional array
        pointcloud_2_transformed_o3d = o3d.geometry.PointCloud()
        pointcloud_2_transformed_o3d.points = o3d.utility.Vector3dVector(pointcloud_2_array.astype(np.float64))
        # transform with matrix
        pointcloud_2_transformed_o3d.transform(config.TRANSFORMATION_MATRIX_SENSOR_2)


        # === Merge pointclouds ===
        pointcloud_merged_array = np.vstack((pointcloud_1_array, np.asarray(pointcloud_2_transformed_o3d.points)))
        # visualize_single_frame(merged_points, vis_merged, pcd_merged, color=[0.7, 0.7, 0.7])  # gray


        # === Filter relevant points ===
        # filter out points outside of the configured polygon (config.py)
        # also drop points that are above certain z coordinate (1m)
        pointcloud_merged_filtered_array = filters.crop_points_within_xy_polygon(pointcloud_merged_array, polygon_xy=config.CROP_POINTCLOUD_POLYGON, visualizer=get_visualizer_by_mode("merged_filtered"), draw_box=True, z_max_height_threshold=1)

        # === Update cached pointcloud that is sent to dashboard via TCP ===
        update_dashboard_key("pointcloud_merged_filtered_seralizednumpyarray", serialize_numpy_array(pointcloud_merged_filtered_array))

        # === Update visualizer windows ===
        # Define context with all needed arrays for visualization functions
        context = {
            "pointcloud_1_array": pointcloud_1_array,
            "pointcloud_2_array": pointcloud_2_array,
            "pointcloud_1_o3d": pointcloud_1_o3d,
            "pointcloud_2_o3d": pointcloud_2_o3d,
            "pc2_transformed": np.asarray(pointcloud_2_transformed_o3d.points),
            "pc_merged": pointcloud_merged_array,
            "pc_filtered": pointcloud_merged_filtered_array,
        }
        # update each visualizer depending on its mode
        update_visualizer_by_mode(config.VISUALIZER_WINDOW_1_MODE, context)
        update_visualizer_by_mode(config.VISUALIZER_WINDOW_2_MODE, context)
        update_visualizer_by_mode(config.VISUALIZER_WINDOW_3_MODE, context)


        # === run Motion Detection ===
        if config.MOTION_DETECTION_ENABLED:
            # determine which visualizer window is configured to display the motion detection output
            # run motion detection
            process_and_visualize_latest_frame(context["pc_filtered"], get_visualizer_by_mode("motion_detection"))


        # === File export of merged frames ===
        # Update exporter class with new Frame. Frame will not automatically be saved, depending on skip_n_frames Attribute
        # This Line does not have to be changed for the event, that File Export will be deactivated
        LazFileSave.save_frame(pointcloud_merged_filtered_array)


        # === handle launch point picker functionality ===
        # Check if a pick was requested by terminal input
        # TODO: the CLI approach does not integrate well anymore, rework?
        pick_req = get_pick_request() # by entering e.g. "pick1" in console and pressing enter
        if pick_req:
            log_info(f"[main] Executing pick request for {pick_req}")
            if pick_req == "sensor1":
                idxs = pick_point_from_cloud(pointcloud_1_array, "Sensor 1")
                log_warn(f"[main] Picked indices of Sensor1: {idxs}, Resuming main loop")
            elif pick_req == "sensor2":
                idxs = pick_point_from_cloud(pointcloud_2_array, "Sensor 2")
                log_warn(f"[main] Picked indices of Sensor2: {idxs}, Resuming main loop")
            clear_pick_request()
            continue  # wait until resumed
        # check if paused
        if not should_advance_frame():
            # when paused, no data update, only run handlers (camera control responsive)
            handle_inputs_of_active_visualizers()
            time.sleep(0.05)
            continue # wait until resumede_frame(pointcloud_merged_filtered_array)


        # === update statistics ===
        # done processing - Log processing duration to file
        stats_processing_duration_ms = int((time.time() - stats_processing_start_time) * 1000)
        update_status_single_key("TIMING_PROCESSING_DURATION_MS", f"{stats_processing_duration_ms} ms")







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
