import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from collections import deque
import time

import config as config
from utils import log_info, log_warn, log_debug, serialize_numpy_array
from visualizer import visualize_dual_frame, draw_2d_polygon, draw_bounding_box, draw_cluster_boxes, remove_near_duplicates
from filters import apply_highpass_filter, remove_isolated_points, crop_points_within_xy_polygon
from people_detection import track_moving_clusters, track_room_occupancy


# cache for cluster tracking
fast_cluster_detection_cache = {}

# === Rolling buffer for motion filtering ===
# Used for highpass, temporal denoise, clustering
# This stores the *XYZ arrays* (after [:, :3])
pointcloud_history_buffer = deque(maxlen=config.POINTCLOUD_HISTORY_BUFFER_SIZE)

# variables for point difference mode
reference_was_saved = False
reference_pointcloud = None

def process_and_visualize_latest_frame(new_pointcloud, visualizer, status_cache):
    """
    Processes the latest frame in the rolling buffer:
    - Applies motion filters (high-pass, denoise)
    - Detects moving clusters
    - Tracks moving clusters
    - counts people leaving and entering
    - Visualizes both raw and filtered data
    - Visualizes clusters polygons etc

    Args:
        pointcloud_history_buffer (deque): Rolling buffer of point clouds (np.ndarray N x 3)
        visualizer (open3d.visualization.Visualizer): Initialized Open3D window
    """

    # Strip to XYZ only (drop intensity/ring/time if present)
    xyz_points = new_pointcloud[:, :3]
    # Add to rolling history buffer
    pointcloud_history_buffer.append(xyz_points)

    # Wait until enough frames for filters
    if len(pointcloud_history_buffer) < config.POINTCLOUD_HISTORY_BUFFER_SIZE:
        log_warn(f"[processor] too few frames in buffer for processing, waiting for buffer to fill up...({len(pointcloud_history_buffer)}/{config.POINTCLOUD_HISTORY_BUFFER_SIZE})")
        return

    # Latest raw LiDAR scan
    latest_frame = pointcloud_history_buffer[-1]

    # Run Filter by configured mode to get the input pointcloud for detection
    t1 = time.time()
    if config.DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE == "OLDEST":
        filtered_frame = pointcloud_history_buffer[0]  # Use oldest frame directly (only useful for testing the buffer)
        #log_info("Visualizing oldest frame (baseline).")

    elif config.DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE == "HIGHPASS":
        filtered_frame = apply_highpass_filter(pointcloud_history_buffer, minMovedMetersThreshold=0.1)
        #log_info("Visualizing high-pass filtered frame.")

    elif config.DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE == "HIGHPASS+DENOISE":
        highpass_points = apply_highpass_filter(pointcloud_history_buffer, minMovedMetersThreshold=0.1)
        filtered_frame = remove_isolated_points(highpass_points, nb_points=40, radius=0.3)

    elif config.DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE == "CHANGED_POINTS_SINCE_START":
        global reference_was_saved, reference_pointcloud
        if not reference_was_saved:
            log_warn("[processor filter] first run - saving current pointcloud as reference")
            reference_pointcloud = new_pointcloud
            reference_was_saved = True
        # remove reference points from latest pointcloud
        filtered_frame = remove_near_duplicates(latest_frame, reference_pointcloud, threshold=0.3)
        # additionaly apply denoise filter (low settings) to get rid of isolated flickering points
        filtered_frame = remove_isolated_points(filtered_frame, nb_points=20, radius=0.2)
    else:
        log_warn(f"Invalid DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE: {config.DETECTION_ALGORITHM_INPUT_DATA_FILTER_MODE} -> fix config.py")
        return
    status_cache.update_status_key("TIMING_MOTION_DETECTION__APPLY_FILTERS", f"{(time.time() - t1)*1000:.0f} ms", trigger_file_update=False)

    # === Update cached pointcloud that is sent to dashboard via TCP ===
    t2 = time.time()
    status_cache.update_dashboard_key("pointcloud_highpass_denoised_numpyarray", filtered_frame)

    import open3d as o3d
    import numpy as np


    if config.COUNT_PEOPLE_ENABLED:
        # advanced tracking of moving clusters
        # slow detection to be sure, long retain
        tracked_clusters_precise = track_moving_clusters(
            filtered_frame,
            status_cache=status_cache,
            cluster_cache=None,
            distance_threshold=0.5, # points within that radius are merged as one cluster
            min_points=80, # min points in a cluster to considered as potential cluster/movement at all
            max_moving_points_ignore_frame=10000, # ignore entire frame if e.g. sensor moved
            min_z_height=0.9, # initial detection threshold
            min_volume_m3=0.23, # initial detection threshold
            min_frames_to_confirm=15, # frame count the initial thresholds have to be fulfilled to be added as cluster
            retain_frames=800, # at 5fps
            match_threshold=0.7, # distance of cluster center from old to new one to be detected as a match
            # when too large e.g. >1.2 matches tracked actually still clusters to noise thus we get more clusters than there are people...
            enable_logging=True,
        ) #using default detection thresholds (see definition)


        # second instance of people/cluster detection
        # fast detection fast forget -> detected clusters only used for counting people entering/leaving thus needs to be quicker
        global fast_cluster_detection_cache
        tracked_clusters_fast = track_moving_clusters(
            filtered_frame,
            cluster_cache=fast_cluster_detection_cache,
            distance_threshold=0.2, # points within that radius are merged as one cluster
            min_points=60, # min points in a cluster to considered as potential cluster/movement at all
            max_moving_points_ignore_frame=10000, # ignore entire frame if e.g. sensor moved
            min_z_height=0.5, # initial detection threshold
            min_volume_m3=0.2, # initial detection threshold
            min_frames_to_confirm=3, # frame count the initial thresholds have to be fulfilled to be added as cluster
            status_cache=None,
            retain_frames=5, # at 5fps
            match_threshold=0.6, # distance of cluster center from old to new one to be detected as a match
            enable_logging=False,
        ) #using default detection thresholds (see definition)
        status_cache.update_status_key("TIMING_MOTION_DETECTION__CLUSTER_DETECTION", f"{(time.time() - t2)*1000:.0f} ms", trigger_file_update=False)

        t3 = time.time()
        if visualizer is not None:
            #=== draw clusters ===
            draw_cluster_boxes(tracked_clusters_precise, visualizer)
            draw_2d_polygon(config.PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON, visualizer, color=(1,0.6,0)) # draw room polygon in orange

        #=== count people leaving/entering ===
        # count people entering and leaving the room
        people_inside = track_room_occupancy(
            #tracked_clusters_fast,
            tracked_clusters_fast,
            polygon_xy_inside_area=config.PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON,
            history_buffer=None,
            status_cache=status_cache,
            visualizer=visualizer
        )
        # === Update cached clusters that is sent to dashboard via TCP ===
        #print(f"Cluster type: {type(clusters[0])}, content: {clusters[0]}")
        status_cache.update_dashboard_key(
            "moving_people_clusters_arrayofserializednumpyarrays",
            [serialize_numpy_array(cluster["pcd"].points) for cluster in tracked_clusters_precise]
        )

    status_cache.update_status_key("TIMING_MOTION_TRACKING_ALGORITHM", f"{(time.time() - t1)*1000:.0f} ms", trigger_file_update=False)


    # Visualize both point clouds
    if visualizer is not None:
        visualize_dual_frame(latest_frame, filtered_frame, visualizer)
        status_cache.update_status_key("TIMING_MOTION_DETECTION__VISUALIZER_UPDATE", f"{(time.time() - t3)*1000:.0f} ms", trigger_file_update=False)



