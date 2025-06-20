import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from collections import deque
import time

from config import COUNT_PEOPLE_ENABLED, COUNT_PEOPLE_DRAW_BOXES, MODE_SECOND_DATA_SET, CROP_POINTCLOUD_POLYGON, CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER, POINTCLOUD_HISTORY_BUFFER_SIZE, PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON
from utils import log_info, log_warn, log_debug, serialize_numpy_array
from visualizer import visualize_dual_frame, draw_2d_polygon, draw_bounding_box, draw_cluster_boxes
from filters import apply_highpass_filter, remove_isolated_points, crop_points_within_xy_polygon
from people_detection import estimate_moving_people, track_moving_clusters, track_room_occupancy


# cache for cluster tracking
fast_cluster_detection_cache = {}

# === Rolling buffer for motion filtering ===
# Used for highpass, temporal denoise, clustering
# This stores the *XYZ arrays* (after [:, :3])
pointcloud_history_buffer = deque(maxlen=POINTCLOUD_HISTORY_BUFFER_SIZE)

def process_and_visualize_latest_frame(new_pointcloud, visualizer, status_cache):
    """
    Processes the latest frame in the rolling buffer:
    - Applies motion filters (high-pass, denoise)
    - Detects moving clusters
    - Visualizes both raw and filtered data

    Args:
        pointcloud_history_buffer (deque): Rolling buffer of point clouds (np.ndarray N x 3)
        visualizer (open3d.visualization.Visualizer): Initialized Open3D window
    """

    # Strip to XYZ only (drop intensity/ring/time if present)
    xyz_points = new_pointcloud[:, :3]
    # Add to rolling history buffer
    pointcloud_history_buffer.append(xyz_points)

    # Wait until enough frames for filters
    if len(pointcloud_history_buffer) < POINTCLOUD_HISTORY_BUFFER_SIZE:
        log_warn(f"[processor] too few frames in buffer for processing, waiting for buffer to fill up...({len(pointcloud_history_buffer)}/{POINTCLOUD_HISTORY_BUFFER_SIZE})")
        return

    # Latest raw LiDAR scan
    latest_frame = pointcloud_history_buffer[-1]

    # TODO: Optimize the selected filter combination configuration (more general approach to chain them)
    # Select transformation/filter mode
    t1 = time.time()
    if MODE_SECOND_DATA_SET == "OLDEST":
        filtered_frame = pointcloud_history_buffer[0]  # Use oldest frame directly
        #log_info("Visualizing oldest frame (baseline).")

    elif MODE_SECOND_DATA_SET == "HIGHPASS":
        filtered_frame = apply_highpass_filter(pointcloud_history_buffer, minMovedMetersThreshold=0.1)
        #log_info("Visualizing high-pass filtered frame.")

    elif MODE_SECOND_DATA_SET == "HIGHPASS+DENOISE":
        highpass_points = apply_highpass_filter(pointcloud_history_buffer, minMovedMetersThreshold=0.1)
        filtered_frame = remove_isolated_points(highpass_points, nb_points=40, radius=0.3)

    elif MODE_SECOND_DATA_SET == "HIGHPASS+CROP+DENOISE":
        # Define rectangular crop region (clockwise or counter-clockwise)
        highpass_points = apply_highpass_filter(pointcloud_history_buffer, minMovedMetersThreshold=0.15)
        cropped_frame = crop_points_within_xy_polygon(highpass_points, polygon_xy=CROP_POINTCLOUD_POLYGON, visualizer=visualizer, draw_box=True)
        filtered_frame = remove_isolated_points(cropped_frame, nb_points=20, radius=0.3)
        #log_info("Visualizing high-pass + denoised frame.")

    else:
        log_warn(f"Invalid MODE_SECOND_DATA_SET: {MODE_SECOND_DATA_SET}")
        return
    status_cache.update_status_key("TIMING_MOTION_DETECTION__APPLY_FILTERS", f"{(time.time() - t1)*1000:.0f} ms", trigger_file_update=False)

    # === Update cached pointcloud that is sent to dashboard via TCP ===
    t2 = time.time()
    status_cache.update_dashboard_key("pointcloud_highpass_denoised_numpyarray", filtered_frame)

    import open3d as o3d
    import numpy as np

    # open additional viewer window to manually get pint coordinates (freezes script here) useful for defining the crop polygon points
    if CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER:
        log_warn("CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER is enabled -> showing current pointcloud for selecting points to get coordinates")
        log_warn("use SHIFT+Click on a point to show coordinates")
        # Example: your pointcloud as Nx3 numpy array
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pointcloud_history_buffer[-1])

        # Open viewer — pick with Shift + Left Click
        o3d.visualization.draw_geometries_with_editing([pc])


    if COUNT_PEOPLE_ENABLED:
        # advanced tracking of moving clusters
        # slow detection to be sure, long retain
        tracked_clusters_precise = track_moving_clusters(
            filtered_frame,
            status_cache=status_cache,
            cluster_cache=None,
            distance_threshold=0.5, # points within that radius are merged as one cluster
            min_points=70, # min points in a cluster to considered as potential cluster/movement at all
            max_moving_points_ignore_frame=4000, # ignore entire frame if e.g. sensor moved
            min_z_height=0.5, # initial detection threshold
            min_volume_m3=0.2, # initial detection threshold
            min_frames_to_confirm=12, # frame count the initial thresholds have to be fulfilled to be added as cluster
            retain_frames=100, # at 5fps
            match_threshold=1.2, # distance of cluster center from old to new one to be detected as a match
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
            max_moving_points_ignore_frame=4000, # ignore entire frame if e.g. sensor moved
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
            draw_cluster_boxes(tracked_clusters_fast, visualizer)
            draw_2d_polygon(PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON, visualizer, color=(1,0.6,0)) # draw room polygon in orange

        #=== count people leaving/entering ===
        # count people entering and leaving the room
        people_inside = track_room_occupancy(
            #tracked_clusters_fast,
            tracked_clusters_precise,
            polygon_xy_inside_area=PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON,
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

        ## old people estimation TODO: drop this
        #estimate_moving_people(filtered_frame, distance_threshold=0.5, min_points=120, visualizer=visualizer)


    # Visualize both point clouds
    if visualizer is not None:
        visualize_dual_frame(latest_frame, filtered_frame, visualizer)
        status_cache.update_status_key("TIMING_MOTION_DETECTION__VISUALIZER_UPDATE", f"{(time.time() - t3)*1000:.0f} ms", trigger_file_update=False)



