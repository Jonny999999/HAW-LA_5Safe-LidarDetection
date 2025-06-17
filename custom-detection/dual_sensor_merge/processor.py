import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from config import COUNT_PEOPLE_ENABLED, COUNT_PEOPLE_DRAW_BOXES, MODE_SECOND_DATA_SET, CROP_POINTCLOUD_POLYGON, CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER, POINTCLOUD_HISTORY_BUFFER_SIZE, PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON
from utils import log_info, log_warn, log_debug, serialize_numpy_array
from visualizer import visualize_dual_frame, draw_2d_polygon, draw_bounding_boxes
from filters import apply_highpass_filter, remove_isolated_points, crop_points_within_xy_polygon
from people_detection import estimate_moving_people, detect_moving_clusters, track_room_occupancy
from collections import deque


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

    # === Update cached pointcloud that is sent to dashboard via TCP ===
    status_cache.update_dashboard_key("pointcloud_highpass_denoised_seralizednumpyarray", serialize_numpy_array(filtered_frame))

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


    # Optional: Count moving people via DBSCAN
    if COUNT_PEOPLE_ENABLED:
        clusters = detect_moving_clusters(filtered_frame, distance_threshold=0.5, min_points=60, max_points=4000, status_cache=status_cache)
        if visualizer is not None:
            draw_bounding_boxes(clusters, visualizer)
            draw_2d_polygon(PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON, visualizer, color=(1,0.6,0)) # draw room polygon in orange
        #people_inside = track_room_occupancy(clusters, polygon_xy_inside_area=PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON, status_cache=status_cache)
            people_inside = track_room_occupancy(
            clusters,
            polygon_xy_inside_area=PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON,
            history_buffer=None,
            status_cache=status_cache,
            visualizer=visualizer
        )
        # === Update cached clusters that is sent to dashboard via TCP ===
        #print(f"Cluster type: {type(clusters[0])}, content: {clusters[0]}")
        status_cache.update_dashboard_key(
            "moving_people_clusters_arrayofserializednumpyarrays",
            [serialize_numpy_array(cluster[2].points) for cluster in clusters]
        )

        ## old people estimation TODO: drop this
        #estimate_moving_people(filtered_frame, distance_threshold=0.5, min_points=120, visualizer=visualizer)


    # Visualize both point clouds
    if visualizer is not None:
        visualize_dual_frame(latest_frame, filtered_frame, visualizer)



