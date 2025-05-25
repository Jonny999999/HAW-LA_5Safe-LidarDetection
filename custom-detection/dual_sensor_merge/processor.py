import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from config import COUNT_PEOPLE_ENABLED, COUNT_PEOPLE_DRAW_BOXES, MODE_SECOND_DATA_SET, CROP_POINTCLOUD_POLYGON, CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER
from utils import log_info, log_warn, log_debug
from visualizer import visualize_dual_frame
from filters import apply_highpass_filter, remove_isolated_points, crop_points_within_xy_polygon
from people_detection import estimate_moving_people



def process_and_visualize_latest_frame(frame_buffer, visualizer):
    """
    Processes the latest frame in the rolling buffer:
    - Applies motion filters (high-pass, denoise)
    - Detects moving clusters
    - Visualizes both raw and filtered data

    Args:
        frame_buffer (deque): Rolling buffer of point clouds (np.ndarray N x 3)
        visualizer (open3d.visualization.Visualizer): Initialized Open3D window
    """
    if not frame_buffer:
        log_warn("Frame buffer is empty.")
        return

    # Latest raw LiDAR scan
    latest_frame = frame_buffer[-1]

    # TODO: Optimize the selected filter combination configuration (more general approach to chain them)
    # Select transformation/filter mode
    if MODE_SECOND_DATA_SET == "OLDEST":
        filtered_frame = frame_buffer[0]  # Use oldest frame directly
        #log_info("Visualizing oldest frame (baseline).")

    elif MODE_SECOND_DATA_SET == "HIGHPASS":
        filtered_frame = apply_highpass_filter(frame_buffer, minMovedMetersThreshold=0.2)
        #log_info("Visualizing high-pass filtered frame.")

    elif MODE_SECOND_DATA_SET == "HIGHPASS+DENOISE":
        highpass_points = apply_highpass_filter(frame_buffer, minMovedMetersThreshold=0.1)
        filtered_frame = remove_isolated_points(highpass_points, nb_points=20, radius=0.3)

    elif MODE_SECOND_DATA_SET == "HIGHPASS+CROP+DENOISE":
        # Define rectangular crop region (clockwise or counter-clockwise)
        highpass_points = apply_highpass_filter(frame_buffer, minMovedMetersThreshold=0.1)
        cropped_frame = crop_points_within_xy_polygon(highpass_points, polygon_xy=CROP_POINTCLOUD_POLYGON, visualizer=visualizer, draw_box=True)
        filtered_frame = remove_isolated_points(cropped_frame, nb_points=20, radius=0.3)
        #log_info("Visualizing high-pass + denoised frame.")

    else:
        log_warn(f"Invalid MODE_SECOND_DATA_SET: {MODE_SECOND_DATA_SET}")
        return

    import open3d as o3d
    import numpy as np

    # open additional viewer window to manually get pint coordinates (freezes script here) useful for defining the crop polygon points
    if CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER:
        log_warn("CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER is enabled -> showing current pointcloud for selecting points to get coordinates")
        log_warn("use SHIFT+Click on a point to show coordinates")
        # Example: your pointcloud as Nx3 numpy array
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(frame_buffer[-1])

        # Open viewer — pick with Shift + Left Click
        o3d.visualization.draw_geometries_with_editing([pc])

    # Optional: Count moving people via DBSCAN
    if COUNT_PEOPLE_ENABLED:
        estimate_moving_people(filtered_frame, distance_threshold=0.5, min_points=50, visualizer=visualizer)

    # Visualize both point clouds
    visualize_dual_frame(latest_frame, filtered_frame, visualizer)



