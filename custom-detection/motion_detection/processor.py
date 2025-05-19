import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from config import COUNT_PEOPLE_ENABLED, COUNT_PEOPLE_DRAW_BOXES, MODE_SECOND_DATA_SET
from utils import log_info, log_warn, log_debug
from visualizer import visualize_dual_frame
from filters import apply_highpass_filter, remove_isolated_points
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
        #log_info("Visualizing high-pass + denoised frame.")

    else:
        log_warn(f"Invalid MODE_SECOND_DATA_SET: {MODE_SECOND_DATA_SET}")
        return

    # Optional: Count moving people via DBSCAN
    if COUNT_PEOPLE_ENABLED:
        estimate_moving_people(filtered_frame, distance_threshold=0.5, min_points=50, visualizer=visualizer)

    # Visualize both point clouds
    visualize_dual_frame(latest_frame, filtered_frame, visualizer)



