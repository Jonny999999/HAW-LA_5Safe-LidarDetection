import numpy as np
import open3d as o3d
from config import COUNT_PEOPLE_DRAW_BOXES
from utils import log_info, log_warn

drawn_bounding_boxes = []  # Persistent list of bounding box geometries


# ===============================
# === PEOPLE DETECTION (CLUSTERS)
# ===============================

def estimate_moving_people(pointcloud_np, distance_threshold=0.5, min_points=30, visualizer=None):
    """
    Detects and optionally visualizes clusters of points likely to be moving people.

    Args:
        pointcloud_np (np.ndarray): N x 3 array of points
        distance_threshold (float): DBSCAN radius (meters)
        min_points (int): Minimum points to consider a cluster
        visualizer (Visualizer): Optional Open3D visualizer for bounding boxes

    Returns:
        int: Estimated number of people
    """
    global drawn_bounding_boxes


    MAX_INPUT_POINTS = 5000  # Prevent performance issues when input is too large
    if pointcloud_np.shape[0] > MAX_INPUT_POINTS:
        log_warn(f"Too many points ({pointcloud_np.shape[0]}), skipping people detection.")
        return 0

    if pointcloud_np.shape[0] == 0:
        log_info("No dynamic points to analyze.")
        return 0

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

    # Run DBSCAN clustering to identify dense regions
    labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))

    # Handle empty/noise result
    if labels.size == 0 or np.max(labels) < 0:
        log_info("No clusters found.")
        return 0

    unique_labels, counts = np.unique(labels, return_counts=True)
    valid_clusters = 0

    if COUNT_PEOPLE_DRAW_BOXES and visualizer is not None:
        # Remove previous bounding boxes from scene
        for box in drawn_bounding_boxes:
            visualizer.remove_geometry(box, reset_bounding_box=False)
        drawn_bounding_boxes.clear()

        # Iterate through valid clusters and draw bounding boxes
        for cluster_id, count in zip(unique_labels, counts):
            if cluster_id == -1 or count < min_points:
                continue  # Skip noise or too-small clusters

            valid_clusters += 1

            # Extract cluster points and compute its bounding box
            indices = np.where(labels == cluster_id)[0]
            cluster_pcd = pcd.select_by_index(indices)
            bbox = cluster_pcd.get_axis_aligned_bounding_box()
            bbox.color = (0.0, 0.0, 0.5)  # Dark blue color for bounding boxes

            # Add to visualizer and store for later removal
            visualizer.add_geometry(bbox, reset_bounding_box=False)
            drawn_bounding_boxes.append(bbox)

    log_info(f"people_detection: Estimated number of moving people: {valid_clusters}")
    return valid_clusters