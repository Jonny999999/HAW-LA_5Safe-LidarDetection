import numpy as np
import open3d as o3d
from config import COUNT_PEOPLE_DRAW_BOXES
from utils import log_info, log_warn
from collections import deque



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







# Globals (optionally refactor to avoid)
drawn_bounding_boxes = []

# Global buffer for room occupancy tracking (optional usage)
_default_history_buffer = deque(maxlen=5)






import numpy as np
import open3d as o3d
from shapely.geometry import Polygon, Point
from collections import deque
from utils import log_info, log_warn, log_debug

# Globals (optionally refactor to avoid)
drawn_bounding_boxes = []

# ========== CLUSTER DETECTION ==========
def detect_moving_clusters(pointcloud_np, distance_threshold=0.5, min_points=30, max_points=5000):
    """
    Detects moving clusters in a high-pass filtered point cloud.

    Returns:
        List of (centroid, cluster_pcd) tuples for valid clusters
    """
    if pointcloud_np.shape[0] > max_points:
        log_warn(f"Too many points ({pointcloud_np.shape[0]}), skipping cluster detection.")
        return []

    if pointcloud_np.shape[0] == 0:
        log_info("No dynamic points to analyze.")
        return []

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_np)
    labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))

    if labels.size == 0 or np.max(labels) < 0:
        log_info("No clusters found.")
        return []

    unique_labels = np.unique(labels)
    result_clusters = []

    valid_clusters_count = 0
    for cluster_id in unique_labels:
        valid_clusters_count += 1
        if cluster_id == -1:
            continue  # noise
        indices = np.where(labels == cluster_id)[0]
        cluster = pcd.select_by_index(indices)
        centroid = np.mean(np.asarray(cluster.points), axis=0)
        result_clusters.append((centroid, cluster))

    log_info(f"[moving clusters] detected moving clusters: {valid_clusters_count}")

    return result_clusters




# ========== ROOM ENTRY/EXIT TRACKING ==========
def track_room_occupancy(clusters, door_polygon, history_buffer=None, distance_threshold=1.0):
    """
    Tracks people entering or leaving a room via a virtual door polygon.

    Args:
        clusters: list of (centroid, cluster_pcd)
        door_polygon: list of (x, y) tuples defining the entry/exit zone
        history_buffer: deque storing previous frame centroids
        distance_threshold: max dist to match person across frames

    Returns:
        int: current estimated people inside
    """
    # use global buffer if not explicitly provided
    if history_buffer is None:
        history_buffer = _default_history_buffer

    poly = Polygon(door_polygon)

    # Track history
    current_centroids = [c for c, _ in clusters]
    history_buffer.append(current_centroids)
    if len(history_buffer) > 5:
        history_buffer.popleft()

    people_inside = 0
    tracked_paths = zip(*history_buffer) if history_buffer else []

    for path in tracked_paths:
        if len(path) < 2:
            continue

        start = path[0][:2]
        end = path[-1][:2]
        was_inside = poly.contains(Point(start))
        is_inside = poly.contains(Point(end))

        if not was_inside and is_inside:
            log_warn("[track_room_occupancy] Person ENTERED room")
            people_inside += 1
        elif was_inside and not is_inside:
            log_warn("[track_room_occupancy] Person LEFT room")
            people_inside -= 1
        elif is_inside:
            people_inside += 1  # still inside
        log_info(f"[track_room_occupancy] people inside: {people_inside}")

    return max(people_inside, 0)

