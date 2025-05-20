import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from utils import log_warn, log_debug, log_info


# ===============================
# === FILTERING FUNCTIONS
# ===============================

def apply_highpass_filter(buffer, minMovedMetersThreshold=0.2):
    """
    Retains only the points from the latest frame that moved significantly
    compared to their nearest neighbor in the oldest frame.

    Args:
        buffer (deque): Rolling buffer of frames
        minMovedMetersThreshold (float): Distance threshold in meters

    Returns:
        np.ndarray: Filtered points from latest frame
    """
    if len(buffer) < 2:
        log_warn("Not enough frames in buffer for high-pass filtering.")
        return buffer[-1]

    # Get most recent and oldest frames in the buffer
    latest = buffer[-1]
    previous = buffer[0]

    # Build KD-Tree from the previous frame to allow efficient nearest-neighbor search
    # We need to do nearest neighbor search because the points in the frames received are not in the same memory position in each frame, even the amount of points varies each frame received
    tree = cKDTree(previous)

    # For each point in the latest frame, find distance to nearest neighbor in previous
    distances, _ = tree.query(latest, k=1)

    # Create a mask of points that moved farther than the threshold
    mask = distances > minMovedMetersThreshold

    # Apply the mask to keep only dynamic/moved points
    filtered = latest[mask]

    log_debug(f"HIGH-PASS: kept {filtered.shape[0]} of {latest.shape[0]} points.")
    return filtered



def remove_isolated_points(points, nb_points=3, radius=0.2):
    """
    Removes sparse outlier points from a cloud using radius filtering.

    Args:
        points (np.ndarray): Input points
        nb_points (int): Minimum neighbors within radius
        radius (float): Radius to consider for neighbors

    Returns:
        np.ndarray: Denoised points
    """
    # Convert to Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Apply Open3D radius outlier removal
    # `cl` is the cleaned point cloud; `ind` = indices of kept points
    cl, _ = pcd.remove_radius_outlier(nb_points, radius)

    # Convert back to NumPy
    filtered_points = np.asarray(cl.points)

    log_debug(f"DENOISE: Removed isolated points. Kept {len(filtered_points)} of {len(points)}.")
    return filtered_points



from shapely.geometry import Polygon, Point

def crop_points_within_xy_polygon(points, polygon_xy, visualizer=None, draw_box=False):
    """
    Filters out all 3D points whose (x, y) coordinates lie outside the given 2D polygon.

    Args:
        points (np.ndarray): Input N x 3 point cloud
        polygon_xy (list): List of 4 (x, y) tuples defining the polygon
        visualizer (Visualizer, optional): Open3D visualizer to draw crop box
        draw_box (bool): Whether to visualize the crop polygon in Open3D

    Returns:
        np.ndarray: Cropped point cloud (still N x 3)
    """

        
    if len(polygon_xy) < 3:
        raise ValueError("At least 3 XY coordinates required to define a polygon")

    poly = Polygon(polygon_xy)
    mask = []

    # Test each point's (x, y) location against polygon
    for pt in points:
        x, y = pt[0], pt[1]
        mask.append(poly.contains(Point(x, y)))

    filtered = points[np.array(mask)]

    log_debug(f"CROP: Kept {filtered.shape[0]} of {points.shape[0]} points within polygon.")

    # Optional: draw the polygon as a flat Open3D line loop
    if draw_box and visualizer is not None:
        import open3d as o3d
        # Convert 2D polygon to 3D line set
        poly_3d = [(x, y, 0.0) for x, y in polygon_xy]
        poly_3d.append(poly_3d[0])  # close loop

        lines = [[i, i + 1] for i in range(len(poly_3d) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(poly_3d)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.2, 0.8, 0.2])  # green crop region

        visualizer.add_geometry(line_set, reset_bounding_box=False)

    return filtered
