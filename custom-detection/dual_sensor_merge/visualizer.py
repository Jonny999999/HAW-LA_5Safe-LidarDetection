import numpy as np
import open3d as o3d
from utils import log_info, log_warn, log_debug
import numpy as np
from scipy.spatial import cKDTree



def initialize_visualizer(title="LiDAR Viewer", width=1280, height=720):
    log_info(f"Initializing visualizer: {title}")
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=title, width=width, height=height)
    return visualizer






# keep track of which PointClouds have been added
_added_pcds = set()

def visualize_single_frame(points, visualizer, pcd_ref, color):
    points = np.asarray(points)

    # Automatically slice down to 3D if needed
    if points.ndim == 2 and points.shape[1] > 3:
        #log_debug(f"[visualizer] Trimming point data from shape {points.shape} to (N, 3) for visualization.")
        points = points[:, :3]

    # Ensure it's (N, 3)
    if points.ndim != 2 or points.shape[1] != 3:
        log_warn(f"[visualizer] Invalid point array shape: {points.shape} — skipping frame.")
        return

    if points.size == 0:
        return

    pcd_ref.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd_ref.paint_uniform_color(color)
    visualizer.update_geometry(pcd_ref)

    # on first use only: add to scene
    pid = id(pcd_ref)
    if pid not in _added_pcds:
        visualizer.add_geometry(pcd_ref)
        _added_pcds.add(pid)

    visualizer.poll_events()
    visualizer.update_renderer()



# plain stupid but inefficient way to remove duplicates (102% cpu)
# TODO: unused function, drop this?
def remove_exact_duplicates(raw, filtered):
    """
    Removes rows from `raw` that exactly match any row in `filtered`.
    Returns the pointcloud the duplicates were removed
    """
    if raw.size == 0 or filtered.size == 0:
        return raw

    # Ensure both are (N, 3)
    raw = np.asarray(raw)
    filtered = np.asarray(filtered)
    assert raw.shape[1] == 3 and filtered.shape[1] == 3

    # Round for floating-point tolerance
    decimals = 4
    raw_rounded = np.round(raw, decimals=decimals)
    filtered_rounded = np.round(filtered, decimals=decimals)

    # Convert to structured arrays
    raw_view = raw_rounded.view(dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]).reshape(-1)
    filt_view = filtered_rounded.view(dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]).reshape(-1)

    # Fast row-wise comparison
    mask = ~np.isin(raw_view, filt_view)

    return raw[mask]




# More efficient/optimized way of removing duplicate points (30% cpu)
def remove_near_duplicates(raw, filtered, threshold=0.001):
    """
    Removes points from `raw` that are less than threshold spaced from any point in `filtered`.
    Returns the pointcloud the duplicates were removed
    """
    if raw.size == 0 or filtered.size == 0:
        return raw

    tree = cKDTree(filtered)
    distances, _ = tree.query(raw, distance_upper_bound=threshold)

    mask = distances > threshold  # Keep only raw points not near any filtered point
    return raw[mask]



geometry_added = False
pcd_raw = o3d.geometry.PointCloud()
pcd_filtered = o3d.geometry.PointCloud()
# update window with two pointclouds in default and red color
def visualize_dual_frame(raw_points, filtered_points, visualizer):
    """
    Visualizes two point clouds in Open3D:
    - raw_points = gray
    - filtered_points = red
    Ensures overlapping points are drawn correctly.

    Args:
        raw_points (np.ndarray): Raw point cloud (Nx3)
        filtered_points (np.ndarray): Processed cloud (Nx3)
        visualizer (Visualizer): Open3D window
    """

    global pcd_filtered, pcd_raw

    if raw_points.size == 0 and filtered_points.size == 0:
        log_warn(" Both frames empty, skipping visualization.")
        return

    # Automatically slice down to 3D if needed
    if filtered_points.ndim == 2 and filtered_points.shape[1] > 3:
        #log_debug(f"[visualizer] Trimming point data from shape {points.shape} to (N, 3) for visualization.")
        filtered_points = filtered_points[:, :3]
    # Automatically slice down to 3D if needed
    if raw_points.ndim == 2 and raw_points.shape[1] > 3:
        #log_debug(f"[visualizer] Trimming point data from shape {points.shape} to (N, 3) for visualization.")
        raw_points = raw_points[:, :3]

    # if both pointclouds are provided, remove equal points from raw_points cloud so filtered_points are always visible (draw order seems random, sometimes red points not visible at all)
    if raw_points.size > 0 and filtered_points.size > 0:
        # Remove raw points that are exactly in filtered_points
        #raw_points = remove_exact_duplicates(raw_points, filtered_points) # this is very inefficient (cpu >100%)
        raw_points = remove_near_duplicates(raw_points, filtered_points)

    if raw_points.size > 0:
        pcd_raw.points = o3d.utility.Vector3dVector(raw_points.astype(np.float64))
        # Optional: reset color if you want raw to be a default gray
        pcd_raw.paint_uniform_color([0.8, 0.8, 0.8])   # gray color
        visualizer.update_geometry(pcd_raw)

    if filtered_points.size > 0:
        pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points.astype(np.float64))
        pcd_filtered.paint_uniform_color([1.0, 0.0, 0.0])  # solid red after updating points!
        visualizer.update_geometry(pcd_filtered)

    # add geometry initially
    global geometry_added
    if not geometry_added:
        visualizer.add_geometry(pcd_raw)
        visualizer.add_geometry(pcd_filtered)
        geometry_added = True

    visualizer.poll_events()
    visualizer.update_renderer()






# Function for creating a independent new window with pointcloud for picking a point 
# (blocks the script until window closed)
def pick_point_from_cloud(points, title="Pick Points"):
    """
    Opens a blocking Open3D editor window for selecting points.
    Returns list of 3D coordinates (user must press 'q' to close).
    """
    # convert pointcloud to open3d format
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])

    log_info(f"[Pick] SHIFT+Click to select points in '{title}', press Q to exit.")
    log_info(f"[Pick] ***close window*** to get selected points logged with ***FULL PRECISION***")

    # create new visualizer with editing enables (-> blocking)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    # draw pointcloud
    vis.add_geometry(pc)
    # start blocking, user selects points
    vis.run()  # user picks points
    vis.destroy_window()

    # extract indices of selected points
    picked_indices = vis.get_picked_points()
    print(f"finished picking points, logging full pcecision coordinates")

    # log all selected points with full precision
    if picked_indices:
        points_np = np.asarray(pc.points)
        coords = [points_np[i] for i in picked_indices]
        print(f"coords: {coords}")
        for i, c in zip(picked_indices, coords):
            print(f"[Pick] Full-precision Coordinates of point-Index {i}: ({c[0]:.9f}, {c[1]:.9f}, {c[2]:.9f})")
        return picked_indices, coords
    return picked_indices







# ========== VISUALIZATION ==========
drawn_bounding_boxes = []  # Persistent list of bounding box geometries
def draw_bounding_boxes(clusters, visualizer, color=(0.0, 0.0, 0.5)):
    """
    Draws bounding boxes around clusters.

    Args:
        clusters: list of (centroid, o3d.geometry.PointCloud)
        visualizer: Open3D visualizer
        color: RGB tuple
    """
    global drawn_bounding_boxes

    # Clear previous
    for box in drawn_bounding_boxes:
        visualizer.remove_geometry(box, reset_bounding_box=False)
    drawn_bounding_boxes.clear()

    # Draw new
    for _, cluster in clusters:
        bbox = cluster.get_axis_aligned_bounding_box()
        bbox.color = color
        visualizer.add_geometry(bbox, reset_bounding_box=False)
        drawn_bounding_boxes.append(bbox)





def draw_2d_polygon(polygon_xy, visualizer, color=(0.2, 0.8, 0.2)):
    """
    Draws a polygon as lines on the scene.
    """
    poly_3d = [(x, y, 0.0) for x, y in polygon_xy] + [(polygon_xy[0][0], polygon_xy[0][1], 0.0)]
    lines = [[i, i + 1] for i in range(len(poly_3d) - 1)]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(poly_3d)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color(color)
    visualizer.add_geometry(line_set, reset_bounding_box=False)