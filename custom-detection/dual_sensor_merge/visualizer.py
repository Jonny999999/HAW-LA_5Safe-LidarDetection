import open3d as o3d
from utils import log_info
import numpy as np

def initialize_visualizer():
    log_info("Initializing Open3D visualizer window...")
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name="LiDAR Streaming", width=1280, height=720)
    return visualizer



# ===============================
# === VISUALIZATION
# ===============================
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


    # if both pointclouds are provided, remove equal points from raw_points cloud so filtered_points are always visible (draw order seems random, sometimes red points not visible at all)
    if raw_points.size > 0 and filtered_points.size > 0:
        # Remove raw points that are exactly in filtered_points
        raw_set = set(map(tuple, raw_points))
        filt_set = set(map(tuple, filtered_points))
        visible_raw_points = np.array(list(raw_set - filt_set))
        raw_points = visible_raw_points

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
