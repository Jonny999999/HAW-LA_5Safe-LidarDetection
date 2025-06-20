import numpy as np
import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import time
import json
import hashlib

from utils import log_info, log_warn, log_debug, log_error
import config


visualizer_instances = {}
visualizer_modes = {}

#########################
######## CONFIG #########
#########################
VISUALIZER_DEFAULT_WINDOW_WIDTH = 3840
VISUALIZER_DEFAULT_WINDOW_HEIGHT = 2160
CUSTOM_DEFAULT_CAMERA_POSITION_ENABLED = True
VISUALIZER_DEFAULT_CAMERA_POSITION = {
  "intrinsic": {
    "width": 1914,
    "height": 1137,
    "fx": 984.6708841029068,
    "fy": 984.6708841029068,
    "cx": 956.5,
    "cy": 568.0
  },
  "extrinsic": [
    [
      -0.6730622953428582,
      -0.7351171121348872,
      0.08117867967802239,
      -0.8071571641729453
    ],
    [
      -0.6322851327318026,
      0.5149955914727914,
      -0.5787875704348846,
      3.20865146732323
    ],
    [
      0.38366998516189654,
      -0.44088816292803046,
      -0.8114277357077564,
      4.39508800354559
    ],
    [
      0.0,
      0.0,
      0.0,
      1.0
    ]
  ]
} 


def initialize_visualizer(
    title="LiDAR Viewer", 
    width=VISUALIZER_DEFAULT_WINDOW_WIDTH, 
    height=VISUALIZER_DEFAULT_WINDOW_HEIGHT):
    log_info(f"Initializing visualizer: {title}")
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=title, width=width, height=height)

    # Draw polygon 
    # (temporary, we need any object present to be able to adjust the camera..)
    if config.PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON:
        draw_2d_polygon(config.PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON, visualizer, fit_camera_to_data=True)

    # Let Open3D render one frame (so internal bounding box is set)
    visualizer.poll_events()
    visualizer.update_renderer()

    # apply configured camera position
    if (CUSTOM_DEFAULT_CAMERA_POSITION_ENABLED):
        apply_camera_parameters(visualizer, VISUALIZER_DEFAULT_CAMERA_POSITION)

    # === uncomment this to create/get the camera position for VISUALIZER_DEFAULT_CAMERA_POSITION ===
    # pause and manually select the camera position to update the configuration
    #time.sleep(1)
    #adjust_and_dump_camera(visualizer)

    # Clear polygon again
    visualizer.clear_geometries()
    #time.sleep(2)

    return visualizer





def apply_camera_parameters(visualizer, cam_data):
    """
    Applies previously saved camera parameters to a visualizer using current window intrinsics
    and the provided extrinsic matrix.
    Note: the cam_data can be obtained with calline adjust_and_dump_camera  durin init (uncomment that line)

    Args:
        visualizer (o3d.visualization.Visualizer): The visualizer instance.
        cam_data (dict): Must contain the "extrinsic" key.
    """
    ctr = visualizer.get_view_control()

    # Start with current camera parameters to get valid intrinsic
    current_params = ctr.convert_to_pinhole_camera_parameters()
    current_params.extrinsic = np.array(cam_data["extrinsic"])

    # Apply updated camera parameters
    ctr.convert_from_pinhole_camera_parameters(current_params)
    visualizer.poll_events()
    visualizer.update_renderer()






def adjust_and_dump_camera(visualizer, duration_sec=10):
    """
    Allows manual camera adjustment for a few seconds and then logs the camera parameters.
    
    Args:
        visualizer (open3d.visualization.Visualizer): Existing visualizer instance.
        duration_sec (int): Duration in seconds to keep updating the window for camera adjustment.
    """
    print("==== CAMERA POSITION SELECTOR =====")
    print("You have 10 seconds time to adjust the camera, then the data will be dumped")
    start = time.time()
    while time.time() - start < duration_sec:
        visualizer.poll_events()
        visualizer.update_renderer()
        time.sleep(0.01)  # small sleep to avoid CPU spinning

    # Extract and dump camera parameters
    ctr = visualizer.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()
    cam_json = {
        "intrinsic": {
            "width": params.intrinsic.width,
            "height": params.intrinsic.height,
            "fx": params.intrinsic.get_focal_length()[0],
            "fy": params.intrinsic.get_focal_length()[1],
            "cx": params.intrinsic.get_principal_point()[0],
            "cy": params.intrinsic.get_principal_point()[1],
        },
        "extrinsic": params.extrinsic.tolist()
    }

    print("\n[Camera Dump] === Copy and paste into code ===")
    print(json.dumps(cam_json, indent=2))




def create_window_if_needed(mode, window_title_prefix):
    if mode == "none":
        return None
    title = f"{window_title_prefix} ({mode})"
    return initialize_visualizer(title)


def initialize_all_used_visualizer_windows():
    global visualizer_instances, visualizer_modes

    visualizer_modes = {
        "Window 1": config.VISUALIZER_WINDOW_1_MODE,
        "Window 2": config.VISUALIZER_WINDOW_2_MODE,
        "Window 3": config.VISUALIZER_WINDOW_3_MODE,
    }

    for win_name, mode in visualizer_modes.items():
        vis = create_window_if_needed(mode, win_name)
        visualizer_instances[win_name] = vis



def get_visualizer_by_mode(mode):
    # TODO: check initialize_all_windows was run?
    for name, configured_mode in visualizer_modes.items():
        if configured_mode == mode:
            return visualizer_instances.get(name)
    #log_error(f"[get_visualizer_by_mode] active visualizer with mode {mode} found, returning null")
    return None  # Fallback if not found


def handle_inputs_of_active_visualizers():
    """
    Updates all currently initialized Open3D visualizer windows.
    Should be called regularly to keep UI responsive (e.g. during pause).
    """
    for vis in visualizer_instances.values():  # <-- fix: use .values()
        if vis is not None:
            vis.poll_events()
            vis.update_renderer()



def update_visualizer_by_mode(mode, context):
    """
    Renders data into the visualizer assigned to the given mode.

    Args:
        mode (str): Visualization mode (e.g., 'sensor1', 'merged_dual', etc.)
        context (dict): Point cloud data arrays + Open3D PointClouds
    """
    vis = get_visualizer_by_mode(mode)
    if mode == "none" or vis is None:
        log_debug("[update_visuzlier_by_mode] no vis assigned to this mode")
        return

    if mode == "sensor1":
        visualize_single_frame(context["pointcloud_1_array"], vis, context["pointcloud_1_o3d"], color=[0.0, 0.5, 1.0])

    elif mode == "sensor2":
        visualize_single_frame(context["pointcloud_2_array"], vis, context["pointcloud_2_o3d"], color=[1.0, 0.5, 0.0])

    elif mode == "merged_dual":
        visualize_dual_frame(context["pointcloud_1_array"], context["pc2_transformed"], vis)

    elif mode == "merged_filtered":
        visualize_dual_frame(context["pc_merged"], context["pc_filtered"], vis)

    elif mode == "motion_detection":
        # Do not render here — handled externally
        pass

    else:
        log_warn(f"[visualizer] Unknown visualization mode: {mode}")



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
        visualizer.add_geometry(pcd_ref, reset_bounding_box=False)
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



# Map visualizer ID -> pointcloud objects and state (replace global variables for use with multiple visualizer windows)

_visualizer_objects = {}  # global cache: vis_id -> {pcd_raw, pcd_filtered, added}
def visualize_dual_frame(pointcloid1_gray, pointcloud2_dominant_red, visualizer):
    """
    Efficiently visualizes pointcloud1 (gray) and pointcloud2 (red) in Open3D.
    Avoids per-frame geometry recreation (performance gain).
    Supports multiple visualizers via internal tracking.

    Args:
        pointcloid1_gray (np.ndarray): Raw point cloud (Nx3)
        pointcloud2_dominant_red (np.ndarray): Processed point cloud (Nx3)
        visualizer (open3d.visualization.Visualizer): Visualizer instance
    """
    if visualizer is None or (pointcloid1_gray.size == 0 and pointcloud2_dominant_red.size == 0):
        log_warn("No visualizer or both frames empty, skipping visualization.")
        return

    # Slice to XYZ
    if pointcloid1_gray.ndim == 2 and pointcloid1_gray.shape[1] > 3:
        pointcloid1_gray = pointcloid1_gray[:, :3]
    if pointcloud2_dominant_red.ndim == 2 and pointcloud2_dominant_red.shape[1] > 3:
        pointcloud2_dominant_red = pointcloud2_dominant_red[:, :3]

    # Remove near duplicates (optional but keeps visibility)
    if pointcloid1_gray.size > 0 and pointcloud2_dominant_red.size > 0:
        pointcloid1_gray = remove_near_duplicates(pointcloid1_gray, pointcloud2_dominant_red)

    # determine which visualizer/window is used to use the correct cache
    vis_id = id(visualizer)

    # First time init per visualizer -> create cached structure
    if vis_id not in _visualizer_objects:
        log_info("visualize_dual_frame: Creating object for tracking visualizer/window id={vis_id}")
        _visualizer_objects[vis_id] = {
            "pcd_raw": o3d.geometry.PointCloud(),
            "pcd_filtered": o3d.geometry.PointCloud(),
            "added": False
        }

    state = _visualizer_objects[vis_id]

    # Update geometry with new points
    # note: when empty pointcloud was provided, the previous points are cleared
    state["pcd_raw"].points = o3d.utility.Vector3dVector(pointcloid1_gray.astype(np.float64))
    state["pcd_raw"].paint_uniform_color([0.8, 0.8, 0.8])

    state["pcd_filtered"].points = o3d.utility.Vector3dVector(pointcloud2_dominant_red.astype(np.float64))
    state["pcd_filtered"].paint_uniform_color([1.0, 0.0, 0.0])

    # Add geometry only once
    if not state["added"]:
        log_info("visualize_dual_frame: visualizer={vis_id} initially adding 2x pointcloud geometry")
        visualizer.add_geometry(state["pcd_raw"], reset_bounding_box=False)
        visualizer.add_geometry(state["pcd_filtered"], reset_bounding_box=False)
        state["added"] = True

    # Always update visuals
    if pointcloid1_gray.size > 0:
        visualizer.update_geometry(state["pcd_raw"])
    if pointcloud2_dominant_red.size > 0:
        visualizer.update_geometry(state["pcd_filtered"])

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
    vis.add_geometry(pc, reset_bounding_box=False)
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





def draw_cluster_boxes(clusters, visualizer):
    """
    Draws bounding boxes around clusters with color-coded status and optional thick line rendering.

    Args:
        clusters (List[Dict]): Output from detect_moving_clusters()
        visualizer (Visualizer): Open3D visualizer
    """
    if visualizer is None:
        return

    # Status styles → RGB + whether to draw thicker lines
    status_styles = {
        "confirmed":            {"color": (0.0, 1.0, 0.0), "thick": True},   # green
        "retained":             {"color": (0.0, 0.0, 1.0), "thick": True},   # blue
        "matched":              {"color": (1.0, 0.8, 0.0), "thick": True},   # orange-yellow (case not possible)
        "pending-confirmation": {"color": (0.0, 0.0, 0.0), "thick": True},  # black
        "detected-but-not-meeting-critera": {"color": (0.5, 0.5, 0.5), "thick": False},   # gray
        "unknown":              {"color": (0.5, 0.5, 0.5), "thick": False}   # gray
    }

    # Clear existing boxes (reset visualizer)
    draw_bounding_box(None, visualizer, clear_existing=True, render_now=False)

    # Draw one box per cluster
    for cluster in clusters:
        status = cluster.get("status", "unknown")
        style = status_styles.get(status, status_styles["unknown"])

        draw_bounding_box(
            cluster["pcd"],
            visualizer,
            color=style["color"],
            min_volume_m3=0.1,
            thick_lines_enabled=style["thick"],
            thickness_hack_layer_offset=0.01,
            thickness_hack_layer_count=3,
            clear_existing=False,
            render_now=False
        )

    # Final update once
    visualizer.poll_events()
    visualizer.update_renderer()





drawn_bounding_boxes = []  # Global cache of drawn box objects
def draw_bounding_box(
    pcd,
    visualizer,
    color=(0.0, 0.8, 0.8),
    min_volume_m3=0.0,
    clear_existing=False,
    render_now=True,
    thick_lines_enabled=False,
    thickness_hack_layer_offset=0.01,
    thickness_hack_layer_count=3
):
    """
    Draws a bounding box around a point cloud in Open3D and tracks drawn objects to prevent buildup.
    Optionally simulates thicker lines by drawing multiple offset boxes.

    Args:
        pcd (PointCloud or None): Cluster point cloud. If None and clear_existing is True, only clears.
        visualizer (Visualizer): The Open3D visualizer instance.
        color (tuple): RGB tuple for box color.
        min_volume_m3 (float): If box is smaller than this, enlarge it around the centroid.
        clear_existing (bool): Whether to clear all previously drawn boxes.
        render_now (bool): Whether to update the visualizer immediately (set False if drawing multiple boxes).
        thick_lines_enabled (bool): If True, draws multiple offset boxes to fake thicker lines.
        thickness_hack_layer_offset (float): Size of offset in meters for each fake thickness layer.
        thickness_hack_layer_count (int): Number of offset layers (3 = center + 2 more)
    """
    import open3d as o3d

    if visualizer is None:
        return

    # === Clear all previously drawn boxes ===
    if clear_existing:
        for box in drawn_bounding_boxes:
            try:
                visualizer.remove_geometry(box, reset_bounding_box=False)
            except Exception as e:
                log_warn(f"[draw_bounding_boxes] Failed to remove box: {e}")
        drawn_bounding_boxes.clear()

    # === Draw new box (if point cloud provided) ===
    if pcd is not None:
        bbox = pcd.get_axis_aligned_bounding_box()
        volume = bbox.volume()

        # Enlarge box to min volume
        if min_volume_m3 > 0 and volume < min_volume_m3:
            center = bbox.get_center()
            half_size = (min_volume_m3 ** (1/3)) / 2
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(center[0] - half_size, center[1] - half_size, center[2] - half_size),
                max_bound=(center[0] + half_size, center[1] + half_size, center[2] + half_size)
            )

        # Either draw once or multiple times with small offsets
        boxes_to_draw = []

        if thick_lines_enabled:
            center = bbox.get_center()
            extents = bbox.get_extent()

            # Create multiple slightly scaled versions around center
            for i in range(-thickness_hack_layer_count//2, thickness_hack_layer_count//2 + 1):
                scale = 1.0 + i * thickness_hack_layer_offset
                half_ext = 0.5 * extents * scale
                min_bound = center - half_ext
                max_bound = center + half_ext
                hack_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
                hack_box.color = color
                boxes_to_draw.append(hack_box)
        else:
            bbox.color = color
            boxes_to_draw.append(bbox)

        # Add to visualizer and cache
        for box in boxes_to_draw:
            visualizer.add_geometry(box, reset_bounding_box=False)
            drawn_bounding_boxes.append(box)

    # === Final visualizer update if requested ===
    if render_now:
        visualizer.poll_events()
        visualizer.update_renderer()





_drawn_polygons_cache = {}  # vis_id → set of polygon hashes

def draw_2d_polygon(polygon_xy, visualizer, color=(0.2, 0.8, 0.2), fit_camera_to_data=False):
    """
    Draws a polygon as lines on the scene, only once per visualizer.
    Prevents duplicate geometries which slow down Open3D rendering over time.
    """
    if visualizer is None or not polygon_xy:
        log_error("draw_2d_polygon: no visualizer or polygon provided")
        return

    vis_id = id(visualizer)
    # create cached object for this visualizer if not existing
    if vis_id not in _drawn_polygons_cache:
        _drawn_polygons_cache[vis_id] = set()

    # hash polygon details to compare later if already existing
    polygon_hash = hashlib.md5(
        np.round(np.array(polygon_xy, dtype=np.float32), 6).tobytes() + 
        bytes(np.round(np.array(color, dtype=np.float32), 4))
    ).hexdigest()
    if polygon_hash in _drawn_polygons_cache[vis_id]:
        return  # Already drawn


    # Convert to 3D and create LineSet
    poly_3d = [(x, y, 0.0) for x, y in polygon_xy] + [(polygon_xy[0][0], polygon_xy[0][1], 0.0)]
    lines = [[i, i + 1] for i in range(len(poly_3d) - 1)]

    log_info(f"draw_2d_polygon: Adding new polygon to visualizer (hash:{polygon_hash})")
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(poly_3d)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color(color)

    try:
        visualizer.add_geometry(line_set, reset_bounding_box=fit_camera_to_data)
        _drawn_polygons_cache[vis_id].add(polygon_hash)
        visualizer.poll_events()
        visualizer.update_renderer()
    except Exception as e:
        log_error(f"[draw_2d_polygon] Failed to add geometry: {e}")