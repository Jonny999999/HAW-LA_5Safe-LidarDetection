import open3d as o3d
import velodyne_decoder as vd
import numpy as np
import time
from collections import deque
from scipy.spatial import cKDTree # to map closest points for highpass filter

# === Configuration ===
BUFFER_SIZE = 10           # Rolling buffer of last N frames
MAX_FRAMES = 500           # Total frames to simulate
FRAME_DELAY = 0.1          # Delay between simulated stream input (in seconds)

# what should be drawn as second RED pointcloud:
mode_second_data_set = "HIGHPASS+DENOISE"
#mode_second_data_set = "OLDEST"
#mode_second_data_set = "HIGHPASS"

COUNT_PEOPLE_ENABLED = True
COUNT_PEOPLE_DRAW_BOXES = True


# === Globals (for simulation) ===
frame_buffer = deque(maxlen=BUFFER_SIZE)  # Rolling buffer
visualizer = None                         # Open3D visualizer
pcd = None                                # Point cloud object
is_initialized = False                    # Initialization flag
geometryOptonAdded = False


##def initialize_visualizer():
##    global visualizer, pcd, is_initialized
##
##    print("[INFO] Initializing Open3D visualizer window...")
##    visualizer = o3d.visualization.Visualizer()
##    visualizer.create_window(window_name="LiDAR Streaming", width=1280, height=720)
##
##    pcd = o3d.geometry.PointCloud()
##    visualizer.add_geometry(pcd)
##    is_initialized = True


### === FRAME VISUALIZATION ONLY ===
##def visualize_frame(frame):
##    global visualizer, pcd, geometryOptonAdded
##
##    if frame.size == 0:
##        print("[WARN] Empty frame, skipping visualization.")
##        return
##
##    pcd.points = o3d.utility.Vector3dVector(frame.astype(np.float64))
##
##    if not geometryOptonAdded:
##        visualizer.add_geometry(pcd)
##        geometryOptonAdded = True
##    else:
##        visualizer.update_geometry(pcd)
##
##    visualizer.poll_events()
##    visualizer.update_renderer()



def initialize_visualizer():
    global visualizer, pcd_raw, pcd_filtered, is_initialized, geometryOptonAdded

    print("[INFO] Initializing Open3D visualizer window...")
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name="LiDAR Streaming", width=1280, height=720)

    # Raw points: white or light gray
    pcd_raw = o3d.geometry.PointCloud()
    #pcd_raw.paint_uniform_color([0.8, 0.8, 0.8])

    # Filtered points: red
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.paint_uniform_color([1.0, 0.0, 0.0])

    visualizer.add_geometry(pcd_raw)
    visualizer.add_geometry(pcd_filtered)

    is_initialized = True



# update window with two pointclouds in default and red color
def visualize_dual_frame(raw_points, filtered_points):
    global visualizer, pcd_raw, pcd_filtered, geometryOptonAdded


    if raw_points.size == 0 and filtered_points.size == 0:
        print("[WARN] Both frames empty, skipping visualization.")
        return

    if raw_points.size > 0:
        pcd_raw.points = o3d.utility.Vector3dVector(raw_points.astype(np.float64))
        # Optional: reset color if you want raw to be a default gray
        # pcd_raw.paint_uniform_color([0.8, 0.8, 0.8])   # gray color
        visualizer.update_geometry(pcd_raw)

    if filtered_points.size > 0:
        pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points.astype(np.float64))
        pcd_filtered.paint_uniform_color([1.0, 0.0, 0.0])  # solid red after updating points!
        visualizer.update_geometry(pcd_filtered)

    if not geometryOptonAdded:
        visualizer.add_geometry(pcd_raw)
        visualizer.add_geometry(pcd_filtered)
        geometryOptonAdded = True

    visualizer.poll_events()
    visualizer.update_renderer()






# serialize pcap logfile to simulate constant UDP stream
def simulate_stream_input(pcap_path, max_frames=MAX_FRAMES):
    """
    Simulates streaming by reading PCAP frame-by-frame.
    Appends each frame to the global rolling buffer.
    """
    print(f"[INFO] Starting simulated stream from: {pcap_path}")
    frame_count = 0

    for stamp, pointcloud in vd.read_pcap(pcap_path):
        if frame_count >= max_frames:
            print("[INFO] Reached max frame limit.")
            break

        if pointcloud.shape[1] < 3:
            print(f"[WARNING] Skipping frame {frame_count}, insufficient dimensions.")
            continue

        xyz = pointcloud[:, :3]  # Keep only XYZ
        frame_buffer.append(xyz)  # Add to rolling buffer
        print(f"[DEBUG] Frame {frame_count:03d} appended to buffer ({xyz.shape[0]} points).")
        

        # Maintain rolling size
        if len(frame_buffer) > BUFFER_SIZE:
            frame_buffer.pop(0)

        # Only start visualizing after buffer is full
        if len(frame_buffer) < BUFFER_SIZE:
            print(f"[INFO] Buffer not full yet ({len(frame_buffer)}/{BUFFER_SIZE}). Waiting...")
            continue

        # now its safe to visualize (buffer full)
        process_and_visualize_latest_frame()

        frame_count += 1
        time.sleep(FRAME_DELAY)  # Simulate streaming rate





# === HIGHPASS FILTER ===
def apply_highpass_filter(buffer, minMovedMetersThreshold=0.2):
    """
    High-pass filter: retains points from latest frame that are significantly
    displaced compared to nearest point in the previous frame.
    
    Uses KDTree to handle differing point counts.
    """
    if len(buffer) < 2:
        print("[WARN] Not enough frames in buffer for high-pass filtering.")
        return buffer[-1]

    # TODO: Adjust selected frames for comparison
    latest = buffer[-1] # -1 = last (last added)
    previous = buffer[0] # 0 = oldest, 

    # Build KDTree from previous frame
    tree = cKDTree(previous)

    # Query nearest neighbor distance for each point in latest frame
    distances, _ = tree.query(latest, k=1)

    mask = distances > minMovedMetersThreshold
    filtered = latest[mask]

    print(f"[DEBUG] HIGH-PASS: kept {filtered.shape[0]} of {latest.shape[0]} points.")
    return filtered




#=== DENOISE FILTER ===
def remove_isolated_points(points, nb_points=3, radius=0.2):
    """
    Removes points with fewer than `nb_points` neighbors within `radius`.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    cl, ind = pcd.remove_radius_outlier(nb_points, radius)
    filtered_points = np.asarray(cl.points)  # Use filtered cloud's points, not original indices

    print(f"[DEBUG] DENOISE: Removed isolated points. Kept {len(filtered_points)} of {len(points)} points.")
    return filtered_points






# === PROCESS FRAME FROM BUFFER ===
frame_counter = 0
def process_and_visualize_latest_frame():
    """
    Selects a frame from the buffer based on selected mode and visualizes it.
    """
    global mode_second_data_set

    if not frame_buffer:
        print("[WARN] Frame buffer is empty.")
        return

    latest_frame = frame_buffer[-1]

    if mode_second_data_set == "OLDEST":
        filtered_frame = frame_buffer[0] # 0 = oldest
        print("[INFO] Visualizing oldest frame...")

    elif mode_second_data_set == "HIGHPASS":
        #highpass: only keep points that moved certain meters since oldest buffer value
        filtered_frame = apply_highpass_filter(frame_buffer, minMovedMetersThreshold=0.2)
        print("[INFO] Visualizing high-pass filtered frame...")

    elif mode_second_data_set == "HIGHPASS+DENOISE":
        #highpass: only keep points that moved certain meters since oldest buffer value
        highpass_points = apply_highpass_filter(frame_buffer, minMovedMetersThreshold=0.1)
        #denoise: Removes points with fewer than `nb_points` neighbors within `radius`.
        filtered_frame = remove_isolated_points(highpass_points, nb_points=3, radius=0.5)
    
    else:
        print("Invalid visualization mode mode_second_data_set", mode_second_data_set)


    # Estimate how many people are likely moving
    # global frame_counter
    # frame_counter += 1

    # if frame_counter % 10 == 0:  # Every 10 frames
    global COUNT_PEOPLE_ENABLED
    if COUNT_PEOPLE_ENABLED:
        num_people = estimate_moving_people(filtered_frame)

    #print("[DEBUG] Sample points:\n", filtered_frame[:5])
    #visualize_frame(selected_frame)
    # draw orignal and manipulated dataset (default + red color)
    visualize_dual_frame(latest_frame, filtered_frame)





drawn_bounding_boxes = []


def estimate_moving_people(pointcloud_np, eps=0.5, min_points=30):
    """
    Estimates number of moving people in a filtered point cloud using DBSCAN clustering.

    Args:
        pointcloud_np (np.ndarray): N x 3 array of points from filtered motion data.
        eps (float): Distance threshold for DBSCAN clustering (meters).
        min_points (int): Minimum number of points to consider a cluster valid.

    Returns:
        int: Estimated number of moving people (clusters above min_points).
    """
    global visualizer, drawn_bounding_boxes

    if pointcloud_np.shape[0] == 0:
        print("[INFO] No dynamic points to analyze.")
        return 0


    # Step 2: Cluster the filtered points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_np)
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if labels.size == 0 or np.max(labels) < 0:
        print("[INFO] No clusters found.")
        return 0

    unique_labels, counts = np.unique(labels, return_counts=True)

    valid_clusters = 0

    global COUNT_PEOPLE_DRAW_BOXES
    if COUNT_PEOPLE_DRAW_BOXES:
        # Step 1: Clear previous boxes
        for box in drawn_bounding_boxes:
            visualizer.remove_geometry(box, reset_bounding_box=False)
        drawn_bounding_boxes.clear()

        for cluster_id, count in zip(unique_labels, counts):
            if cluster_id == -1 or count < min_points:
                continue  # skip noise

            valid_clusters += 1
            indices = np.where(labels == cluster_id)[0]
            cluster_pcd = pcd.select_by_index(indices)

            # Draw bounding box
            bbox = cluster_pcd.get_axis_aligned_bounding_box()
            #bbox.color = (0, 1, 0)  # green
            bbox.color = (0.0, 0.0, 0.5)  # Dark blue
            #visualizer.add_geometry(bbox)
            visualizer.add_geometry(bbox, reset_bounding_box=False)
            drawn_bounding_boxes.append(bbox)

    print(f"[INFO] Estimated number of moving people: {valid_clusters}")
    return valid_clusters





if __name__ == "__main__":
    #pcap_path = "../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"  
    pcap_path = "../data/testdata/2025-05-06_4ppl-walking_sensor-tilted_VLP-32C.pcap"

    initialize_visualizer()
    simulate_stream_input(pcap_path)

    print("[INFO] Stream ended. Closing window.")
    visualizer.destroy_window()

