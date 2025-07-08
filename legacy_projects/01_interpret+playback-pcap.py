import open3d as o3d
import velodyne_decoder as vd
import numpy as np
import time

# === CONFIG ===
PCAP_PATH = "../../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"
MAX_FRAMES = 100
DELAY = 0.1  # seconds between frames (simulate real-time)
VISUALIZE = True  # Toggle visualization (helpful for headless testing)




# === LOAD & BUFFER ===
def load_pointcloud_frames(path, max_frames=100):
    """
    Load LiDAR frames from a PCAP file using velodyne_decoder.

    Each frame is one 360° sweep and returns XYZ-only data per frame.

    Returns:
        List[np.ndarray]: List of Nx3 arrays for each LiDAR scan.
    """
    print(f"[INFO] Opening PCAP file: {path}")
    frame_buffer = []

    for i, (stamp, pointcloud) in enumerate(vd.read_pcap(path)):
        print(f"[DEBUG] Frame {i:03d}: Device Time = {stamp.device:.3f}s | Host Time = {stamp.host:.3f}s | Points = {pointcloud.shape}")

        if pointcloud.shape[1] < 3:
            print(f"[WARNING] Frame {i:03d} has fewer than 3 columns, skipping.")
            continue

        xyz = pointcloud[:, :3]
        frame_buffer.append(xyz)

        if i + 1 >= max_frames:
            print(f"[INFO] Reached max_frames limit ({max_frames}).")
            break

    print(f"[INFO] Total frames loaded: {len(frame_buffer)}")
    return frame_buffer




# === VISUALIZATION ===
def play_frames_with_open3d(frame_buffer, delay=0.1):
    """
    Simulates real-time playback of LiDAR frames using Open3D.
    Frame-by-frame update in a persistent window.
    """
    if not frame_buffer:
        print("[ERROR] No frames to display.")
        return

    print("[INFO] Initializing Open3D visualizer...")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="LiDAR Playback", width=1280, height=720)

    pcd = o3d.geometry.PointCloud()
    added = False

    for idx, frame in enumerate(frame_buffer):
        print(f"[DEBUG] Displaying frame {idx+1}/{len(frame_buffer)} with {frame.shape[0]} points.")
        print(frame[:5])  # show a sample of the frame

        pcd.clear()  # clear existing geometry
        pcd.points = o3d.utility.Vector3dVector(frame.astype(np.float64))  # Open3D prefers float64

        if not added:
            vis.add_geometry(pcd)
            added = True
        else:
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()
        time.sleep(delay)

    print("[INFO] Finished playback. Closing window.")
    vis.destroy_window()





# === MAIN EXECUTION ===
if __name__ == "__main__":
    print("[INFO] Starting LiDAR frame-by-frame processing...")

    frame_buffer = load_pointcloud_frames(PCAP_PATH, max_frames=MAX_FRAMES)

    print("[INFO] Frame buffering complete. Now beginning visualization loop...")
    print("[DEBUG] First frame sample (first 5 points):")
    if frame_buffer:
        print(frame_buffer[0][:5])

    if VISUALIZE:
        play_frames_with_open3d(frame_buffer, delay=DELAY)

    print("[INFO] All done.")

