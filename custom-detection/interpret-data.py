import open3d as o3d
import velodyne_decoder as vd
import numpy as np

def visualize_pointcloud(points_xyz):
    """
    Visualize a point cloud using Open3D.

    Parameters:
        points_xyz (np.ndarray): Nx3 array of XYZ coordinates
    """
    if points_xyz.shape[1] != 3:
        raise ValueError(f"Expected Nx3 array for visualization, got shape {points_xyz.shape}")
    
    print("[INFO] Initializing Open3D point cloud visualization...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    print("[INFO] Rendering window...")
    o3d.visualization.draw_geometries([pcd])
    print("[INFO] Visualization complete.")

def load_pointcloud_from_pcap(path, max_frames=100):
    """
    Load and stack point clouds from a Velodyne PCAP file using velodyne_decoder.

    Parameters:
        path (str): Path to the .pcap file.
        max_frames (int): Maximum number of Lidar frames to process.

    Returns:
        np.ndarray: Stacked Nx3 point cloud array (X, Y, Z)
    """
    print(f"[INFO] Reading pointcloud from: {path}")
    points_list = []

    for i, (stamp, pointcloud) in enumerate(vd.read_pcap(path)):
        print(f"Frame {i:03d}: device time={stamp.device:.3f}s | host time={stamp.host:.3f}s | shape={pointcloud.shape}")

        # pointcloud is Nx8: [x, y, z, intensity, ring, time, azimuth, distance]
        if pointcloud.shape[1] < 3:
            print("[WARNING] Frame has fewer than 3 columns, skipping")
            continue

        xyz = pointcloud[:, :3]  # Only take X, Y, Z
        points_list.append(xyz)

        if i + 1 >= max_frames:
            print(f"[INFO] Reached max frame limit ({max_frames}). Stopping.")
            break

    if not points_list:
        print("[WARNING] No valid point clouds found in the file.")
        return np.empty((0, 3))

    all_points = np.vstack(points_list)
    print(f"[INFO] Total accumulated points: {all_points.shape}")
    return all_points




# === MAIN EXECUTION ===
if __name__ == "__main__":
    pcap_path = "../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"  # <-- Update path as needed

    print("[INFO] Starting PCAP point cloud extraction and visualization...")
    points_xyz = load_pointcloud_from_pcap(pcap_path, max_frames=100)

    print("[DEBUG] extracted XYZ Points:")
    print(points_xyz)

    print("[INFO] Visualizing extracted point cloud...")
    visualize_pointcloud(points_xyz)
