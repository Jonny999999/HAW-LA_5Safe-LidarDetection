import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from sklearn.cluster import DBSCAN
import open3d as o3d

import time
from queue import Empty
from status_file import GlobalStatusCache


# Thread for running AI detection on a pointcloud from a queue
# returns output via queue as well
def ai_detection_thread(merged_filtered_pointcoud_queue, ai_model_output_queue, status_cache_class_shared_params):

    # status cache setup
    status_cache = GlobalStatusCache(
        shared_status_dict=status_cache_class_shared_params.status_dict,
        shared_dashboard_dict=status_cache_class_shared_params.dashboard_dict,
        lock=status_cache_class_shared_params.lock,
        status_file_path=status_cache_class_shared_params.status_file_path,
        max_log_entries=status_cache_class_shared_params.max_log_entries,
        status_file_enabled=status_cache_class_shared_params.status_file_enabled,
        status_file_creation_enabled=status_cache_class_shared_params.status_file_creation_enabled,
    )

    people_detector = PointCloudPeopleDetector(model_path="model.pth")

    while True:
        #=== wait for new pointcloud input ===
        merged_pointcloud = merged_filtered_pointcoud_queue.get()

        # === Run PointNet AI Model and Clustering ===
        t0 = time.perf_counter()
        num_people, Ai_HumanPoints, ai_clusters, input_pointcloud = people_detector.detect(merged_pointcloud)

        # logging
        t1 = time.perf_counter()
        elapsed_ms = (t1 - t0) * 1000
        status_cache.update_status_key("DETECTION_AI_PEOPLE_COUNT", f"{num_people}")
        status_cache.update_status_key("TIMING_AI-MODEL_MS", f"{elapsed_ms:.1f} ms")

        # === Push result into output queue ===
        if ai_model_output_queue.full():
            try:
                ai_model_output_queue.get_nowait()  # drop oldest result
            except Empty:
                pass
        ai_model_output_queue.put_nowait((num_people, Ai_HumanPoints, ai_clusters, input_pointcloud))



# AI detection class
class PointCloudPeopleDetector:
    """
    Self-contained class for detecting the number of people in a point cloud.
    Includes model definition, clustering, and inference.
    """

    class SimplePointNet(nn.Module):
        def __init__(self, num_classes=2):
            super().__init__()
            self.fc1 = nn.Linear(3, 64)
            self.fc2 = nn.Linear(64, 128)
            self.fc3 = nn.Linear(128, 256)
            self.fc4 = nn.Linear(256, 128)
            self.fc5 = nn.Linear(128, num_classes)

        def forward(self, x):
            x = F.relu(self.fc1(x))
            x = F.relu(self.fc2(x))
            x = F.relu(self.fc3(x))
            x = F.relu(self.fc4(x))
            x = self.fc5(x)
            return x

    def __init__(self, model_path="model.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.SimplePointNet(num_classes=2).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

    def cluster_predictions(self, points, pred_labels, eps=0.5, min_samples=10):
        """
        Clusters points labeled as human using DBSCAN.

        Returns:
            human_points (np.ndarray): Points predicted as humans.
            cluster_labels (np.ndarray): Cluster labels from DBSCAN.
        """
        human_points = points[pred_labels == 1]
        if len(human_points) == 0:
            return np.array([]), np.array([])
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(human_points)
        labels = clustering.labels_

        # Convert clustered points to a list of Open3D PointClouds
        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue  # skip noise
            indices = np.where(labels == label)[0]
            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(human_points[indices])
            clusters.append(cluster_pcd)
        
        # Now return the raw points, labels, and the list of cluster point clouds
        return human_points, labels, clusters

    def predict_count(self, pointcloud: np.ndarray, eps=0.5, min_samples=17) -> int:
        """
        Runs inference on the point cloud and returns the number of detected people clusters.

        Args:
            pointcloud (np.ndarray): Nx3 array of point cloud points.
            eps (float): DBSCAN epsilon parameter.
            min_samples (int): DBSCAN min_samples parameter.

        Returns:
            int: Number of detected people clusters.
        """
        if pointcloud.ndim != 2 or pointcloud.shape[1] != 3:
            raise ValueError("Pointcloud must be Nx3 shaped (x, y, z)")

        points = pointcloud.astype(np.float32)
        points_tensor = torch.from_numpy(points).float().to(self.device)

        with torch.no_grad():
            pred_logits = self.model(points_tensor.unsqueeze(0))  # shape: (1, N, 2)
            pred_labels = pred_logits.argmax(dim=2).squeeze(0).cpu().numpy()

        human_points, cluster_ids, clusters = self.cluster_predictions(points, pred_labels, eps=eps, min_samples=min_samples)

        if len(human_points) == 0:
            return 0

        num_clusters = len(set(cluster_ids)) - (1 if -1 in cluster_ids else 0)
        return num_clusters

    def predict_labels(self, pointcloud: np.ndarray) -> np.ndarray:
        """
        Returns the per-point predicted class labels for the point cloud.

        Args:
            pointcloud (np.ndarray): Nx3 point cloud.

        Returns:
            np.ndarray: Predicted labels for each point.
        """
        if pointcloud.ndim != 2 or pointcloud.shape[1] != 3:
            raise ValueError("Pointcloud must be Nx3 shaped (x, y, z)")

        points = pointcloud.astype(np.float32)
        points_tensor = torch.from_numpy(points).float().to(self.device)

        with torch.no_grad():
            pred_logits = self.model(points_tensor.unsqueeze(0))
            pred_labels = pred_logits.argmax(dim=2).squeeze(0).cpu().numpy()

        return pred_labels
    def detect(self, pointcloud: np.ndarray, eps=0.3, min_samples=100) -> tuple[int, np.ndarray, list[np.ndarray]]:
        """
        Runs inference and clustering on the point cloud.

        Returns:
            num_clusters (int): Number of detected people clusters.
            human_points (np.ndarray): Mx3 array of points classified as human.
            cluster_arrays (List[np.ndarray]): List of M_i x 3 arrays for each detected cluster.
        """
        if pointcloud.ndim != 2 or pointcloud.shape[1] != 3:
            raise ValueError("Pointcloud must be Nx3 shaped (x, y, z)")

        points = pointcloud.astype(np.float32)
        points_tensor = torch.from_numpy(points).float().to(self.device)

        with torch.no_grad():
            pred_logits = self.model(points_tensor.unsqueeze(0))
            pred_labels = pred_logits.argmax(dim=2).squeeze(0).cpu().numpy()

        human_points, labels, _ = self.cluster_predictions(points, pred_labels, eps=eps, min_samples=min_samples)

        if len(human_points) == 0:
            return 0, np.empty((0, 3), dtype=np.float32), []

        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        cluster_arrays = []
        for label in np.unique(labels):
            if label == -1:
                continue
            indices = np.where(labels == label)[0]
            cluster_np = human_points[indices]
            cluster_arrays.append(cluster_np)

        return num_clusters, human_points, cluster_arrays, pointcloud # note: also returning input pointcloud