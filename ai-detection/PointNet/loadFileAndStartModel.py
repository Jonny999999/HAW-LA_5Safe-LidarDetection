import sys
import os
import torch
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN

# Modellarchitektur (wie vorher definiert)
import torch.nn as nn
import torch.nn.functional as F

class SimplePointNet(nn.Module):
    def __init__(self, num_classes=2):
        super(SimplePointNet, self).__init__()
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

def cluster_predictions(points, pred_labels, eps=0.5, min_samples=10):
    human_points = points[pred_labels == 1]
    if len(human_points) == 0:
        return np.array([]), np.array([])  # Keine Menschen
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(human_points)
    return human_points, clustering.labels_

def main():
    if len(sys.argv) < 2:
        print("Usage: python loadFileAndStartModel.py <pointcloud.pcd>")
        sys.exit(1)

    pcd_path = sys.argv[1]
    if not os.path.exists(pcd_path):
        print(f"Datei nicht gefunden: {pcd_path}")
        sys.exit(1)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Modell laden
    model = SimplePointNet(num_classes=2).to(device)
    model.load_state_dict(torch.load("model.pth", map_location=device))
    model.eval()

    # Punktwolke laden
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        print("Keine Punkte in der Punktwolke gefunden.")
        sys.exit(1)

    points_tensor = torch.from_numpy(points).float().to(device)  # (N, 3)

    # Vorhersage
    with torch.no_grad():
        pred_logits = model(points_tensor.unsqueeze(0))  # (1, N, 2)
        pred_labels = pred_logits.argmax(dim=2).squeeze(0).cpu().numpy()  # (N,)

    # Clustering der Mensch-Punkte
    human_points, cluster_ids = cluster_predictions(points, pred_labels, eps=0.5, min_samples=10)

    if len(human_points) == 0:
        print("Keine Menschen erkannt.")
    else:
        num_people = len(set(cluster_ids)) - (1 if -1 in cluster_ids else 0)
        print(f"Menschen erkannt (Cluster-Anzahl): {num_people}")

        # Optional: Details ausgeben
        for cid in set(cluster_ids):
            if cid == -1:
                continue
            count = np.sum(cluster_ids == cid)
            print(f"Cluster {cid}: {count} Punkte")

if __name__ == "__main__":
    main()