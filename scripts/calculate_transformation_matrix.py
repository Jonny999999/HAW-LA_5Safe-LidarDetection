import open3d as o3d
import numpy as np


# === Configure reference points ===

# 2025.05.28:
# Determined Reference points during live setup, works for `data/testdata/2025-05-28_dual-sensor-test_sensor-lamp.pcap.gz`
points_pc1 = np.array([
    [-1.855261207, -7.051774979, 0.591801226],  # Greenscreen cabinets
    [2.209504128, -7.254121304, 0.439154744],   # Greenscreen center
    [2.961946249, -11.077330589, 0.465429336],  # Greenscreen door
])

points_pc2 = np.array([
    [-5.001546383, -6.413216591, 0.660472870],  # Greenscreen cabinets
    [-2.166234493, -3.555836439, -0.047979854], # Greenscreen center
    [1.149047494, -5.592765808, 0.164931178],   # Greenscreen door
])


# 2025.05.26:
# Extrected reference point from files `data/testdata/2025-05-20_dual-sensor-test_sensor-xxx.pcap.gz`
# points_pc1 = np.array([
#     [0.47, -5.8, 0.71],
#     [-0.61, -4, -0.24],
#     [-5, -2.3, -0.58],
# ])
# 
# points_pc2 = np.array([
#     [2.3, -10, 0.87],
#     [3, -8.7, 0.27],
#     [1.7, -4.3, 0.02],
# ])




# === Calculate Transformation matrix ===

# Punktwolken erzeugen
pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(points_pc1)
pcd2.points = o3d.utility.Vector3dVector(points_pc2)

# Korrespondenzen (Index-Paare: point i in pc2 ↔ point i in pc1)
corres = np.array([[i, i] for i in range(len(points_pc1))])
corres_vector = o3d.utility.Vector2iVector(corres)

# Transformation berechnen
estimator = o3d.pipelines.registration.TransformationEstimationPointToPoint()
transformation = estimator.compute_transformation(pcd2, pcd1, corres_vector)

print("Transformation matrix:")
print(transformation)