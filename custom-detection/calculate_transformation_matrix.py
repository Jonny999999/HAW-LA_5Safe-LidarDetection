import open3d as o3d
import numpy as np

# Korrespondierende Punkte
points_pc1 = np.array([
    [0.47, -5.8, 0.71],
    [-0.61, -4, -0.24],
    [-5, -2.3, -0.58],
])

points_pc2 = np.array([
    [2.3, -10, 0.87],
    [3, -8.7, 0.27],
    [1.7, -4.3, 0.02],
])

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