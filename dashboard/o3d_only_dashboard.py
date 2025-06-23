import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

class PointCloudApp:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()

        self.window = self.app.create_window("PointCloud Viewer", 1024, 768)

        # Scene widget
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)

        # Side panel
        self.panel = gui.Vert(300, gui.Margins(10, 10, 10, 10))
        self.label = gui.Label("Initial Frame")
        self.panel.add_child(self.label)
        self.window.add_child(self.panel)

        # Point cloud setup
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.random.rand(1000, 3))
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.scene.scene.add_geometry("pcd", self.pcd, self.material)

        bounds = self.pcd.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60, bounds, bounds.get_center())

        self.frame = 0
        self.window.set_on_tick(self.update)  # ✅ Use this instead of add_timer

    def update(self):
        self.frame += 1
        # Update point cloud
        self.pcd.points = o3d.utility.Vector3dVector(np.random.rand(1000, 3))
        self.scene.scene.update_geometry("pcd", self.pcd)
        self.label.text = f"Frame: {self.frame}"
        return True  # returning True keeps the update loop going

    def run(self):
        self.app.run()

if __name__ == "__main__":
    PointCloudApp().run()