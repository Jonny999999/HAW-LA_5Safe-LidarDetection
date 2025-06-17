import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import time


class ViewerWithText:
    def __init__(self):
        self.geometry_name = "pcd"
        self.update_interval = 1.0
        self.counter = 0

        self.app = gui.Application.instance
        self.app.initialize()

        # Window and scene
        self.window = self.app.create_window("Live Viewer with Text", 1000, 600)
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background([0, 0, 0, 1])
        self.scene_widget.scene.show_axes(True)

        # === GUI: sidebar layout ===
        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(0.5 * em, gui.Margins(margin))

        self.label_status = gui.Label("Viewer running...")
        self.label_counter = gui.Label("Updates: 0")
        for lbl in [self.label_status, self.label_counter]:
            lbl.text_color = gui.Color(1, 1, 1)

        self.panel.add_child(self.label_status)
        self.panel.add_child(self.label_counter)

        self.window.add_child(self.panel)
        self.window.add_child(self.scene_widget)
        self.window.set_on_layout(self._on_layout)

        # Init point cloud and camera
        self.add_new_pointcloud()
        bbox = self.pcd.get_axis_aligned_bounding_box()
        self.scene_widget.setup_camera(60, bbox, bbox.get_center())

        self.last_update = time.time()

    def _on_layout(self, context):
        content_rect = self.window.content_rect
        panel_width = 200
        self.panel.frame = gui.Rect(content_rect.x, content_rect.y,
                                    panel_width, content_rect.height)
        self.scene_widget.frame = gui.Rect(content_rect.x + panel_width, content_rect.y,
                                           content_rect.width - panel_width, content_rect.height)

    def add_new_pointcloud(self):
        # Generate new cloud
        points = np.random.rand(1000, 3) * 2 - 1
        colors = np.random.rand(1000, 3)

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        self.scene_widget.scene.remove_geometry(self.geometry_name)
        self.scene_widget.scene.add_geometry(self.geometry_name, self.pcd, rendering.MaterialRecord())
        self.scene_widget.force_redraw()

        # Update labels
        self.counter += 1
        self.label_counter.text = f"Updates: {self.counter}"
        self.label_status.text = time.strftime("Last update: %H:%M:%S")

    def run(self):
        while self.app.run_one_tick():
            now = time.time()
            if now - self.last_update > self.update_interval:
                self.add_new_pointcloud()
                self.last_update = now
            self.window.post_redraw()


if __name__ == "__main__":
    try:
        viewer = ViewerWithText()
        viewer.run()
    except Exception as e:
        print("Exception:", e)
