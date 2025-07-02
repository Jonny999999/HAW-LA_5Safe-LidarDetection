import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import threading
import pickle
import socket
import time
import struct


class ViewerWithText:
    def __init__(self):
        self.geometry_name = "pcd"
        self.counter = 0
        self.data_buffer = b""

        self.app = gui.Application.instance
        self.app.initialize()

        # Window and scene
        self.window = self.app.create_window("Live Viewer with Text", 1000, 600)
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background([0, 0, 0, 1])
        self.scene_widget.scene.show_axes(True)

        # Sidebar panel
        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(0.5 * em, gui.Margins(margin))

        self.label_status = gui.Label("Viewer running...")
        self.label_counter = gui.Label("Updates: 0")
        self.label_people_count = gui.Label("People count: 0")
        for lbl in [self.label_status, self.label_counter, self.label_people_count]:
            lbl.text_color = gui.Color(1, 1, 1)

        self.panel.add_child(self.label_status)
        self.panel.add_child(self.label_counter)
        self.panel.add_child(self.label_people_count)

        self.window.add_child(self.panel)
        self.window.add_child(self.scene_widget)
        self.window.set_on_layout(self._on_layout)

        # Initialize empty point cloud
        self.pcd = o3d.geometry.PointCloud()
        self.scene_widget.scene.add_geometry(self.geometry_name, self.pcd, rendering.MaterialRecord())

        bbox = o3d.geometry.AxisAlignedBoundingBox([-10, -10, -10], [10, 10, 10])
        self.scene_widget.setup_camera(60, bbox, bbox.get_center())

        # Start background thread to receive socket data
        self.socket_thread = threading.Thread(target=self.socket_receive_loop, daemon=True)
        self.socket_thread.start()

    def _on_layout(self, context):
        content_rect = self.window.content_rect
        panel_width = 200
        self.panel.frame = gui.Rect(content_rect.x, content_rect.y,
                                    panel_width, content_rect.height)
        self.scene_widget.frame = gui.Rect(content_rect.x + panel_width, content_rect.y,
                                           content_rect.width - panel_width, content_rect.height)

    def clip_large_content(self, obj, max_items=10):
        if isinstance(obj, np.ndarray):
            return f"<np.ndarray shape={obj.shape} dtype={obj.dtype}>"
        elif isinstance(obj, list):
            if len(obj) > max_items:
                return obj[:max_items] + ["..."]
            else:
                return [self.clip_large_content(i, max_items) for i in obj]
        elif isinstance(obj, dict):
            return {k: self.clip_large_content(v, max_items) for k, v in obj.items()}
        else:
            return obj

    def recv_full_message(self, sock):
        try:
            while len(self.data_buffer) < 4:
                self.data_buffer += sock.recv(4096)
            msg_len = struct.unpack('!I', self.data_buffer[:4])[0]
            self.data_buffer = self.data_buffer[4:]

            while len(self.data_buffer) < msg_len:
                self.data_buffer += sock.recv(4096)

            msg_data = self.data_buffer[:msg_len]
            self.data_buffer = self.data_buffer[msg_len:]

            return pickle.loads(msg_data)
        except (pickle.UnpicklingError, struct.error, ValueError, EOFError) as e:
            print(f"[Receiver] Unpickle failed or corrupt data: {e}")
            self.data_buffer = b""
            return None

    def socket_receive_loop(self):
        HOST = '10.215.111.111'
        PORT = 65432

        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connected = False
            try:
                while not connected:
                    try:
                        print(f"[Receiver] Connecting to {HOST}:{PORT} ...")
                        sock.connect((HOST, PORT))
                        connected = True
                        print("[Receiver] Connected.")
                    except ConnectionRefusedError:
                        print("[Receiver] Connection refused, retrying in 1s...")
                        time.sleep(1)

                sock.setblocking(False)

                while True:
                    try:
                        data = sock.recv(8192)
                        if data:
                            self.data_buffer += data
                            obj = self.recv_full_message(sock)
                            if obj is not None:
                                print("\n\n========= Received Pickled Object =========")
                                print(self.clip_large_content(obj))

                                pc_data = obj.get("dashboard", {}).get("pointcloud_merged_filtered")
                                people_count = obj.get("dashboard", {}).get("people_count", 0)

                                if isinstance(pc_data, np.ndarray):
                                    self.update_pointcloud_and_label(pc_data, people_count)
                        else:
                            print("[Receiver] Connection closed by peer.")
                            break
                    except BlockingIOError:
                        time.sleep(0.01)
                    except Exception as e:
                        print("[Receiver] Error:", e)
                        break
            except Exception as e:
                print("[Receiver] Fatal error:", e)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

            print("[Receiver] Connection lost, retrying in 2 seconds...")
            time.sleep(2)

    def update_pointcloud_and_label(self, points, people_count):
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.scene_widget.scene.remove_geometry(self.geometry_name)
        self.scene_widget.scene.add_geometry(self.geometry_name, self.pcd, rendering.MaterialRecord())

        self.counter += 1
        self.label_counter.text = f"Updates: {self.counter}"
        self.label_people_count.text = f"People count: {people_count}"
        self.label_status.text = time.strftime("Last update: %H:%M:%S")

        self.scene_widget.force_redraw()

    def run(self):
        self.app.run()


if __name__ == "__main__":
    viewer = ViewerWithText()
    viewer.run()