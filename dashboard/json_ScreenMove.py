import pygame
import numpy as np
import open3d as o3d
import json
from PIL import Image

# === JSON-Daten (Dummy, wie vom TCP-Server) ===
json_string = '''
{
  "dashboard": {
    "DETECTION_MOVING_PEOPLE_INSIDE": "1",
    "pointcloud_merged_filtered_serializednumpyarray": {
      "data": [0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0],
      "dtype": "float64",
      "shape": [3, 3]
    }
  },
  "status": {
    "DETECTION_LAST_EVENTS": ["14:35:22 - Person ENTERED"],
    "DETECTION_TRACKED_PEOPLE_INSIDE": "3",
    "LOG_LAST_ERRORS": [],
    "LOG_LAST_WARNINGS": [],
    "TIMING_FRAMERATE_DECODER_1": "5.0 fps",
    "TIMING_PROCESSING_DURATION_MS": "112 ms"
  }
}
'''

def deserialize_numpy_array(obj):
    return np.array(obj["data"], dtype=obj["dtype"]).reshape(obj["shape"])

# === Daten aus JSON extrahieren ===
data = json.loads(json_string)
pointcloud_np = deserialize_numpy_array(data["dashboard"]["pointcloud_merged_filtered_serializednumpyarray"])
people_inside = data["dashboard"].get("DETECTION_MOVING_PEOPLE_INSIDE", "0")
last_event = data["status"].get("DETECTION_LAST_EVENTS", ["Keine Events"])[0]
processing_time = data["status"].get("TIMING_PROCESSING_DURATION_MS", "N/A")
framerate = data["status"].get("TIMING_FRAMERATE_DECODER_1", "N/A")

# === Open3D Visualizer (offscreen) ===
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=800, height=600)
vis.add_geometry(pcd)
ctr = vis.get_view_control()

# === Kamera-Steuerung ===
angle_x, angle_y = 0.0, 0.0
last_mouse_pos = None
rotating = False

# === Pygame Setup ===
pygame.init()
WIDTH, HEIGHT = 1200, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("PointCloud + Dashboard")
font = pygame.font.SysFont("Arial", 22)
clock = pygame.time.Clock()

# === Main Loop ===
running = True
while running:
    screen.fill((25, 25, 25))

    # === Events ===
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            rotating = True
            last_mouse_pos = event.pos
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            rotating = False
            last_mouse_pos = None
        elif event.type == pygame.MOUSEMOTION and rotating:
            x, y = event.pos
            last_x, last_y = last_mouse_pos
            dx, dy = x - last_x, y - last_y
            last_mouse_pos = (x, y)
            angle_x += dy * 0.5
            angle_y += dx * 0.5

    # === Kamera setzen & drehen ===
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, 1, 0])
    ctr.set_front([0, 0, -1])
    ctr.set_zoom(0.5)
    ctr.rotate(angle_y, angle_x)

    # === Punktwolke rendern ===
    vis.poll_events()
    vis.update_renderer()
    image = vis.capture_screen_float_buffer(do_render=True)
    image_np = (255 * np.asarray(image)).astype(np.uint8)
    image_pil = Image.fromarray(image_np)
    image_pil = image_pil.resize((800, 600))
    image_pg = pygame.image.fromstring(image_pil.tobytes(), image_pil.size, "RGB")

    # === Punktwolke anzeigen ===
    screen.blit(image_pg, (380, 50))

    # === Text-Overlay ===
    info_lines = [
        f"People inside: {people_inside}",
        f"Last event: {last_event}",
        f"Processing time: {processing_time}",
        f"Framerate: {framerate}",
        "Linke Maustaste + Ziehen = Rotation"
    ]
    for i, line in enumerate(info_lines):
        text = font.render(line, True, (200, 200, 255))
        screen.blit(text, (20, 40 + i * 40))

    pygame.display.flip()
    clock.tick(20)  # 20 FPS

# === Cleanup ===
vis.destroy_window()
pygame.quit()