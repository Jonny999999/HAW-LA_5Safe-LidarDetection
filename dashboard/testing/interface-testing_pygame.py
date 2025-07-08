import pygame
import open3d as o3d
import numpy as np
import json
from PIL import Image

# === Dummy JSON input ===
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

# === Parse JSON and extract data ===
data = json.loads(json_string)
pointcloud_np = deserialize_numpy_array(data["dashboard"]["pointcloud_merged_filtered_serializednumpyarray"])
last_event = data["status"].get("DETECTION_LAST_EVENTS", ["Keine Events"])[0]
processing_time = data["status"].get("TIMING_PROCESSING_DURATION_MS", "N/A")
framerate = data["status"].get("TIMING_FRAMERATE_DECODER_1", "N/A")

# === Open3D: create static point cloud image ===
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=800, height=600)
vis.add_geometry(pcd)
ctr = vis.get_view_control()
ctr.set_lookat(pcd.get_center())
ctr.set_up([0, 1, 0])
ctr.set_front([0, 0, -1])
ctr.set_zoom(0.5)

for _ in range(5):
    vis.poll_events()
    vis.update_renderer()

image = vis.capture_screen_float_buffer(do_render=True)
image_np = (255 * np.asarray(image)).astype(np.uint8)
image_pil = Image.fromarray(image_np).resize((800, 600))
image_pg = pygame.image.fromstring(image_pil.tobytes(), image_pil.size, "RGB")
vis.destroy_window()

# === Pygame setup ===
pygame.init()
screen = pygame.display.set_mode((1200, 700))
pygame.display.set_caption("Open3D + Dashboard Info")
font = pygame.font.SysFont("Arial", 22)
clock = pygame.time.Clock()

# === Initial value for counter ===
people_inside = 0

running = True
while running:
    screen.fill((25, 25, 25))

    # Update counter each frame
    people_inside += 1

    # Recreate text lines each frame (only one changes)
    text_lines = [
        f"People inside: {people_inside}",
        f"Last event: {last_event}",
        f"Processing time: {processing_time}",
        f"Framerate: {framerate}"
    ]

    # Show point cloud on the right
    screen.blit(image_pg, (380, 50))

    # Show info text on the left
    for i, line in enumerate(text_lines):
        text = font.render(line, True, (200, 200, 255))
        screen.blit(text, (20, 40 + i * 40))

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(1)  # 1 FPS to make count visible

pygame.quit()
