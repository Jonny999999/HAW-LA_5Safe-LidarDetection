import open3d as o3d
import numpy as np
import pygame
import time
import io
from PIL import Image

# === Dummy Punktwolke ===
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.random.rand(500, 3))

# === Open3D-Visualizer im Headless-Modus (Offscreen Rendering) ===
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False)  # Kein eigenes Fenster
vis.add_geometry(pcd)
vis.poll_events()
vis.update_renderer()

# === Pygame Setup ===
pygame.init()
WIDTH, HEIGHT = 1200, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Combined Dashboard")
font = pygame.font.SysFont("Arial", 24)
clock = pygame.time.Clock()

# === Dummy-Daten ===
people_inside = 2
last_event = "12:34:56 - Person ENTERED"
processing_time = "136 ms"

running = True
while running:
    screen.fill((20, 20, 20))

    # === Punktwolke Screenshot erzeugen (Open3D -> NumPy -> Pygame) ===
    vis.poll_events()
    vis.update_renderer()
    image_o3d = vis.capture_screen_float_buffer(do_render=True)
    image_np = (255 * np.asarray(image_o3d)).astype(np.uint8)
    image_pil = Image.fromarray(image_np)
    image_pil = image_pil.resize((800, 600))
    image_str = image_pil.tobytes()
    image_pg = pygame.image.fromstring(image_str, image_pil.size, "RGB")

    # === Punktwolke (rechts) anzeigen ===
    screen.blit(image_pg, (400, 0))  # Rechts platzieren

    # === Text (links) anzeigen ===
    info_lines = [
        "== STATUS ==",
        f"People inside: {people_inside}",
        f"Last event: {last_event}",
        f"Processing: {processing_time}",
    ]
    for i, line in enumerate(info_lines):
        txt = font.render(line, True, (200, 200, 200))
        screen.blit(txt, (20, 30 + i * 40))

    # === Events + Refresh ===
    pygame.display.flip()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(10)  # 10 FPS genügen

# === Aufräumen ===
vis.destroy_window()
pygame.quit()