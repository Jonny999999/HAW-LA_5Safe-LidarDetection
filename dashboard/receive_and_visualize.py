import open3d as o3d
import numpy as np
import socket
import json
import time
import pprint

# Konfiguration
HOST = 'localhost'   # IP-Adresse des Senders
PORT = 65432

# Open3D-Setup
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Empfänger: Punktwolke + Personenanzahl")
vis.add_geometry(pcd)
vis.get_render_option().point_size = 3.0



def deserialize_numpy_array(obj):
    return np.array(obj["data"], dtype=obj["dtype"]).reshape(obj["shape"])

# Example:
# raw = '{"pointcloud_merged_filtered": {"data": [...], "shape": [...], "dtype": "float32"}}'


def clip_large_lists(obj, max_items=20):
    if isinstance(obj, list):
        if len(obj) > max_items:
            return obj[:max_items] + ["..."]
        else:
            return [clip_large_lists(i, max_items) for i in obj]
    elif isinstance(obj, dict):
        return {k: clip_large_lists(v, max_items) for k, v in obj.items()}
    else:
        return obj


# === Verbindung aufbauen mit Retry ===
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
connected = False
while not connected:
    try:
        print(f"[Empfänger] Verbinde mit {HOST}:{PORT} ...")
        sock.connect((HOST, PORT))
        connected = True
        print("[Empfänger] Verbindung erfolgreich hergestellt.")
    except ConnectionRefusedError:
        print("[Empfänger] Verbindung fehlgeschlagen. Neuer Versuch in 1 Sekunde...")
        time.sleep(1)

sock.setblocking(False)  # Nicht blockierend für vis-Loop

# Zustand
buffer = ""
people_count = 0
points = np.zeros((1, 3))

# Hauptloop: Pollt Socket & updated Open3D
while True:
    # Versuch, Daten vom Socket zu lesen
    try:
        data = sock.recv(8192)
        if data:
            buffer += data.decode()

            # Verarbeitung von Zeilen (eine Zeile = eine JSON-Nachricht)
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                try:
                    obj = json.loads(line)
                    #moving_people = obj.get("DETECTION_MOVING_PEOPLE_INSIDE")
                    print("\n\n=======================")
                    print("==== Received dict: ====")
                    print("========================")
                    pprint.pprint(clip_large_lists(obj, max_items=20), depth=10, width=150, compact=True)
                    # print(moving_people)
                    # vis.update_geometry(pcd)

                    # de-serialize pointcloud from json to numpy array
                    pointcloud_data = obj["dashboard"]["pointcloud_merged_filtered_serializednumpyarray"]
                    pointcloud_merged_filtered_numpyarray = deserialize_numpy_array(pointcloud_data)

                except json.JSONDecodeError:
                    print("[Empfänger] Ungültige JSON-Zeile.")
    except BlockingIOError:
        pass  # Kein neues Datenpaket da – weitermachen
    except Exception as e:
        print(f"[Empfänger] Fehler: {e}")
        #break

    vis.poll_events()
    vis.update_renderer()