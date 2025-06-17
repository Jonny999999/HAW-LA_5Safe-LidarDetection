
import socket
import json
import numpy as np
import threading
import time
import gzip

from status_file import GlobalStatusCache

HOST = '0.0.0.0'
PORT = 65432


DATA_SEND_INTERVAL_MS = 500


import pickle
import struct

def handle_client(conn, addr, status_cache):
    global DATA_SEND_INTERVAL_MS
    print(f"Empfänger verbunden: {addr}")
    try:
        while True:
            # Get raw status and dashboard data as Python objects
            data = status_cache.get_dashboard_and_status_data()

            # Serialize the object with pickle
            msg = pickle.dumps(data)
            length = struct.pack('!I', len(msg))  # 4 bytes, network byte order

            # Optionally: add newline or framing logic if needed
            conn.sendall(length + msg)
            print("[TCP] Pickle data sent.")

            time.sleep(DATA_SEND_INTERVAL_MS / 1000)
    except Exception as e:
        print(f"Verbindung zu {addr} verloren: {e}")
    finally:
        conn.close()



def dashboard_tcp_server(status_cache_class_shared_params):

    status_cache = GlobalStatusCache(
        shared_status_dict=status_cache_class_shared_params.status_dict,
        shared_dashboard_dict=status_cache_class_shared_params.dashboard_dict,
        lock=status_cache_class_shared_params.lock,
        status_file_path=status_cache_class_shared_params.status_file_path,
        max_log_entries=status_cache_class_shared_params.max_log_entries,
        status_file_enabled=status_cache_class_shared_params.status_file_enabled,
        status_file_creation_enabled=status_cache_class_shared_params.status_file_creation_enabled,
    )
    status_cache.update_dashboard_key("example_key", "main process test value")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        s.settimeout(2)  # Avoids blocking forever waiting for connection
        print(f"[TCP-Dashboard Sender] Listening on {HOST}:{PORT} ...")

        while True:
            try:
                conn, addr = s.accept() # waits for connection with timeout
                client_thread = threading.Thread(target=handle_client, args=(conn, addr, status_cache), daemon=True)
                client_thread.start()
            except socket.timeout:
                continue  # no connection within timeout try again
            except Exception as e:
                print(f"[TCP-Dashboard Sender] Error accepting connection: {e}")

