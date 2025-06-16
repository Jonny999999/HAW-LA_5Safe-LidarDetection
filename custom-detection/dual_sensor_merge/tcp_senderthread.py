
import socket
import json
import numpy as np
import threading
import time
import gzip

HOST = '0.0.0.0'
PORT = 65432


SEND_DATA_DELAY_MS = 500

def handle_client(conn, addr, status_cache):
    global DATA_SEND_INTERVAL_MS
    print(f"Empfänger verbunden: {addr}")
    try:
        while True:
            # TODO: USE compression for sending
            # TODO: instead of json use pickle to send the dict directly from python to python
            # conn.sendall(pickle.dumps(your_dict))
            print("[TCP], preparing message for sending...")
            msg = status_cache.get_dashboard_and_status_data_as_json()
            msg = f"{msg}\n"
            #print(f"Data received from status_cache: {msg}")
            print("[TCP], sending...")
            conn.sendall(msg.encode())
            #conn.sendall(gzip.compress(msg.encode()))
            print("[TCP], done sending")
            time.sleep(SEND_DATA_DELAY_MS/1000)
    except Exception as e:
        print(f"Verbindung zu {addr} verloren: {e}")
    finally:
        conn.close()




def dashboard_tcp_server(status_cache):
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

