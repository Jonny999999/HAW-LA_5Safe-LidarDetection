
import socket
import json
import numpy as np
import threading
import time

from status_file import get_full_status_as_json, get_dashboard_and_status_data_as_json

HOST = '0.0.0.0'
PORT = 65432



def handle_client(conn, addr):
    print(f"Empfänger verbunden: {addr}")
    try:
        while True:
            #msg = get_full_status_as_json()
            msg = get_dashboard_and_status_data_as_json()
            msg = f"{msg}\n"
            # print(f"Data received {msg}")
            conn.sendall(msg.encode())
            time.sleep(1)
    except Exception as e:
        print(f"Verbindung zu {addr} verloren: {e}")
    finally:
        conn.close()




def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"[Sender] Lausche auf {HOST}:{PORT} ...")

        while True:
            try:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
                client_thread.start()
            except Exception as e:
                print(f"[Sender] Fehler beim Akzeptieren: {e}")



# Starte den Sender als separaten Thread
def run_sender_thread():
    sender_thread = threading.Thread(target=start_server, daemon=True)
    sender_thread.start()