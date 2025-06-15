
import socket
import json
import numpy as np
import threading
import time

from status_file import get_full_status_as_json

HOST = '0.0.0.0'
PORT = 65432



def handle_client(conn, addr):
    print(f"Empfänger verbunden: {addr}")
    try:
        while True:
            msg = get_full_status_as_json()
            msg = f"{msg}\n"
            # print(f"Data received {msg}")
            conn.sendall(msg.encode())
            time.sleep(1)
    except Exception as e:
        print(f"Verbindung zu {addr} verloren: {e}")
    finally:
        conn.close()

def start_server():
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((HOST, PORT))
                s.listen(1)
                print(f"[Sender] Lausche auf {HOST}:{PORT} ...")
                conn, addr = s.accept()
                handle_client(conn, addr)
        except Exception as e:
            print(f"[Sender] Fehler im Server: {e}")
            print("[Sender] Neustart in 3 Sekunden...")
            time.sleep(3)

# Starte den Sender als separaten Thread
def run_sender_thread():
    sender_thread = threading.Thread(target=start_server, daemon=True)
    sender_thread.start()