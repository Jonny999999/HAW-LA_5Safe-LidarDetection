import socket
import pickle
import struct
import time
import pprint
import numpy as np

# Configuration
HOST = 'localhost'
PORT = 65432

def clip_large_content(obj, max_items=10):
    return obj
    if isinstance(obj, np.ndarray):
        return f"<np.ndarray shape={obj.shape} dtype={obj.dtype}>"
    elif isinstance(obj, list):
        if len(obj) > max_items:
            return obj[:max_items] + ["..."]
        else:
            return [clip_large_content(i, max_items) for i in obj]
    elif isinstance(obj, dict):
        return {k: clip_large_content(v, max_items) for k, v in obj.items()}
    else:
        return obj

def recv_full_message(sock, buffer):
    try:
        # Wait for 4-byte header (message length)
        while len(buffer) < 4:
            buffer += sock.recv(4096)
        msg_len = struct.unpack('!I', buffer[:4])[0]
        buffer = buffer[4:]

        # Wait for full payload
        while len(buffer) < msg_len:
            buffer += sock.recv(4096)
        msg_data = buffer[:msg_len]
        buffer = buffer[msg_len:]

        # Try to deserialize
        obj = pickle.loads(msg_data)
        return obj, buffer

    except (pickle.UnpicklingError, struct.error, ValueError, EOFError) as e:
        print(f"[Receiver] Unpickle failed or corrupt data: {e}")
        # Drop entire buffer and continue listening
        return None, b""

# Establish TCP connection with retry
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
connected = False
while not connected:
    try:
        print(f"[Receiver] Connecting to {HOST}:{PORT} ...")
        sock.connect((HOST, PORT))
        connected = True
        print("[Receiver] Connected successfully.")
    except ConnectionRefusedError:
        print("[Receiver] Connection refused. Retrying in 1 second...")
        time.sleep(1)

sock.setblocking(False)

# Main loop
data_buffer = b""
while True:
    try:
        obj, data_buffer = recv_full_message(sock, data_buffer)
        if obj is not None:
            print("\n\n========= Received Pickled Object =========")
            pprint.pprint(clip_large_content(obj), compact=True, width=120, depth=5)

    except BlockingIOError:
        pass  # No data yet
    except Exception as e:
        print(f"[Receiver] Unexpected error: {e}")
        break
