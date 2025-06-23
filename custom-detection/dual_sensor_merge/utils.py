import os
import sys
import time
import signal
import multiprocessing
import platform
from colorama import Fore, Style
import numpy as np

from config import LOG_INFO_ENABLED, LOG_DEBUG_ENABLED, LOG_WARN_ENABLED, LOG_ERROR_ENABLED


_status_cache_instance = None

def utils_init_global_status_cache(status_cache):
    """Call this once from main to enable logging/status globally."""
    global _status_cache_instance
    _status_cache_instance = status_cache

def log_debug(msg): 
    if LOG_DEBUG_ENABLED:
        print(f"{Fore.WHITE}[DEBUG] {msg} {Style.RESET_ALL}")

def log_info(msg): 
    if LOG_INFO_ENABLED:
        print(f"{Fore.GREEN}[INFO]{Style.RESET_ALL} {msg}")

def log_warn(msg):
    if LOG_WARN_ENABLED:
        print(f"{Fore.YELLOW}[WARN] {msg}{Style.RESET_ALL}")
        if _status_cache_instance:
            _status_cache_instance.add_log_entry(f"LOG_LAST_WARNINGS", f"'{msg}'")

def log_error(msg): 
    if LOG_ERROR_ENABLED:
        print(f"{Fore.RED}[ERROR] {msg}{Style.RESET_ALL}")
        if _status_cache_instance:
            _status_cache_instance.add_log_entry(f"LOG_LAST_ERRORS", f"'{msg}'")



# for encoding a numpy array in json, we need to serialize it
def serialize_numpy_array(arr):
    """
    Converts a numpy array (or anything convertible) to a JSON-serializable structure.
    Accepts both np.ndarray and regular Python lists.
    """
    if not isinstance(arr, np.ndarray):
        try:
            arr = np.array(arr)
        except Exception as e:
            raise TypeError(f"serialize_numpy_array: Could not convert input to ndarray. Got {type(arr)}. Error: {e}")

    return {
        "data": arr.flatten().tolist(),
        "shape": arr.shape,
        "dtype": str(arr.dtype)
    }

def deserialize_numpy_array(obj):
    return np.array(obj["data"], dtype=obj["dtype"]).reshape(obj["shape"])




def kill_all_processes_and_terminate_script():
    print("\n\n\n[EXIT] Killing processes and terminating python script...")
    system = platform.system()

    # Terminate active children first (on all platforms)
    print("[EXIT] Terminating active child processes...")
    for p in multiprocessing.active_children():
        print(f"[EXIT]  Killing process {p.pid}")
        try:
            p.terminate()
            p.join(timeout=0.2)
            if p.is_alive():
                print(f"[EXIT] Process {p.pid} did not terminate within timeout.")
        except Exception as e:
            print(f"[EXIT] Failed to terminate process {p.pid}: {e}")

    # on windows we cant use killpg to force kill all python processes, os._exit should be enough
    if system != "Windows":
        try:
            print("[EXIT] Killing all processes in this process group...")
            pgid = os.getpgid(os.getpid())
            print(f"[EXIT]  PGID is {pgid}, sending SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            print(f"[EXIT] Failed to kill process group: {e}")
    
    print("[EXIT] 3. force-killing script...")
    time.sleep(2)
    os._exit(0)