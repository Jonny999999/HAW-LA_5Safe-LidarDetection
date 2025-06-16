from colorama import Fore, Style
import numpy as np

from config import LOG_INFO_ENABLED, LOG_DEBUG_ENABLED, LOG_WARN_ENABLED, LOG_ERROR_ENABLED
from status_file import add_log_entry_to_status_file

def log_debug(msg): 
    if LOG_DEBUG_ENABLED:
        print(f"{Fore.WHITE}[DEBUG] {msg} {Style.RESET_ALL}")

def log_info(msg): 
    if LOG_INFO_ENABLED:
        print(f"{Fore.GREEN}[INFO]{Style.RESET_ALL} {msg}")

def log_warn(msg):
    if LOG_WARN_ENABLED:
        print(f"{Fore.YELLOW}[WARN] {msg}{Style.RESET_ALL}")
        add_log_entry_to_status_file(f"LOG_LAST_WARNINGS", f"'{msg}'")

def log_error(msg): 
    if LOG_ERROR_ENABLED:
        print(f"{Fore.RED}[ERROR] {msg}{Style.RESET_ALL}")
        add_log_entry_to_status_file(f"LOG_LAST_ERRORS", f"'{msg}'")



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