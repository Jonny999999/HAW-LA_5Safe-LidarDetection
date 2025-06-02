# thread safe functions for updating variables in one common json file

import os
import json
import threading
from config import STATUS_FILE_ENABLED

# config
STATUS_FILE_PATH = "output/status.json"

# variables
_lock = threading.Lock()
_status_cache = {}
_first_write_done = False  # Ensures folder is created and file is cleared only once



def update_status_single_key(key, value):
    """Update or add a single key-value pair in status.json."""
    if not STATUS_FILE_ENABLED: return
    with _lock:
        _ensure_output_file()
        _status_cache[key] = value
        _write_status()



def update_status_bulk(new_data: dict):
    """Update multiple keys at once.
    example:
    update_status_bulk({
        "ABS-MOVING-PEOPLE-INSIDE": 1,
        "INCREMENTED-LEFT-ENTERED-PEOPLE": people_inside_incremented 
    })
    """
    if not STATUS_FILE_ENABLED: return
    with _lock:
        _ensure_output_file()
        _status_cache.update(new_data)
        _write_status()



def get_status(key, default=None):
    """Read a single value from the cached status."""
    if not STATUS_FILE_ENABLED:
        print("[ERR] [status_file.json] cant `get_status` because status file is disabled")
    with _lock:
        return _status_cache.get(key, default)




#=== local functions ===
def _ensure_output_file():
    os.makedirs(os.path.dirname(STATUS_FILE_PATH), exist_ok=True)

    global _first_write_done
    if not _first_write_done:
        print("[status.json] first update → clearing or creating file")
        _status_cache = {}
        _write_status()
        #with open(STATUS_FILE_PATH, "w") as f:
        #    f.write("{}\n")
        _first_write_done = True



def _write_status():
    try:
        with open(STATUS_FILE_PATH, "w") as f:
            json.dump(_status_cache, f, indent=2)
    except Exception as e:
        print(f"[status_io] Failed to write {STATUS_FILE_PATH}: {e}")



def _load_status():
    """Optional: Load existing status from file at startup."""
    global _status_cache
    if os.path.exists(STATUS_FILE_PATH):
        try:
            with open(STATUS_FILE_PATH, "r") as f:
                _status_cache = json.load(f)
        except Exception as e:
            print(f"[status_io] Failed to load existing {STATUS_FILE_PATH}: {e}")


# Load at import (optional)
# _load_status()
