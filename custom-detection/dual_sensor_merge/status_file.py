import os
import json
import time
from datetime import datetime
from multiprocessing import Manager, Lock

from config import STATUS_FILE_ENABLED

# === Config ===
STATUS_FILE_PATH = "output/status.json"
MAX_LOG_ENTRIES = 5

# === Shared memory dictionary and locking ===
_manager = Manager()
_status_cache = _manager.dict()
_status_lock = Lock()
_first_write_done = False


# === Public API ===

def update_status_single_key(key, value, trigger_file_update=True):
    """
    Update or insert a single key into the shared status cache.
    If trigger_file_update=True, write the entire status to disk immediately.
    """
    if not STATUS_FILE_ENABLED:
        return
    with _status_lock:
        _ensure_output_file()
        _status_cache[key] = value
        if trigger_file_update:
            _write_status()


def update_status_bulk(new_data: dict, trigger_file_update=True):
    """
    Update multiple keys at once.
    If trigger_file_update=True, write the entire status to disk immediately.
    """
    if not STATUS_FILE_ENABLED:
        return
    with _status_lock:
        _ensure_output_file()
        _status_cache.update(new_data)
        if trigger_file_update:
            _write_status()


def get_status(key, default=None):
    """
    Read a single key from the status cache.
    """
    if not STATUS_FILE_ENABLED:
        print("[ERR] [status_file.json] Can't `get_status` because status file is disabled")
        return default
    with _status_lock:
        return _status_cache.get(key, default)


def add_log_entry_to_status_file(key, message, trigger_file_update=True):
    """
    Append a timestamped message to a list in the status file.
    Automatically keeps only the latest MAX_LOG_ENTRIES.
    """
    if not STATUS_FILE_ENABLED:
        return
    now_str = datetime.now().strftime("%H:%M:%S")
    entry = f"{now_str} - {message}"

    with _status_lock:
        _ensure_output_file()
        log_list = list(_status_cache.get(key, []))  # ensure it's a real list
        log_list.append(entry)
        _status_cache[key] = log_list[-MAX_LOG_ENTRIES:]
        if trigger_file_update:
            _write_status()


def flush_status_to_file():
    """
    Manually flush the in-memory status cache to disk.
    Can be used periodically in a background thread or process.
    """
    if not STATUS_FILE_ENABLED:
        return
    with _status_lock:
        _write_status()


# === Internal helpers ===

def _ensure_output_file():
    """
    Creates the output directory and clears the file on first write.
    """
    global _first_write_done
    os.makedirs(os.path.dirname(STATUS_FILE_PATH), exist_ok=True)
    if not _first_write_done:
        _status_cache.clear()
        _write_status()
        _first_write_done = True


def _write_status():
    try:
        def sort_key(k):
            if k.startswith("DETECTION_"):
                return (0, k)  # Highest priority
            elif k.startswith("LOG_") or k.startswith("LAST_WARNINGS"):
                return (2, k)  # Lowest priority
            else:
                return (1, k)  # Middle

        sorted_status = {k: _status_cache[k] for k in sorted(_status_cache.keys(), key=sort_key)}

        with open(STATUS_FILE_PATH, "w") as f:
            json.dump(sorted_status, f, indent=2)
    except Exception as e:
        print(f"[status_io] Failed to write {STATUS_FILE_PATH}: {e}")


def _load_status():
    """
    Optional: Load existing status.json into memory at startup.
    Can be called once from main before updates start.
    """
    global _status_cache
    if os.path.exists(STATUS_FILE_PATH):
        try:
            with open(STATUS_FILE_PATH, "r") as f:
                loaded = json.load(f)
                with _status_lock:
                    _status_cache.update(loaded)
        except Exception as e:
            print(f"[status_io] Failed to load {STATUS_FILE_PATH}: {e}")
