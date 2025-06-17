import os
import json
import time
from datetime import datetime
from multiprocessing import Manager, Lock


class GlobalStatusCache:
    """
    Thread- and process-safe global cache for storing runtime status and dashboard data.
    Allows concurrent updates from multiple processes using shared memory (Manager.dict()).
    """
    def __init__(self,
                 shared_status_dict=None,
                 shared_dashboard_dict=None,
                 lock=None,
                 status_file_path="output/status.json",
                 max_log_entries=5,
                 status_file_enabled=True,
                 status_file_creation_enabled=True):
        # === Validate external shared memory objects ===
        if shared_status_dict is None:
            raise ValueError("shared_status_dict (Manager().dict()) must be provided")
        if shared_dashboard_dict is None:
            raise ValueError("shared_dashboard_dict (Manager().dict()) must be provided")
        if lock is None:
            raise ValueError("lock (multiprocessing.Lock()) must be provided")

        # === Shared memory references ===
        self._status_cache = shared_status_dict
        self._dashboard_cache = shared_dashboard_dict
        self._lock = lock

        # === Configuration ===
        self.status_file_enabled = status_file_enabled
        self.status_file_path = status_file_path
        self.max_log_entries = max_log_entries
        self.status_file_creation_enabled = status_file_creation_enabled
        self._first_write_done = False

        # === Debug output for validation ===
        print(f"[GlobalStatusCache.__init__] Shared status dict id: {id(self._status_cache)}")
        print(f"[GlobalStatusCache.__init__] Shared dashboard dict id: {id(self._dashboard_cache)}")


    # === Public API ===

    def update_status_key(self, key, value, trigger_file_update=True):
        if not self.status_file_enabled:
            return
        with self._lock:
            self._ensure_output_file()
            self._status_cache[key] = value
            if trigger_file_update:
                self._write_status()

    def update_status_bulk(self, data: dict, trigger_file_update=True):
        if not self.status_file_enabled:
            return
        with self._lock:
            self._ensure_output_file()
            self._status_cache.update(data)
            if trigger_file_update:
                self._write_status()

    def get_status_key(self, key, default=None):
        if not self.status_file_enabled:
            return default
        with self._lock:
            return self._status_cache.get(key, default)

    def get_full_status_json(self):
        if not self.status_file_enabled:
            return "{}"
        with self._lock:
            return json.dumps(dict(self._status_cache))

    def add_log_entry(self, key, message, trigger_file_update=True):
        if not self.status_file_enabled:
            return
        with self._lock:
            self._ensure_output_file()
            now = datetime.now().strftime("%H:%M:%S")
            entry = f"{now} - {message}"
            lst = list(self._status_cache.get(key, []))
            lst.append(entry)
            self._status_cache[key] = lst[-self.max_log_entries:]
            if trigger_file_update:
                self._write_status()

    def update_dashboard_key(self, key, value):
        if not self.status_file_enabled:
            return
        with self._lock:
            self._dashboard_cache[key] = value

    def get_dashboard_and_status_data_as_json(self):
        if not self.status_file_enabled:
            return "{}"
        with self._lock:
            return json.dumps({
                "status": dict(self._status_cache),
                "dashboard": dict(self._dashboard_cache)
            })

    def get_dashboard_and_status_data(self):
        """
        Returns the raw status and dashboard dictionaries.
        Can be pickled directly for transmission.
        """
        if not self.status_file_enabled:
            return {"status": {}, "dashboard": {}}
        with self._lock:
            return {
                "status": dict(self._status_cache),
                "dashboard": dict(self._dashboard_cache)
            }


    def flush(self):
        if not self.status_file_enabled:
            return
        with self._lock:
            self._write_status()

    def load_from_file(self):
        if not os.path.exists(self.status_file_path):
            return
        try:
            with open(self.status_file_path, "r") as f:
                loaded = json.load(f)
                with self._lock:
                    self._status_cache.update(loaded)
        except Exception as e:
            print(f"[GlobalStatusCache] Failed to load from file: {e}")

    # === Internal Helpers ===

    def _ensure_output_file(self):
        if self._first_write_done:
            return
        os.makedirs(os.path.dirname(self.status_file_path), exist_ok=True)
        self._status_cache.clear()
        self._write_status()
        self._first_write_done = True

    def _write_status(self):
        if self.status_file_creation_enabled:
            try:
                def sort_key(k):
                    if k.startswith("DETECTION_"):
                        return (0, k)
                    elif k.startswith("LOG_") or k.startswith("LAST_WARNINGS"):
                        return (2, k)
                    else:
                        return (1, k)
                sorted_dict = {k: self._status_cache[k] for k in sorted(self._status_cache.keys(), key=sort_key)}
                with open(self.status_file_path, "w") as f:
                    json.dump(sorted_dict, f, indent=2)
            except Exception as e:
                print(f"[GlobalStatusCache] Failed to write status file: {e}")
