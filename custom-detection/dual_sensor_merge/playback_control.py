import time
import numpy as np
import threading

from utils import *


# Global shared state for all viewers
_playback_state = {
    "paused": False,
    "step": False,
    "picking": False, 
}



def start_playback_input_thread():
    """
    Starts the input listener thread.
    """
    thread = threading.Thread(target=input_thread, daemon=True)
    thread.start()




def input_thread():
    """
    Thread that listens for user input from the terminal.
    Controls pause/play and step-by-step navigation.
    """
    help="""\
  =============== [playback_control]  CLI-Controls ================
  === Usage: Type command in terminal and press enter to run    ===
  === Available Commands:                                       ===
  ===     - 'p' = pause/resume                                  ===
  ===     - 'n' = next-frame                                    ===
  ===     - 'pick1'|'pick2'|'1'|'2'                             ===
  ===       launches point picker window for that sensor        ===   
  ===     - 'q' = quit")                                        ===
  === Note: Some commands might be broken, since this is legacy ===
  ================================================================="""
    print(help)
    while True:
        try:
            cmd = input(">> ").strip().lower()
        except EOFError:
            print("[playback-control (CLI)] No stdin available (EOF). Launched from a script? Input thread exiting.")
            break

        if cmd == "p":
            _playback_state["paused"] = not _playback_state["paused"]
            print("[Playback] Paused" if _playback_state["paused"] else "[Playback] Playing")
        elif cmd == "n":
            _playback_state["step"] = True
        elif cmd == "resume":
            _playback_state["paused"] = False
            _playback_state["picking"] = False
            print("[Playback] Resuming playback...")
        elif cmd == "pick1" or cmd == "1" :
            log_warn("pick1 request via input")
            request_pick("sensor1")
        elif cmd == "pick2" or cmd == "2" :
            log_warn("pick2 request via input")
            request_pick("sensor2")
        elif cmd == "q":
            print("Exiting...")
            exit(0)
        else:
            log_error(f"[playback-control] unhandled command `{cmd}")




def should_advance_frame():
    """
    Determines whether the next frame should be processed.
    In pause mode, waits until step is triggered.
    """
    if _playback_state["paused"]:
        if _playback_state["step"]:
            _playback_state["step"] = False
            return True
        else:
            time.sleep(0.01)
            return False
    return True





# Functions for triggering point picking request

_pick_request = None  # or "sensor1", "sensor2"


def get_pick_request():
    return _pick_request


def clear_pick_request():
    global _pick_request
    _pick_request = None
    _playback_state["picking"] = False


def request_pick(sensor):
    global _pick_request
    log_warn(f"requesting pick for sensor {sensor}, and pausing playback")
    _pick_request = sensor
    _playback_state["paused"] = True
    _playback_state["picking"] = True






def should_advance_frame():
    """
    Determines whether the next frame should be processed.
    In pause mode, waits until step is triggered.
    """
    if _playback_state["picking"]:
        return False
    if _playback_state["paused"]:
        if _playback_state["step"]:
            _playback_state["step"] = False
            return True
        return False
    return True

