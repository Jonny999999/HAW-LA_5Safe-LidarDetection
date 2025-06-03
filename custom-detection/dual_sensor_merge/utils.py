from colorama import Fore, Style

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
