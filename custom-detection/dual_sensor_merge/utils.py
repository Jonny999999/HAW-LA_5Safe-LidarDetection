
from colorama import Fore, Style

def log_info(msg): print(f"{Fore.GREEN}[INFO]{Style.RESET_ALL} {msg}")
def log_warn(msg): print(f"{Fore.YELLOW}[WARN] {msg}{Style.RESET_ALL}")
def log_error(msg): print(f"{Fore.RED}[ERROR] {msg}{Style.RESET_ALL}")
def log_debug(msg): print(f"{Fore.WHITE}[DEBUG] {msg} {Style.RESET_ALL}")