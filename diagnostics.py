# -*- coding: utf-8 -*-

import threading
import collections
import time
import serial.tools.list_ports
from config import DEBUG, READ_ONLY_MODE, ENABLE_COMMANDS

diag_lock = threading.Lock()
diag = {
    "comm_thread_alive": False,
    "serial_open": False,
    "serial_session_id": 0,
    "selected_port": "",
    "last_rx_time": 0.0,
    "last_tx_time": 0.0,
    "last_loop_time": 0.0,
    "rx_count": 0,
    "tx_count": 0,
    "read_errors": 0,
    "write_errors": 0,
    "build_errors": 0,
    "plot_errors": 0,
    "gui_errors": 0,
    "consecutive_read_errors": 0,
    "consecutive_write_errors": 0,
    "last_read_error": "",
    "last_write_error": "",
    "last_build_error": "",
    "last_plot_error": "",
    "last_gui_error": "",
    "last_comm_error": "",
    "last_response_summary": "",
    "last_sample_age_s": float("inf"),
    "read_only_mode": READ_ONLY_MODE,
    "commands_enabled": ENABLE_COMMANDS,
}

event_log = collections.deque(maxlen=500)


def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


def log_event(msg):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    dprint(line)
    with diag_lock:
        event_log.appendleft(line)


def set_diag(**kwargs):
    with diag_lock:
        diag.update(kwargs)


def inc_diag(key, amount=1):
    with diag_lock:
        diag[key] = diag.get(key, 0) + amount


def get_diag_snapshot():
    with diag_lock:
        d = dict(diag)
        d["event_log"] = list(event_log)
        return d


def debug_list_ports():
    if not DEBUG:
        return
    try:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            log_event("Available ports: <none>")
            return
        port_text = ", ".join(f"{p.device} ({p.description})" for p in ports)
        log_event(f"Available ports: {port_text}")
    except Exception as e:
        log_event(f"Port scan failed: {type(e).__name__}: {e}")
