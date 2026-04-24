# -*- coding: utf-8 -*-

import threading
import collections
import time
from config import DEFAULT_TARGET_CAN_ID

vesc_com_flag = threading.Event()
prog_flag = threading.Event()

vesc_values = {}
vesc_values_lock = threading.Lock()

vesc_history = {}
vesc_history_lock = threading.Lock()

selected_port = None
control_value = 0.0
control_mode = "Current"

sample_counter = 0
sample_counter_lock = threading.Lock()

vesc_session = None
vesc_session_lock = threading.RLock()

target_can_id = DEFAULT_TARGET_CAN_ID
target_can_id_lock = threading.Lock()

param_state_lock = threading.Lock()
param_state = {
    "runtime": {},
    "bike": {},
    "control": {},
    "pending_initial_refresh": False,
    "ui_sync_needed": False,
    "ui_sync_update_targets": False,
}

custom_msg_test_lock = threading.Lock()
custom_msg_test_results = collections.deque(maxlen=100)


def get_target_can_id():
    with target_can_id_lock:
        return int(target_can_id)


def set_target_can_id(can_id):
    global target_can_id
    with target_can_id_lock:
        target_can_id = int(can_id)


def get_active_vesc_session():
    with vesc_session_lock:
        return vesc_session


def set_active_vesc_session(session):
    global vesc_session
    with vesc_session_lock:
        vesc_session = session


def add_custom_msg_result(name, ok, detail):
    with custom_msg_test_lock:
        custom_msg_test_results.appendleft({
            "t": time.strftime("%H:%M:%S"),
            "name": name,
            "ok": bool(ok),
            "detail": str(detail),
        })


def get_custom_msg_results():
    with custom_msg_test_lock:
        return list(custom_msg_test_results)
