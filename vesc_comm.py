# -*- coding: utf-8 -*-

import time
import threading
import traceback
import types

from pyvesc.VESC.VESC import VESC
from pyvesc.protocol.interface import encode, encode_request, decode

from config import *
from diagnostics import log_event, set_diag, inc_diag, debug_list_ports
from telemetry import build_vesc_values, append_history, response_summary
from state import (
    get_target_can_id, set_target_can_id, vesc_values_lock, vesc_com_flag,
    prog_flag, vesc_session_lock, set_active_vesc_session, param_state_lock,
    param_state, add_custom_msg_result
)
import state
from vesc_messages import (
    #SetCurrent, SetDuty, SetServoPos,
    StartBikeSim, StopBikeSim, GetValuesExp,
    GetBikeRuntime, GetBikeSimParams, GetControlParams,
    SetBikeRuntime, SetBikeSimParams, SetControlParams,
    cmd_id,
)


COMM_PING_CAN = cmd_id("COMM_PING_CAN", 62)


def fwd_msg(msg):
    if USE_CAN_FORWARD:
        msg.can_id = get_target_can_id()
    return msg


def attach_custom_io(vesc):
    if getattr(vesc, "_gui_custom_io_attached", False):
        return

    serial_lock = threading.RLock()
    timeout_s = float(getattr(getattr(vesc, "serial_port", None), "timeout", SERIAL_TIMEOUT_S) or SERIAL_TIMEOUT_S)

    def recover_from_timeout(self):
        try:
            with serial_lock:
                for fn in ("reset_input_buffer", "reset_output_buffer"):
                    try:
                        getattr(self.serial_port, fn)()
                    except Exception:
                        pass
        except Exception:
            pass

    def request_custom(self, msg):
        frame = encode_request(msg)
        with serial_lock:
            try:
                try:
                    self.serial_port.reset_input_buffer()
                except Exception:
                    pass

                self.serial_port.write(frame)
                try:
                    self.serial_port.flush()
                except Exception:
                    pass

                deadline = time.perf_counter() + timeout_s
                raw = bytearray()

                while True:
                    waiting = self.serial_port.in_waiting
                    if waiting > 0:
                        chunk = self.serial_port.read(waiting)
                        if chunk:
                            raw.extend(chunk)
                            try:
                                response, consumed = decode(bytes(raw))
                                if consumed > 0 and response is not None:
                                    return response
                            except Exception:
                                pass

                    if time.perf_counter() >= deadline:
                        try:
                            self.serial_port.reset_input_buffer()
                        except Exception:
                            pass
                        raise TimeoutError(f"Timed out waiting for custom reply, buffered_len={len(raw)}")

                    time.sleep(0.001)
            except OSError as e:
                raise OSError(f"Serial custom request failed: {e}") from e

    def send_custom_no_reply(self, msg):
        frame = encode(msg)
        with serial_lock:
            try:
                written = self.serial_port.write(frame)
                try:
                    self.serial_port.flush()
                except Exception:
                    pass
                return written
            except OSError as e:
                raise OSError(f"Serial custom write failed: {e}") from e

    def request_raw_payload(self, payload):
        frame = encode_raw_payload(payload)
        with serial_lock:
            try:
                try:
                    self.serial_port.reset_input_buffer()
                except Exception:
                    pass
                self.serial_port.write(frame)
                try:
                    self.serial_port.flush()
                except Exception:
                    pass

                deadline = time.perf_counter() + timeout_s
                raw = bytearray()
                while time.perf_counter() < deadline:
                    waiting = self.serial_port.in_waiting
                    if waiting > 0:
                        raw.extend(self.serial_port.read(waiting))
                        payload_rx = extract_first_vesc_payload(bytes(raw))
                        if payload_rx is not None:
                            return payload_rx
                    time.sleep(0.001)
                raise TimeoutError(f"Timed out waiting for raw reply, buffered_len={len(raw)}")
            except OSError as e:
                raise OSError(f"Serial raw request failed: {e}") from e

    vesc.recover_from_timeout = types.MethodType(recover_from_timeout, vesc)
    vesc.request_custom = types.MethodType(request_custom, vesc)
    vesc.send_custom_no_reply = types.MethodType(send_custom_no_reply, vesc)
    vesc.request_raw_payload = types.MethodType(request_raw_payload, vesc)
    vesc._gui_custom_io_attached = True


def encode_raw_payload(payload):
    # VESC packet framing, compatible with short and long packets.
    data = bytes(payload)
    crc = crc16(data)
    if len(data) <= 255:
        return bytes([2, len(data)]) + data + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 3])
    return bytes([3, (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 3])


def extract_first_vesc_payload(raw):
    i = 0
    while i < len(raw):
        st = raw[i]
        if st == 2 and i + 5 <= len(raw):
            ln = raw[i + 1]
            end = i + 2 + ln + 3
            if end <= len(raw) and raw[end - 1] == 3:
                return raw[i + 2:i + 2 + ln]
        elif st == 3 and i + 6 <= len(raw):
            ln = (raw[i + 1] << 8) | raw[i + 2]
            end = i + 3 + ln + 3
            if end <= len(raw) and raw[end - 1] == 3:
                return raw[i + 3:i + 3 + ln]
        i += 1
    return None


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def ping_can_ids(vesc):
    payload = vesc.request_raw_payload(bytes([COMM_PING_CAN]))
    if not payload or payload[0] != COMM_PING_CAN:
        return []
    return [int(x) for x in payload[1:]]


def get_measurements_exp(vesc):
    return vesc.request_custom(fwd_msg(GetValuesExp()))


def get_bike_runtime(vesc):
    return vesc.request_custom(fwd_msg(GetBikeRuntime()))


def get_bike_sim_params(vesc):
    return vesc.request_custom(fwd_msg(GetBikeSimParams()))


def get_control_params(vesc):
    return vesc.request_custom(fwd_msg(GetControlParams()))


def start_bike_sim(vesc):
    vesc.send_custom_no_reply(fwd_msg(StartBikeSim()))
    log_event(f"COMM_START_BIKE_SIM sent to CAN ID {get_target_can_id()}")


def stop_bike_sim(vesc):
    vesc.send_custom_no_reply(fwd_msg(StopBikeSim()))
    log_event(f"COMM_STOP_BIKE_SIM sent to CAN ID {get_target_can_id()}")


def send_current_zero(vesc):
    msg = SetCurrent()
    msg.current = 0.0
    vesc.send_custom_no_reply(fwd_msg(msg))


def send_command(vesc, mode, value):
    with vesc_values_lock:
        local_values = dict(state.vesc_values)

    ctrl_active = int(local_values.get("Ctrl Active", 0))

    if mode == "Duty Cycle":
        msg = SetDuty(); msg.duty = float(value)
        vesc.send_custom_no_reply(fwd_msg(msg))
    elif mode == "Current":
        if ctrl_active:
            stop_bike_sim(vesc)
        else:
            send_current_zero(vesc)
    elif mode == "Position":
        msg = SetServoPos(); msg.servo_pos = float(value)
        vesc.send_custom_no_reply(fwd_msg(msg))
    elif mode == "Speed":
        if not ctrl_active:
            start_bike_sim(vesc)
    else:
        if ctrl_active:
            stop_bike_sim(vesc)
        else:
            send_current_zero(vesc)


def read_param_blocks_from_session(vesc, update_targets=False):
    runtime = get_bike_runtime(vesc)
    bike = get_bike_sim_params(vesc)
    control = get_control_params(vesc)

    # Validate types explicitly to avoid stale/wrong decoded packets.
    for obj, attrs, name in [
        (runtime, ["gear_ratio_bike", "incline_deg"], "runtime"),
        (bike, ["p_air_ro", "p_weight"], "bike"),
        (control, ["p_fo_hz", "p_gz_hz"], "control"),
    ]:
        missing = [a for a in attrs if not hasattr(obj, a)]
        if missing:
            raise TypeError(f"Wrong {name} response: {type(obj).__name__}, missing {missing}")

    with param_state_lock:
        param_state["runtime"] = {
            "gear_ratio_bike": float(runtime.gear_ratio_bike),
            "incline_deg": float(runtime.incline_deg),
            "pumptrack_enabled": bool(runtime.pumptrack_enabled),
            "freewheel_enabled": bool(runtime.freewheel_enabled),
            "pumptrack_period_min": float(runtime.pumptrack_period_min),
        }
        param_state["bike"] = {k: float(getattr(bike, k)) for k in [
            "p_air_ro", "p_c_rr", "p_weight", "p_As", "p_c_air", "p_c_bw", "p_c_wl",
            "p_wheel_radius", "p_mech_gearing", "p_r_bearings", "p_k_v_bw", "p_J",
            "p_B", "p_k_area", "p_height", "p_speed_limit_pos_control_activation"]}
        param_state["control"] = {k: float(getattr(control, k)) for k in [
            "p_fo_hz", "p_gz_hz", "p_fc_TLPF", "p_adrc_scale", "p_sched_spd_floor",
            "p_sched_pos_floor", "p_sched_pos_dead_erpm", "p_sched_spd_sat_erpm", "p_sched_pos_sat_erpm"]}
        param_state["ui_sync_needed"] = True
        param_state["ui_sync_update_targets"] = bool(update_targets)


def response_probe(vesc, name, fn):
    try:
        res = fn()
        detail = response_summary(res) if hasattr(res, "__dict__") else str(res)
        add_custom_msg_result(name, True, detail)
        log_event(f"{name}: OK -> {detail}")
        return res
    except Exception as e:
        detail = f"{type(e).__name__}: {e}"
        add_custom_msg_result(name, False, detail)
        log_event(f"{name}: FAIL -> {detail}")
        return None


def run_custom_message_selftest(vesc):
    log_event("Running custom message self-test...")
    response_probe(vesc, "COMM_GET_BIKE_RUNTIME", lambda: get_bike_runtime(vesc)); time.sleep(0.05)
    response_probe(vesc, "COMM_GET_BIKE_SIM_PARAMS", lambda: get_bike_sim_params(vesc)); time.sleep(0.05)
    response_probe(vesc, "COMM_GET_CONTROL_PARAMS", lambda: get_control_params(vesc)); time.sleep(0.05)
    response_probe(vesc, "COMM_GET_VALUES_EXP", lambda: get_measurements_exp(vesc)); time.sleep(0.05)
    log_event("Custom message self-test finished")


def snapshot_command():
    return (state.control_mode, float(state.control_value))


def command_changed(prev_cmd, new_cmd):
    if prev_cmd is None:
        return True
    prev_mode, prev_val = prev_cmd
    new_mode, new_val = new_cmd
    if prev_mode != new_mode:
        return True
    if new_mode == "Duty Cycle":
        return abs(new_val - prev_val) >= DUTY_EPS
    if new_mode == "Current":
        return abs(new_val - prev_val) >= CURRENT_EPS
    if new_mode == "Position":
        return abs(new_val - prev_val) >= SERVO_EPS
    if new_mode == "Speed":
        return False
    return abs(new_val - prev_val) >= CURRENT_EPS


def vesc_communication():
    log_event("Comm thread started")
    while True:
        set_diag(comm_thread_alive=True, read_only_mode=READ_ONLY_MODE, commands_enabled=ENABLE_COMMANDS)

        if not (vesc_com_flag.is_set() and state.selected_port):
            set_diag(serial_open=False)
            time.sleep(0.1)
            continue

        port = state.selected_port
        session_id = int(time.time() * 1000) % 100000000
        set_diag(serial_session_id=session_id, selected_port=port)

        try:
            log_event(f"Opening VESC on {port} (session {session_id})")
            debug_list_ports()
            with VESC(serial_port=port, start_heartbeat=False, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT_S) as vesc:
                attach_custom_io(vesc)
                set_active_vesc_session(vesc)
                set_diag(serial_open=True)
                log_event(f"Serial open OK on {port} (session {session_id}); target CAN {get_target_can_id()}")

                try:
                    vals = get_measurements_exp(vesc)
                    log_event(f"Forwarded values OK from CAN {get_target_can_id()}: rpm={getattr(vals, 'rpm', 'n/a')}, vin={getattr(vals, 'v_in', 'n/a')}")
                except Exception as e:
                    log_event(f"Forwarded values failed: {type(e).__name__}: {e}")

                try:
                    run_custom_message_selftest(vesc)
                except Exception as e:
                    log_event(f"Custom self-test crashed: {type(e).__name__}: {e}")

                time.sleep(0.2)
                try:
                    vesc.recover_from_timeout()
                except Exception:
                    pass

                next_param_refresh_time = 0.0
                try:
                    with param_state_lock:
                        need_initial = bool(param_state["pending_initial_refresh"])
                        param_state["pending_initial_refresh"] = False
                    if need_initial:
                        read_param_blocks_from_session(vesc, update_targets=True)
                        next_param_refresh_time = time.perf_counter() + PARAM_REFRESH_PERIOD_S
                        log_event("Initial parameter refresh done")
                except Exception as e:
                    log_event(f"Initial parameter refresh failed: {type(e).__name__}: {e}")

                consecutive_timeouts = 0
                last_cmd_sent = None
                last_command_tx_time = 0.0
                next_telem_time = time.perf_counter()

                while vesc_com_flag.is_set() and state.selected_port == port:
                    now = time.perf_counter()
                    set_diag(last_loop_time=now)
                    state.control_mode = "Speed" if prog_flag.is_set() else "Current"
                    state.control_value = 0.0

                    if now < next_telem_time:
                        time.sleep(min(0.001, next_telem_time - now))
                        continue

                    cycle_start = time.perf_counter()
                    try:
                        response = get_measurements_exp(vesc)
                        new_values = build_vesc_values(response)
                        with vesc_values_lock:
                            state.vesc_values = new_values
                        append_history(new_values)
                        rx_now = time.perf_counter()
                        from diagnostics import diag, diag_lock
                        with diag_lock:
                            diag["last_rx_time"] = rx_now
                            diag["rx_count"] += 1
                            diag["consecutive_read_errors"] = 0
                            diag["consecutive_write_errors"] = 0
                            diag["last_response_summary"] = response_summary(response)
                        consecutive_timeouts = 0
                    except TimeoutError as e:
                        err = f"{type(e).__name__}: {e}"
                        inc_diag("read_errors")
                        set_diag(last_read_error=err, last_comm_error=err)
                        from diagnostics import diag, diag_lock
                        with diag_lock:
                            diag["consecutive_read_errors"] += 1
                        consecutive_timeouts += 1
                        log_event(f"READ TIMEOUT on {port}: {err} ({consecutive_timeouts})")
                        try:
                            vesc.recover_from_timeout()
                        except Exception:
                            pass
                        if consecutive_timeouts >= HARD_TIMEOUT_LIMIT:
                            raise
                        next_telem_time = time.perf_counter() + TELEMETRY_PERIOD_S
                        time.sleep(0.05)
                        continue
                    except Exception as e:
                        err = f"{type(e).__name__}: {e}"
                        inc_diag("read_errors")
                        set_diag(last_read_error=err, last_comm_error=err, serial_open=False)
                        log_event(f"READ ERROR on {port}: {err}")
                        raise

                    time.sleep(TX_GUARD_S)
                    if not READ_ONLY_MODE and ENABLE_COMMANDS:
                        cmd_now = snapshot_command()
                        should_send = command_changed(last_cmd_sent, cmd_now) or ((time.perf_counter() - last_command_tx_time) >= COMMAND_KEEPALIVE_S)
                        if should_send or (cmd_now[0] == "Speed" and last_cmd_sent is None):
                            try:
                                time.sleep(TX_GUARD_S * 2.0)
                                send_command(vesc, cmd_now[0], cmd_now[1])
                                time.sleep(TX_GUARD_S * 2.0)
                                tx_now = time.perf_counter()
                                from diagnostics import diag, diag_lock
                                with diag_lock:
                                    diag["last_tx_time"] = tx_now
                                    diag["tx_count"] += 1
                                    diag["consecutive_write_errors"] = 0
                                last_command_tx_time = tx_now
                                last_cmd_sent = cmd_now
                            except Exception as e:
                                err = f"{type(e).__name__}: {e}"
                                inc_diag("write_errors")
                                set_diag(last_write_error=err, last_comm_error=err, serial_open=False)
                                log_event(f"WRITE ERROR on {port}: {err}")
                                raise
                        time.sleep(TX_GUARD_S)

                    try:
                        now_after = time.perf_counter()
                        if now_after >= next_param_refresh_time:
                            read_param_blocks_from_session(vesc, update_targets=False)
                            next_param_refresh_time = now_after + PARAM_REFRESH_PERIOD_S
                    except Exception as e:
                        log_event(f"Periodic parameter refresh failed: {type(e).__name__}: {e}")
                        next_param_refresh_time = time.perf_counter() + PARAM_REFRESH_PERIOD_S

                    next_telem_time = cycle_start + TELEMETRY_PERIOD_S
        except Exception:
            err = traceback.format_exc()
            set_diag(serial_open=False, last_comm_error=err)
            log_event(f"Communication session failed on {port}")
            time.sleep(REOPEN_BACKOFF_S)
        finally:
            set_active_vesc_session(None)
            set_diag(serial_open=False)
