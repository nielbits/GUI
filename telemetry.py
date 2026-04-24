# -*- coding: utf-8 -*-

import math
import time
from config import *
from state import (
    param_state_lock, param_state, vesc_history, vesc_history_lock,
    sample_counter_lock
)
import state


def get_status_bit(word, bit):
    return int(bool((int(word) & 0xFFFFFFFF) & (1 << bit)))


def get_speed_conversion_params():
    with param_state_lock:
        bike = dict(param_state.get("bike", {}))

    wheel_radius = float(bike.get("p_wheel_radius", WHEEL_RADIUS_M))
    mech_gearing = float(bike.get("p_mech_gearing", GM))

    if abs(wheel_radius) < 1e-9:
        wheel_radius = WHEEL_RADIUS_M
    if abs(mech_gearing) < 1e-9:
        mech_gearing = GM

    return wheel_radius, mech_gearing


def estimate_speed_kmh_from_motor_mech_radps(motor_mech_radps, gear_ratio):
    try:
        gear_ratio = float(gear_ratio)
        if abs(gear_ratio) < 1e-9:
            return 0.0
        wheel_radius, mech_gearing = get_speed_conversion_params()
        gearing = mech_gearing / gear_ratio
        if abs(gearing) < 1e-9:
            return 0.0
        speed_mps = float(motor_mech_radps) * wheel_radius / gearing
        return speed_mps * 3.6
    except Exception:
        return 0.0


def estimate_real_speed_kmh(erpm, gear_ratio):
    try:
        motor_mech_radps = (float(erpm) / POLE_PAIRS) * (2.0 * math.pi / 60.0)
        return estimate_speed_kmh_from_motor_mech_radps(motor_mech_radps, gear_ratio)
    except Exception:
        return 0.0


def estimate_setpoint_speed_kmh(model_speed, gear_ratio):
    try:
        return estimate_speed_kmh_from_motor_mech_radps(float(model_speed), gear_ratio)
    except Exception:
        return 0.0


def build_vesc_values(response):
    v_in = getattr(response, "v_in", 0.0)
    avg_input_current = getattr(response, "avg_input_current", 0.0)
    duty_cycle_now = getattr(response, "duty_cycle_now", 0.0)
    avg_motor_current = getattr(response, "avg_motor_current", 0.0)
    status_bits_ext = int(getattr(response, "status_bits_ext", 0)) & 0xFFFFFFFF

    rpm_erpm = float(getattr(response, "rpm", 0.0))
    erpm_soll = float(getattr(response, "erpm_soll", 0.0))
    gear_ratio = float(getattr(response, "gear_ratio", 0.0))
    model_speed = float(getattr(response, "model_speed", 0.0))
    uw_theta = float(getattr(response, "uw_theta", 0.0))
    uw_angle_sp = float(getattr(response, "uw_angle_sp", 0.0))

    position_error_rad = uw_angle_sp - uw_theta
    position_error_deg = math.degrees(position_error_rad) / GM
    real_speed_kmh = estimate_real_speed_kmh(rpm_erpm, gear_ratio)
    setpoint_speed_kmh = estimate_setpoint_speed_kmh(model_speed, gear_ratio)

    return {
        "RPM Motor": rpm_erpm / POLE_PAIRS,
        "RPM Set": erpm_soll / POLE_PAIRS,
        "LESO RPM": float(getattr(response, "leso_omega", 0.0)) / POLE_PAIRS,
        "Motor Current": avg_motor_current,
        "Battery Current": avg_input_current,
        "IQ Filtered": getattr(response, "avg_iq", 0.0),
        "IQ Instant": getattr(response, "iq_current", 0.0),
        "IQ Set": getattr(response, "iq_set", 0.0),
        "Input Voltage": v_in,
        "Duty Cycle": duty_cycle_now,
        "T_friction": getattr(response, "tf", 0.0),
        "Gear Ratio": gear_ratio,
        "Setpoint Speed km/h": setpoint_speed_kmh,
        "Real Speed km/h": real_speed_kmh,
        "F_combine": getattr(response, "f_combine", 0.0),
        "UW Theta": uw_theta,
        "Pedal Torque Observed": getattr(response, "tp_observed", 0.0),
        "Torque Motor": -getattr(response, "torque_motor", 0.0),
        "Torque FF": -getattr(response, "torque_ff", 0.0),
        "Param Index": getattr(response, "param_index", 0),
        "Param Value": getattr(response, "param_from_index", 0.0),
        "Pos Term Speed": getattr(response, "pos_term_speed", 0.0),
        "Speed Error": getattr(response, "speed_error", 0.0),
        "T_F_combine": getattr(response, "t_f_combine", 0.0),
        "Incline Deg Ist": getattr(response, "incline_deg_ist", 0.0),
        "UW Angle SP": uw_angle_sp,
        "Position Error deg": position_error_deg,
        "Ctrl Active": get_status_bit(status_bits_ext, STATUS_BIT_CTRL_ACTIVE),
        "Forced FW": get_status_bit(status_bits_ext, STATUS_BIT_FORCED_FW),
        "START": get_status_bit(status_bits_ext, STATUS_BIT_START),
        "INDEX_FOUND": get_status_bit(status_bits_ext, STATUS_BIT_INDEX_FOUND),
        "ENABLE": get_status_bit(status_bits_ext, STATUS_BIT_ENABLE),
        "Status Bits Ext": status_bits_ext,
        "Power In": v_in * avg_input_current,
    }


def response_summary(response):
    try:
        if response is None:
            return "<none>"
        if hasattr(response, "__dict__"):
            items = []
            for k, v in response.__dict__.items():
                if not k.startswith("_"):
                    items.append(f"{k}={v}")
            return ", ".join(items[:12])
        return str(response)
    except Exception:
        return "<unavailable>"


def append_history(values_dict):
    now = time.perf_counter()
    with sample_counter_lock:
        state.sample_counter += 1
        sc = state.sample_counter

    with vesc_history_lock:
        if "time_s" not in vesc_history:
            vesc_history["time_s"] = []
        if "sample_idx" not in vesc_history:
            vesc_history["sample_idx"] = []

        numeric_items = {k: float(v) for k, v in values_dict.items() if isinstance(v, (int, float))}
        numeric_items["sample_idx"] = float(sc)

        for key in numeric_items.keys():
            if key not in vesc_history:
                vesc_history[key] = [float("nan")] * len(vesc_history["time_s"])

        for key in list(vesc_history.keys()):
            if key == "time_s":
                continue
            if key not in numeric_items:
                numeric_items[key] = float("nan")

        vesc_history["time_s"].append(now)
        for key, value in numeric_items.items():
            vesc_history[key].append(value)
