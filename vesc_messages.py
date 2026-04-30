# -*- coding: utf-8 -*-

from pyvesc.protocol.base import VESCMessage
from pyvesc.VESC.messages import VedderCmd


def cmd_id(name, fallback):
    value = getattr(VedderCmd, name, fallback)
    return int(getattr(value, "value", value))

"""
class SetCurrent(metaclass=VESCMessage):
    id = cmd_id("COMM_SET_CURRENT", 6)
    fields = [('current', 'i', 1000)]


class SetDuty(metaclass=VESCMessage):
    id = cmd_id("COMM_SET_DUTY", 5)
    fields = [('duty', 'i', 100000)]


class SetServoPos(metaclass=VESCMessage):
    id = cmd_id("COMM_SET_SERVO_POS", 12)
    fields = [('servo_pos', 'h', 1000)]
"""

class StartBikeSim(metaclass=VESCMessage):
    id = 167
    fields = []


class StopBikeSim(metaclass=VESCMessage):
    id = 168
    fields = []


class GetValuesExp(metaclass=VESCMessage):
    id = 164
    fields = [
        ('temp_fet', 'h', 10),
        ('temp_motor', 'h', 10),
        ('avg_motor_current', 'i', 100),
        ('avg_input_current', 'i', 100),
        ('avg_id', 'i', 100),
        ('avg_iq', 'i', 100),
        ('duty_cycle_now', 'h', 1000),
        ('rpm', 'i', 1),
        ('v_in', 'h', 10),
        ('amp_hours', 'i', 10000),
        ('amp_hours_charged', 'i', 10000),
        ('watt_hours', 'i', 10000),
        ('watt_hours_charged', 'i', 10000),
        ('tachometer', 'i', 1),
        ('tachometer_abs', 'i', 1),
        ('mc_fault_code', 'c', 0),
        ('pid_pos_now', 'i', 1000000),
        ('app_controller_id', 'c', 0),
        ('temp_mos1', 'h', 10),
        ('temp_mos2', 'h', 10),
        ('temp_mos3', 'h', 10),
        ('avg_vd', 'i', 1000),
        ('avg_vq', 'i', 1000),
        ('status', 'c', 0),
        ('erpm_soll', 'h', 1),
        ('tf', 'h', 10000),
        ('gear_ratio', 'i', 100),
        ('id_current', 'i', 100),
        ('iq_current', 'i', 100),
        ('model_speed', 'h', 100),
        ('f_combine', 'h', 10),
        ('iq_set', 'i', 10000),
        ('uw_theta', 'i', 10000),
        ('leso_omega', 'i', 10000),
        ('tp_observed', 'i', 10000),
        ('ctrl_sm_reset_reason', 'i', 1),
        ('i_res', 'i', 1000000),
        ('uw_angle_sp', 'i', 10000),
        ('param_from_index', 'i', 1000),
        ('pos_term_speed', 'i', 1000),
        ('speed_error', 'i', 1000),
        ('t_f_combine', 'i', 1000),
        ('incline_deg_ist', 'i', 1000),
        ('torque_motor', 'i', 1000),
        ('torque_ff', 'i', 1000),
        ('ctrl_sm_state', 'i', 1000),
        ('ctrl_sm_still_cycles', 'i', 1000),
        ('ctrl_sm_index_lost_cycles', 'i', 1000),
        ('status_bits_ext', 'I', 1),
    ]


class GetBikeRuntime(metaclass=VESCMessage):
    id = 166
    fields = [
        ('gear_ratio_bike', 'i', 1000000),
        ('incline_deg', 'i', 1000),
        ('pumptrack_enabled', 'B', 1),
        ('freewheel_enabled', 'B', 1),
        ('pumptrack_period_min', 'i', 1000),
    ]


class GetBikeSimParams(metaclass=VESCMessage):
    id = 161
    fields = [
        ('p_air_ro', 'i', 1000000),
        ('p_c_rr', 'i', 1000000),
        ('p_weight', 'i', 1000),
        ('p_As', 'i', 1000000),
        ('p_c_air', 'i', 1000000),
        ('p_c_bw', 'i', 1000000),
        ('p_c_wl', 'i', 1000000),
        ('p_wheel_radius', 'i', 1000000),
        ('p_mech_gearing', 'i', 1000000),
        ('p_r_bearings', 'i', 1000000),
        ('p_k_v_bw', 'i', 1000000),
        ('p_J', 'i', 1000000),
        ('p_B', 'i', 1000000),
        ('p_k_area', 'i', 1000000),
        ('p_height', 'i', 1000000),
        ('p_speed_limit_pos_control_activation', 'i', 1000000),
    ]


class GetControlParams(metaclass=VESCMessage):
    id = 163
    fields = [
        ('p_fo_hz', 'i', 1000000),
        ('p_gz_hz', 'i', 1000000),
        ('p_fc_TLPF', 'i', 1000000),
        ('p_adrc_scale', 'i', 1000000),
        ('p_sched_spd_floor', 'i', 1000000),
        ('p_sched_pos_floor', 'i', 1000000),
        ('p_sched_pos_dead_erpm', 'i', 1000000),
        ('p_sched_spd_sat_erpm', 'i', 1000000),
        ('p_sched_pos_sat_erpm', 'i', 1000000),
    ]


class SetBikeRuntime(metaclass=VESCMessage):
    id = 165
    fields = [
        ('gear_ratio_bike', 'i', 1000000),
        ('incline_deg', 'i', 1000),
        ('pumptrack_enabled', 'B', 1),
        ('freewheel_enabled', 'B', 1),
        ('pumptrack_period_min', 'i', 1000),
    ]


class SetBikeSimParams(metaclass=VESCMessage):
    id = 160
    fields = GetBikeSimParams.fields


class SetControlParams(metaclass=VESCMessage):
    id = 162
    fields = GetControlParams.fields
