# -*- coding: utf-8 -*-

import sys
import time
import math
import threading
import traceback
import collections
import serial
import serial.tools.list_ports

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from pyvesc.VESC.VESC import VESC


# =========================
# Global state
# =========================
DEBUG = False


def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


vesc_com_flag = threading.Event()   # user wants communication
prog_flag = threading.Event()
freewheel = threading.Event()
freewheel.set()

pumptrack = threading.Event()
pumptrack.clear()

vesc_values = {}
vesc_values_lock = threading.Lock()

vesc_history = {}
vesc_history_lock = threading.Lock()

selected_port = None
control_value = 0.0
control_mode = "Current"

WHEEL_CIRCUMFERENCE_M = 2.105
WHEEL_RADIUS_M = WHEEL_CIRCUMFERENCE_M / (2.0 * math.pi)
POLE_PAIRS = 23.0
GM = 2.603036876355748

PLOT_SCROLL_WINDOW_S = 20.0
GUI_REFRESH_MS = 100
STALE_WARNING_S = 0.5
STALE_TIMEOUT_S = 1.0

sample_counter = 0
sample_counter_lock = threading.Lock()


# =========================
# Communication debug config
# =========================
READ_ONLY_MODE = False
ENABLE_COMMANDS = True

TELEMETRY_PERIOD_S = 0.1
COMMAND_KEEPALIVE_S = 0.5
TX_GUARD_S = 0.02
REOPEN_BACKOFF_S = 3.0

SOFT_TIMEOUT_LIMIT = 3
HARD_TIMEOUT_LIMIT = 8

DUTY_EPS = 1e-4
CURRENT_EPS = 0.02
SERVO_EPS = 1e-3
SPEED_EPS = 1.0


# =========================
# Diagnostics / debug state
# =========================
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


# =========================
# Parameter definitions
# =========================
PARAM_DEFS = [
    (1, "gear_ratio_bike"),
    (2, "p_air_ro"),
    (3, "p_c_rr"),
    (4, "p_weight"),
    (5, "p_As"),
    (6, "p_c_air"),
    (7, "p_c_bw"),
    (8, "p_c_wl"),
    (9, "p_wheel_radius"),
    (10, "p_r_bearings"),
    (11, "p_k_v_bw"),
    (12, "p_k_area"),
    (13, "p_height"),
    (14, "p_fo_hz"),
    (15, "p_gz_hz"),
    (16, "p_fc_TLPF"),
    (17, "p_adrc_scale"),
    (18, "p_kp_pos"),
    (19, "p_ki_pos"),
    (20, "p_kd_pos"),
    (21, "p_J"),
    (22, "p_incline_deg"),
    (23, "p_mech_gearing"),
    (24, "pumptrack_enabled"),
    (25, "freewheel_enabled"),
]


# =========================
# Telemetry helpers
# =========================
def estimate_real_speed_kmh(erpm, gear_ratio):
    try:
        gear_ratio = float(gear_ratio)
        if abs(gear_ratio) < 1e-9:
            return 0.0

        gearing = GM / gear_ratio

        motor_mech_radps = (float(erpm) / POLE_PAIRS) * (2.0 * math.pi / 60.0)

        speed_mps = motor_mech_radps * WHEEL_RADIUS_M / gearing
        return speed_mps * 3.6
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

    rpm_motor = rpm_erpm / POLE_PAIRS
    rpm_set = erpm_soll / POLE_PAIRS
    leso_rpm = float(getattr(response, "leso_omega", 0.0)) / POLE_PAIRS

    setpoint_speed_kmh = model_speed * gear_ratio

    return {
        "RPM Motor": rpm_motor,
        "RPM Set": rpm_set,
        "LESO RPM": leso_rpm,

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
        "Param Index": getattr(response, "param_index", 0),
        "Param Value": getattr(response, "param_from_index", 0.0),
        "Pos Term Speed": getattr(response, "pos_term_speed", 0.0),
        "Speed Error": getattr(response, "speed_error", 0.0),
        "T_F_combine": getattr(response, "t_f_combine", 0.0),
        "Incline Deg Ist": getattr(response, "incline_deg_ist", 0.0),
        "UW Angle SP": uw_angle_sp,
        "Position Error deg": position_error_deg,

        "Ctrl Active": int(bool(status_bits_ext & (1 << 0))),
        "Forced FW": int(bool(status_bits_ext & (1 << 1))),
        "START": int(bool(status_bits_ext & (1 << 2))),
        "INDEX_FOUND": int(bool(status_bits_ext & (1 << 3))),
        "ENABLE": int(bool(status_bits_ext & (1 << 4))),

        "Status Bits Ext": status_bits_ext,
        "Power In": v_in * avg_input_current,
    }


def response_summary(response):
    try:
        rpm = getattr(response, "rpm", None)
        v_in = getattr(response, "v_in", None)
        status_bits_ext = getattr(response, "status_bits_ext", None)
        return f"rpm={rpm}, v_in={v_in}, status_bits_ext={status_bits_ext}"
    except Exception:
        return "<unavailable>"


def append_history(values_dict):
    global sample_counter

    now = time.perf_counter()

    with sample_counter_lock:
        sample_counter += 1
        sc = sample_counter

    with vesc_history_lock:
        if "time_s" not in vesc_history:
            vesc_history["time_s"] = []
        if "sample_idx" not in vesc_history:
            vesc_history["sample_idx"] = []

        numeric_items = {
            key: float(value)
            for key, value in values_dict.items()
            if isinstance(value, (int, float))
        }
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


# =========================
# Small UI helpers
# =========================
class LiveValueRow(QtWidgets.QWidget):
    def __init__(self, label_text, unit_text=""):
        super().__init__()
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(2, 1, 2, 1)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(0)

        self.label = QtWidgets.QLabel(label_text)
        self.edit = QtWidgets.QLineEdit("0")
        self.edit.setReadOnly(True)
        self.edit.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.unit = QtWidgets.QLabel(unit_text)

        font_label = QtGui.QFont()
        font_label.setPointSize(9)
        font_label.setBold(True)

        font_edit = QtGui.QFont()
        font_edit.setPointSize(9)

        self.label.setFont(font_label)
        self.edit.setFont(font_edit)
        self.unit.setFont(font_label)

        self.label.setMinimumWidth(75)
        self.edit.setMinimumWidth(72)
        self.edit.setMaximumWidth(95)
        self.edit.setMinimumHeight(22)
        self.unit.setMinimumWidth(35)

        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.edit, 0, 1)
        layout.addWidget(self.unit, 0, 2)

    def set_value(self, value):
        self.edit.setText(value)


class FlagIndicator(QtWidgets.QFrame):
    def __init__(self, size=15):
        super().__init__()
        self.setFixedSize(size, size)
        self.setFrameShape(QtWidgets.QFrame.Box)
        self.setLineWidth(1)
        self.set_active(False)

    def set_active(self, active):
        if active:
            self.setStyleSheet("background-color: rgb(0, 200, 0); border: 1px solid black;")
        else:
            self.setStyleSheet("background-color: rgb(90, 90, 90); border: 1px solid black;")


class ReadableFlag(QtWidgets.QWidget):
    def __init__(self, text):
        super().__init__()
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.flag = FlagIndicator(15)
        self.label = QtWidgets.QLabel(text)

        font = QtGui.QFont()
        font.setPointSize(9)
        self.label.setFont(font)

        layout.addWidget(self.flag)
        layout.addWidget(self.label)

    def set_state(self, active):
        self.flag.set_active(active)


class ParamBox(QtWidgets.QGroupBox):
    def __init__(self, param_index, param_name):
        super().__init__(param_name)
        self.param_index = param_index
        self.param_name = param_name

        font = QtGui.QFont()
        font.setPointSize(8)
        self.setFont(font)

        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(3)

        self.edit_target = QtWidgets.QLineEdit()
        self.edit_target.setPlaceholderText("set value")

        self.edit_actual = QtWidgets.QLineEdit()
        self.edit_actual.setReadOnly(True)
        self.edit_actual.setPlaceholderText("actual value")

        self.edit_target.setMinimumHeight(22)
        self.edit_actual.setMinimumHeight(22)

        layout.addWidget(QtWidgets.QLabel("Set"), 0, 0)
        layout.addWidget(self.edit_target, 0, 1)
        layout.addWidget(QtWidgets.QLabel("Act"), 1, 0)
        layout.addWidget(self.edit_actual, 1, 1)


class PopupChartsWindow(QtWidgets.QMainWindow):
    def __init__(self, parent_ui):
        super().__init__()
        self.parent_ui = parent_ui
        self.setWindowTitle("All Charts")
        self.resize(1400, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        toolbar = QtWidgets.QHBoxLayout()
        self.btn_popup_autorange = QtWidgets.QPushButton("Auto Range")
        self.btn_popup_zoomfit = QtWidgets.QPushButton("Zoom to Fit")
        self.btn_popup_zoomout_all = QtWidgets.QPushButton("Zoom Out All")
        self.btn_popup_reset_x = QtWidgets.QPushButton("Reset X")
        self.btn_popup_reset_y = QtWidgets.QPushButton("Reset Y")
        self.btn_popup_mouse = QtWidgets.QPushButton("Mouse Zoom/Pan: ON")
        self.btn_popup_mouse.setCheckable(True)
        self.btn_popup_mouse.setChecked(True)

        toolbar.addWidget(self.btn_popup_autorange)
        toolbar.addWidget(self.btn_popup_zoomfit)
        toolbar.addWidget(self.btn_popup_zoomout_all)
        toolbar.addWidget(self.btn_popup_reset_x)
        toolbar.addWidget(self.btn_popup_reset_y)
        toolbar.addWidget(self.btn_popup_mouse)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plotWidgetPopup = pg.GraphicsLayoutWidget()
        self.plotWidgetPopup.setBackground('k')
        layout.addWidget(self.plotWidgetPopup)

        self.plot_popup_currents = parent_ui.make_plot(self.plotWidgetPopup, 0, 0, "Currents")
        self.curve_popup_iq_filtered = self.plot_popup_currents.plot(pen=pg.mkPen('g', width=1.2), name='IQ Filtered')
        self.curve_popup_iq_set = self.plot_popup_currents.plot(pen=pg.mkPen((255, 165, 0), width=1.2), name='IQ Set')

        self.plot_popup_rpms = parent_ui.make_plot(self.plotWidgetPopup, 0, 1, "RPMs")
        self.curve_popup_rpm = self.plot_popup_rpms.plot(pen=pg.mkPen('y', width=1.2), name='RPM Motor')
        self.curve_popup_rpm_set = self.plot_popup_rpms.plot(pen=pg.mkPen('c', width=1.2), name='RPM Set')
        self.curve_popup_leso_rpm = self.plot_popup_rpms.plot(pen=pg.mkPen('m', width=1.2), name='LESO RPM')

        self.plot_popup_torques = parent_ui.make_plot(self.plotWidgetPopup, 0, 2, "Torques")
        self.curve_popup_tf = self.plot_popup_torques.plot(pen=pg.mkPen('y', width=1.2), name='T_friction')
        self.curve_popup_tp_obs = self.plot_popup_torques.plot(pen=pg.mkPen('c', width=1.2), name='Tp_Observed')
        self.curve_popup_tf_combine = self.plot_popup_torques.plot(pen=pg.mkPen('m', width=1.2), name='T_F_combine')

        self.plot_popup_errors = parent_ui.make_plot(self.plotWidgetPopup, 0, 3, "Errors")
        self.curve_popup_speed_error = self.plot_popup_errors.plot(pen=pg.mkPen('r', width=1.2), name='Speed Error')
        self.curve_popup_pos_term_speed = self.plot_popup_errors.plot(pen=pg.mkPen('c', width=1.2), name='Pos Term Speed')

        self.plot_popup_speed_kmh = parent_ui.make_plot(self.plotWidgetPopup, 1, 0, "Speed km/h")
        self.curve_popup_model_speed = self.plot_popup_speed_kmh.plot(pen=pg.mkPen('y', width=1.2), name='Setpoint Speed')
        self.curve_popup_real_speed = self.plot_popup_speed_kmh.plot(pen=pg.mkPen('c', width=1.2), name='Real Speed')

        self.plot_popup_force_incline = parent_ui.make_plot(self.plotWidgetPopup, 1, 1, "Force / Incline")
        self.curve_popup_fcombine = self.plot_popup_force_incline.plot(pen=pg.mkPen('m', width=1.2), name='F_combine')
        self.curve_popup_incline = self.plot_popup_force_incline.plot(pen=pg.mkPen('g', width=1.2), name='Incline')

        self.plot_popup_uw = parent_ui.make_plot(self.plotWidgetPopup, 1, 2, "UW Theta [rad]")
        self.curve_popup_uw_angle_sp = self.plot_popup_uw.plot(pen=pg.mkPen('c', width=1.2), name='SP')
        self.curve_popup_uw_theta = self.plot_popup_uw.plot(pen=pg.mkPen('y', width=1.2), name='Theta')

        self.plot_popup_pos_error = parent_ui.make_plot(self.plotWidgetPopup, 1, 3, "Position Error [deg]")
        self.curve_popup_position_error = self.plot_popup_pos_error.plot(pen=pg.mkPen('r', width=1.2), name='Err')

        self.popup_plots = [
            self.plot_popup_currents,
            self.plot_popup_rpms,
            self.plot_popup_torques,
            self.plot_popup_errors,
            self.plot_popup_speed_kmh,
            self.plot_popup_force_incline,
            self.plot_popup_uw,
            self.plot_popup_pos_error,
        ]

        self.popup_scroll_mode = True

        self.btn_popup_autorange.clicked.connect(lambda: self.parent_ui.autorange_plots(self.popup_plots))
        self.btn_popup_zoomfit.clicked.connect(self.zoom_fit_popup)
        self.btn_popup_zoomout_all.clicked.connect(self.zoom_out_all_popup)
        self.btn_popup_reset_x.clicked.connect(self.reset_x_popup)
        self.btn_popup_reset_y.clicked.connect(lambda: self.parent_ui.reset_plots_y(self.popup_plots))
        self.btn_popup_mouse.clicked.connect(
            lambda: self.parent_ui.toggle_plot_mouse(self.popup_plots, self.btn_popup_mouse)
        )

    def zoom_fit_popup(self):
        self.popup_scroll_mode = False
        self.parent_ui.autorange_plots(self.popup_plots)

    def zoom_out_all_popup(self):
        self.popup_scroll_mode = False
        with vesc_history_lock:
            if "time_s" not in vesc_history or len(vesc_history["time_s"]) == 0:
                return
            t = vesc_history["time_s"]
            t0 = t[0]
            x0 = 0.0
            x1 = max(1.0, t[-1] - t0)

        for p in self.popup_plots:
            p.setXRange(x0, x1, padding=0.02)
            p.enableAutoRange(axis='y', enable=True)
            p.autoRange()

    def reset_x_popup(self):
        self.popup_scroll_mode = True


# =========================
# Main GUI
# =========================
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 800)
        MainWindow.setMinimumSize(1000, 650)
        MainWindow.setMaximumSize(1920, 1080)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)

        self.param_boxes = {}
        self.chart_tabs_1 = []
        self.chart_tabs_2 = []
        self.popup_window = None

        self.charts1_scroll_mode = True
        self.charts2_scroll_mode = True

        self.build_ui(MainWindow)
        self.make_connections()
        self.refresh_ports()

        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.refresh_live_data)
        self.gui_timer.start(GUI_REFRESH_MS)

    def build_ui(self, MainWindow):
        root = QtWidgets.QVBoxLayout(self.centralwidget)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        topbar = QtWidgets.QHBoxLayout()
        self.comboBox_portselect = QtWidgets.QComboBox()
        self.comboBox_portselect.setMinimumWidth(260)
        self.pushButton_refresh = QtWidgets.QPushButton("Refresh Ports")
        self.pushButton_refresh.setMaximumWidth(120)
        self.pushButton_popup_charts = QtWidgets.QPushButton("Open All Charts Popup")
        self.pushButton_popup_charts.setMinimumWidth(170)

        topbar.addWidget(QtWidgets.QLabel("Serial Port:"))
        topbar.addWidget(self.comboBox_portselect)
        topbar.addWidget(self.pushButton_refresh)
        topbar.addWidget(self.pushButton_popup_charts)
        topbar.addStretch(1)
        root.addLayout(topbar)

        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs)

        self.tab_main = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_main, "Main")
        self.build_main_tab()

        self.tab_charts_1 = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_charts_1, "Charts 1")
        self.build_charts_tab_1()

        self.tab_charts_2 = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_charts_2, "Charts 2")
        self.build_charts_tab_2()

        self.tab_params = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_params, "Params")
        self.build_params_tab()

        self.tab_telemetry = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_telemetry, "Telemetry / Debug")
        self.build_telemetry_tab()

        self.menubar = QtWidgets.QMenuBar(MainWindow)
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)

        MainWindow.setWindowTitle("VESC Telemetry GUI")

    def build_main_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_main)
        layout.setSpacing(4)
        layout.setContentsMargins(4, 4, 4, 4)

        controls_row = QtWidgets.QHBoxLayout()
        controls_row.setSpacing(6)

        self.pushButton_connect = QtWidgets.QPushButton("Connect")
        self.pushButton_disconnect = QtWidgets.QPushButton("Disconnect")
        self.start = QtWidgets.QPushButton("Start")
        self.stop = QtWidgets.QPushButton("Stop")
        self.Reset = QtWidgets.QPushButton("Reset")

        for btn in [self.pushButton_connect, self.pushButton_disconnect, self.start, self.stop, self.Reset]:
            btn.setMinimumHeight(30)

        self.label_link_state = QtWidgets.QLabel("LINK: IDLE")
        self.label_link_state.setMinimumWidth(220)
        self.label_link_state.setStyleSheet("font-weight: bold;")

        controls_row.addWidget(self.pushButton_connect)
        controls_row.addWidget(self.pushButton_disconnect)
        controls_row.addWidget(self.start)
        controls_row.addWidget(self.stop)
        controls_row.addWidget(self.Reset)
        controls_row.addSpacing(12)
        controls_row.addWidget(self.label_link_state)
        controls_row.addStretch(1)
        layout.addLayout(controls_row)

        top_boxes_row = QtWidgets.QHBoxLayout()
        top_boxes_row.setSpacing(6)

        modes_box = QtWidgets.QGroupBox("Modes")
        modes_box.setMaximumHeight(120)
        modes_layout = QtWidgets.QVBoxLayout(modes_box)
        modes_layout.setContentsMargins(6, 6, 6, 6)
        modes_layout.setSpacing(4)

        self.radioButton_freewheel = QtWidgets.QCheckBox("Freewheeling enabled")
        self.radioButton_freewheel.setChecked(True)
        self.checkBox_pumptrack = QtWidgets.QCheckBox("Pump track enabled")
        self.checkBox_pumptrack.setChecked(False)

        font_mode = QtGui.QFont()
        font_mode.setPointSize(10)
        self.radioButton_freewheel.setFont(font_mode)
        self.checkBox_pumptrack.setFont(font_mode)

        modes_layout.addWidget(self.radioButton_freewheel)
        modes_layout.addWidget(self.checkBox_pumptrack)

        flags_box = QtWidgets.QGroupBox("Status Flags")
        flags_box.setMaximumHeight(120)
        flags_layout = QtWidgets.QGridLayout(flags_box)
        flags_layout.setContentsMargins(6, 6, 6, 6)
        flags_layout.setHorizontalSpacing(10)
        flags_layout.setVerticalSpacing(4)

        self.flag_connected = ReadableFlag("Connected")
        self.flag_ctrl_active = ReadableFlag("Ctrl active")
        self.flag_forced_fw = ReadableFlag("Forced FW")
        self.flag_start = ReadableFlag("START")
        self.flag_index = ReadableFlag("INDEX_FOUND")
        self.flag_enable = ReadableFlag("Enable")

        flags_layout.addWidget(self.flag_connected, 0, 0)
        flags_layout.addWidget(self.flag_ctrl_active, 0, 1)
        flags_layout.addWidget(self.flag_forced_fw, 1, 0)
        flags_layout.addWidget(self.flag_start, 1, 1)
        flags_layout.addWidget(self.flag_index, 2, 0)
        flags_layout.addWidget(self.flag_enable, 2, 1)

        live_box = QtWidgets.QGroupBox("Live Values")
        live_box.setMaximumHeight(120)
        live_layout = QtWidgets.QVBoxLayout(live_box)
        live_layout.setContentsMargins(6, 4, 6, 4)
        live_layout.setSpacing(1)

        self.row_real_speed = LiveValueRow("Real Speed", "km/h")
        self.row_incline = LiveValueRow("Incline", "deg")
        self.row_gear_ratio = LiveValueRow("Gear Ratio", "-")
        self.row_power = LiveValueRow("Power", "W")

        live_layout.addWidget(self.row_real_speed)
        live_layout.addWidget(self.row_incline)
        live_layout.addWidget(self.row_gear_ratio)
        live_layout.addWidget(self.row_power)

        debug_box = QtWidgets.QGroupBox("Runtime Health")
        debug_box.setMaximumHeight(120)
        debug_layout = QtWidgets.QVBoxLayout(debug_box)
        debug_layout.setContentsMargins(6, 4, 6, 4)
        debug_layout.setSpacing(1)

        self.row_sample_age = LiveValueRow("Sample Age", "s")
        self.row_rx_count = LiveValueRow("RX Count", "-")
        self.row_read_errors = LiveValueRow("Read Err", "-")
        self.row_session = LiveValueRow("Session", "-")

        debug_layout.addWidget(self.row_sample_age)
        debug_layout.addWidget(self.row_rx_count)
        debug_layout.addWidget(self.row_read_errors)
        debug_layout.addWidget(self.row_session)

        top_boxes_row.addWidget(modes_box, 1)
        top_boxes_row.addWidget(flags_box, 2)
        top_boxes_row.addWidget(live_box, 1)
        top_boxes_row.addWidget(debug_box, 1)
        layout.addLayout(top_boxes_row)

        self.mainPlotWidget = pg.GraphicsLayoutWidget()
        self.mainPlotWidget.setBackground("k")
        pg.setConfigOptions(antialias=True)

        self.main_plot_torque = self.make_plot(self.mainPlotWidget, 0, 0, "Observer Torque")
        self.main_curve_torque = self.main_plot_torque.plot(
            pen=pg.mkPen('c', width=2), name='Pedal Torque'
        )

        self.main_plot_speed = self.make_plot(self.mainPlotWidget, 1, 0, "Real Speed")
        self.main_curve_speed = self.main_plot_speed.plot(
            pen=pg.mkPen('y', width=2), name='Real Speed'
        )

        self.main_plot_speed.setXLink(self.main_plot_torque)

        self.main_plot_torque.getAxis('bottom').setLabel("")
        self.main_plot_torque.getAxis('bottom').setStyle(showValues=False)

        layout.addWidget(self.mainPlotWidget, 1)

    def build_charts_tab_1(self):
        layout = QtWidgets.QVBoxLayout(self.tab_charts_1)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        toolbar = QtWidgets.QHBoxLayout()
        self.btn_c1_autorange = QtWidgets.QPushButton("Auto Range")
        self.btn_c1_zoomfit = QtWidgets.QPushButton("Zoom to Fit")
        self.btn_c1_zoomout_all = QtWidgets.QPushButton("Zoom Out All")
        self.btn_c1_reset_x = QtWidgets.QPushButton("Reset X")
        self.btn_c1_reset_y = QtWidgets.QPushButton("Reset Y")
        self.btn_c1_mouse = QtWidgets.QPushButton("Mouse Zoom/Pan: ON")
        self.btn_c1_mouse.setCheckable(True)
        self.btn_c1_mouse.setChecked(True)

        toolbar.addWidget(self.btn_c1_autorange)
        toolbar.addWidget(self.btn_c1_zoomfit)
        toolbar.addWidget(self.btn_c1_zoomout_all)
        toolbar.addWidget(self.btn_c1_reset_x)
        toolbar.addWidget(self.btn_c1_reset_y)
        toolbar.addWidget(self.btn_c1_mouse)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plotWidget1 = pg.GraphicsLayoutWidget()
        self.plotWidget1.setBackground('k')

        self.plot_currents = self.make_plot(self.plotWidget1, 0, 0, "Currents")
        self.curve_iq_filtered = self.plot_currents.plot(pen=pg.mkPen('g', width=1.2), name='IQ Filtered')
        self.curve_iq_set = self.plot_currents.plot(pen=pg.mkPen((255, 165, 0), width=1.2), name='IQ Set')

        self.plot_erpms = self.make_plot(self.plotWidget1, 0, 1, "RPMs")
        self.curve_rpm = self.plot_erpms.plot(pen=pg.mkPen('y', width=1.2), name='RPM Motor')
        self.curve_erpm_soll = self.plot_erpms.plot(pen=pg.mkPen('c', width=1.2), name='RPM Set')
        self.curve_leso_omega = self.plot_erpms.plot(pen=pg.mkPen('m', width=1.2), name='LESO RPM')

        self.plot_torques = self.make_plot(self.plotWidget1, 1, 0, "Torques")
        self.curve_tf = self.plot_torques.plot(pen=pg.mkPen('y', width=1.2), name='T_friction')
        self.curve_tp_obs = self.plot_torques.plot(pen=pg.mkPen('c', width=1.2), name='Tp_Observed')
        self.curve_tf_combine = self.plot_torques.plot(pen=pg.mkPen('m', width=1.2), name='T_F_combine')

        self.plot_errors = self.make_plot(self.plotWidget1, 1, 1, "Errors")
        self.curve_speed_error = self.plot_errors.plot(pen=pg.mkPen('r', width=1.2), name='Speed Error')
        self.curve_pos_term_speed = self.plot_errors.plot(pen=pg.mkPen('c', width=1.2), name='Pos Term Speed')

        self.chart_tabs_1 = [
            self.plot_currents, self.plot_erpms, self.plot_torques, self.plot_errors
        ]

        layout.addWidget(self.plotWidget1)

    def build_charts_tab_2(self):
        layout = QtWidgets.QVBoxLayout(self.tab_charts_2)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        toolbar = QtWidgets.QHBoxLayout()
        self.btn_c2_autorange = QtWidgets.QPushButton("Auto Range")
        self.btn_c2_zoomfit = QtWidgets.QPushButton("Zoom to Fit")
        self.btn_c2_zoomout_all = QtWidgets.QPushButton("Zoom Out All")
        self.btn_c2_reset_x = QtWidgets.QPushButton("Reset X")
        self.btn_c2_reset_y = QtWidgets.QPushButton("Reset Y")
        self.btn_c2_mouse = QtWidgets.QPushButton("Mouse Zoom/Pan: ON")
        self.btn_c2_mouse.setCheckable(True)
        self.btn_c2_mouse.setChecked(True)

        toolbar.addWidget(self.btn_c2_autorange)
        toolbar.addWidget(self.btn_c2_zoomfit)
        toolbar.addWidget(self.btn_c2_zoomout_all)
        toolbar.addWidget(self.btn_c2_reset_x)
        toolbar.addWidget(self.btn_c2_reset_y)
        toolbar.addWidget(self.btn_c2_mouse)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plotWidget2 = pg.GraphicsLayoutWidget()
        self.plotWidget2.setBackground('k')

        self.plot_speed_kmh = self.make_plot(self.plotWidget2, 0, 0, "Speed km/h")
        self.curve_model_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('y', width=1.2), name='Setpoint Speed')
        self.curve_real_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('c', width=1.2), name='Real Speed')

        self.plot_force_incline = self.make_plot(self.plotWidget2, 0, 1, "Force / Incline")
        self.curve_fcombine = self.plot_force_incline.plot(pen=pg.mkPen('m', width=1.2), name='F_combine')
        self.curve_incline = self.plot_force_incline.plot(pen=pg.mkPen('g', width=1.2), name='Incline')

        self.plot_uw = self.make_plot(self.plotWidget2, 1, 0, "UW Theta [rad]")
        self.curve_uw_angle_sp = self.plot_uw.plot(pen=pg.mkPen('c', width=1.2), name='SP')
        self.curve_uw_theta = self.plot_uw.plot(pen=pg.mkPen('y', width=1.2), name='Theta')

        self.plot_pos_error = self.make_plot(self.plotWidget2, 1, 1, "Position Error [deg]")
        self.curve_position_error = self.plot_pos_error.plot(pen=pg.mkPen('r', width=1.2), name='Err')

        self.chart_tabs_2 = [
            self.plot_speed_kmh, self.plot_force_incline, self.plot_uw, self.plot_pos_error
        ]

        layout.addWidget(self.plotWidget2)

    def build_params_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_params)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        button_row = QtWidgets.QHBoxLayout()
        self.btn_params_read_all = QtWidgets.QPushButton("Read All")
        self.btn_params_set_all = QtWidgets.QPushButton("Set All")
        self.btn_params_update_set_all = QtWidgets.QPushButton("Update Set All")

        button_row.addWidget(self.btn_params_read_all)
        button_row.addWidget(self.btn_params_set_all)
        button_row.addWidget(self.btn_params_update_set_all)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)

        content = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(content)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        cols = 4
        for n, (idx, name) in enumerate(PARAM_DEFS):
            box = ParamBox(idx, name)
            self.param_boxes[idx] = box

            r = n // cols
            c = n % cols
            grid.addWidget(box, r, c)

        scroll.setWidget(content)
        layout.addWidget(scroll)

    def build_telemetry_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_telemetry)
        layout.setContentsMargins(4, 4, 4, 4)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        self.telemetryTable = QtWidgets.QTableWidget()
        self.telemetryTable.setColumnCount(2)
        self.telemetryTable.setHorizontalHeaderLabels(["Variable", "Value"])
        self.telemetryTable.horizontalHeader().setStretchLastSection(True)
        self.telemetryTable.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.ResizeToContents
        )
        self.telemetryTable.verticalHeader().setVisible(False)
        self.telemetryTable.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.telemetryTable.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)

        self.debugText = QtWidgets.QPlainTextEdit()
        self.debugText.setReadOnly(True)
        self.debugText.setVisible(DEBUG)

        splitter.addWidget(self.telemetryTable)
        splitter.addWidget(self.debugText)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(splitter)

    def make_plot(self, parent_widget, row, col, title):
        axis_pen = pg.mkPen((180, 180, 180))
        text_pen = (220, 220, 220)

        plot = parent_widget.addPlot(row=row, col=col, title=title)
        plot.showGrid(x=True, y=True, alpha=0.20)
        plot.getAxis('left').setPen(axis_pen)
        plot.getAxis('bottom').setPen(axis_pen)
        plot.getAxis('left').setTextPen(text_pen)
        plot.getAxis('bottom').setTextPen(text_pen)
        plot.getAxis('bottom').setLabel("t", units="s")
        plot.addLegend(offset=(5, 5))
        plot.setMouseEnabled(x=True, y=True)
        plot.getViewBox().setMouseMode(plot.getViewBox().RectMode)
        return plot

    def make_connections(self):
        self.pushButton_refresh.clicked.connect(self.refresh_ports)
        self.pushButton_connect.clicked.connect(self.start_com)
        self.pushButton_disconnect.clicked.connect(self.stop_com)
        self.start.clicked.connect(self.start_prog)
        self.stop.clicked.connect(self.stop_prog)
        self.Reset.clicked.connect(self.reset_prog)
        self.pushButton_popup_charts.clicked.connect(self.open_popup_charts)

        self.radioButton_freewheel.clicked.connect(self.set_freewheel)
        self.checkBox_pumptrack.clicked.connect(self.set_pumptrack)

        self.btn_c1_autorange.clicked.connect(lambda: self.autorange_plots(self.chart_tabs_1))
        self.btn_c1_zoomfit.clicked.connect(lambda: self.zoom_fit_plots(self.chart_tabs_1, 1))
        self.btn_c1_zoomout_all.clicked.connect(lambda: self.zoom_out_all_plots(self.chart_tabs_1, 1))
        self.btn_c1_reset_x.clicked.connect(lambda: self.reset_plots_x(self.chart_tabs_1, 1))
        self.btn_c1_reset_y.clicked.connect(lambda: self.reset_plots_y(self.chart_tabs_1))
        self.btn_c1_mouse.clicked.connect(lambda: self.toggle_plot_mouse(self.chart_tabs_1, self.btn_c1_mouse))

        self.btn_c2_autorange.clicked.connect(lambda: self.autorange_plots(self.chart_tabs_2))
        self.btn_c2_zoomfit.clicked.connect(lambda: self.zoom_fit_plots(self.chart_tabs_2, 2))
        self.btn_c2_zoomout_all.clicked.connect(lambda: self.zoom_out_all_plots(self.chart_tabs_2, 2))
        self.btn_c2_reset_x.clicked.connect(lambda: self.reset_plots_x(self.chart_tabs_2, 2))
        self.btn_c2_reset_y.clicked.connect(lambda: self.reset_plots_y(self.chart_tabs_2))
        self.btn_c2_mouse.clicked.connect(lambda: self.toggle_plot_mouse(self.chart_tabs_2, self.btn_c2_mouse))

        self.btn_params_read_all.clicked.connect(self.read_all_parameters)
        self.btn_params_set_all.clicked.connect(self.set_all_parameters)
        self.btn_params_update_set_all.clicked.connect(self.update_set_all_parameters)

    def open_popup_charts(self):
        if self.popup_window is None:
            self.popup_window = PopupChartsWindow(self)
        self.popup_window.show()
        self.popup_window.raise_()
        self.popup_window.activateWindow()

    def set_freewheel(self):
        if self.radioButton_freewheel.isChecked():
            freewheel.set()
        else:
            freewheel.clear()

    def set_pumptrack(self):
        if self.checkBox_pumptrack.isChecked():
            pumptrack.set()
        else:
            pumptrack.clear()

    def refresh_ports(self):
        self.comboBox_portselect.clear()
        ports = list(serial.tools.list_ports.comports())

        if not ports:
            self.comboBox_portselect.addItem("No ports found", None)
            log_event("Port refresh: no ports found")
            return

        for port in ports:
            display_text = f"{port.device} - {port.description}"
            self.comboBox_portselect.addItem(display_text, port.device)

        log_event(f"Port refresh: found {len(ports)} port(s)")

    def start_com(self):
        global selected_port
        idx = self.comboBox_portselect.currentIndex()
        selected_port = self.comboBox_portselect.itemData(idx)

        if selected_port:
            set_diag(selected_port=selected_port)
            log_event(f"Connect requested on {selected_port}")
            vesc_com_flag.set()
            self.statusbar.showMessage(f"Connect requested to {selected_port}")
        else:
            self.statusbar.showMessage("No valid serial port selected")
            log_event("Connect requested but no valid port selected")

    def stop_com(self):
        vesc_com_flag.clear()
        set_diag(serial_open=False)
        log_event("Communication stopped by user")
        self.statusbar.showMessage("Communication stopped")

    def start_prog(self):
        prog_flag.set()
        log_event("Program started")
        self.statusbar.showMessage("Program started")

    def stop_prog(self):
        global control_value, control_mode
        control_value = 0.0
        control_mode = "Current"
        prog_flag.clear()
        log_event("Program stopped")
        self.statusbar.showMessage("Program stopped")

    def reset_prog(self):
        global control_value, control_mode, sample_counter
        if prog_flag.is_set():
            prog_flag.clear()

        control_value = 0.0
        control_mode = "Current"

        self.row_real_speed.set_value("0")
        self.row_incline.set_value("0")
        self.row_gear_ratio.set_value("0")
        self.row_power.set_value("0")

        self.row_sample_age.set_value("0")
        self.row_rx_count.set_value("0")
        self.row_read_errors.set_value("0")
        self.row_session.set_value("0")

        self.flag_connected.set_state(False)
        self.flag_ctrl_active.set_state(False)
        self.flag_forced_fw.set_state(False)
        self.flag_start.set_state(False)
        self.flag_index.set_state(False)
        self.flag_enable.set_state(False)

        with vesc_history_lock:
            vesc_history.clear()

        with vesc_values_lock:
            vesc_values.clear()

        with sample_counter_lock:
            sample_counter = 0

        self.telemetryTable.setRowCount(0)
        self.label_link_state.setText("LINK: RESET")
        self.debugText.clear()

        if self.popup_window is not None:
            for curve in [
                self.popup_window.curve_popup_iq_filtered,
                self.popup_window.curve_popup_iq_set,
                self.popup_window.curve_popup_rpm,
                self.popup_window.curve_popup_rpm_set,
                self.popup_window.curve_popup_leso_rpm,
                self.popup_window.curve_popup_tf,
                self.popup_window.curve_popup_tp_obs,
                self.popup_window.curve_popup_tf_combine,
                self.popup_window.curve_popup_speed_error,
                self.popup_window.curve_popup_pos_term_speed,
                self.popup_window.curve_popup_model_speed,
                self.popup_window.curve_popup_real_speed,
                self.popup_window.curve_popup_fcombine,
                self.popup_window.curve_popup_incline,
                self.popup_window.curve_popup_uw_angle_sp,
                self.popup_window.curve_popup_uw_theta,
                self.popup_window.curve_popup_position_error,
            ]:
                curve.setData([], [])

        log_event("Reset done")
        self.statusbar.showMessage("Reset done")

    def read_parameter(self, param_index):
        with vesc_values_lock:
            local_values = dict(vesc_values)

        box = self.param_boxes[param_index]
        current_idx = int(local_values.get("Param Index", -1))
        current_value = float(local_values.get("Param Value", 0.0))

        if current_idx == param_index:
            box.edit_actual.setText(f"{current_value:.6f}")
            self.statusbar.showMessage(f"Read parameter {param_index} from live telemetry")
        else:
            self.statusbar.showMessage(f"Param {param_index}: no direct read command implemented yet")

    def apply_parameter(self, param_index):
        box = self.param_boxes[param_index]
        target = box.edit_target.text().strip()

        if not target:
            return False

        try:
            float(target)
        except ValueError:
            return False

        dprint(f"[PARAM APPLY PLACEHOLDER] index={param_index}, value={target}")
        return True

    def read_all_parameters(self):
        for idx, _name in PARAM_DEFS:
            self.read_parameter(idx)
        self.statusbar.showMessage("Read All executed")

    def set_all_parameters(self):
        ok_count = 0
        for idx, _name in PARAM_DEFS:
            if self.apply_parameter(idx):
                ok_count += 1
        self.statusbar.showMessage(f"Set All executed ({ok_count} parameters queued)")

    def update_set_all_parameters(self):
        with vesc_values_lock:
            local_values = dict(vesc_values)

        current_idx = int(local_values.get("Param Index", -1))
        current_value = float(local_values.get("Param Value", 0.0))

        if current_idx in self.param_boxes:
            self.param_boxes[current_idx].edit_target.setText(f"{current_value:.6f}")
            self.statusbar.showMessage(f"Updated set field for parameter {current_idx}")
        else:
            self.statusbar.showMessage("Update Set All: no matching live parameter index")

    def update_telemetry_table(self, values_dict):
        items = list(values_dict.items())
        self.telemetryTable.setRowCount(len(items))
        for row, (key, value) in enumerate(items):
            key_item = QtWidgets.QTableWidgetItem(str(key))
            if key == "Status Bits Ext" and isinstance(value, (int, float)):
                value_str = f"{int(value)} (0b{int(value):032b})"
            else:
                value_str = f"{value:.6f}" if isinstance(value, float) else str(value)
            val_item = QtWidgets.QTableWidgetItem(value_str)
            self.telemetryTable.setItem(row, 0, key_item)
            self.telemetryTable.setItem(row, 1, val_item)

    def update_debug_view(self):
        if not DEBUG:
            self.debugText.clear()
            return

        d = get_diag_snapshot()

        now = time.perf_counter()
        last_rx = d.get("last_rx_time", 0.0)
        sample_age = (now - last_rx) if last_rx > 0.0 else float("inf")

        lines = [
            "=== Diagnostics ===",
            f"comm_thread_alive       = {d.get('comm_thread_alive')}",
            f"serial_open             = {d.get('serial_open')}",
            f"selected_port           = {d.get('selected_port')}",
            f"serial_session_id       = {d.get('serial_session_id')}",
            f"sample_age_s            = {sample_age:.3f}",
            f"rx_count                = {d.get('rx_count')}",
            f"tx_count                = {d.get('tx_count')}",
            f"read_errors             = {d.get('read_errors')}",
            f"write_errors            = {d.get('write_errors')}",
            f"build_errors            = {d.get('build_errors')}",
            f"plot_errors             = {d.get('plot_errors')}",
            f"gui_errors              = {d.get('gui_errors')}",
            f"consecutive_read_errors = {d.get('consecutive_read_errors')}",
            f"consecutive_write_errors= {d.get('consecutive_write_errors')}",
            f"read_only_mode          = {d.get('read_only_mode')}",
            f"commands_enabled        = {d.get('commands_enabled')}",
            "",
            "=== Last errors ===",
            f"last_read_error   = {d.get('last_read_error')}",
            f"last_write_error  = {d.get('last_write_error')}",
            f"last_build_error  = {d.get('last_build_error')}",
            f"last_plot_error   = {d.get('last_plot_error')}",
            f"last_gui_error    = {d.get('last_gui_error')}",
            f"last_comm_error   = {d.get('last_comm_error')}",
            "",
            "=== Last response ===",
            f"{d.get('last_response_summary')}",
            "",
            "=== Recent events ===",
        ]
        lines.extend(d.get("event_log", []))
        self.debugText.setPlainText("\n".join(lines))

    def autorange_plots(self, plots):
        for p in plots:
            p.enableAutoRange(axis='xy', enable=True)
            p.autoRange()

    def reset_plots_x(self, plots, group_id):
        if group_id == 1:
            self.charts1_scroll_mode = True
        else:
            self.charts2_scroll_mode = True

    def reset_plots_y(self, plots):
        for p in plots:
            p.enableAutoRange(axis='y', enable=True)
            p.autoRange()

    def zoom_fit_plots(self, plots, group_id):
        if group_id == 1:
            self.charts1_scroll_mode = False
        else:
            self.charts2_scroll_mode = False
        for p in plots:
            p.enableAutoRange(axis='xy', enable=True)
            p.autoRange()

    def zoom_out_all_plots(self, plots, group_id):
        if group_id == 1:
            self.charts1_scroll_mode = False
        else:
            self.charts2_scroll_mode = False

        with vesc_history_lock:
            if "time_s" not in vesc_history or len(vesc_history["time_s"]) == 0:
                return
            t = vesc_history["time_s"]
            t0 = t[0]
            x0 = 0.0
            x1 = max(1.0, t[-1] - t0)

        for p in plots:
            p.setXRange(x0, x1, padding=0.02)
            p.enableAutoRange(axis='y', enable=True)
            p.autoRange()

    def toggle_plot_mouse(self, plots, button):
        enabled = button.isChecked()
        for p in plots:
            p.setMouseEnabled(x=enabled, y=enabled)
        button.setText(f"Mouse Zoom/Pan: {'ON' if enabled else 'OFF'}")

    def refresh_live_data(self):
        global vesc_values

        try:
            with vesc_values_lock:
                local_values = dict(vesc_values)

            d = get_diag_snapshot()
            now = time.perf_counter()
            last_rx = d.get("last_rx_time", 0.0)
            sample_age = (now - last_rx) if last_rx > 0.0 else float("inf")
            set_diag(last_sample_age_s=sample_age)

            requested = vesc_com_flag.is_set()
            serial_open = d.get("serial_open", False)
            truly_connected = requested and serial_open and (sample_age < STALE_WARNING_S)

            self.flag_connected.set_state(truly_connected)

            if not requested:
                self.label_link_state.setText("LINK: IDLE")
            elif serial_open and sample_age < STALE_WARNING_S:
                self.label_link_state.setText(f"LINK: OK ({sample_age*1000.0:.0f} ms)")
            elif serial_open:
                self.label_link_state.setText(f"LINK: STALE ({sample_age:.2f} s)")
            else:
                self.label_link_state.setText("LINK: RECONNECTING")

            self.row_sample_age.set_value(f"{sample_age:.3f}" if math.isfinite(sample_age) else "inf")
            self.row_rx_count.set_value(str(d.get("rx_count", 0)))
            self.row_read_errors.set_value(str(d.get("read_errors", 0)))
            self.row_session.set_value(str(d.get("serial_session_id", 0)))

            if sample_age >= STALE_WARNING_S and requested:
                self.statusbar.showMessage(f"Telemetry stale: {sample_age:.2f} s since last packet")

            if sample_age >= STALE_TIMEOUT_S:
                self.flag_ctrl_active.set_state(False)

            if not local_values:
                self.flag_ctrl_active.set_state(False)
                self.flag_forced_fw.set_state(False)
                self.flag_start.set_state(False)
                self.flag_index.set_state(False)
                self.flag_enable.set_state(False)
                self.update_debug_view()
                return

            self.row_real_speed.set_value(f"{local_values.get('Real Speed km/h', 0):.2f}")
            self.row_incline.set_value(f"{local_values.get('Incline Deg Ist', 0):.2f}")
            self.row_gear_ratio.set_value(f"{local_values.get('Gear Ratio', 0):.3f}")
            self.row_power.set_value(f"{local_values.get('Power In', 0):.1f}")

            self.flag_ctrl_active.set_state(bool(local_values.get("Ctrl Active", 0)))
            self.flag_forced_fw.set_state(bool(local_values.get("Forced FW", 0)))
            self.flag_start.set_state(bool(local_values.get("START", 0)))
            self.flag_index.set_state(bool(local_values.get("INDEX_FOUND", 0)))
            self.flag_enable.set_state(bool(local_values.get("ENABLE", 0)))

            try:
                p_idx = int(local_values.get("Param Index", -1))
                p_val = float(local_values.get("Param Value", 0.0))
                if p_idx in self.param_boxes:
                    self.param_boxes[p_idx].edit_actual.setText(f"{p_val:.6f}")
            except Exception:
                pass

            self.update_telemetry_table(local_values)
            self.refresh_plots()
            self.update_debug_view()

        except Exception:
            err = traceback.format_exc()
            inc_diag("gui_errors")
            set_diag(last_gui_error=err)
            log_event("GUI refresh error")
            dprint(err)

    def safe_set_curve(self, curve, x, y, name):
        try:
            if len(x) != len(y):
                raise ValueError(f"{name}: len(x)={len(x)} != len(y)={len(y)}")
            curve.setData(x, y)
        except Exception:
            err = traceback.format_exc()
            inc_diag("plot_errors")
            set_diag(last_plot_error=err)
            log_event(f"Plot update failed for {name}")
            dprint(err)

    def refresh_plots(self):
        with vesc_history_lock:
            if "time_s" not in vesc_history or len(vesc_history["time_s"]) == 0:
                return
            hist_copy = {k: v[:] for k, v in vesc_history.items()}

        t = hist_copy.get("time_s", [])
        if not t:
            return

        x_full = [ti - t[0] for ti in t]
        x_last = x_full[-1]

        def aligned_xy(key, visible_only=False):
            y_full = hist_copy.get(key, [])
            if not y_full:
                return [], []

            n = min(len(x_full), len(y_full))
            if n <= 0:
                return [], []

            x = x_full[:n]
            y = y_full[:n]

            if visible_only:
                x_min = max(0.0, x_last - PLOT_SCROLL_WINDOW_S)

                lo = 0
                hi = len(x)
                while lo < hi:
                    mid = (lo + hi) // 2
                    if x[mid] < x_min:
                        lo = mid + 1
                    else:
                        hi = mid

                x = x[lo:]
                y = y[lo:]

            return x, y

        x, y = aligned_xy("Pedal Torque Observed", visible_only=True)
        if x and y:
            self.safe_set_curve(self.main_curve_torque, x, y, "main_curve_torque")

        x, y = aligned_xy("Real Speed km/h", visible_only=True)
        if x and y:
            self.safe_set_curve(self.main_curve_speed, x, y, "main_curve_speed")

        self.main_plot_torque.setXRange(
            max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
            max(PLOT_SCROLL_WINDOW_S, x_last),
            padding=0.0
        )

        vis1 = self.charts1_scroll_mode

        for key, curve, name in [
            ("IQ Filtered", self.curve_iq_filtered, "curve_iq_filtered"),
            ("IQ Set", self.curve_iq_set, "curve_iq_set"),
            ("RPM Motor", self.curve_rpm, "curve_rpm"),
            ("RPM Set", self.curve_erpm_soll, "curve_erpm_soll"),
            ("LESO RPM", self.curve_leso_omega, "curve_leso_omega"),
            ("T_friction", self.curve_tf, "curve_tf"),
            ("Pedal Torque Observed", self.curve_tp_obs, "curve_tp_obs"),
            ("T_F_combine", self.curve_tf_combine, "curve_tf_combine"),
            ("Speed Error", self.curve_speed_error, "curve_speed_error"),
            ("Pos Term Speed", self.curve_pos_term_speed, "curve_pos_term_speed"),
        ]:
            x, y = aligned_xy(key, visible_only=vis1)
            if x and y:
                self.safe_set_curve(curve, x, y, name)

        if vis1:
            for p in self.chart_tabs_1:
                p.setXRange(
                    max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
                    max(PLOT_SCROLL_WINDOW_S, x_last),
                    padding=0.0
                )

        vis2 = self.charts2_scroll_mode

        for key, curve, name in [
            ("Setpoint Speed km/h", self.curve_model_speed, "curve_model_speed"),
            ("Real Speed km/h", self.curve_real_speed, "curve_real_speed"),
            ("F_combine", self.curve_fcombine, "curve_fcombine"),
            ("Incline Deg Ist", self.curve_incline, "curve_incline"),
            ("UW Angle SP", self.curve_uw_angle_sp, "curve_uw_angle_sp"),
            ("UW Theta", self.curve_uw_theta, "curve_uw_theta"),
            ("Position Error deg", self.curve_position_error, "curve_position_error"),
        ]:
            x, y = aligned_xy(key, visible_only=vis2)
            if x and y:
                self.safe_set_curve(curve, x, y, name)

        if vis2:
            for p in self.chart_tabs_2:
                p.setXRange(
                    max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
                    max(PLOT_SCROLL_WINDOW_S, x_last),
                    padding=0.0
                )

        if self.popup_window is not None and self.popup_window.isVisible():
            visp = self.popup_window.popup_scroll_mode

            for key, curve, name in [
                ("IQ Filtered", self.popup_window.curve_popup_iq_filtered, "curve_popup_iq_filtered"),
                ("IQ Set", self.popup_window.curve_popup_iq_set, "curve_popup_iq_set"),
                ("RPM Motor", self.popup_window.curve_popup_rpm, "curve_popup_rpm"),
                ("RPM Set", self.popup_window.curve_popup_rpm_set, "curve_popup_rpm_set"),
                ("LESO RPM", self.popup_window.curve_popup_leso_rpm, "curve_popup_leso_rpm"),
                ("T_friction", self.popup_window.curve_popup_tf, "curve_popup_tf"),
                ("Pedal Torque Observed", self.popup_window.curve_popup_tp_obs, "curve_popup_tp_obs"),
                ("T_F_combine", self.popup_window.curve_popup_tf_combine, "curve_popup_tf_combine"),
                ("Speed Error", self.popup_window.curve_popup_speed_error, "curve_popup_speed_error"),
                ("Pos Term Speed", self.popup_window.curve_popup_pos_term_speed, "curve_popup_pos_term_speed"),
                ("Setpoint Speed km/h", self.popup_window.curve_popup_model_speed, "curve_popup_model_speed"),
                ("Real Speed km/h", self.popup_window.curve_popup_real_speed, "curve_popup_real_speed"),
                ("F_combine", self.popup_window.curve_popup_fcombine, "curve_popup_fcombine"),
                ("Incline Deg Ist", self.popup_window.curve_popup_incline, "curve_popup_incline"),
                ("UW Angle SP", self.popup_window.curve_popup_uw_angle_sp, "curve_popup_uw_angle_sp"),
                ("UW Theta", self.popup_window.curve_popup_uw_theta, "curve_popup_uw_theta"),
                ("Position Error deg", self.popup_window.curve_popup_position_error, "curve_popup_position_error"),
            ]:
                x, y = aligned_xy(key, visible_only=visp)
                if x and y:
                    self.safe_set_curve(curve, x, y, name)

            if visp:
                for p in self.popup_window.popup_plots:
                    p.setXRange(
                        max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
                        max(PLOT_SCROLL_WINDOW_S, x_last),
                        padding=0.0
                    )


# =========================
# Communication helpers
# =========================
def snapshot_command():
    return (control_mode, float(control_value))


def command_changed(prev_cmd, new_cmd):
    if prev_cmd is None:
        return True

    prev_mode, prev_val = prev_cmd
    new_mode, new_val = new_cmd

    if prev_mode != new_mode:
        return True

    if new_mode == "Duty Cycle":
        return abs(new_val - prev_val) >= DUTY_EPS
    elif new_mode == "Current":
        return abs(new_val - prev_val) >= CURRENT_EPS
    elif new_mode == "Position":
        return abs(new_val - prev_val) >= SERVO_EPS
    elif new_mode == "Speed":
        return abs(new_val - prev_val) >= SPEED_EPS
    else:
        return abs(new_val - prev_val) >= CURRENT_EPS


def send_command(vesc, mode, value):
    if mode == "Duty Cycle":
        vesc.set_duty_cycle(value)
    elif mode == "Current":
        vesc.set_current(value)
    elif mode == "Position":
        vesc.set_servo(value)
    elif mode == "Speed":
        vesc.set_rpm(1000)
    else:
        vesc.set_current(0.0)


# =========================
# Threads
# =========================
def vesc_communication():
    global vesc_values, selected_port, control_value, control_mode

    log_event("Comm thread started")

    while True:
        set_diag(
            comm_thread_alive=True,
            read_only_mode=READ_ONLY_MODE,
            commands_enabled=ENABLE_COMMANDS,
        )

        if not (vesc_com_flag.is_set() and selected_port):
            set_diag(serial_open=False)
            time.sleep(0.1)
            continue

        port = selected_port
        session_id = int(time.time() * 1000) % 100000000
        set_diag(serial_session_id=session_id, selected_port=port)

        try:
            log_event(f"Opening VESC on {port} (session {session_id})")
            debug_list_ports()

            with VESC(serial_port=port, start_heartbeat=False, baudrate=1200, timeout=0.25) as vesc:
                set_diag(serial_open=True)
                log_event(f"Serial open OK on {port} (session {session_id})")

                time.sleep(0.2)
                try:
                    vesc.recover_from_timeout()
                    log_event("Initial RX/TX buffer reset after open")
                except Exception as e:
                    log_event(f"Initial recovery after open failed: {type(e).__name__}: {e}")

                consecutive_timeouts = 0
                last_cmd_sent = None
                last_command_tx_time = 0.0
                next_telem_time = time.perf_counter()

                while vesc_com_flag.is_set() and selected_port == port:
                    now = time.perf_counter()
                    set_diag(last_loop_time=now)

                    if prog_flag.is_set():
                        control_mode = "Speed"
                        control_value = 0.0
                    else:
                        control_mode = "Current"
                        control_value = 0.0

                    if now < next_telem_time:
                        time.sleep(min(0.001, next_telem_time - now))
                        continue

                    cycle_start = time.perf_counter()

                    try:
                        log_event("TX: telemetry request")
                        response = vesc.get_measurements_exp()

                        new_values = build_vesc_values(response)

                        with vesc_values_lock:
                            vesc_values = new_values

                        append_history(new_values)

                        rx_now = time.perf_counter()
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
                        with diag_lock:
                            diag["consecutive_read_errors"] += 1

                        consecutive_timeouts += 1
                        log_event(f"READ TIMEOUT on {port}: {err} ({consecutive_timeouts})")

                        try:
                            vesc.recover_from_timeout()
                            log_event("Soft recovery: RX/TX buffers reset")
                        except Exception as rec_e:
                            log_event(f"Soft recovery failed: {type(rec_e).__name__}: {rec_e}")

                        if consecutive_timeouts >= HARD_TIMEOUT_LIMIT:
                            log_event("Too many timeouts, forcing reopen")
                            raise

                        next_telem_time = time.perf_counter() + TELEMETRY_PERIOD_S
                        time.sleep(0.05)
                        continue

                    except Exception as e:
                        err = f"{type(e).__name__}: {e}"
                        inc_diag("read_errors")
                        set_diag(
                            last_read_error=err,
                            last_comm_error=err,
                            serial_open=False,
                        )
                        with diag_lock:
                            diag["consecutive_read_errors"] += 1

                        log_event(f"READ ERROR on {port}: {err}")
                        raise

                    time.sleep(TX_GUARD_S)

                    if not READ_ONLY_MODE and ENABLE_COMMANDS:
                        cmd_now = snapshot_command()

                        should_send = False
                        if command_changed(last_cmd_sent, cmd_now):
                            should_send = True
                        elif (time.perf_counter() - last_command_tx_time) >= COMMAND_KEEPALIVE_S:
                            should_send = True

                        if should_send:
                            try:
                                time.sleep(TX_GUARD_S * 2.0)
                                log_event(f"TX: command mode={cmd_now[0]} value={cmd_now[1]:.3f}")
                                send_command(vesc, cmd_now[0], cmd_now[1])
                                time.sleep(TX_GUARD_S * 2.0)

                                tx_now = time.perf_counter()
                                with diag_lock:
                                    diag["last_tx_time"] = tx_now
                                    diag["tx_count"] += 1
                                    diag["consecutive_write_errors"] = 0

                                last_command_tx_time = tx_now
                                last_cmd_sent = cmd_now

                            except Exception as e:
                                err = f"{type(e).__name__}: {e}"
                                inc_diag("write_errors")
                                set_diag(
                                    last_write_error=err,
                                    last_comm_error=err,
                                    serial_open=False,
                                )
                                with diag_lock:
                                    diag["consecutive_write_errors"] += 1

                                log_event(f"WRITE ERROR on {port}: {err}")
                                raise

                        time.sleep(TX_GUARD_S)

                    next_telem_time = cycle_start + TELEMETRY_PERIOD_S

        except Exception:
            err = traceback.format_exc()
            set_diag(
                serial_open=False,
                last_comm_error=err,
            )
            log_event(f"Communication session failed on {port}")
            dprint(err)
            time.sleep(REOPEN_BACKOFF_S)

        finally:
            set_diag(serial_open=False)


# =========================
# Main
# =========================
if __name__ == "__main__":
    communication_thread = threading.Thread(target=vesc_communication, daemon=True)
    communication_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())