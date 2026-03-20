# -*- coding: utf-8 -*-

import sys
import time
import threading
import serial
import serial.tools.list_ports

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from pyvesc.VESC import VESC


# =========================
# Global state
# =========================
vesc_com_flag = threading.Event()
prog_flag = threading.Event()
freewheel = threading.Event()
freewheel.set()

pumptrack = threading.Event()
pumptrack.clear()

vesc_values = {}
vesc_values_lock = threading.Lock()

# Full run history is stored here
vesc_history = {}
vesc_history_lock = threading.Lock()

selected_port = None
control_value = 0.0
control_mode = "Current"

WHEEL_CIRCUMFERENCE_M = 2.105
POLE_PAIRS = 23.0

PLOT_SCROLL_WINDOW_S = 20.0


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
def estimate_real_speed_kmh(rpm, gear_ratio):
    try:
        rpm = float(rpm) / POLE_PAIRS
        gear_ratio = float(gear_ratio)
        if abs(gear_ratio) < 1e-9:
            wheel_rpm = rpm
        else:
            wheel_rpm = rpm / gear_ratio
        return wheel_rpm * WHEEL_CIRCUMFERENCE_M * 60.0 / 1000.0
    except Exception:
        return 0.0


def build_vesc_values(response):

    v_in = getattr(response, "v_in", 0.0)
    avg_input_current = getattr(response, "avg_input_current", 0.0)
    duty_cycle_now = getattr(response, "duty_cycle_now", 0.0)
    avg_motor_current = getattr(response, "avg_motor_current", 0.0)
    status_bits_ext = int(getattr(response, "status_bits_ext", 0)) & 0xFFFFFFFF

    rpm = getattr(response, "rpm", 0.0)
    erpm_soll = getattr(response, "erpm_soll", 0.0)
    gear_ratio = getattr(response, "gear_ratio", 0.0)
    model_speed = getattr(response, "model_speed", 0.0)
    uw_theta = getattr(response, "uw_theta", 0.0)
    uw_angle_sp = getattr(response, "uw_angle_sp", 0.0)

    position_error = uw_angle_sp - uw_theta
    real_speed_kmh = estimate_real_speed_kmh(rpm, gear_ratio)

    return {
        "RPM": rpm,
        "ERPM Soll": erpm_soll,
        "Motor Current": avg_motor_current,
        "Battery Current": avg_input_current,
        "IQ Filtered": getattr(response, "avg_iq", 0.0),
        "IQ Instant": getattr(response, "iq_current", 0.0),
        "IQ Set": getattr(response, "iq_set", 0.0),
        "Input Voltage": v_in,
        "Duty Cycle": duty_cycle_now,

        "Tf": getattr(response, "tf", 0.0),
        "Gear Ratio": gear_ratio,
        "Model Speed": model_speed,
        "Real Speed km/h": real_speed_kmh,
        "F Combine": getattr(response, "f_combine", 0.0),
        "UW Theta": uw_theta,
        "LESO Omega": getattr(response, "leso_omega", 0.0),
        "Pedal Torque Observed": getattr(response, "tp_observed", 0.0),
        "Param Index": getattr(response, "param_index", 0),
        "Param Value": getattr(response, "param_from_index", 0.0),
        "Pos Term Speed": getattr(response, "pos_term_speed", 0.0),
        "Speed Error": getattr(response, "speed_error", 0.0),
        "Tf Combine": getattr(response, "t_f_combine", 0.0),
        "Incline Deg Ist": getattr(response, "incline_deg_ist", 0.0),
        "UW Angle SP": uw_angle_sp,
        "Position Error": position_error,

        "Ctrl Active": int(bool(status_bits_ext & (1 << 0))),
        "Forced FW": int(bool(status_bits_ext & (1 << 1))),
        "START": int(bool(status_bits_ext & (1 << 2))),
        "INDEX_FOUND": int(bool(status_bits_ext & (1 << 3))),
        "ENABLE": int(bool(status_bits_ext & (1 << 4))),

        "Status Bits Ext": status_bits_ext,
        "Power In": v_in * avg_input_current,
    }


def append_history(values_dict):
    now = time.perf_counter()
    with vesc_history_lock:
        if "time_s" not in vesc_history:
            vesc_history["time_s"] = []
        vesc_history["time_s"].append(now)

        for key, value in values_dict.items():
            if isinstance(value, (int, float)):
                if key not in vesc_history:
                    vesc_history[key] = []
                vesc_history[key].append(float(value))


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


# =========================
# Main GUI
# =========================
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 480)
        MainWindow.setMinimumSize(800, 480)
        MainWindow.setMaximumSize(1920, 1080)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)

        self.param_boxes = {}
        self.chart_tabs_1 = []
        self.chart_tabs_2 = []

        # whether charts should keep scrolling or show full-fit/manual zoom view
        self.charts1_scroll_mode = True
        self.charts2_scroll_mode = True

        self.build_ui(MainWindow)
        self.make_connections()
        self.refresh_ports()

        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.refresh_live_data)
        self.gui_timer.start(100)

    def build_ui(self, MainWindow):
        root = QtWidgets.QVBoxLayout(self.centralwidget)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        topbar = QtWidgets.QHBoxLayout()
        self.comboBox_portselect = QtWidgets.QComboBox()
        self.comboBox_portselect.setMinimumWidth(220)
        self.pushButton_refresh = QtWidgets.QPushButton("Refresh Ports")
        self.pushButton_refresh.setMaximumWidth(110)

        topbar.addWidget(QtWidgets.QLabel("Serial Port:"))
        topbar.addWidget(self.comboBox_portselect)
        topbar.addWidget(self.pushButton_refresh)
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
        self.tabs.addTab(self.tab_telemetry, "Telemetry")
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

        controls_row.addWidget(self.pushButton_connect)
        controls_row.addWidget(self.pushButton_disconnect)
        controls_row.addWidget(self.start)
        controls_row.addWidget(self.stop)
        controls_row.addWidget(self.Reset)
        controls_row.addStretch(1)
        layout.addLayout(controls_row)

        top_boxes_row = QtWidgets.QHBoxLayout()
        top_boxes_row.setSpacing(6)

        modes_box = QtWidgets.QGroupBox("Modes")
        modes_box.setMaximumHeight(106)
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
        flags_box.setMaximumHeight(106)
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
        live_box.setMaximumHeight(106)
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

        top_boxes_row.addWidget(modes_box, 1)
        top_boxes_row.addWidget(flags_box, 2)
        top_boxes_row.addWidget(live_box, 1)
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

        # remove bottom x label and numbers from top chart
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
        self.curve_motor_current = self.plot_currents.plot(pen=pg.mkPen('y', width=1.2), name='Motor')
        self.curve_iq_filtered = self.plot_currents.plot(pen=pg.mkPen('g', width=1.2), name='IQ F')
        self.curve_iq_instant = self.plot_currents.plot(pen=pg.mkPen('r', width=1.2), name='IQ I')
        self.curve_iq_set = self.plot_currents.plot(pen=pg.mkPen((255, 165, 0), width=1.2), name='IQ Set')

        self.plot_erpms = self.make_plot(self.plotWidget1, 0, 1, "ERPMs")
        self.curve_rpm = self.plot_erpms.plot(pen=pg.mkPen('y', width=1.2), name='RPM')
        self.curve_erpm_soll = self.plot_erpms.plot(pen=pg.mkPen('c', width=1.2), name='ERPM Soll')
        self.curve_leso_omega = self.plot_erpms.plot(pen=pg.mkPen('m', width=1.2), name='LESO')

        self.plot_torques = self.make_plot(self.plotWidget1, 1, 0, "Torques")
        self.curve_tf = self.plot_torques.plot(pen=pg.mkPen('y', width=1.2), name='Tf')
        self.curve_tp_obs = self.plot_torques.plot(pen=pg.mkPen('c', width=1.2), name='Tp Obs')
        self.curve_tf_combine = self.plot_torques.plot(pen=pg.mkPen('m', width=1.2), name='Tf Comb')

        self.plot_errors = self.make_plot(self.plotWidget1, 1, 1, "Errors")
        self.curve_speed_error = self.plot_errors.plot(pen=pg.mkPen('r', width=1.2), name='Speed Err')
        self.curve_pos_term_speed = self.plot_errors.plot(pen=pg.mkPen('c', width=1.2), name='Pos Term')

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
        self.curve_model_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('y', width=1.2), name='Setpoint')
        self.curve_real_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('c', width=1.2), name='Real')

        self.plot_force_incline = self.make_plot(self.plotWidget2, 0, 1, "Force / Incline")
        self.curve_fcombine = self.plot_force_incline.plot(pen=pg.mkPen('m', width=1.2), name='F Comb')
        self.curve_incline = self.plot_force_incline.plot(pen=pg.mkPen('g', width=1.2), name='Incline')

        self.plot_uw = self.make_plot(self.plotWidget2, 1, 0, "UW Theta")
        self.curve_uw_angle_sp = self.plot_uw.plot(pen=pg.mkPen('c', width=1.2), name='SP')
        self.curve_uw_theta = self.plot_uw.plot(pen=pg.mkPen('y', width=1.2), name='Theta')

        self.plot_pos_error = self.make_plot(self.plotWidget2, 1, 1, "Position Error")
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
        layout.addWidget(self.telemetryTable)

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
            return

        for port in ports:
            display_text = f"{port.device} - {port.description}"
            self.comboBox_portselect.addItem(display_text, port.device)

    def start_com(self):
        global selected_port
        idx = self.comboBox_portselect.currentIndex()
        selected_port = self.comboBox_portselect.itemData(idx)

        if selected_port:
            print(f"Connecting to {selected_port}")
            vesc_com_flag.set()
            self.statusbar.showMessage(f"Connected request to {selected_port}")
        else:
            self.statusbar.showMessage("No valid serial port selected")

    def stop_com(self):
        vesc_com_flag.clear()
        self.statusbar.showMessage("Communication stopped")

    def start_prog(self):
        prog_flag.set()
        self.statusbar.showMessage("Program started")

    def stop_prog(self):
        global control_value, control_mode
        control_value = 0.0
        control_mode = "Current"
        prog_flag.clear()
        self.statusbar.showMessage("Program stopped")

    def reset_prog(self):
        global control_value, control_mode
        if prog_flag.is_set():
            prog_flag.clear()

        control_value = 0.0
        control_mode = "Current"

        self.row_real_speed.set_value("0")
        self.row_incline.set_value("0")
        self.row_gear_ratio.set_value("0")
        self.row_power.set_value("0")

        self.flag_connected.set_state(False)
        self.flag_ctrl_active.set_state(False)
        self.flag_forced_fw.set_state(False)
        self.flag_start.set_state(False)
        self.flag_index.set_state(False)
        self.flag_enable.set_state(False)

        with vesc_history_lock:
            vesc_history.clear()

        self.telemetryTable.setRowCount(0)
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

        print(f"[PARAM APPLY PLACEHOLDER] index={param_index}, value={target}")
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
            value_str = f"{value:.6f}" if isinstance(value, float) else str(value)
            val_item = QtWidgets.QTableWidgetItem(value_str)
            self.telemetryTable.setItem(row, 0, key_item)
            self.telemetryTable.setItem(row, 1, val_item)

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

        with vesc_values_lock:
            local_values = dict(vesc_values)

        self.flag_connected.set_state(vesc_com_flag.is_set())

        if not local_values:
            self.flag_ctrl_active.set_state(False)
            self.flag_forced_fw.set_state(False)
            self.flag_start.set_state(False)
            self.flag_index.set_state(False)
            self.flag_enable.set_state(False)
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

    def _last_window_index(self, x, window_s):
        if not x:
            return 0
        x_last = x[-1]
        x_min = max(0.0, x_last - window_s)
        lo = 0
        hi = len(x)
        while lo < hi:
            mid = (lo + hi) // 2
            if x[mid] < x_min:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def refresh_plots(self):
        with vesc_history_lock:
            if "time_s" not in vesc_history or len(vesc_history["time_s"]) == 0:
                return

            # copy only what we need for plotting
            t = list(vesc_history["time_s"])

            x_full = [ti - t[0] for ti in t]
            x_last = x_full[-1]

            def get_visible(key):
                vals = vesc_history.get(key, [])
                if not vals:
                    return [], []

                x_min = max(0.0, x_last - PLOT_SCROLL_WINDOW_S)

                i0 = 0
                lo = 0
                hi = len(x_full)
                while lo < hi:
                    mid = (lo + hi) // 2
                    if x_full[mid] < x_min:
                        lo = mid + 1
                    else:
                        hi = mid
                i0 = lo

                return x_full[i0:], list(vals[i0:])

            def get_full(key):
                vals = vesc_history.get(key, [])
                return x_full, list(vals) if vals else []

        # Main tab: always scrolling recent window
        x_vis, y_vis = get_visible("Pedal Torque Observed")
        self.main_curve_torque.setData(x_vis, y_vis)

        x_vis, y_vis = get_visible("Real Speed km/h")
        self.main_curve_speed.setData(x_vis, y_vis)

        self.main_plot_torque.setXRange(
            max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
            max(PLOT_SCROLL_WINDOW_S, x_last),
            padding=0.0
        )

        # Charts 1
        if self.charts1_scroll_mode:
            x, y = get_visible("Motor Current")
            self.curve_motor_current.setData(x, y)
            x, y = get_visible("IQ Filtered")
            self.curve_iq_filtered.setData(x, y)
            x, y = get_visible("IQ Instant")
            self.curve_iq_instant.setData(x, y)
            x, y = get_visible("IQ Set")
            self.curve_iq_set.setData(x, y)

            x, y = get_visible("RPM")
            self.curve_rpm.setData(x, y)
            x, y = get_visible("ERPM Soll")
            self.curve_erpm_soll.setData(x, y)
            x, y = get_visible("LESO Omega")
            self.curve_leso_omega.setData(x, y)

            x, y = get_visible("Tf")
            self.curve_tf.setData(x, y)
            x, y = get_visible("Pedal Torque Observed")
            self.curve_tp_obs.setData(x, y)
            x, y = get_visible("Tf Combine")
            self.curve_tf_combine.setData(x, y)

            x, y = get_visible("Speed Error")
            self.curve_speed_error.setData(x, y)
            x, y = get_visible("Pos Term Speed")
            self.curve_pos_term_speed.setData(x, y)

            for p in self.chart_tabs_1:
                p.setXRange(
                    max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
                    max(PLOT_SCROLL_WINDOW_S, x_last),
                    padding=0.0
                )
        else:
            x, y = get_full("Motor Current")
            self.curve_motor_current.setData(x, y)
            x, y = get_full("IQ Filtered")
            self.curve_iq_filtered.setData(x, y)
            x, y = get_full("IQ Instant")
            self.curve_iq_instant.setData(x, y)
            x, y = get_full("IQ Set")
            self.curve_iq_set.setData(x, y)

            x, y = get_full("RPM")
            self.curve_rpm.setData(x, y)
            x, y = get_full("ERPM Soll")
            self.curve_erpm_soll.setData(x, y)
            x, y = get_full("LESO Omega")
            self.curve_leso_omega.setData(x, y)

            x, y = get_full("Tf")
            self.curve_tf.setData(x, y)
            x, y = get_full("Pedal Torque Observed")
            self.curve_tp_obs.setData(x, y)
            x, y = get_full("Tf Combine")
            self.curve_tf_combine.setData(x, y)

            x, y = get_full("Speed Error")
            self.curve_speed_error.setData(x, y)
            x, y = get_full("Pos Term Speed")
            self.curve_pos_term_speed.setData(x, y)

        # Charts 2
        if self.charts2_scroll_mode:
            x, y = get_visible("Model Speed")
            self.curve_model_speed.setData(x, y)
            x, y = get_visible("Real Speed km/h")
            self.curve_real_speed.setData(x, y)

            x, y = get_visible("F Combine")
            self.curve_fcombine.setData(x, y)
            x, y = get_visible("Incline Deg Ist")
            self.curve_incline.setData(x, y)

            x, y = get_visible("UW Angle SP")
            self.curve_uw_angle_sp.setData(x, y)
            x, y = get_visible("UW Theta")
            self.curve_uw_theta.setData(x, y)

            x, y = get_visible("Position Error")
            self.curve_position_error.setData(x, y)

            for p in self.chart_tabs_2:
                p.setXRange(
                    max(0.0, x_last - PLOT_SCROLL_WINDOW_S),
                    max(PLOT_SCROLL_WINDOW_S, x_last),
                    padding=0.0
                )
        else:
            x, y = get_full("Model Speed")
            self.curve_model_speed.setData(x, y)
            x, y = get_full("Real Speed km/h")
            self.curve_real_speed.setData(x, y)

            x, y = get_full("F Combine")
            self.curve_fcombine.setData(x, y)
            x, y = get_full("Incline Deg Ist")
            self.curve_incline.setData(x, y)

            x, y = get_full("UW Angle SP")
            self.curve_uw_angle_sp.setData(x, y)
            x, y = get_full("UW Theta")
            self.curve_uw_theta.setData(x, y)

            x, y = get_full("Position Error")
            self.curve_position_error.setData(x, y)

# =========================
# Threads
# =========================
def vesc_communication():
    global vesc_values, selected_port, control_value, control_mode

    while True:
        if vesc_com_flag.is_set() and selected_port:
            try:
                with VESC(serial_port=selected_port) as vesc:
                    while vesc_com_flag.is_set():
                        start_time = time.perf_counter()

                        try:
                            response = vesc.get_measurements_exp()
                        except Exception as e:
                            print("READ ERROR:", repr(e))
                            raise

                        if response is not None:
                            new_values = build_vesc_values(response)
                            with vesc_values_lock:
                                vesc_values = new_values
                            append_history(new_values)

                        try:
                            if control_mode == "Duty Cycle":
                                vesc.set_duty_cycle(control_value)
                            elif control_mode == "Current":
                                vesc.set_current(control_value)
                            elif control_mode == "Position":
                                vesc.set_servo(control_value)
                            elif control_mode == "Speed":
                                vesc.set_rpm(0)
                            else:
                                vesc.set_current(0.0)
                        except Exception as e:
                            print("WRITE ERROR:", repr(e))
                            raise

                        elapsed_time = time.perf_counter() - start_time
                        sleep_time = max(0, 0.01 - elapsed_time)
                        time.sleep(sleep_time)

            except Exception as e:
                print(f"VESC Communication Error: {repr(e)}")
                vesc_com_flag.clear()
        else:
            time.sleep(0.1)


def control_logic():
    global control_value, control_mode

    while True:
        if vesc_com_flag.is_set() and prog_flag.is_set():
            control_mode = "Speed"
            control_value = 0.0
            time.sleep(0.01)
        else:
            if not prog_flag.is_set():
                control_mode = "Current"
                control_value = 0.0
            time.sleep(0.1)


# =========================
# Main
# =========================
if __name__ == "__main__":
    communication_thread = threading.Thread(target=vesc_communication, daemon=True)
    control_logic_thread = threading.Thread(target=control_logic, daemon=True)
    communication_thread.start()
    control_logic_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())