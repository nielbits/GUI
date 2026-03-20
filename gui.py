# -*- coding: utf-8 -*-

import sys
import time
import threading
import collections
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

vesc_values = {}
vesc_values_lock = threading.Lock()

history_len = 1000
vesc_history = {}
vesc_history_lock = threading.Lock()

selected_port = None
control_value = 0.0
control_mode = "Current"
bicycle_mode = "dyn"

speed = 0
dist = 0
elev = 0
is_free = 1

# Adjust if needed for better real-speed estimate from cadence + gear ratio
WHEEL_CIRCUMFERENCE_M = 2.105  # typical 700c road wheel


# =========================
# Telemetry helpers
# =========================
def safe_byte_to_int(value, default=0):
    """
    Convert bytes or bytearray to an integer, with fallback to a default value.
    
    Args:
        value: Input value that can be bytes, bytearray, int, or None.
        default: Default value to return if value is None or empty bytes/bytearray.
                 Defaults to 0.
    
    Returns:
        int: The integer representation of the input value. If value is bytes or
             bytearray with length 1, returns the byte value directly. If longer,
             converts from big-endian bytes to unsigned integer. If value is None,
             empty bytes/bytearray, or already an int, returns the appropriate value
             or the default.
    
    Examples:
        >>> safe_byte_to_int(b'\x05')
        5
        >>> safe_byte_to_int(b'\x00\x10')
        16
        >>> safe_byte_to_int(None)
        0
        >>> safe_byte_to_int(b'', default=42)
        42
        >>> safe_byte_to_int(10)
        10
    """
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 0:
            return default
        if len(value) == 1:
            return value[0]
        return int.from_bytes(value, byteorder="big", signed=False)
    return value if value is not None else default


def estimate_real_speed_kmh(rpm, gear_ratio):
    """
    Estimates bike speed from pedal RPM and gear ratio.

    Assumption:
    wheel_rpm = pedal_rpm / gear_ratio

    If your firmware defines gear_ratio the other way around,
    just invert the relation below.
    """
    try:
        rpm = float(rpm)
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
    mc_fault_code = safe_byte_to_int(getattr(response, "mc_fault_code", 0), 0)
    app_controller_id = safe_byte_to_int(getattr(response, "app_controller_id", 0), 0)
    status = safe_byte_to_int(getattr(response, "status", 0), 0)

    rpm = getattr(response, "rpm", 0.0)
    erpm_soll = getattr(response, "erpm_soll", 0.0)
    gear_ratio = getattr(response, "gear_ratio", 0.0)
    model_speed = getattr(response, "model_speed", 0.0)
    uw_theta = getattr(response, "uw_theta", 0.0)
    uw_angle_sp = getattr(response, "uw_angle_sp", 0.0)

    position_error = uw_angle_sp - uw_theta
    real_speed_kmh = estimate_real_speed_kmh(rpm, gear_ratio)

    return {
        # Core / original block
        "RPM": rpm,
        "ERPM Soll": erpm_soll,
        "Position": getattr(response, "pid_pos_now", 0.0),
        "Motor Current": avg_motor_current,
        "Battery Current": avg_input_current,
        "ID Filtered": getattr(response, "avg_id", 0.0),
        "IQ Filtered": getattr(response, "avg_iq", 0.0),
        "ID Instant": getattr(response, "id_current", 0.0),
        "IQ Instant": getattr(response, "iq_current", 0.0),
        "IQ Set": getattr(response, "iq_set", 0.0),
        "Input Voltage": v_in,
        "Duty Cycle": duty_cycle_now,
        "FET Temperature": getattr(response, "temp_fet", 0.0),
        "Motor Temperature": getattr(response, "temp_motor", 0.0),
        "MOS1 Temperature": getattr(response, "temp_mos1", 0.0),
        "MOS2 Temperature": getattr(response, "temp_mos2", 0.0),
        "MOS3 Temperature": getattr(response, "temp_mos3", 0.0),
        "Amp Hours": getattr(response, "amp_hours", 0.0),
        "Amp Hours Charged": getattr(response, "amp_hours_charged", 0.0),
        "Watt Hours": getattr(response, "watt_hours", 0.0),
        "Watt Hours Charged": getattr(response, "watt_hours_charged", 0.0),
        "Tachometer": getattr(response, "tachometer", 0),
        "Tachometer Abs": getattr(response, "tachometer_abs", 0),
        "Fault Code": mc_fault_code,
        "Controller ID": app_controller_id,
        "Avg Vd": getattr(response, "avg_vd", 0.0),
        "Avg Vq": getattr(response, "avg_vq", 0.0),
        "Status Bits": status,

        # Extended custom block
        "Tf": getattr(response, "tf", 0.0),
        "Gear Ratio": gear_ratio,
        "Model Speed": model_speed,
        "Real Speed km/h": real_speed_kmh,
        "F Combine": getattr(response, "f_combine", 0.0),
        "UW Theta": uw_theta,
        "LESO Omega": getattr(response, "leso_omega", 0.0),
        "Pedal Torque Observed": getattr(response, "tp_observed", 0.0),
        "Param Index": getattr(response, "param_index", 0),
        "I_res": getattr(response, "i_res", 0.0),
        "UW Angle SP": uw_angle_sp,
        "Position Error": position_error,
        "Param Value": getattr(response, "param_from_index", 0.0),
        "Pos Term Speed": getattr(response, "pos_term_speed", 0.0),
        "Speed Error": getattr(response, "speed_error", 0.0),
        "Tf Combine": getattr(response, "t_f_combine", 0.0),
        "Incline Deg Ist": getattr(response, "incline_deg_ist", 0.0),

        # Status word
        "Status Bits Ext": status_bits_ext,
        "Speed Ctrl Active": int(bool(status_bits_ext & (1 << 0))),
        "Forced Freewheel Active": int(bool(status_bits_ext & (1 << 1))),
        "Ctrl State START": int(bool(status_bits_ext & (1 << 2))),
        "Ctrl State INDEX_FOUND": int(bool(status_bits_ext & (1 << 3))),
        "Ctrl State ENABLE": int(bool(status_bits_ext & (1 << 4))),
        "Status Bits Bits": f"{status_bits_ext:032b}",

        # Debug / test channels
        "Test 5.005": getattr(response, "test_5_005", 0.0),
        "Test 6.006": getattr(response, "test_6_006", 0.0),
        "Test 7.007": getattr(response, "test_7_007", 0.0),
        "Test 8.008": getattr(response, "test_8_008", 0.0),
        "Test 9.009": getattr(response, "test_9_009", 0.0),

        # Derived values
        "Power In": v_in * avg_input_current,
        "Power Motor Approx": v_in * duty_cycle_now * avg_motor_current,
    }


def append_history(values_dict):
    with vesc_history_lock:
        for key, value in values_dict.items():
            if isinstance(value, (int, float)):
                if key not in vesc_history:
                    vesc_history[key] = collections.deque(maxlen=history_len)
                vesc_history[key].append(float(value))


# =========================
# Popup telemetry window
# =========================
class TelemetryWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetry Table")
        self.resize(480, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        layout = QtWidgets.QVBoxLayout(central)

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

    def update_table(self, values_dict):
        items = list(values_dict.items())
        self.telemetryTable.setRowCount(len(items))
        for row, (key, value) in enumerate(items):
            key_item = QtWidgets.QTableWidgetItem(str(key))
            if isinstance(value, float):
                value_str = f"{value:.6f}"
            else:
                value_str = str(value)
            val_item = QtWidgets.QTableWidgetItem(value_str)
            self.telemetryTable.setItem(row, 0, key_item)
            self.telemetryTable.setItem(row, 1, val_item)


# =========================
# GUI
# =========================
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1800, 980)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.telemetryWindow = TelemetryWindow()
        self.telemetryWindow.show()

        # ---------- Port / connection ----------
        self.comboBox_portselect = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_portselect.setGeometry(QtCore.QRect(10, 10, 320, 28))
        self.comboBox_portselect.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.comboBox_portselect.setObjectName("comboBox_portselect")

        self.pushButton_refresh = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_refresh.setGeometry(QtCore.QRect(340, 10, 80, 28))
        self.pushButton_refresh.setObjectName("pushButton_refresh")

        self.pushButton_connect = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_connect.setGeometry(QtCore.QRect(10, 50, 90, 36))
        self.pushButton_connect.setObjectName("pushButton_connect")

        self.pushButton_disconnect = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_disconnect.setGeometry(QtCore.QRect(110, 50, 100, 36))
        self.pushButton_disconnect.setObjectName("pushButton_disconnect")

        self.start = QtWidgets.QPushButton(self.centralwidget)
        self.start.setGeometry(QtCore.QRect(230, 50, 80, 36))
        self.start.setObjectName("start")

        self.stop = QtWidgets.QPushButton(self.centralwidget)
        self.stop.setGeometry(QtCore.QRect(320, 50, 80, 36))
        self.stop.setObjectName("stop")

        self.Reset = QtWidgets.QPushButton(self.centralwidget)
        self.Reset.setGeometry(QtCore.QRect(410, 50, 80, 36))
        self.Reset.setObjectName("Reset")

        self.pushButton_show_table = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_show_table.setGeometry(QtCore.QRect(500, 50, 130, 36))
        self.pushButton_show_table.setObjectName("pushButton_show_table")

        # ---------- Status indicators ----------
        self.label_status_title = QtWidgets.QLabel(self.centralwidget)
        self.label_status_title.setGeometry(QtCore.QRect(20, 580, 220, 25))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_status_title.setFont(font)
        self.label_status_title.setObjectName("label_status_title")

        self.label_speed_ctrl = QtWidgets.QLabel(self.centralwidget)
        self.label_speed_ctrl.setGeometry(QtCore.QRect(20, 615, 170, 22))
        self.label_speed_ctrl.setObjectName("label_speed_ctrl")

        self.frame_speed_ctrl = QtWidgets.QFrame(self.centralwidget)
        self.frame_speed_ctrl.setGeometry(QtCore.QRect(200, 613, 22, 22))
        self.frame_speed_ctrl.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_speed_ctrl.setLineWidth(1)

        self.label_forced_fw = QtWidgets.QLabel(self.centralwidget)
        self.label_forced_fw.setGeometry(QtCore.QRect(20, 645, 170, 22))
        self.label_forced_fw.setObjectName("label_forced_fw")

        self.frame_forced_fw = QtWidgets.QFrame(self.centralwidget)
        self.frame_forced_fw.setGeometry(QtCore.QRect(200, 643, 22, 22))
        self.frame_forced_fw.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_forced_fw.setLineWidth(1)

        self.label_ctrl_start = QtWidgets.QLabel(self.centralwidget)
        self.label_ctrl_start.setGeometry(QtCore.QRect(20, 675, 170, 22))
        self.label_ctrl_start.setObjectName("label_ctrl_start")

        self.frame_ctrl_start = QtWidgets.QFrame(self.centralwidget)
        self.frame_ctrl_start.setGeometry(QtCore.QRect(200, 673, 22, 22))
        self.frame_ctrl_start.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_ctrl_start.setLineWidth(1)

        self.label_ctrl_index = QtWidgets.QLabel(self.centralwidget)
        self.label_ctrl_index.setGeometry(QtCore.QRect(20, 705, 170, 22))
        self.label_ctrl_index.setObjectName("label_ctrl_index")

        self.frame_ctrl_index = QtWidgets.QFrame(self.centralwidget)
        self.frame_ctrl_index.setGeometry(QtCore.QRect(200, 703, 22, 22))
        self.frame_ctrl_index.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_ctrl_index.setLineWidth(1)

        self.label_ctrl_enable = QtWidgets.QLabel(self.centralwidget)
        self.label_ctrl_enable.setGeometry(QtCore.QRect(20, 735, 170, 22))
        self.label_ctrl_enable.setObjectName("label_ctrl_enable")

        self.frame_ctrl_enable = QtWidgets.QFrame(self.centralwidget)
        self.frame_ctrl_enable.setGeometry(QtCore.QRect(200, 733, 22, 22))
        self.frame_ctrl_enable.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_ctrl_enable.setLineWidth(1)

        self.set_indicator(self.frame_speed_ctrl, False)
        self.set_indicator(self.frame_forced_fw, False)
        self.set_indicator(self.frame_ctrl_start, False)
        self.set_indicator(self.frame_ctrl_index, False)
        self.set_indicator(self.frame_ctrl_enable, False)

        # ---------- Mode ----------
        self.Mode = QtWidgets.QGroupBox(self.centralwidget)
        self.Mode.setGeometry(QtCore.QRect(20, 110, 170, 150))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.Mode.setFont(font)
        self.Mode.setObjectName("Mode")

        self.Button_dyn = QtWidgets.QRadioButton(self.Mode)
        self.Button_dyn.setGeometry(QtCore.QRect(15, 30, 120, 24))
        self.Button_dyn.setChecked(True)
        self.Button_dyn.setObjectName("Button_dyn")

        self.Button_spin = QtWidgets.QRadioButton(self.Mode)
        self.Button_spin.setGeometry(QtCore.QRect(15, 62, 120, 24))
        self.Button_spin.setObjectName("Button_spin")

        self.Button_Ergometer = QtWidgets.QRadioButton(self.Mode)
        self.Button_Ergometer.setGeometry(QtCore.QRect(15, 94, 120, 24))
        self.Button_Ergometer.setObjectName("Button_Ergometer")

        # ---------- Sliders / setup ----------
        self.label_incline = QtWidgets.QLabel(self.centralwidget)
        self.label_incline.setGeometry(QtCore.QRect(220, 110, 120, 36))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_incline.setFont(font)
        self.label_incline.setObjectName("label_incline")

        self.Slider_incline = QtWidgets.QSlider(self.centralwidget)
        self.Slider_incline.setGeometry(QtCore.QRect(350, 110, 231, 36))
        self.Slider_incline.setMinimum(-45)
        self.Slider_incline.setMaximum(45)
        self.Slider_incline.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_incline.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.Slider_incline.setTickInterval(5)
        self.Slider_incline.setObjectName("Slider_incline")

        self.line_incline = QtWidgets.QLineEdit(self.centralwidget)
        self.line_incline.setGeometry(QtCore.QRect(595, 108, 70, 36))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.line_incline.setFont(font)
        self.line_incline.setAlignment(QtCore.Qt.AlignCenter)
        self.line_incline.setObjectName("line_incline")

        self.pushButton_incline_res = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_incline_res.setGeometry(QtCore.QRect(675, 108, 45, 36))
        self.pushButton_incline_res.setObjectName("pushButton_incline_res")

        self.label_airspeed = QtWidgets.QLabel(self.centralwidget)
        self.label_airspeed.setGeometry(QtCore.QRect(185, 160, 155, 36))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_airspeed.setFont(font)
        self.label_airspeed.setObjectName("label_airspeed")

        self.Slider_airspeed = QtWidgets.QSlider(self.centralwidget)
        self.Slider_airspeed.setGeometry(QtCore.QRect(350, 160, 231, 36))
        self.Slider_airspeed.setMinimum(-30)
        self.Slider_airspeed.setMaximum(30)
        self.Slider_airspeed.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_airspeed.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.Slider_airspeed.setTickInterval(5)
        self.Slider_airspeed.setObjectName("Slider_airspeed")

        self.line_airspeed = QtWidgets.QLineEdit(self.centralwidget)
        self.line_airspeed.setGeometry(QtCore.QRect(595, 158, 70, 36))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.line_airspeed.setFont(font)
        self.line_airspeed.setAlignment(QtCore.Qt.AlignCenter)
        self.line_airspeed.setObjectName("line_airspeed")

        self.pushButton_airspeed_reset = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_airspeed_reset.setGeometry(QtCore.QRect(675, 158, 45, 36))
        self.pushButton_airspeed_reset.setObjectName("pushButton_airspeed_reset")

        # ---------- Gear ----------
        self.label_gear = QtWidgets.QLabel(self.centralwidget)
        self.label_gear.setGeometry(QtCore.QRect(260, 215, 70, 36))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_gear.setFont(font)
        self.label_gear.setObjectName("label_gear")

        self.gearselector = QtWidgets.QSpinBox(self.centralwidget)
        self.gearselector.setGeometry(QtCore.QRect(350, 210, 70, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.gearselector.setFont(font)
        self.gearselector.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.gearselector.setMinimum(1)
        self.gearselector.setMaximum(11)
        self.gearselector.setObjectName("gearselector")

        self.pushButton_upshift = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_upshift.setGeometry(QtCore.QRect(430, 210, 45, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.pushButton_upshift.setFont(font)
        self.pushButton_upshift.setObjectName("pushButton_upshift")

        self.pushButton_downshift = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_downshift.setGeometry(QtCore.QRect(485, 210, 45, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.pushButton_downshift.setFont(font)
        self.pushButton_downshift.setObjectName("pushButton_downshift")

        # ---------- Freewheel ----------
        self.radioButton_freewheel = QtWidgets.QRadioButton(self.centralwidget)
        self.radioButton_freewheel.setGeometry(QtCore.QRect(30, 275, 110, 32))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radioButton_freewheel.setFont(font)
        self.radioButton_freewheel.setChecked(True)
        self.radioButton_freewheel.setObjectName("radioButton_freewheel")

        # ---------- Mass ----------
        self.mass = QtWidgets.QFrame(self.centralwidget)
        self.mass.setGeometry(QtCore.QRect(20, 330, 220, 90))
        self.mass.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mass.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mass.setObjectName("mass")

        self.label_drivermass = QtWidgets.QLabel(self.mass)
        self.label_drivermass.setGeometry(QtCore.QRect(10, 10, 130, 30))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_drivermass.setFont(font)
        self.label_drivermass.setObjectName("label_drivermass")

        self.line_drivermass = QtWidgets.QLineEdit(self.mass)
        self.line_drivermass.setGeometry(QtCore.QRect(155, 10, 55, 30))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.line_drivermass.setFont(font)
        self.line_drivermass.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.line_drivermass.setObjectName("line_drivermass")

        self.label_bikemass = QtWidgets.QLabel(self.mass)
        self.label_bikemass.setGeometry(QtCore.QRect(10, 50, 130, 30))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_bikemass.setFont(font)
        self.label_bikemass.setObjectName("label_bikemass")

        self.line_bikemass = QtWidgets.QLineEdit(self.mass)
        self.line_bikemass.setGeometry(QtCore.QRect(155, 50, 55, 30))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.line_bikemass.setFont(font)
        self.line_bikemass.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.line_bikemass.setObjectName("line_bikemass")

        # ---------- Numeric monitor ----------
        self.energy = QtWidgets.QFrame(self.centralwidget)
        self.energy.setGeometry(QtCore.QRect(260, 280, 280, 220))
        self.energy.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.energy.setFrameShadow(QtWidgets.QFrame.Raised)
        self.energy.setObjectName("energy")

        row_y = [10, 50, 90, 130, 170]
        labels = ["Elevation:", "Potential Energy:", "Speed:", "Kinetic Energy:", "Distance:"]
        line_names = ["line_elevation", "line_pot_energy", "line_speed", "line_kin_energy", "line_distance"]
        self.energy_lines = {}

        for y, lab, obj_name in zip(row_y, labels, line_names):
            label = QtWidgets.QLabel(self.energy)
            label.setGeometry(QtCore.QRect(10, y, 150, 30))
            font = QtGui.QFont()
            font.setPointSize(14)
            label.setFont(font)
            label.setText(lab)

            line = QtWidgets.QLineEdit(self.energy)
            line.setGeometry(QtCore.QRect(180, y, 85, 30))
            font = QtGui.QFont()
            font.setPointSize(14)
            line.setFont(font)
            line.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            line.setReadOnly(True)
            line.setObjectName(obj_name)
            line.setText("0")
            self.energy_lines[obj_name] = line

        self.line_elevation = self.energy_lines["line_elevation"]
        self.line_pot_energy = self.energy_lines["line_pot_energy"]
        self.line_speed = self.energy_lines["line_speed"]
        self.line_kin_energy = self.energy_lines["line_kin_energy"]
        self.line_distance = self.energy_lines["line_distance"]

        self.rpm = QtWidgets.QFrame(self.centralwidget)
        self.rpm.setGeometry(QtCore.QRect(20, 440, 220, 120))
        self.rpm.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.rpm.setFrameShadow(QtWidgets.QFrame.Raised)
        self.rpm.setObjectName("rpm")

        rpm_labels = ["Ped RPM:", "Sys RPM:", "Power:"]
        rpm_names = ["line_pedrpm", "line_sysrpm", "line_power"]
        self.rpm_lines = {}
        for i, (lab, obj_name) in enumerate(zip(rpm_labels, rpm_names)):
            y = 10 + i * 35
            label = QtWidgets.QLabel(self.rpm)
            label.setGeometry(QtCore.QRect(10, y, 90, 26))
            font = QtGui.QFont()
            font.setPointSize(14)
            label.setFont(font)
            label.setText(lab)

            line = QtWidgets.QLineEdit(self.rpm)
            line.setGeometry(QtCore.QRect(110, y, 90, 26))
            font = QtGui.QFont()
            font.setPointSize(14)
            line.setFont(font)
            line.setReadOnly(True)
            line.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            line.setObjectName(obj_name)
            line.setText("0")
            self.rpm_lines[obj_name] = line

        self.line_pedrpm = self.rpm_lines["line_pedrpm"]
        self.line_sysrpm = self.rpm_lines["line_sysrpm"]
        self.line_power = self.rpm_lines["line_power"]

        # ---------- Live plots ----------
        self.plotWidget = pg.GraphicsLayoutWidget(self.centralwidget)
        self.plotWidget.setGeometry(QtCore.QRect(760, 10, 1020, 920))
        self.plotWidget.setBackground('k')

        pg.setConfigOptions(antialias=True)
        axis_pen = pg.mkPen((180, 180, 180))
        text_pen = (220, 220, 220)

        def make_plot(row, col, title):
            plot = self.plotWidget.addPlot(row=row, col=col, title=title)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.getAxis('left').setPen(axis_pen)
            plot.getAxis('bottom').setPen(axis_pen)
            plot.getAxis('left').setTextPen(text_pen)
            plot.getAxis('bottom').setTextPen(text_pen)
            plot.addLegend(offset=(5, 5))
            return plot

        # 1) all currents
        self.plot_currents = make_plot(0, 0, "1) All Currents")
        self.curve_motor_current = self.plot_currents.plot(pen=pg.mkPen('y', width=1.5), name='Motor Current')
        #self.curve_battery_current = self.plot_currents.plot(pen=pg.mkPen('c', width=1.5), name='Battery Current')
        #self.curve_id_filtered = self.plot_currents.plot(pen=pg.mkPen('m', width=1.5), name='ID Filtered')
        self.curve_iq_filtered = self.plot_currents.plot(pen=pg.mkPen('g', width=1.5), name='IQ Filtered')
        #self.curve_id_instant = self.plot_currents.plot(pen=pg.mkPen('b', width=1.5), name='ID Instant')
        self.curve_iq_instant = self.plot_currents.plot(pen=pg.mkPen('r', width=1.5), name='IQ Instant')
        self.curve_iq_set = self.plot_currents.plot(pen=pg.mkPen((255, 165, 0), width=1.5), name='IQ Set')

        # 2) all erpms
        self.plot_erpms = make_plot(0, 1, "2) All ERPMs")
        self.curve_rpm = self.plot_erpms.plot(pen=pg.mkPen('y', width=1.5), name='RPM')
        self.curve_erpm_soll = self.plot_erpms.plot(pen=pg.mkPen('c', width=1.5), name='ERPM Soll')
        self.curve_leso_omega = self.plot_erpms.plot(pen=pg.mkPen('m', width=1.5), name='LESO Omega')

        # 3) all torques
        self.plot_torques = make_plot(1, 0, "3) All Torques")
        self.curve_tf = self.plot_torques.plot(pen=pg.mkPen('y', width=1.5), name='Tf')
        self.curve_tp_obs = self.plot_torques.plot(pen=pg.mkPen('c', width=1.5), name='Pedal Torque Observed')
        self.curve_tf_combine = self.plot_torques.plot(pen=pg.mkPen('m', width=1.5), name='Tf Combine')

        # 4) speed error and pos error term
        self.plot_errors = make_plot(1, 1, "4) Speed Error and Pos Error Term")
        self.curve_speed_error = self.plot_errors.plot(pen=pg.mkPen('r', width=1.5), name='Speed Error')
        self.curve_pos_term_speed = self.plot_errors.plot(pen=pg.mkPen('c', width=1.5), name='Pos Term Speed')

        # 5) speed in km/h (setpoint x real)
        self.plot_speed_kmh = make_plot(2, 0, "5) Speed in km/h (Setpoint x Real)")
        self.curve_model_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('y', width=1.5), name='Setpoint km/h')
        self.curve_real_speed = self.plot_speed_kmh.plot(pen=pg.mkPen('c', width=1.5), name='Real km/h')

        # 6) f_combine and inclination
        self.plot_force_incline = make_plot(2, 1, "6) F Combine and Inclination")
        self.curve_fcombine = self.plot_force_incline.plot(pen=pg.mkPen('m', width=1.5), name='F Combine')
        self.curve_incline = self.plot_force_incline.plot(pen=pg.mkPen('g', width=1.5), name='Incline Deg Ist')

        # 7) uw theta setpoint and real(filtered)
        self.plot_uw = make_plot(3, 0, "7) UW Theta Setpoint and Real (Filtered)")
        self.curve_uw_angle_sp = self.plot_uw.plot(pen=pg.mkPen('c', width=1.5), name='UW Angle SP')
        self.curve_uw_theta = self.plot_uw.plot(pen=pg.mkPen('y', width=1.5), name='UW Theta')

        # 8) values from 7 subtracted (position error)
        self.plot_pos_error = make_plot(3, 1, "8) Position Error")
        self.curve_position_error = self.plot_pos_error.plot(pen=pg.mkPen('r', width=1.5), name='Position Error')

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1800, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.pushButton_refresh.clicked.connect(self.refresh_ports)
        self.pushButton_connect.clicked.connect(self.start_com)
        self.pushButton_disconnect.clicked.connect(self.stop_com)
        self.start.clicked.connect(self.start_prog)
        self.stop.clicked.connect(self.stop_prog)
        self.Reset.clicked.connect(self.reset_prog)
        self.pushButton_show_table.clicked.connect(self.show_table_window)

        self.Button_dyn.clicked.connect(self.set_dyn)
        self.Button_spin.clicked.connect(self.set_spin)
        self.Button_Ergometer.clicked.connect(self.set_erg)

        self.Slider_incline.valueChanged.connect(self.updateIncline)
        self.line_incline.textChanged.connect(self.validateIncline)
        self.pushButton_incline_res.clicked.connect(self.resetIncline)

        self.Slider_airspeed.valueChanged.connect(self.updateAirspeed)
        self.line_airspeed.textChanged.connect(self.validateAirspeed)
        self.pushButton_airspeed_reset.clicked.connect(self.resetAirspeed)

        self.pushButton_upshift.clicked.connect(self.incrementGear)
        self.pushButton_downshift.clicked.connect(self.decrementGear)

        self.radioButton_freewheel.clicked.connect(self.set_freewheel)

        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.refresh_live_data)
        self.gui_timer.start(100)

        self.refresh_ports()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "VESC Telemetry GUI"))
        self.pushButton_refresh.setText(_translate("MainWindow", "Refresh"))
        self.pushButton_connect.setText(_translate("MainWindow", "Connect"))
        self.pushButton_disconnect.setText(_translate("MainWindow", "Disconnect"))
        self.start.setText(_translate("MainWindow", "Start"))
        self.stop.setText(_translate("MainWindow", "Stop"))
        self.Reset.setText(_translate("MainWindow", "Reset"))
        self.pushButton_show_table.setText(_translate("MainWindow", "Show Table"))
        self.label_status_title.setText(_translate("MainWindow", "Controller Status Bits"))
        self.label_speed_ctrl.setText(_translate("MainWindow", "Speed Ctrl Active"))
        self.label_forced_fw.setText(_translate("MainWindow", "Forced Freewheel"))
        self.label_ctrl_start.setText(_translate("MainWindow", "CTRL_SM = START"))
        self.label_ctrl_index.setText(_translate("MainWindow", "CTRL_SM = INDEX_FOUND"))
        self.label_ctrl_enable.setText(_translate("MainWindow", "CTRL_SM = ENABLE"))
        self.Mode.setTitle(_translate("MainWindow", "Mode"))
        self.Button_dyn.setText(_translate("MainWindow", "Dynamics"))
        self.Button_spin.setText(_translate("MainWindow", "Spinning"))
        self.Button_Ergometer.setText(_translate("MainWindow", "Ergometer"))
        self.label_incline.setText(_translate("MainWindow", "Steigung"))
        self.label_airspeed.setText(_translate("MainWindow", "Windgeschw."))
        self.label_gear.setText(_translate("MainWindow", "Gang"))
        self.pushButton_incline_res.setText(_translate("MainWindow", "0"))
        self.pushButton_airspeed_reset.setText(_translate("MainWindow", "0"))
        self.pushButton_upshift.setText(_translate("MainWindow", "+"))
        self.pushButton_downshift.setText(_translate("MainWindow", "-"))
        self.radioButton_freewheel.setText(_translate("MainWindow", "Freilauf"))
        self.label_drivermass.setText(_translate("MainWindow", "Driver Mass:"))
        self.label_bikemass.setText(_translate("MainWindow", "Bike Mass:"))

        self.line_incline.setText(_translate("MainWindow", "0"))
        self.line_airspeed.setText(_translate("MainWindow", "0"))
        self.line_drivermass.setText(_translate("MainWindow", "85"))
        self.line_bikemass.setText(_translate("MainWindow", "10"))

    def show_table_window(self):
        self.telemetryWindow.show()
        self.telemetryWindow.raise_()
        self.telemetryWindow.activateWindow()

    def set_indicator(self, frame, active):
        if active:
            frame.setStyleSheet("background-color: rgb(0, 200, 0); border: 1px solid black;")
        else:
            frame.setStyleSheet("background-color: rgb(80, 80, 80); border: 1px solid black;")

    def set_dyn(self):
        global bicycle_mode
        bicycle_mode = "dyn"

    def set_spin(self):
        global bicycle_mode
        bicycle_mode = "spin"

    def set_erg(self):
        global bicycle_mode
        bicycle_mode = "erg"

    def incrementGear(self):
        if self.gearselector.value() < self.gearselector.maximum():
            self.gearselector.setValue(self.gearselector.value() + 1)

    def decrementGear(self):
        if self.gearselector.value() > self.gearselector.minimum():
            self.gearselector.setValue(self.gearselector.value() - 1)

    def updateIncline(self, value):
        self.line_incline.setText(str(value))

    def validateIncline(self):
        try:
            value = int(self.line_incline.text())
            if value < self.Slider_incline.minimum() or value > self.Slider_incline.maximum():
                print("Warning: Incline value out of range!")
        except ValueError:
            pass

    def resetIncline(self):
        self.Slider_incline.setValue(0)
        self.line_incline.setText("0")

    def updateAirspeed(self, value):
        self.line_airspeed.setText(str(value))

    def validateAirspeed(self):
        try:
            value = int(self.line_airspeed.text())
            if value < self.Slider_airspeed.minimum() or value > self.Slider_airspeed.maximum():
                print("Warning: Airspeed value out of range!")
        except ValueError:
            pass

    def resetAirspeed(self):
        self.Slider_airspeed.setValue(0)
        self.line_airspeed.setText("0")

    def set_freewheel(self):
        if self.radioButton_freewheel.isChecked():
            freewheel.set()
        else:
            freewheel.clear()

    def refresh_ports(self):
        self.comboBox_portselect.clear()
        ports = list(serial.tools.list_ports.comports())

        print("Detected serial ports:")
        for port in ports:
            print(f"  device={port.device}, description={port.description}")

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
        else:
            print("No valid serial port selected.")

    def stop_com(self):
        vesc_com_flag.clear()

    def start_prog(self):
        prog_flag.set()

    def stop_prog(self):
        global control_value, control_mode
        control_value = 0.0
        control_mode = "Current"
        prog_flag.clear()

    def reset_prog(self):
        global speed, control_value, control_mode, dist, elev

        if prog_flag.is_set():
            prog_flag.clear()

        speed = 0
        dist = 0
        elev = 0
        control_value = 0.0
        control_mode = "Current"

        self.Slider_incline.setValue(0)
        self.line_incline.setText("0")
        self.Slider_airspeed.setValue(0)
        self.line_airspeed.setText("0")
        self.line_speed.setText("0")
        self.line_kin_energy.setText("0")
        self.line_elevation.setText("0")
        self.line_pot_energy.setText("0")
        self.line_sysrpm.setText("0")
        self.line_pedrpm.setText("0")
        self.line_power.setText("0")
        self.line_distance.setText("0")
        self.gearselector.setValue(1)

        with vesc_history_lock:
            vesc_history.clear()

    def refresh_live_data(self):
        global vesc_values

        with vesc_values_lock:
            local_values = dict(vesc_values)

        if not local_values:
            return

        self.set_indicator(self.frame_speed_ctrl, bool(local_values.get("Speed Ctrl Active", 0)))
        self.set_indicator(self.frame_forced_fw, bool(local_values.get("Forced Freewheel Active", 0)))
        self.set_indicator(self.frame_ctrl_start, bool(local_values.get("Ctrl State START", 0)))
        self.set_indicator(self.frame_ctrl_index, bool(local_values.get("Ctrl State INDEX_FOUND", 0)))
        self.set_indicator(self.frame_ctrl_enable, bool(local_values.get("Ctrl State ENABLE", 0)))

        self.line_pedrpm.setText(f"{local_values.get('RPM', 0):.0f}")
        self.line_sysrpm.setText(f"{local_values.get('ERPM Soll', 0):.0f}")
        self.line_power.setText(f"{local_values.get('Power In', 0):.1f}")

        self.line_speed.setText(f"{local_values.get('Model Speed', 0):.2f}")
        self.line_elevation.setText(f"{local_values.get('Incline Deg Ist', 0):.2f}")
        self.line_kin_energy.setText("0")
        self.line_pot_energy.setText("0")
        self.line_distance.setText("0")

        self.telemetryWindow.update_table(local_values)
        self.refresh_plots()

    def refresh_plots(self):
        with vesc_history_lock:
            h = {k: list(v) for k, v in vesc_history.items()}

        if not h:
            return

        # 1) currents
        #self.curve_motor_current.setData(h.get("Motor Current", []))
#        self.curve_battery_current.setData(h.get("Battery Current", []))
        #self.curve_id_filtered.setData(h.get("ID Filtered", []))
        self.curve_iq_filtered.setData(h.get("IQ Filtered", []))
        #self.curve_id_instant.setData(h.get("ID Instant", []))
        self.curve_iq_instant.setData(h.get("IQ Instant", []))
        self.curve_iq_set.setData(h.get("IQ Set", []))

        # 2) erpms
        self.curve_rpm.setData(h.get("RPM", []))
        self.curve_erpm_soll.setData(h.get("ERPM Soll", []))
        self.curve_leso_omega.setData(h.get("LESO Omega", []))

        # 3) torques
        self.curve_tf.setData(h.get("Tf", []))
        self.curve_tp_obs.setData(h.get("Pedal Torque Observed", []))
        self.curve_tf_combine.setData(h.get("Tf Combine", []))

        # 4) speed error + pos error term
        self.curve_speed_error.setData(h.get("Speed Error", []))
        self.curve_pos_term_speed.setData(h.get("Pos Term Speed", []))

        # 5) speed in km/h
        self.curve_model_speed.setData(h.get("Model Speed", []))
        self.curve_real_speed.setData(h.get("Real Speed km/h", []))

        # 6) f_combine + inclination
        self.curve_fcombine.setData(h.get("F Combine", []))
        self.curve_incline.setData(h.get("Incline Deg Ist", []))

        # 7) uw theta setpoint and real(filtered)
        self.curve_uw_angle_sp.setData(h.get("UW Angle SP", []))
        self.curve_uw_theta.setData(h.get("UW Theta", []))

        # 8) position error
        self.curve_position_error.setData(h.get("Position Error", []))


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