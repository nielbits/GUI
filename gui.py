# -*- coding: utf-8 -*-

import math
import sys
import time
import traceback
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from config import *
import state
from state import (
    get_active_vesc_session, set_target_can_id, get_target_can_id,
    vesc_values_lock, vesc_history_lock, param_state_lock, param_state,
    get_custom_msg_results, sample_counter_lock
)
from diagnostics import log_event, set_diag, inc_diag, get_diag_snapshot
from widgets import LiveValueRow, ReadableFlag, ParamEditRow, make_plot
from vesc_comm import (
    ping_can_ids, read_param_blocks_from_session, start_bike_sim, stop_bike_sim, fwd_msg
)
from vesc_messages import SetBikeRuntime, SetBikeSimParams, SetControlParams


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 800)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)
        self.chart_plots = []
        self.chart_curves = {}
        self.scroll_mode = True
        self.build_ui(MainWindow)
        self.make_connections()
        self.refresh_ports()
        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.refresh_live_data)
        self.gui_timer.start(GUI_REFRESH_MS)

    def build_ui(self, MainWindow):
        root = QtWidgets.QVBoxLayout(self.centralwidget)
        root.setContentsMargins(4, 4, 4, 4); root.setSpacing(4)
        topbar = QtWidgets.QHBoxLayout()
        self.comboBox_portselect = QtWidgets.QComboBox(); self.comboBox_portselect.setMinimumWidth(260)
        self.pushButton_refresh = QtWidgets.QPushButton("Refresh Ports")
        self.pushButton_connect = QtWidgets.QPushButton("Connect")
        self.pushButton_disconnect = QtWidgets.QPushButton("Disconnect")
        self.comboBox_can_id = QtWidgets.QComboBox(); self.comboBox_can_id.setMinimumWidth(90)
        self.comboBox_can_id.addItem(f"CAN {get_target_can_id()}", get_target_can_id())
        self.pushButton_scan_can = QtWidgets.QPushButton("Scan CAN")
        topbar.addWidget(QtWidgets.QLabel("Serial Port:")); topbar.addWidget(self.comboBox_portselect)
        topbar.addWidget(self.pushButton_refresh); topbar.addWidget(self.pushButton_connect); topbar.addWidget(self.pushButton_disconnect)
        topbar.addSpacing(10); topbar.addWidget(QtWidgets.QLabel("Target:")); topbar.addWidget(self.comboBox_can_id); topbar.addWidget(self.pushButton_scan_can)
        topbar.addStretch(1); root.addLayout(topbar)

        self.tabs = QtWidgets.QTabWidget(); root.addWidget(self.tabs)
        self.tab_main = QtWidgets.QWidget(); self.tabs.addTab(self.tab_main, "Main")
        self.tab_params = QtWidgets.QWidget(); self.tabs.addTab(self.tab_params, "Params")
        self.tab_charts = QtWidgets.QWidget(); self.tabs.addTab(self.tab_charts, "Charts")
        self.tab_debug = QtWidgets.QWidget(); self.tabs.addTab(self.tab_debug, "Telemetry / Debug")
        self.build_main_tab(); self.build_params_tab(); self.build_charts_tab(); self.build_debug_tab()
        self.statusbar = QtWidgets.QStatusBar(MainWindow); MainWindow.setStatusBar(self.statusbar)
        MainWindow.setWindowTitle("VESC Telemetry GUI")

    def build_main_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_main)
        controls = QtWidgets.QHBoxLayout()
        self.start = QtWidgets.QPushButton("Start"); self.stop = QtWidgets.QPushButton("Stop"); self.Reset = QtWidgets.QPushButton("Reset")
        self.label_link_state = QtWidgets.QLabel("LINK: IDLE"); self.label_link_state.setStyleSheet("font-weight: bold;")
        controls.addWidget(self.start); controls.addWidget(self.stop); controls.addWidget(self.Reset); controls.addSpacing(12); controls.addWidget(self.label_link_state); controls.addStretch(1)
        layout.addLayout(controls)

        row = QtWidgets.QHBoxLayout()
        runtime_box = QtWidgets.QGroupBox("Runtime")
        runtime_layout = QtWidgets.QVBoxLayout(runtime_box)
        self.runtime_gear_ratio = ParamEditRow("Gear Ratio Bike")
        self.runtime_incline_deg = ParamEditRow("Incline Deg")
        self.runtime_pumptrack_period = ParamEditRow("Pumptrack Period [min]")
        self.runtime_pumptrack_enabled = ParamEditRow("Pumptrack Enabled", is_bool=True)
        self.runtime_freewheel_enabled = ParamEditRow("Freewheel Enabled", is_bool=True)
        self.btn_set_runtime = QtWidgets.QPushButton("Set Runtime")
        for w in [self.runtime_gear_ratio, self.runtime_incline_deg, self.runtime_pumptrack_period, self.runtime_pumptrack_enabled, self.runtime_freewheel_enabled, self.btn_set_runtime]:
            runtime_layout.addWidget(w)

        flags_box = QtWidgets.QGroupBox("Status Flags")
        flags_layout = QtWidgets.QGridLayout(flags_box)
        self.flag_connected = ReadableFlag("Connected"); self.flag_ctrl_active = ReadableFlag("Ctrl active")
        self.flag_forced_fw = ReadableFlag("Forced FW"); self.flag_start = ReadableFlag("START")
        self.flag_index = ReadableFlag("INDEX_FOUND"); self.flag_enable = ReadableFlag("Enable")
        for i, f in enumerate([self.flag_connected, self.flag_ctrl_active, self.flag_forced_fw, self.flag_start, self.flag_index, self.flag_enable]):
            flags_layout.addWidget(f, i // 2, i % 2)

        live_box = QtWidgets.QGroupBox("Live Values")
        live_layout = QtWidgets.QVBoxLayout(live_box)
        self.row_real_speed = LiveValueRow("Real Speed", "km/h"); self.row_incline = LiveValueRow("Incline", "deg")
        self.row_gear_ratio = LiveValueRow("Gear Ratio", "-"); self.row_power = LiveValueRow("Power", "W")
        self.row_sample_age = LiveValueRow("Sample Age", "s"); self.row_rx_count = LiveValueRow("RX Count", "-"); self.row_read_errors = LiveValueRow("Read Err", "-")
        for w in [self.row_real_speed, self.row_incline, self.row_gear_ratio, self.row_power, self.row_sample_age, self.row_rx_count, self.row_read_errors]: live_layout.addWidget(w)
        row.addWidget(runtime_box, 2); row.addWidget(flags_box, 1); row.addWidget(live_box, 1); layout.addLayout(row)

        self.mainPlotWidget = pg.GraphicsLayoutWidget(); self.mainPlotWidget.setBackground('k')
        self.main_plot_torque = make_plot(self.mainPlotWidget, 0, 0, "Observer Torque")
        self.main_curve_torque = self.main_plot_torque.plot(pen=pg.mkPen('c', width=2), name='Pedal Torque')
        self.main_plot_speed = make_plot(self.mainPlotWidget, 1, 0, "Real Speed")
        self.main_curve_speed = self.main_plot_speed.plot(pen=pg.mkPen('y', width=2), name='Real Speed')
        self.main_plot_speed.setXLink(self.main_plot_torque)
        layout.addWidget(self.mainPlotWidget, 1)

    def build_params_tab(self):
        layout = QtWidgets.QHBoxLayout(self.tab_params)
        self.bike_param_rows = {}
        self.control_param_rows = {}
        bike_keys = ["p_air_ro", "p_c_rr", "p_weight", "p_As", "p_c_air", "p_c_bw", "p_c_wl", "p_wheel_radius", "p_mech_gearing", "p_r_bearings", "p_k_v_bw", "p_J", "p_B", "p_k_area", "p_height", "p_speed_limit_pos_control_activation"]
        control_keys = ["p_fo_hz", "p_gz_hz", "p_fc_TLPF", "p_adrc_scale", "p_sched_spd_floor", "p_sched_pos_floor", "p_sched_pos_dead_erpm", "p_sched_spd_sat_erpm", "p_sched_pos_sat_erpm"]
        for title, keys, target in [("Bike", bike_keys, self.bike_param_rows), ("Advanced", control_keys, self.control_param_rows)]:
            box = QtWidgets.QGroupBox(title); v = QtWidgets.QVBoxLayout(box)
            scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); cont = QtWidgets.QWidget(); cv = QtWidgets.QVBoxLayout(cont)
            for k in keys:
                target[k] = ParamEditRow(k); cv.addWidget(target[k])
            cv.addStretch(1); scroll.setWidget(cont); v.addWidget(scroll)
            btn = QtWidgets.QPushButton(f"Set {title} Params"); v.addWidget(btn)
            if title == "Bike": self.btn_set_bike_params = btn
            else: self.btn_set_control_params = btn
            layout.addWidget(box)

    def build_charts_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_charts)

        toolbar = QtWidgets.QHBoxLayout()
        self.btn_autorange = QtWidgets.QPushButton("Auto Range")
        self.btn_reset_x = QtWidgets.QPushButton("Reset X")
        toolbar.addWidget(self.btn_autorange)
        toolbar.addWidget(self.btn_reset_x)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plotWidget = pg.GraphicsLayoutWidget()
        self.plotWidget.setBackground('k')
        layout.addWidget(self.plotWidget)

        plot_defs = [
            ("Currents", ["IQ Filtered", "IQ Set", "IQ Instant"]),            
            ("RPMs", ["RPM Motor", "RPM Set", "LESO RPM"]),
            ("Torques", ["Torque FF", "Torque Motor", "Pedal Torque Observed", "T_F_combine", "T_friction"]),
            ("Errors", ["Speed Error", "Pos Term Speed", "Position Error deg"]),
            ("Force / Incline", ["F_combine", "Incline Deg Ist"]),
            ("UW", ["UW Angle SP", "UW Theta"]),
            ("Speed km/h", ["Setpoint Speed km/h", "Real Speed km/h"]),
            ("Input Electrical", ["Input Voltage", "Battery Current", "Power In"]),
            ("Ctrl SM Counts", ["Ctrl SM Still Cycles", "Ctrl SM Index Lost Cycles"]),
            ("Ctrl SM Reset Reason", ["Ctrl SM Reset Reason"]),
        ]

        pens = [
            pg.mkPen('c', width=1.5),
            pg.mkPen('y', width=1.5),
            pg.mkPen('g', width=1.5),
            pg.mkPen('m', width=1.5),
            pg.mkPen('r', width=1.5),
            pg.mkPen((255, 165, 0), width=1.5),
            pg.mkPen((180, 180, 255), width=1.5),
        ]

        for i, (title, keys) in enumerate(plot_defs):
            row = i // 2
            col = i % 2

            p = make_plot(self.plotWidget, row, col, title)
            self.chart_plots.append(p)
            if title == "Ctrl SM Reset Reason":
                p.setYRange(0.0, 3.0, padding=0.0)

            for j, key in enumerate(keys):
                self.chart_curves[key] = p.plot(
                    pen=pens[j % len(pens)],
                    name=key
                )

    def build_debug_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_debug)
        self.telemetryTable = QtWidgets.QTableWidget(); self.telemetryTable.setColumnCount(2); self.telemetryTable.setHorizontalHeaderLabels(["Variable", "Value"])
        self.telemetryTable.horizontalHeader().setStretchLastSection(True)
        self.debugText = QtWidgets.QPlainTextEdit(); self.debugText.setReadOnly(True)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical); splitter.addWidget(self.telemetryTable); splitter.addWidget(self.debugText); layout.addWidget(splitter)

    def make_connections(self):
        self.pushButton_refresh.clicked.connect(self.refresh_ports); self.pushButton_connect.clicked.connect(self.start_com); self.pushButton_disconnect.clicked.connect(self.stop_com)
        self.comboBox_can_id.currentIndexChanged.connect(self.can_id_changed); self.pushButton_scan_can.clicked.connect(self.scan_can_ids_clicked)
        self.start.clicked.connect(self.start_prog); self.stop.clicked.connect(self.stop_prog); self.Reset.clicked.connect(self.reset_prog)
        self.btn_set_runtime.clicked.connect(self.set_runtime_params_clicked); self.btn_set_bike_params.clicked.connect(self.set_bike_params_clicked); self.btn_set_control_params.clicked.connect(self.set_control_params_clicked)
        self.btn_autorange.clicked.connect(self.autorange_plots); self.btn_reset_x.clicked.connect(lambda: setattr(self, 'scroll_mode', True))

    def refresh_ports(self):
        self.comboBox_portselect.clear(); ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.comboBox_portselect.addItem("No ports found", None); log_event("Port refresh: no ports found"); return
        for port in ports:
            self.comboBox_portselect.addItem(f"{port.device} - {port.description}", port.device)
        log_event(f"Port refresh: found {len(ports)} port(s)")

    def start_com(self):
        idx = self.comboBox_portselect.currentIndex(); state.selected_port = self.comboBox_portselect.itemData(idx)
        if state.selected_port:
            with param_state_lock:
                param_state["pending_initial_refresh"] = True; param_state["ui_sync_needed"] = False; param_state["ui_sync_update_targets"] = False
            set_diag(selected_port=state.selected_port); log_event(f"Connect requested on {state.selected_port}"); state.vesc_com_flag.set()
        else:
            self.statusbar.showMessage("No valid serial port selected")

    def stop_com(self):
        state.vesc_com_flag.clear(); set_diag(serial_open=False); log_event("Communication stopped by user")

    def can_id_changed(self):
        can_id = self.comboBox_can_id.currentData()
        if can_id is not None:
            set_target_can_id(can_id); log_event(f"Target CAN ID set to {can_id}")

    def scan_can_ids_clicked(self):
        session = get_active_vesc_session()
        if session is None:
            self.statusbar.showMessage("No active VESC session"); return
        log_event("CAN scan started using COMM_PING_CAN")
        def worker():
            try:
                found = ping_can_ids(session)
            except Exception as e:
                found = []; log_event(f"CAN scan failed: {type(e).__name__}: {e}")
            QtCore.QTimer.singleShot(0, lambda: self.update_can_id_list(found))
        import threading; threading.Thread(target=worker, daemon=True).start()

    def update_can_id_list(self, found):
        old_id = get_target_can_id(); self.comboBox_can_id.blockSignals(True); self.comboBox_can_id.clear()
        ids = found if found else [old_id]
        for cid in ids: self.comboBox_can_id.addItem(f"CAN {cid}", cid)
        idx = ids.index(old_id) if old_id in ids else 0; self.comboBox_can_id.setCurrentIndex(idx)
        set_target_can_id(self.comboBox_can_id.currentData()); self.comboBox_can_id.blockSignals(False)
        log_event(f"CAN scan finished: found {found}, selected CAN ID {get_target_can_id()}")

    def start_prog(self):
        state.prog_flag.set(); log_event("Program started")

    def stop_prog(self):
        state.control_value = 0.0; state.control_mode = "Current"; state.prog_flag.clear()
        session = get_active_vesc_session()
        if session is not None:
            try: stop_bike_sim(session)
            except Exception as e: log_event(f"Stop command failed: {type(e).__name__}: {e}")
        log_event("Program stopped")

    def reset_prog(self):
        state.prog_flag.clear(); state.control_value = 0.0; state.control_mode = "Current"
        with vesc_history_lock: state.vesc_history.clear()
        with vesc_values_lock: state.vesc_values.clear()
        with sample_counter_lock: state.sample_counter = 0
        self.telemetryTable.setRowCount(0); log_event("Reset done")

    def apply_param_state_to_ui(self):
        with param_state_lock:
            runtime = dict(param_state["runtime"]); bike = dict(param_state["bike"]); control = dict(param_state["control"])
            needs_sync = bool(param_state["ui_sync_needed"]); update_targets = bool(param_state["ui_sync_update_targets"])
            param_state["ui_sync_needed"] = False; param_state["ui_sync_update_targets"] = False
        if not needs_sync: return
        if runtime:
            rows = [(self.runtime_gear_ratio,"gear_ratio_bike"),(self.runtime_incline_deg,"incline_deg"),(self.runtime_pumptrack_period,"pumptrack_period_min"),(self.runtime_pumptrack_enabled,"pumptrack_enabled"),(self.runtime_freewheel_enabled,"freewheel_enabled")]
            for row, key in rows:
                row.set_actual(runtime.get(key, 0.0));
                if update_targets: row.set_target(runtime.get(key, 0.0))
        for key, row in self.bike_param_rows.items():
            if key in bike: row.set_actual(bike[key]); row.set_target(bike[key]) if update_targets else None
        for key, row in self.control_param_rows.items():
            if key in control: row.set_actual(control[key]); row.set_target(control[key]) if update_targets else None

    def set_runtime_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetBikeRuntime(); msg.gear_ratio_bike = self.runtime_gear_ratio.get_target(); msg.incline_deg = self.runtime_incline_deg.get_target(); msg.pumptrack_enabled = int(self.runtime_pumptrack_enabled.get_target()); msg.freewheel_enabled = int(self.runtime_freewheel_enabled.get_target()); msg.pumptrack_period_min = self.runtime_pumptrack_period.get_target()
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Runtime parameters sent")
        except Exception as e: log_event(f"Set runtime failed: {type(e).__name__}: {e}")

    def set_bike_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetBikeSimParams()
            for key, row in self.bike_param_rows.items(): setattr(msg, key, row.get_target())
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Bike parameters sent")
        except Exception as e: log_event(f"Set bike params failed: {type(e).__name__}: {e}")

    def set_control_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetControlParams()
            for key, row in self.control_param_rows.items(): setattr(msg, key, row.get_target())
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Advanced parameters sent")
        except Exception as e: log_event(f"Set advanced params failed: {type(e).__name__}: {e}")

    def update_telemetry_table(self, values_dict):
        items = list(values_dict.items()); self.telemetryTable.setRowCount(len(items))
        ctrl_sm_state_labels = {0: "START", 1: "INDEX_FOUND", 2: "ENABLE"}
        for row, (key, value) in enumerate(items):
            self.telemetryTable.setItem(row, 0, QtWidgets.QTableWidgetItem(str(key)))
            if key == "Status Bits Ext" and isinstance(value, (int, float)):
                value_str = f"{int(value)} (0b{int(value):032b})"
            elif key == "Ctrl SM State" and isinstance(value, (int, float)):
                sm_state = int(value); value_str = f"{sm_state} ({ctrl_sm_state_labels.get(sm_state, 'UNKNOWN')})"
            else:
                value_str = f"{value:.6f}" if isinstance(value, float) else str(value)
            self.telemetryTable.setItem(row, 1, QtWidgets.QTableWidgetItem(value_str))

    def update_debug_view(self):
        d = get_diag_snapshot(); now = time.perf_counter(); last_rx = d.get("last_rx_time", 0.0); sample_age = (now - last_rx) if last_rx > 0 else float("inf")
        lines = ["=== Diagnostics ===", f"serial_open = {d.get('serial_open')}", f"selected_port = {d.get('selected_port')}", f"target_can_id = {get_target_can_id()}", f"sample_age_s = {sample_age:.3f}", f"rx_count = {d.get('rx_count')}", f"tx_count = {d.get('tx_count')}", f"read_errors = {d.get('read_errors')}", f"last_comm_error = {d.get('last_comm_error')}", "", "=== Last response ===", str(d.get('last_response_summary')), "", "=== Recent events ==="]
        lines.extend(d.get("event_log", [])); lines.append(""); lines.append("=== Custom message tests ===")
        for item in get_custom_msg_results(): lines.append(f"[{item['t']}] {item['name']} -> {'OK' if item['ok'] else 'FAIL'} -> {item['detail']}")
        self.debugText.setPlainText("\n".join(lines))

    def autorange_plots(self):
        for p in self.chart_plots + [self.main_plot_torque, self.main_plot_speed]: p.autoRange()

    def refresh_live_data(self):
        try:
            with vesc_values_lock: local_values = dict(state.vesc_values)
            d = get_diag_snapshot(); now = time.perf_counter(); last_rx = d.get("last_rx_time", 0.0); sample_age = (now - last_rx) if last_rx > 0 else float("inf")
            requested = state.vesc_com_flag.is_set(); serial_open = d.get("serial_open", False); connected = requested and serial_open and sample_age < STALE_WARNING_S
            self.flag_connected.set_state(connected); self.label_link_state.setText("LINK: OK" if connected else ("LINK: STALE" if serial_open else "LINK: IDLE"))
            self.row_sample_age.set_value(f"{sample_age:.3f}" if math.isfinite(sample_age) else "inf"); self.row_rx_count.set_value(str(d.get("rx_count", 0))); self.row_read_errors.set_value(str(d.get("read_errors", 0)))
            if local_values:
                self.row_real_speed.set_value(f"{local_values.get('Real Speed km/h', 0):.2f}"); self.row_incline.set_value(f"{local_values.get('Incline Deg Ist', 0):.2f}"); self.row_gear_ratio.set_value(f"{local_values.get('Gear Ratio', 0):.3f}"); self.row_power.set_value(f"{local_values.get('Power In', 0):.1f}")
                self.flag_ctrl_active.set_state(bool(local_values.get("Ctrl Active", 0))); self.flag_forced_fw.set_state(bool(local_values.get("Forced FW", 0))); self.flag_start.set_state(bool(local_values.get("START", 0))); self.flag_index.set_state(bool(local_values.get("INDEX_FOUND", 0))); self.flag_enable.set_state(bool(local_values.get("ENABLE", 0)))
                self.update_telemetry_table(local_values); self.refresh_plots()
            self.apply_param_state_to_ui(); self.update_debug_view()
        except Exception:
            inc_diag("gui_errors"); set_diag(last_gui_error=traceback.format_exc()); log_event("GUI refresh error")

    def refresh_plots(self):
        with vesc_history_lock:
            if "time_s" not in state.vesc_history or not state.vesc_history["time_s"]: return
            hist = {k: v[:] for k, v in state.vesc_history.items()}
        t = hist.get("time_s", []); x_full = [ti - t[0] for ti in t]; x_last = x_full[-1]
        def xy(key, visible=True):
            y = hist.get(key, []); n = min(len(x_full), len(y)); x = x_full[:n]; y = y[:n]
            if visible:
                x_min = max(0.0, x_last - PLOT_SCROLL_WINDOW_S); lo = next((i for i, xv in enumerate(x) if xv >= x_min), len(x)); x = x[lo:]; y = y[lo:]
            return x, y
        for key, curve in [("Pedal Torque Observed", self.main_curve_torque), ("Real Speed km/h", self.main_curve_speed)]:
            x, y = xy(key); curve.setData(x, y)
        self.main_plot_torque.setXRange(max(0.0, x_last - PLOT_SCROLL_WINDOW_S), max(PLOT_SCROLL_WINDOW_S, x_last), padding=0.0)
        for key, curve in self.chart_curves.items():
            x, y = xy(key, self.scroll_mode); curve.setData(x, y)
        if self.scroll_mode:
            for p in self.chart_plots: p.setXRange(max(0.0, x_last - PLOT_SCROLL_WINDOW_S), max(PLOT_SCROLL_WINDOW_S, x_last), padding=0.0)
