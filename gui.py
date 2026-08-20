# -*- coding: utf-8 -*-

import json
import math
import sys
import threading
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


CHART_PLOT_DEFS = [
    ("Currents", ["IQ Filtered", "IQ Set", "IQ Instant"]),
    ("RPMs", ["RPM Motor", "RPM Set", "LESO RPM"]),
    ("Torques", ["Torque FF", "Torque Motor", "Pedal Torque Observed", "T_F_combine", "T_friction"]),
    ("Errors", ["Speed Error", "Pos Term Speed", "Position Error deg"]),
    ("Force / Incline", ["F_combine", "Incline Deg Ist"]),
    ("UW", ["UW Angle SP", "UW Theta"]),
    ("Speed km/h", ["Setpoint Speed km/h", "Real Speed km/h"]),
    ("Input Electrical", ["Input Voltage", "Battery Current", "Power In"]),
   # ("Ctrl SM Counts", ["Ctrl SM Still Cycles", "Ctrl SM Index Lost Cycles"]),
   # ("Ctrl SM Reset Reason", ["Ctrl SM Reset Reason"]),
]

MAIN_PLOT_DEFS = [
    ("Observer Torque", ["Pedal Torque Observed"]),
    ("Real Speed", ["Real Speed km/h"]),
]


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 800)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)
        self.chart_plots = []
        self.chart_curves = {}
        self.chart_plot_widgets = {}
        self.chart_config_items = {}
        self.chart_selections = {}
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
        self.btn_dump_charts = QtWidgets.QPushButton("Dump Charts")
        toolbar.addWidget(self.btn_autorange)
        toolbar.addWidget(self.btn_reset_x)
        toolbar.addWidget(self.btn_dump_charts)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.chart_tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.chart_tabs, 1)

        self.tab_chart1 = QtWidgets.QWidget()
        self.tab_chart2 = QtWidgets.QWidget()
        self.tab_chart_config = QtWidgets.QWidget()
        self.chart_tabs.addTab(self.tab_chart1, "Chart 1")
        self.chart_tabs.addTab(self.tab_chart2, "Chart 2")
        self.chart_tabs.addTab(self.tab_chart_config, "Configure")

        for tab_index, tab_widget in [(0, self.tab_chart1), (1, self.tab_chart2)]:
            tab_layout = QtWidgets.QVBoxLayout(tab_widget)
            tab_layout.setContentsMargins(0, 0, 0, 0)
            plot_widget = pg.GraphicsLayoutWidget()
            plot_widget.setBackground('k')
            self.chart_plot_widgets[tab_index] = plot_widget
            tab_layout.addWidget(plot_widget, 1)

        split_at = (len(CHART_PLOT_DEFS) + 1) // 2
        self.chart_selections = {
            0: {title for i, (title, _) in enumerate(CHART_PLOT_DEFS) if i < split_at},
            1: {title for i, (title, _) in enumerate(CHART_PLOT_DEFS) if i >= split_at},
        }
        self.build_chart_config_tab()
        self.rebuild_chart_tabs()

    def build_chart_config_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_chart_config)

        button_row = QtWidgets.QHBoxLayout()
        self.btn_chart1_all = QtWidgets.QPushButton("Select All Chart 1")
        self.btn_chart1_clear = QtWidgets.QPushButton("Clear Chart 1")
        self.btn_chart2_all = QtWidgets.QPushButton("Select All Chart 2")
        self.btn_chart2_clear = QtWidgets.QPushButton("Clear Chart 2")
        for button in [self.btn_chart1_all, self.btn_chart1_clear, self.btn_chart2_all, self.btn_chart2_clear]:
            button_row.addWidget(button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self.chart_config_table = QtWidgets.QTableWidget()
        self.chart_config_table.setColumnCount(4)
        self.chart_config_table.setHorizontalHeaderLabels(["Chart", "Signals", "Chart 1", "Chart 2"])
        self.chart_config_table.setRowCount(len(CHART_PLOT_DEFS))
        self.chart_config_table.setAlternatingRowColors(True)
        self.chart_config_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.chart_config_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.chart_config_table.verticalHeader().setVisible(False)

        header = self.chart_config_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)

        for row, (title, keys) in enumerate(CHART_PLOT_DEFS):
            title_item = QtWidgets.QTableWidgetItem(title)
            title_item.setFlags(title_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.chart_config_table.setItem(row, 0, title_item)

            signals_item = QtWidgets.QTableWidgetItem(", ".join(keys))
            signals_item.setFlags(signals_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.chart_config_table.setItem(row, 1, signals_item)

            for col, tab_index in [(2, 0), (3, 1)]:
                item = QtWidgets.QTableWidgetItem()
                item.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                item.setCheckState(QtCore.Qt.Checked if title in self.chart_selections[tab_index] else QtCore.Qt.Unchecked)
                item.setData(QtCore.Qt.UserRole, title)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.chart_config_table.setItem(row, col, item)
                self.chart_config_items[(tab_index, title)] = item

        layout.addWidget(self.chart_config_table, 1)

    def chart_pens(self):
        return [
            pg.mkPen('c', width=1.5),
            pg.mkPen('y', width=1.5),
            pg.mkPen('g', width=1.5),
            pg.mkPen('m', width=1.5),
            pg.mkPen('r', width=1.5),
            pg.mkPen((255, 165, 0), width=1.5),
            pg.mkPen((180, 180, 255), width=1.5),
        ]

    def selected_chart_defs(self, tab_index):
        selected_titles = self.chart_selections.get(tab_index, set())
        return [(title, keys) for title, keys in CHART_PLOT_DEFS if title in selected_titles]

    def chart_grid_position(self, index, count):
        if count <= 2:
            return index, 0, 1
        if count % 2 == 1 and index == count - 1:
            return index // 2, 0, 2
        return index // 2, index % 2, 1

    def rebuild_chart_tabs(self):
        self.chart_plots = []
        self.chart_curves = {}
        pens = self.chart_pens()

        for tab_index, plot_widget in self.chart_plot_widgets.items():
            plot_widget.clear()
            plot_defs = self.selected_chart_defs(tab_index)
            if not plot_defs:
                label = pg.LabelItem("No charts selected", color=(220, 220, 220), size="14pt")
                plot_widget.addItem(label, row=0, col=0)
                continue

            first_plot = None
            for i, (title, keys) in enumerate(plot_defs):
                row, col, colspan = self.chart_grid_position(i, len(plot_defs))
                plot = make_plot(plot_widget, row, col, title, colspan=colspan)
                self.chart_plots.append(plot)
                if first_plot is None:
                    first_plot = plot
                else:
                    plot.setXLink(first_plot)
                if title == "Ctrl SM Reset Reason":
                    plot.setYRange(0.0, 3.0, padding=0.0)

                for j, key in enumerate(keys):
                    curve = plot.plot(
                        pen=pens[j % len(pens)],
                        name=key
                    )
                    self.chart_curves.setdefault(key, []).append(curve)

        self.refresh_plots()

    def chart_config_item_changed(self, item):
        if item.column() not in (2, 3):
            return

        title = item.data(QtCore.Qt.UserRole)
        tab_index = item.column() - 2
        if item.checkState() == QtCore.Qt.Checked:
            self.chart_selections[tab_index].add(title)
        else:
            self.chart_selections[tab_index].discard(title)
        self.rebuild_chart_tabs()

    def set_chart_selection(self, tab_index, checked):
        self.chart_config_table.blockSignals(True)
        selected_titles = self.chart_selections[tab_index]
        selected_titles.clear()
        if checked:
            selected_titles.update(title for title, _ in CHART_PLOT_DEFS)

        check_state = QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked
        for title, _ in CHART_PLOT_DEFS:
            self.chart_config_items[(tab_index, title)].setCheckState(check_state)
        self.chart_config_table.blockSignals(False)
        self.rebuild_chart_tabs()

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
        self.btn_dump_charts.clicked.connect(self.dump_charts_clicked)
        self.chart_config_table.itemChanged.connect(self.chart_config_item_changed)
        self.btn_chart1_all.clicked.connect(lambda: self.set_chart_selection(0, True))
        self.btn_chart1_clear.clicked.connect(lambda: self.set_chart_selection(0, False))
        self.btn_chart2_all.clicked.connect(lambda: self.set_chart_selection(1, True))
        self.btn_chart2_clear.clicked.connect(lambda: self.set_chart_selection(1, False))

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

    def queue_param_readback(self, session, label):
        def worker():
            try:
                read_param_blocks_from_session(session, update_targets=False)
                log_event(f"{label} readback done")
            except Exception as e:
                log_event(f"{label} readback failed: {type(e).__name__}: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def set_runtime_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetBikeRuntime(); msg.gear_ratio_bike = self.runtime_gear_ratio.get_target(); msg.p_incline_deg = self.runtime_incline_deg.get_target(); msg.pumptrack_enabled = int(self.runtime_pumptrack_enabled.get_target()); msg.freewheel_enabled = int(self.runtime_freewheel_enabled.get_target()); msg.pumptrack_period_min = self.runtime_pumptrack_period.get_target()
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Runtime parameters sent"); self.queue_param_readback(session, "Runtime parameters")
        except Exception as e: log_event(f"Set runtime failed: {type(e).__name__}: {e}")

    def set_bike_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetBikeSimParams()
            for key, row in self.bike_param_rows.items(): setattr(msg, key, row.get_target())
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Bike parameters sent"); self.queue_param_readback(session, "Bike parameters")
        except Exception as e: log_event(f"Set bike params failed: {type(e).__name__}: {e}")

    def set_control_params_clicked(self):
        session = get_active_vesc_session()
        if session is None: return
        try:
            msg = SetControlParams()
            for key, row in self.control_param_rows.items(): setattr(msg, key, row.get_target())
            session.send_custom_no_reply(fwd_msg(msg)); log_event("Advanced parameters sent"); self.queue_param_readback(session, "Advanced parameters")
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

    def build_chart_dump(self):
        with vesc_history_lock:
            hist = {k: v[:] for k, v in state.vesc_history.items()}
        with vesc_values_lock:
            latest_values = dict(state.vesc_values)

        t = hist.get("time_s", [])
        relative_time_s = [ti - t[0] for ti in t] if t else []
        chart_keys = []
        for _, keys in MAIN_PLOT_DEFS + CHART_PLOT_DEFS:
            chart_keys.extend(keys)
        chart_keys = sorted(set(chart_keys))

        series = {}
        for key in chart_keys:
            values = hist.get(key, [])
            n = min(len(relative_time_s), len(values))
            series[key] = {
                "time_s": relative_time_s[:n],
                "values": values[:n],
            }

        def chart_entries(plot_defs):
            entries = []
            for i, (title, keys) in enumerate(plot_defs):
                row, col, colspan = self.chart_grid_position(i, len(plot_defs))
                entries.append({
                    "title": title,
                    "row": row,
                    "column": col,
                    "column_span": colspan,
                    "signals": list(keys),
                })
            return entries

        return {
            "generated_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            "target_can_id": get_target_can_id(),
            "plot_scroll_window_s": PLOT_SCROLL_WINDOW_S,
            "scroll_mode": bool(self.scroll_mode),
            "main_charts": chart_entries(MAIN_PLOT_DEFS),
            "charts": chart_entries(CHART_PLOT_DEFS),
            "chart_tabs": {
                "chart_1": chart_entries(self.selected_chart_defs(0)),
                "chart_2": chart_entries(self.selected_chart_defs(1)),
            },
            "latest_values": latest_values,
            "diagnostics": get_diag_snapshot(),
            "history_sample_count": len(relative_time_s),
            "series": series,
        }

    def dump_charts_clicked(self):
        filename = time.strftime("charts_dump_%Y%m%d_%H%M%S.json")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.tab_charts,
            "Dump Charts",
            filename,
            "JSON Files (*.json);;All Files (*)",
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.build_chart_dump(), f, indent=2, allow_nan=True)
            self.statusbar.showMessage(f"Charts dumped to {path}", 8000)
            log_event(f"Charts dumped to {path}")
        except Exception as e:
            self.statusbar.showMessage(f"Chart dump failed: {e}", 8000)
            log_event(f"Chart dump failed: {type(e).__name__}: {e}")

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
        for key, curves in self.chart_curves.items():
            x, y = xy(key, self.scroll_mode)
            for curve in curves:
                curve.setData(x, y)
        if self.scroll_mode:
            for p in self.chart_plots: p.setXRange(max(0.0, x_last - PLOT_SCROLL_WINDOW_S), max(PLOT_SCROLL_WINDOW_S, x_last), padding=0.0)
