# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


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
        font_label = QtGui.QFont(); font_label.setPointSize(9); font_label.setBold(True)
        font_edit = QtGui.QFont(); font_edit.setPointSize(9)
        self.label.setFont(font_label); self.edit.setFont(font_edit); self.unit.setFont(font_label)
        self.label.setMinimumWidth(75); self.edit.setMinimumWidth(72); self.edit.setMaximumWidth(95); self.edit.setMinimumHeight(22); self.unit.setMinimumWidth(35)
        layout.addWidget(self.label, 0, 0); layout.addWidget(self.edit, 0, 1); layout.addWidget(self.unit, 0, 2)

    def set_value(self, value):
        self.edit.setText(str(value))


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
        layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(4)
        self.flag = FlagIndicator(15)
        self.label = QtWidgets.QLabel(text)
        font = QtGui.QFont(); font.setPointSize(9)
        self.label.setFont(font)
        layout.addWidget(self.flag); layout.addWidget(self.label)

    def set_state(self, active):
        self.flag.set_active(active)


class ParamEditRow(QtWidgets.QWidget):
    def __init__(self, label_text, is_bool=False):
        super().__init__()
        self.is_bool = is_bool
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setHorizontalSpacing(8); layout.setVerticalSpacing(2)
        self.label = QtWidgets.QLabel(label_text); self.label.setMinimumWidth(220)
        if is_bool:
            self.edit_target = QtWidgets.QCheckBox()
            self.edit_actual = QtWidgets.QCheckBox(); self.edit_actual.setEnabled(False)
        else:
            self.edit_target = QtWidgets.QLineEdit()
            self.edit_actual = QtWidgets.QLineEdit(); self.edit_actual.setReadOnly(True)
            self.edit_target.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.edit_actual.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.edit_target.setMinimumWidth(90); self.edit_target.setMaximumWidth(110)
            self.edit_actual.setMinimumWidth(90); self.edit_actual.setMaximumWidth(110)
        layout.addWidget(self.label, 0, 0); layout.addWidget(self.edit_target, 0, 1); layout.addWidget(self.edit_actual, 0, 2)

    def set_actual(self, value):
        if self.is_bool:
            self.edit_actual.setChecked(bool(value))
        else:
            self.edit_actual.setText(f"{float(value):.6f}")

    def set_target(self, value):
        if self.is_bool:
            self.edit_target.setChecked(bool(value))
        else:
            self.edit_target.setText(f"{float(value):.6f}")

    def get_target(self):
        if self.is_bool:
            return bool(self.edit_target.isChecked())
        return float(self.edit_target.text().strip())


def make_plot(parent_widget, row, col, title):
    axis_pen = pg.mkPen((180, 180, 180)); text_pen = (220, 220, 220)
    plot = parent_widget.addPlot(row=row, col=col, title=title)
    plot.showGrid(x=True, y=True, alpha=0.20)
    plot.getAxis('left').setPen(axis_pen); plot.getAxis('bottom').setPen(axis_pen)
    plot.getAxis('left').setTextPen(text_pen); plot.getAxis('bottom').setTextPen(text_pen)
    plot.getAxis('bottom').setLabel("t", units="s")
    plot.addLegend(offset=(5, 5))
    plot.setMouseEnabled(x=True, y=True)
    plot.getViewBox().setMouseMode(plot.getViewBox().RectMode)
    return plot
