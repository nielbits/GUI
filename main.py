# -*- coding: utf-8 -*-

import sys
import threading
from PyQt5 import QtWidgets
from vesc_comm import vesc_communication
from gui import Ui_MainWindow


if __name__ == "__main__":
    communication_thread = threading.Thread(target=vesc_communication, daemon=True)
    communication_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
