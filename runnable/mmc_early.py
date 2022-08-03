#
# @file mmc_early.py
# @author Mit Bailey (mitbailey@outlook.com)
# @brief An early, basic version of the MMC GUI and program.
# @version See Git tags for version information.
# @date 2022.08.03
# 
# @copyright Copyright (c) 2022
# 
#

from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets
# from PyQt5.QtCore import (pyqtSignal, pyqtSlot, Q_ARG, QAbstractItemModel, QFileInfo, qFuzzyCompare, QMetaObject, 
#         QModelIndex, QObject, Qt, QThread, QTime, QUrl)
# from PyQt5.QtGui import QColor, qGray, QImage, QPainter, QPalette
from PyQt5.QtMultimedia import (QAbstractVideoBuffer, QMediaContent, QMediaMetaData, QMediaPlayer, QMediaPlaylist, 
        QVideoFrame, QVideoProbe)
from PyQt5.QtMultimediaWidgets import QVideoWidget
# from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog, QFileDialog, QFormLayout, QHBoxLayout, QLabel, QListView,
        # QMessageBox, QPushButton, QSizePolicy, QSlider, QStyle, QToolButton, QVBoxLayout, QWidget, QMainWindow)
# import runnable._thorlabs_kst_wrap_basic
import mmc_early_mid

# Imports .ui file.
class Ui(QtWidgets.QMainWindow):
    prefix = ''
    start = 0
    stop = 0
    step = 0

    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi("mmc_early_gui.ui", self)
        self.setWindowTitle("MMC Early GUI")

        self.scan_button = self.findChild(QtWidgets.QPushButton, "pushbutton_scan")
        self.prefix_box = self.findChild(QtWidgets.QLineEdit, "lineedit_prefix")
        self.start_spin = self.findChild(QtWidgets.QDoubleSpinBox, "spinbox_start")
        self.stop_spin = self.findChild(QtWidgets.QDoubleSpinBox, "spinbox_stop")
        self.step_spin = self.findChild(QtWidgets.QDoubleSpinBox, "spinbox_step")

        self.scan_button.clicked.connect(self.button_pressed)
        self.prefix_box.editingFinished.connect(self.prefix_changed)
        self.start_spin.valueChanged.connect(self.start_changed)

        self.show()

    def button_pressed(self):
        print("Button pressed!")
        self.initiate_scan()

    def prefix_changed(self):
        print("Prefix changed to: %s"%(self.prefix_box.text()))
        self.prefix = self.prefix_box.text()

    def start_changed(self):
        print("Start changed to: %s nm"%(self.start_spin.value()))
        self.start = self.start_spin.value()

    def stop_changed(self):
        print("Stop changed to: %s nm"%(self.stop_spin.value()))
        self.stop = self.stop_spin.value()

    def step_changed(self):
        print("Step changed to: %s nm"%(self.step_spin.value()))
        self.step = self.step_spin.value()

    def initiate_scan(self):
        mmc_early_mid.begin_scan(self.start, self.stop, self.step)


# Main function.
if __name__ == '__main__':
    import sys
    application = QtWidgets.QApplication(sys.argv)

    # Initializes the GUI.
    mainWindow = Ui()
    
    # Example: Creating a new instrument. Should be done in a UI callback of some sort.
     # new_mono = instruments.Monochromator(241.0536, 32, 1)

    # Example: Getting a UI element from the .ui file, setting LCDNumber value.
     # lcd_milli = mainWindow.findChild(QtWidgets.QLCDNumber, "lcdNumber")
     # lcd_nano = mainWindow.findChild(QtWidgets.QLCDNumber, "lcdNumber_2")
     # lcd_milli.display(1)
     # lcd_nano.display(2)
    
    # Wait for the Qt loop to exit before exiting.
    sys.exit(application.exec_())