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
from PyQt5.Qt import QTextOption
from PyQt5.QtCore import (pyqtSignal, pyqtSlot, Q_ARG, QAbstractItemModel,
                          QFileInfo, qFuzzyCompare, QMetaObject, QModelIndex, QObject, Qt,
                          QThread, QTime, QUrl, QSize, QEvent, QCoreApplication)
from PyQt5.QtGui import QColor, qGray, QImage, QPainter, QPalette, QIcon, QKeyEvent, QMouseEvent
from PyQt5.QtMultimedia import (QAbstractVideoBuffer, QMediaContent,
                                QMediaMetaData, QMediaPlayer, QMediaPlaylist, QVideoFrame, QVideoProbe)
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QMainWindow, QDoubleSpinBox, QApplication, QComboBox, QDialog, QFileDialog,
                             QFormLayout, QHBoxLayout, QLabel, QListView, QMessageBox, QPushButton,
                             QSizePolicy, QSlider, QStyle, QToolButton, QVBoxLayout, QWidget, QLineEdit, QPlainTextEdit,
                             QTableWidget, QTableWidgetItem, QSplitter, QAbstractItemView, QStyledItemDelegate, QHeaderView, QFrame, QProgressBar, QCheckBox, QToolTip, QGridLayout)
from io import TextIOWrapper

import _thorlabs_kst_wrap_basic as tlkt
import picoammeter as pico
import math as m

import datetime as dt

MM_TO_IDX = 2184560.64
# NM_TO_MM = 0

# def dcos(deg):
#     return m.degrees((m.cos(m.radians(deg))))

# def nm_to_idx(pos_nm):
#     order = 1
#     zero_order_offset = 1
#     L = 550
#     grating_density = 0.0012
#     dX = pos_nm
#     a = ((2) * (1 / grating_density) * dcos(32) * ((dX + zero_order_offset)/(L)) * (10e6)) / (order)
#     return a * MM_TO_IDX

# Imports .ui file.
class Ui(QMainWindow):
    prefix = ''
    start = 0
    stop = 0
    step = 0

    pa = None
    motor_ctrl = None

    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi("mmc_early_gui.ui", self)
        self.setWindowTitle("MMC Early GUI")

        #  Picoammeter init.
        self.pa = pico.Picoammeter()

        #  KST101 init.
        serials = serials = tlkt.Thorlabs.ListDevicesAny()
        self.motor_ctrl = tlkt.Thorlabs.KST101(serials[0])
        if (self.motor_ctrl._CheckConnection() == False):
            raise RuntimeError('Connection with motor controller failed.')
        self.motor_ctrl.set_stage('ZST25')

        # Move to 1mm (0nm)
        self.motor_ctrl.move_to(1 * MM_TO_IDX, True)
        
        # GUI init.
        self.scan_button = self.findChild(QPushButton, "pushbutton_scan")
        self.prefix_box = self.findChild(QLineEdit, "lineedit_prefix")
        self.start_spin = self.findChild(QDoubleSpinBox, "spinbox_start")
        self.stop_spin = self.findChild(QDoubleSpinBox, "spinbox_stop")
        self.step_spin = self.findChild(QDoubleSpinBox, "spinbox_step")

        self.scan_button.clicked.connect(self.button_pressed)
        self.prefix_box.editingFinished.connect(self.prefix_changed)
        self.start_spin.valueChanged.connect(self.start_changed)
        self.stop_spin.valueChanged.connect(self.stop_changed)
        self.step_spin.valueChanged.connect(self.step_changed)

        self.show()

    def button_pressed(self):
        print("Button pressed!")
        self.initiate_scan()

    def prefix_changed(self):
        print("Prefix changed to: %s"%(self.prefix_box.text()))
        self.prefix = self.prefix_box.text()

    def start_changed(self):
        print("Start changed to: %s mm"%(self.start_spin.value()))
        self.start = self.start_spin.value() * MM_TO_IDX

    def stop_changed(self):
        print("Stop changed to: %s mm"%(self.stop_spin.value()))
        self.stop = self.stop_spin.value() * MM_TO_IDX

    def step_changed(self):
        print("Step changed to: %s mm"%(self.step_spin.value()))
        self.step = self.step_spin.value() * MM_TO_IDX

    def initiate_scan(self):
        tnow = dt.datetime.now()
        self.sav_file = open(self.prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv", 'w')
        print(type(self.sav_file))
        # Move to start and collect data.
        self.motor_ctrl.move_to(self.start, True)
        self.sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
        self.sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
        self.sav_file.write('# Position (step),Current(A),Timestamp,Error Code\n')
        self.sav_file.write(self.pa.sample_data(10, self.motor_ctrl.get_position()))
        print(self.start, self.stop, self.step)
        if self.step > 0:
            while self.motor_ctrl.get_position() < self.stop:
                self.motor_ctrl.move_by(self.step, True)
                pos = self.motor_ctrl.get_position()
                self.sav_file.write(self.pa.sample_data(10, pos))

        else:
            while self.motor_ctrl.get_position() > self.stop:
                self.motor_ctrl.move_by(self.step, True)
                self.sav_file.write(self.pa.sample_data(10, pos))
        self.sav_file.close()
                
# Main function.
if __name__ == '__main__':
    import sys
    application = QApplication(sys.argv)

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
    ret = application.exec_() # block until
    sys.exit(ret)