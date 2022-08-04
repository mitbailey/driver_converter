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
import os

import datetime as dt

import asyncio

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
    manual_prefix = 'manual'
    auto_prefix = 'automatic'
    manual_dir = './data'
    auto_dir = './data'
    start = 0
    stop = 0
    step = 0
    save_data = False

    manual_position = 0

    pa = None
    motor_ctrl = None

    def __init__(self, application):
        self.application = application

        super(Ui, self).__init__()
        uic.loadUi("mainwindow.ui", self)
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
        # self.motor_ctrl.move_to(1 * MM_TO_IDX, True)
        
        # GUI init.
        self.scan_button = self.findChild(QPushButton, "pushButton_3")
        self.save_data_checkbox = self.findChild(QCheckBox, "checkBox_4")
        self.auto_prefix_box = self.findChild(QLineEdit, "lineEdit_14")
        self.manual_prefix_box = self.findChild(QLineEdit, "lineEdit_10")
        self.auto_dir_box = self.findChild(QLineEdit, "lineEdit_7")
        self.manual_dir_box = self.findChild(QLineEdit, "lineEdit_9")
        self.start_spin = self.findChild(QDoubleSpinBox, "doubleSpinBox_7")
        self.stop_spin = self.findChild(QDoubleSpinBox, "doubleSpinBox_8")
        self.step_spin = self.findChild(QDoubleSpinBox, "doubleSpinBox_9")
        self.currpos_mm_disp = self.findChild(QLabel, "position_nm_value_label")
        self.currpos_steps_disp = self.findChild(QLabel, "position_steps_value_label")
        self.scan_status = self.findChild(QLabel, "scan_status_value_label")
        self.scan_progress = self.findChild(QProgressBar, "scan_status_progress_bar")
        self.pos_spin = self.findChild(QDoubleSpinBox, "doubleSpinBox_5")
        self.move_to_position_button = self.findChild(QPushButton, "pushButton_12")
        self.collect_data = self.findChild(QPushButton, "pushButton_13")

        self.manual_prefix_box.setText(self.manual_prefix)
        self.auto_prefix_box.setText(self.auto_prefix)
        self.manual_dir_box.setText(self.manual_dir)
        self.auto_dir_box.setText(self.auto_dir)

        self.scan_button.clicked.connect(self.scan_button_pressed)
        self.collect_data.clicked.connect(self.manual_collect_button_pressed)
        self.move_to_position_button.clicked.connect(self.move_to_position_button_pressed)
        self.save_data_checkbox.stateChanged.connect(self.save_checkbox_toggled)
        self.auto_prefix_box.editingFinished.connect(self.auto_prefix_changed)
        self.manual_prefix_box.editingFinished.connect(self.manual_prefix_changed)
        self.auto_dir_box.editingFinished.connect(self.auto_dir_changed)
        self.manual_dir_box.editingFinished.connect(self.manual_dir_changed)
        self.start_spin.valueChanged.connect(self.start_changed)
        self.stop_spin.valueChanged.connect(self.stop_changed)
        self.step_spin.valueChanged.connect(self.step_changed)
        self.pos_spin.valueChanged.connect(self.manual_pos_changed)

        self.show()

    def scan_button_pressed(self):
        print("Scan button pressed!")
        self.initiate_scan()
        # asyncio.run(self.initiate_scan())

    def manual_collect_button_pressed(self):
        print("Manual collect button pressed!")
        self.take_data()

    def move_to_position_button_pressed(self):
        print("Move to position button pressed!")
        self.motor_ctrl.move_to(self.manual_position, True)

    def save_checkbox_toggled(self):
        print("Save checkbox toggled.")
        self.save_data = not self.save_data

    # def prefix_changed(self):
    #     print("Prefix changed to: %s"%(self.prefix_box.text()))
    #     self.prefix = self.prefix_box.text()

    def manual_prefix_changed(self):
        print("Prefix changed to: %s"%(self.manual_prefix_box.text()))
        self.manual_prefix = self.manual_prefix_box.text()

    def auto_prefix_changed(self):
        print("Prefix changed to: %s"%(self.auto_prefix_box.text()))
        self.auto_prefix = self.auto_prefix_box.text()

    def manual_dir_changed(self):
        print("Prefix changed to: %s"%(self.manual_dir_box.text()))
        self.manual_dir = self.manual_dir_box.text()

    def auto_dir_changed(self):
        print("Prefix changed to: %s"%(self.auto_dir_box.text()))
        self.auto_dir = self.auto_dir_box.text()

    def start_changed(self):
        print("Start changed to: %s mm"%(self.start_spin.value()))
        self.start = self.start_spin.value() * MM_TO_IDX

    def stop_changed(self):
        print("Stop changed to: %s mm"%(self.stop_spin.value()))
        self.stop = self.stop_spin.value() * MM_TO_IDX

    def step_changed(self):
        print("Step changed to: %s mm"%(self.step_spin.value()))
        self.step = self.step_spin.value() * MM_TO_IDX

    def manual_pos_changed(self):
        print("Manual position changed to: %s mm"%(self.pos_spin.value()))
        self.manual_position = self.pos_spin.value() * MM_TO_IDX

    def take_data(self):

        tnow = dt.datetime.now()

        filename = self.manual_dir + '/' + self.manual_prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv"
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        self.sav_file = open(filename, 'w')
        
        self.sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
        self.sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
        self.sav_file.write('# Position (step),Current(A),Timestamp,Error Code\n')
        self.sav_file.write(self.pa.sample_data(10, self.motor_ctrl.get_position()))

        self.sav_file.close()

    # TODO: Use QThreads to prevent freezing GUI. For now, `self.application.processEvents()`.
    # TODO: (cont.): https://www.xingyulei.com/post/qt-threading/
    async def initiate_scan(self):
        print("Save to file? " + str(self.save_data))

        self.scan_status.setText("PREPARING")
        self.scan_progress.setMinimum(self.start)
        self.scan_progress.setMaximum(self.stop)
        self.application.processEvents()

        if (self.save_data):
            tnow = dt.datetime.now()
            
            filename = self.auto_dir + '/' + self.auto_prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            self.sav_file = open(filename, 'w')

            # print(type(self.sav_file))
        # Move to start and collect data.
        self.scan_status.setText("MOVING")
        self.application.processEvents()
        self.motor_ctrl.move_to(self.start, True)
        
        self.scan_status.setText("SAMPLING")
        self.application.processEvents()
        if (self.save_data):
            self.sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
            self.sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
            self.sav_file.write('# Position (step),Current(A),Timestamp,Error Code\n')
            # pos = self.motor_ctrl.get_position()
            self.sav_file.write(self.pa.sample_data(10, self.motor_ctrl.get_position()))

            # self.scan_progress.setValue(pos)
            # self.application.processEvents()

        else:
            print(self.pa.sample_data(10, self.motor_ctrl.get_position()))
        print(self.start, self.stop, self.step)
        if self.step > 0:
            while self.motor_ctrl.get_position() < self.stop:
                self.scan_status.setText("MOVING")
                self.application.processEvents()
                self.motor_ctrl.move_by(self.step, True)
                pos = self.motor_ctrl.get_position()
                self.scan_status.setText("SAMPLING")
                self.application.processEvents()
                if (self.save_data):
                    self.sav_file.write(self.pa.sample_data(10, pos))
                else:
                    print(self.pa.sample_data(10, pos))

                self.scan_progress.setValue(pos)
                self.application.processEvents()

        else:
            while self.motor_ctrl.get_position() > self.stop:
                self.scan_status.setText("MOVING")
                self.application.processEvents()
                self.motor_ctrl.move_by(self.step, True)
                self.scan_status.setText("SAMPLING")
                self.application.processEvents()
                if (self.save_data):
                    self.sav_file.write(self.pa.sample_data(10, pos))
                else:
                    print(self.pa.sample_data(10, pos))

                self.scan_progress.setValue(pos)
                self.application.processEvents()

        if (self.save_data):
            self.sav_file.close()
        self.scan_status.setText("IDLE")
        self.scan_progress.reset()
        self.scan_progress.setMinimum(0)
        self.scan_progress.setMaximum(100)
        self.application.processEvents()
                
# Main function.
if __name__ == '__main__':
    import sys
    application = QApplication(sys.argv)

    # Initializes the GUI.
    mainWindow = Ui(application)
    
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