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

# %% Set up paths

import os
import sys

try:
    exeDir = sys._MEIPASS
except Exception:
    exeDir = os.getcwd()

if getattr(sys, 'frozen', False):
    appDir = os.path.dirname(sys.executable)
elif __file__:
    appDir = os.path.dirname(__file__)

# %% More Imports

from email.charset import QP
from time import sleep
import weakref
from PyQt5 import uic
from PyQt5.Qt import QTextOption
from PyQt5.QtCore import (pyqtSignal, pyqtSlot, Q_ARG, QAbstractItemModel,
                          QFileInfo, qFuzzyCompare, QMetaObject, QModelIndex, QObject, Qt,
                          QThread, QTime, QUrl, QSize, QEvent, QCoreApplication, QFile, QIODevice)
from PyQt5.QtGui import QColor, qGray, QImage, QPainter, QPalette, QIcon, QKeyEvent, QMouseEvent
from PyQt5.QtMultimedia import (QAbstractVideoBuffer, QMediaContent,
                                QMediaMetaData, QMediaPlayer, QMediaPlaylist, QVideoFrame, QVideoProbe)
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QMainWindow, QDoubleSpinBox, QApplication, QComboBox, QDialog, QFileDialog,
                             QFormLayout, QHBoxLayout, QLabel, QListView, QMessageBox, QPushButton,
                             QSizePolicy, QSlider, QStyle, QToolButton, QVBoxLayout, QWidget, QLineEdit, QPlainTextEdit,
                             QTableWidget, QTableWidgetItem, QSplitter, QAbstractItemView, QStyledItemDelegate, QHeaderView, QFrame, QProgressBar, QCheckBox, QToolTip, QGridLayout)
from PyQt5.QtCore import QTimer
from io import TextIOWrapper

import _thorlabs_kst_advanced as tlkt
import picoammeter as pico
import math as m
import os
import numpy as np
import datetime as dt

import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi, tight_layout = True)
        self.axes = fig.add_subplot(111)
        self.axes.set_xlabel('Position (nm)')
        self.axes.set_ylabel('Current (pA)')
        super(MplCanvas, self).__init__(fig)

# TODO: Change this to a per-device variable.
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
class Scan(QThread):
    pass

class Ui(QMainWindow):
    manual_prefix = 'manual'
    auto_prefix = 'automatic'
    manual_dir = './data'
    auto_dir = './data'
    startpos = 0
    stoppos = 0
    steppos = 0
    save_data = False

    manual_position = 0

    current_position = -1

    def __del__(self):
        # del self.scan # workaround for cross referencing: delete scan externally
        del self.motor_ctrl
        del self.pa

    def __init__(self, application, uiresource = None):

        self.machine_conf_win: QDialog = None
        self.grating_density_in: QDoubleSpinBox = None
        self.diff_order_in: QDoubleSpinBox = None
        self.zero_ofst_in: QDoubleSpinBox = None
        self.arm_length_in: QDoubleSpinBox = None
        self.incidence_ang_in: QDoubleSpinBox = None
        self.tangent_ang_in: QDoubleSpinBox = None
        self.machine_conf_btn: QPushButton = None

        self.grating_density = 833.333 # grating density
        self.diff_order = 1
        self.zero_ofst = 0.34 # mm
        self.arm_length = 550.0 # mm
        self.incidence_ang = 32 # deg
        self.tangent_ang = 0 # deg
        self.conversion_slope = 2 / self.grating_density * 1e3 * np.cos(np.pi * self.incidence_ang / 180) * np.cos(np.pi * self.tangent_ang / 180) / self.arm_length * MM_TO_IDX / self.diff_order # verify eqn

        print('\n\nConversion constant: %f\n'%(self.conversion_slope))

        self.manual_position = 0 # 0 nm
        self.startpos = 0
        self.stoppos = 0
        self.steppos = 0

        self.application = application

        super(Ui, self).__init__()
        uic.loadUi(uiresource, self)
        self.setWindowTitle("MMC Early GUI")

        self.is_conv_set = False # Use this flag to set conversion

        #  Picoammeter init.
        try:
            self.pa = pico.Picoammeter(3)
        except:
            self.pa = pico.Picodummy(3)

        #  KST101 init.
        try:
            serials = tlkt.Thorlabs.ListDevicesAny()
            if len(serials) == 0:
                raise RuntimeError('No KST101 controller found')
            self.motor_ctrl = tlkt.Thorlabs.KST101(serials[0])
            if (self.motor_ctrl._CheckConnection() == False):
                raise RuntimeError('Connection with motor controller failed.')
            self.motor_ctrl.set_stage('ZST25')
        except:
            serials = tlkt.Thorlabs.KSTDummy._ListDevices()
            self.motor_ctrl = tlkt.Thorlabs.KSTDummy(serials[0])

        # Move to 1mm (0nm)
        # self.motor_ctrl.move_to(1 * MM_TO_IDX, True)

        # GUI init.
        self.scan_button = self.findChild(QPushButton, "begin_scan_button")
        self.save_data_checkbox = self.findChild(QCheckBox, "save_data_checkbox")
        self.auto_prefix_box = self.findChild(QLineEdit, "scancon_prefix_lineedit")
        self.manual_prefix_box = self.findChild(QLineEdit, "mancon_prefix_lineedit")
        self.dir_box = self.findChild(QLineEdit, "save_dir_lineedit")
        # self.auto_dir_box = self.findChild(QLineEdit, "lineEdit_7")
        # self.manual_dir_box = self.findChild(QLineEdit, "lineEdit_9")
        self.start_spin = self.findChild(QDoubleSpinBox, "start_set_spinbox")
        self.stop_spin = self.findChild(QDoubleSpinBox, "end_set_spinbox")
        self.step_spin = self.findChild(QDoubleSpinBox, "step_set_spinbox")

        # These UI elements moved to begin programmatically created within the status bar.
        self.currpos_mm_disp = self.findChild(QLabel, "currpos_nm")
        self.currpos_steps_disp = self.findChild(QLabel, "currpos_steps")
        self.scan_status = self.findChild(QLabel, "status_label")
        self.scan_progress = self.findChild(QProgressBar, "progressbar")
        

        save_config_btn: QPushButton = self.findChild(QPushButton, 'save_config_button')
        save_config_btn.clicked.connect(self.showConfigWindow)


        self.pos_spin: QDoubleSpinBox = self.findChild(QDoubleSpinBox, "pos_set_spinbox")
        self.move_to_position_button: QPushButton = self.findChild(QPushButton, "move_pos_button")
        self.collect_data: QPushButton = self.findChild(QPushButton, "collect_data_button")
        self.plotFrame: QWidget = self.findChild(QWidget, "data_graph")
        # self.mainPlotFrame: QWidget = self.findChild(QWidget, "mainGraph")
        # self.plotBtnFrame: QWidget = self.findChild(QWidget, 'frame')
        self.xmin_in: QLineEdit = self.findChild(QLineEdit, "xmin_in")
        self.ymin_in: QLineEdit = self.findChild(QLineEdit, "ymin_in")
        self.xmax_in: QLineEdit = self.findChild(QLineEdit, "xmax_in")
        self.ymax_in: QLineEdit = self.findChild(QLineEdit, "ymax_in")
        self.plot_autorange: QCheckBox = self.findChild(QCheckBox, "autorange_checkbox")
        self.plot_clear_plots: QPushButton = self.findChild(QPushButton, "clear_plots_button")

        self.xdata: list = [] # collection of xdata
        self.ydata: list = [] # collection of ydata

        self.plotCanvas = MplCanvas(self, width=5, height=4, dpi=100)
        # self.plotCanvas.axes.plot([], [])
        self.scanRunning = False
        self.clearPlotFcn()
        toolbar = NavigationToolbar(self.plotCanvas, self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(toolbar)
        layout.addWidget(self.plotCanvas)
        self.plotFrame.setLayout(layout)

        self.plot_clear_plots.clicked.connect(self.clearPlotFcn)

        self.manual_prefix_box.setText(self.manual_prefix)
        self.auto_prefix_box.setText(self.auto_prefix)
        self.dir_box.setText(self.manual_dir)

        self.scan_button.clicked.connect(self.scan_button_pressed)
        self.collect_data.clicked.connect(self.manual_collect_button_pressed)
        self.move_to_position_button.clicked.connect(self.move_to_position_button_pressed)
        self.save_data_checkbox.stateChanged.connect(self.save_checkbox_toggled)
        self.auto_prefix_box.editingFinished.connect(self.auto_prefix_changed)
        self.manual_prefix_box.editingFinished.connect(self.manual_prefix_changed)
        self.dir_box.editingFinished.connect(self.manual_dir_changed)
        self.start_spin.valueChanged.connect(self.start_changed)
        self.stop_spin.valueChanged.connect(self.stop_changed)
        self.step_spin.valueChanged.connect(self.step_changed)
        self.pos_spin.setValue(self.manual_position / self.conversion_slope - self.zero_ofst)
        self.pos_spin.valueChanged.connect(self.manual_pos_changed)

        self.scan = Scan(weakref.proxy(self))

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position_displays)
        self.timer.start(100)

        self.show()

    def clearPlotFcn(self):
        print('clear called')
        if not self.scanRunning:
            self.plotCanvas.axes.cla()
            self.plotCanvas.axes.set_xlabel('Location (mm)')
            self.plotCanvas.axes.set_ylabel('Photo Current (pA)')
            self.plotCanvas.axes.grid()
            self.plotCanvas.draw()
            self.xdata = []
            self.ydata = []
        return

    def updatePlot(self):
        print('Update called')
        self.plotCanvas.axes.cla()
        self.plotCanvas.axes.set_xlabel('Location (mm)')
        self.plotCanvas.axes.set_ylabel('Photo Current (pA)')
        for idx in range(len(self.xdata)):
            if len(self.xdata[idx]) == len(self.ydata[idx]):
                self.plotCanvas.axes.plot(self.xdata[idx], self.ydata[idx], label = 'Scan %d'%(idx + 1))
        self.plotCanvas.axes.legend()
        self.plotCanvas.axes.grid()
        self.plotCanvas.draw()
        return

    def scan_statusUpdate_slot(self, status):
        self.scan_status.setText('System status: <b>%s</b>'%(status))

    def scan_progress_slot(self, curr_percent):
        self.scan_progress.setValue(curr_percent)

    def scan_complete_slot(self):
        self.scan_button.setText('Begin Scan')
        self.scan_status.setText("System status: <b>IDLE</b>")
        self.scan_progress.reset()

    def update_position_displays(self):
        self.current_position = self.motor_ctrl.get_position()
        self.moving = self.motor_ctrl.is_moving()
        self.currpos_mm_disp.setText('Current: %.4f nm'%(self.current_position / self.conversion_slope - self.zero_ofst))
        self.currpos_steps_disp.setText('%d steps'%(self.current_position))

    def scan_button_pressed(self):
        print("Scan button pressed!")
        if not self.scanRunning:
            self.scan.start()
            self.scan_button.setText('Stop Scan')
        else:
            self.scanRunning = False
            self.scan_button.setText('Begin Scan')

    def manual_collect_button_pressed(self):
        print("Manual collect button pressed!")
        self.take_data()

    def move_to_position_button_pressed(self):
        self.moving = True
        print("Move to position button pressed, moving to %d"%(self.manual_position))
        self.motor_ctrl.move_to(self.manual_position, False)

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
        self.startpos = (self.start_spin.value() + self.zero_ofst) * self.conversion_slope
        print(self.startpos)

    def stop_changed(self):
        print("Stop changed to: %s mm"%(self.stop_spin.value()))
        self.stoppos = (self.stop_spin.value() + self.zero_ofst) * self.conversion_slope
        print(self.stoppos)

    def step_changed(self):
        print("Step changed to: %s mm"%(self.step_spin.value()))
        self.steppos = (self.step_spin.value()) * self.conversion_slope
        print(self.steppos)

    def manual_pos_changed(self):
        print("Manual position changed to: %s mm"%(self.pos_spin.value()))
        self.manual_position = (self.pos_spin.value() + self.zero_ofst) * self.conversion_slope

    def take_data(self):

        tnow = dt.datetime.now()

        filename = self.manual_dir + '/' + self.manual_prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv"
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        self.sav_file = open(filename, 'w')
        
        self.sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
        self.sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
        self.sav_file.write('# Position (step),Current(A),Timestamp,Error Code\n')
        pos = self.motor_ctrl.get_position()
        self.current_position = pos
        self.sav_file.write(self.pa.sample_data())

        self.sav_file.close()

    def showConfigWindow(self):
        if self.machine_conf_win is None:
            ui_file_name = exeDir + '/grating_input.ui'
            ui_file = QFile(ui_file_name)
            if not ui_file.open(QIODevice.ReadOnly):
                print(f"Cannot open {ui_file_name}: {ui_file.errorString()}")
                raise RuntimeError('Could not load grating input UI file')
            
            self.machine_conf_win = QDialog()
            uic.loadUi(ui_file, self.machine_conf_win)

            self.machine_conf_win.setWindowTitle('Monochromator Configuration')

            self.grating_density_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'grating_density_in')
            self.grating_density_in.setValue(self.grating_density)
            
            self.zero_ofst_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'zero_offset_in')
            self.zero_ofst_in.setValue(self.zero_ofst)
            
            self.incidence_ang_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'incidence_angle_in')
            self.incidence_ang_in.setValue(self.incidence_ang)
            
            self.tangent_ang_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'tangent_angle_in')
            self.tangent_ang_in.setValue(self.tangent_ang)

            self.arm_length_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'arm_length_in')
            self.arm_length_in.setValue(self.arm_length)

            self.diff_order_in = self.machine_conf_win.findChild(QDoubleSpinBox, 'diff_order_in')
            self.diff_order_in.setValue(self.diff_order)

            self.machine_conf_btn = self.machine_conf_win.findChild(QPushButton, 'update_conf_btn')
            self.machine_conf_btn.clicked.connect(self.applyMachineConf)
        
        self.machine_conf_win.show()

    def applyMachineConf(self):
        print('Apply config called')
        self.grating_density = self.grating_density_in.value()
        self.diff_order = self.diff_order_in.value()
        self.zero_ofst = self.zero_ofst_in.value()
        self.incidence_ang = self.incidence_ang_in.value()
        self.arm_length = self.arm_length_in.value()

        self.conversion_slope = 2 / self.grating_density * 1e3 * np.cos(np.pi * self.incidence_ang / 180) * np.cos(np.pi * self.tangent_ang / 180) / self.arm_length * MM_TO_IDX / self.diff_order

        self.machine_conf_win.close()


    # TODO: Use QThreads to prevent freezing GUI. For now, `self.application.processEvents()`.
    # TODO: (cont.): https://www.xingyulei.com/post/qt-threading/
    # def initiate_scan(self):
    #     print("Save to file? " + str(self.save_data))

    #     self.scan_status.setText("PREPARING")
    #     self.scan_progress.setMinimum(self.start)
    #     self.scan_progress.setMaximum(self.stop)
    #     self.application.processEvents()

    #     if (self.save_data):
    #         tnow = dt.datetime.now()
            
    #         filename = self.auto_dir + '/' + self.auto_prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv"
    #         os.makedirs(os.path.dirname(filename), exist_ok=True)
    #         self.sav_file = open(filename, 'w')

    #         # print(type(self.sav_file))
    #     # Move to start and collect data.
    #     self.scan_status.setText("MOVING")
    #     self.application.processEvents()
    #     self.motor_ctrl.move_to(self.start, True)
        
    #     self.scan_status.setText("SAMPLING")
    #     self.application.processEvents()
    #     if (self.save_data):
    #         self.sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
    #         self.sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
    #         self.sav_file.write('# Position (step),Current(A),Timestamp,Error Code\n')
    #         # pos = self.motor_ctrl.get_position()
    #         self.sav_file.write(self.pa.sample_data(10, self.motor_ctrl.get_position()))

    #         # self.scan_progress.setValue(pos)
    #         # self.application.processEvents()

    #     else:
    #         print(self.pa.sample_data(10, self.motor_ctrl.get_position()))
    #     print(self.start, self.stop, self.step)
    #     if self.step > 0:
    #         while self.motor_ctrl.get_position() < self.stop:
    #             self.scan_status.setText("MOVING")
    #             self.application.processEvents()
    #             self.motor_ctrl.move_by(self.step, True)
    #             pos = self.motor_ctrl.get_position()
    #             self.scan_status.setText("SAMPLING")
    #             self.application.processEvents()
    #             if (self.save_data):
    #                 self.sav_file.write(self.pa.sample_data(10, pos))
    #             else:
    #                 print(self.pa.sample_data(10, pos))

    #             self.scan_progress.setValue(pos)
    #             self.application.processEvents()

    #     else:
    #         while self.motor_ctrl.get_position() > self.stop:
    #             self.scan_status.setText("MOVING")
    #             self.application.processEvents()
    #             self.motor_ctrl.move_by(self.step, True)
    #             self.scan_status.setText("SAMPLING")
    #             self.application.processEvents()
    #             if (self.save_data):
    #                 self.sav_file.write(self.pa.sample_data(10, pos))
    #             else:
    #                 print(self.pa.sample_data(10, pos))

    #             self.scan_progress.setValue(pos)
    #             self.application.processEvents()

    #     if (self.save_data):
    #         self.sav_file.close()
    #     self.scan_status.setText("IDLE")
    #     self.scan_progress.reset()
    #     self.scan_progress.setMinimum(0)
    #     self.scan_progress.setMaximum(100)
    #     self.application.processEvents()

class Scan(QThread):
    statusUpdate = pyqtSignal(str)
    progress = pyqtSignal(int)
    complete = pyqtSignal()

    def __init__(self, parent: QMainWindow):
        super(Scan, self).__init__()
        self.other: Ui = parent
        self.statusUpdate.connect(self.other.scan_statusUpdate_slot)
        self.progress.connect(self.other.scan_progress_slot)
        self.complete.connect(self.other.scan_complete_slot)
        print('mainWindow reference in scan init: %d'%(sys.getrefcount(self.other) - 1))

    def __del__(self):
        self.wait()

    def run(self):
        print(self.other)
        print("Save to file? " + str(self.other.save_data))

        self.statusUpdate.emit("PREPARING")
        sav_file = None
        if (self.other.save_data):
            tnow = dt.datetime.now()
            
            filename = self.other.auto_dir + '/' + self.other.auto_prefix + '_' + tnow.strftime('%Y%m%d%H%M%S') + "_data.csv"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            sav_file = open(filename, 'w')

        # self.statusUpdate.emit("MOVING")
        
        # self.motor_ctrl.move_to(self.startpos, True)
        
        # self.statusUpdate.emit("SAMPLING")

        print(self.other.startpos, self.other.stoppos, self.other.steppos)

        scanrange = np.arange(self.other.startpos, self.other.stoppos + self.other.steppos, self.other.steppos)
        # self.other.pa.set_samples(3)
        nidx = len(scanrange)
        if len(self.other.xdata) != len(self.other.ydata):
            self.other.xdata = []
            self.other.ydata = []
        pidx = len(self.other.xdata)
        self.other.xdata.append([])
        self.other.ydata.append([])
        self.other.scanRunning = True
        for idx, dpos in enumerate(scanrange):
            if not self.other.scanRunning:
                break
            self.statusUpdate.emit("MOVING")
            self.other.motor_ctrl.move_to(dpos, True)
            pos = self.other.motor_ctrl.get_position()
            self.statusUpdate.emit("SAMPLING")
            buf = self.other.pa.sample_data()
            print(buf)
            self.progress.emit(round((idx + 1) * 100 / nidx))
            # process buf
            words = buf.split(',') # split at comma
            # print(words)
            if len(words) != 3:
                continue
            try:
                mes = float(words[0][:-1]) # skip the A (unit suffix)
                err = int(float(words[2])) # skip timestamp
            except Exception:
                continue
            # print(mes, err)
            self.other.xdata[pidx].append(pos / MM_TO_IDX)
            self.other.ydata[pidx].append(-mes * 1e12)
            # print(self.other.xdata[pidx], self.other.ydata[pidx])
            self.other.updatePlot()
            if sav_file is not None:
                if idx == 0:
                    sav_file.write('# %s\n'%(tnow.strftime('%Y-%m-%d %H:%M:%S')))
                    sav_file.write('# Steps/mm: %f\n'%(MM_TO_IDX))
                    sav_file.write('# Position (step),Mean Current(A),Status/Error Code\n')
                # process buf
                # 1. split by \n
                buf = '%d,%e,%d\n'%(pos, -mes, err)
                sav_file.write(buf)

        if (sav_file is not None):
            sav_file.close()
        self.other.scanRunning = False
        self.complete.emit()
        print('mainWindow reference in scan end: %d'%(sys.getrefcount(self.other) - 1))

# Main function.
if __name__ == '__main__':
    # There will be three separate GUIs:
    # 1. Initialization loading screen, where devices are being searched for and the current status and tasks are displayed. If none are found, display an error and an exit button.
    # 2. The device selection display, where devices can be selected and their settings can be changed prior to entering the control program.
    # 3. The control GUI (mainwindow.ui), where the user has control over what the device(s) do.
    
    application = QApplication(sys.argv)

    # First, the loading screen.

    # Then, we load up the device selection UI.
    ui_file_name = exeDir + '/grating_input.ui'
    ui_file = QFile(ui_file_name)
    if not ui_file.open(QIODevice.ReadOnly):
        print(f"Cannot open {ui_file_name}: {ui_file.errorString()}")
        sys.exit(-1)

    # Main GUI bootup.
    ui_file_name = exeDir + '/' + "mainwindow_mk2.ui"
    ui_file = QFile(ui_file_name) # workaround to load UI file with pyinstaller
    if not ui_file.open(QIODevice.ReadOnly):
        print(f"Cannot open {ui_file_name}: {ui_file.errorString()}")
        sys.exit(-1)

    # Initializes the GUI.
    mainWindow = Ui(application, ui_file)
    
    # Example: Creating a new instrument. Should be done in a UI callback of some sort.
     # new_mono = instruments.Monochromator(241.0536, 32, 1)

    # Example: Getting a UI element from the .ui file, setting LCDNumber value.
     # lcd_milli = mainWindow.findChild(QtWidgets.QLCDNumber, "lcdNumber")
     # lcd_nano = mainWindow.findChild(QtWidgets.QLCDNumber, "lcdNumber_2")
     # lcd_milli.display(1)
     # lcd_nano.display(2)
    
    # Wait for the Qt loop to exit before exiting.
    ret = application.exec_() # block until
    print('mainwindow: %d'%(sys.getrefcount(mainWindow) - 1))
    print('motor: %d'%(sys.getrefcount(mainWindow.motor_ctrl) - 1))
    print('pa: %d'%(sys.getrefcount(mainWindow.pa) - 1))
    del mainWindow
    sys.exit(ret)