#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice


class SimpleFOCGraphicWidget(QtWidgets.QGroupBox):
    disconnectedState = 0
    initialConnectedState = 1
    connectedPausedState = 2
    connectedPlottingStartedState = 3

    signals = ['Target', 'Vq', 'Vd', 'Cq', 'Cd', 'Vel', 'Angle']
    signal_tooltip = [
        '目标',
        '电压 Q [V]',
        '电压 D [V]',
        '电流 Q [mA]',
        '电流 D [mA]',
        '速度 [rad/sec]',
        '角度 [rad]',
    ]
    signalColors = [
        GUIToolKit.RED_COLOR,
        GUIToolKit.BLUE_COLOR,
        GUIToolKit.PURPLE_COLOR,
        GUIToolKit.YELLOW_COLOR,
        GUIToolKit.MAROON_COLOR,
        GUIToolKit.ORANGE_COLOR,
        GUIToolKit.GREEN_COLOR,
    ]
    signalIcons = [
        'reddot',
        'bluedot',
        'purpledot',
        'yellowdot',
        'maroondot',
        'orangedot',
        'greendot',
    ]

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setObjectName('plotWidget')
        self.setTitle('实时电机数据:')

        self.device = SimpleFOCDevice.getInstance()
        self.numberOfSamples = 300
        self.timeArray = np.arange(-self.numberOfSamples + 1, 1, 1, dtype=float)
        self.signalDataArrays = np.full(
            (len(self.signals), self.numberOfSamples), np.nan, dtype=float)
        self.sampleCursor = 0
        self.samplesFilled = 0
        self.plotDirty = False

        self.horizontalLayout = QtWidgets.QVBoxLayout(self)

        pg.setConfigOptions(antialias=False)
        self.plotWidget = pg.PlotWidget()
        self.plotWidget.showGrid(x=True, y=True, alpha=0.35)
        self.plotWidget.addLegend()

        self.controlPlotWidget = ControlPlotPanel(controllerPlotWidget=self)

        self.signalPlots = []
        self.signalPlotFlags = []
        for signal_name, signal_color, check_box, tooltip in zip(
                self.signals,
                self.signalColors,
                self.controlPlotWidget.signalCheckBox,
                self.signal_tooltip):
            signal_pen = pg.mkPen(color=signal_color, width=1.5)
            plot_item = pg.PlotDataItem(
                self.timeArray,
                np.full(self.numberOfSamples, np.nan, dtype=float),
                pen=signal_pen,
                name=tooltip)
            self.signalPlots.append(plot_item)
            self.plotWidget.addItem(plot_item)
            self.signalPlotFlags.append(True)
            check_box.stateChanged.connect(self.signalPlotFlagUpdate)

        self.horizontalLayout.addWidget(self.plotWidget)
        self.horizontalLayout.addWidget(self.controlPlotWidget)

        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.setInterval(33)
        self.redrawTimer.timeout.connect(self.refreshPlotIfNeeded)
        self.redrawTimer.start()

        self.device.commProvider.monitoringDataReceived.connect(self.upDateGraphic)

        self.currentStatus = self.disconnectedState
        self.controlPlotWidget.pauseContinueButton.setDisabled(True)

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)

    def connectionStateChanged(self, deviceConnected):
        if deviceConnected:
            self.currentStatus = self.initialConnectedState
            self.enabeUI()
        else:
            self.controlPlotWidget.startStoPlotAction()
            self.controlPlotWidget.stopAndResetPlot()
            self.currentStatus = self.disconnectedState
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)

    def disableUI(self):
        self.setEnabled(False)

    def signalPlotFlagUpdate(self):
        self.controlPlotWidget.updateMonitorVariables()
        for index, (check_box, plot_flag) in enumerate(
                zip(self.controlPlotWidget.signalCheckBox, self.signalPlotFlags)):
            if check_box.isChecked() and not plot_flag:
                self.signalPlotFlags[index] = True
                self.plotWidget.addItem(self.signalPlots[index])
            elif (not check_box.isChecked()) and plot_flag:
                self.signalPlotFlags[index] = False
                self.plotWidget.removeItem(self.signalPlots[index])
        self.plotDirty = True
        self.updatePlot(force=True)

    def upDateGraphic(self, signalList):
        if self.currentStatus not in (
                self.connectedPlottingStartedState,
                self.connectedPausedState):
            return

        try:
            signals = np.asarray(signalList, dtype=float)
        except ValueError:
            logging.warning('Arrived corrupted data')
            return

        enabled_indices = np.where(np.asarray(self.signalPlotFlags) == True)[0]
        if len(enabled_indices) != len(signals):
            logging.warning('Arrived corrupted data')
            return

        write_index = self.sampleCursor
        for data_index, signal_index in enumerate(enabled_indices):
            self.signalDataArrays[signal_index, write_index] = signals[data_index]

        self.sampleCursor = (self.sampleCursor + 1) % self.numberOfSamples
        self.samplesFilled = min(self.samplesFilled + 1, self.numberOfSamples)
        self.plotDirty = True

    def clearPlotData(self, reset_view=False):
        self.signalDataArrays.fill(np.nan)
        self.sampleCursor = 0
        self.samplesFilled = 0
        self.plotDirty = True
        self.updatePlot(force=True)
        if reset_view:
            self.plotWidget.enableAutoRange()

    def _orderedSignalMatrix(self):
        if self.samplesFilled == 0:
            return np.empty((len(self.signals), 0), dtype=float)

        if self.samplesFilled < self.numberOfSamples:
            return self.signalDataArrays[:, :self.samplesFilled].copy()

        return np.concatenate(
            (
                self.signalDataArrays[:, self.sampleCursor:],
                self.signalDataArrays[:, :self.sampleCursor],
            ),
            axis=1)

    def resizeSampleWindow(self, sample_count):
        sample_count = max(10, int(sample_count))
        if sample_count == self.numberOfSamples:
            return

        ordered_matrix = self._orderedSignalMatrix()
        keep_count = min(ordered_matrix.shape[1], sample_count)
        resized_arrays = np.full(
            (len(self.signals), sample_count), np.nan, dtype=float)

        if keep_count > 0:
            resized_arrays[:, :keep_count] = ordered_matrix[:, -keep_count:]

        self.numberOfSamples = sample_count
        self.timeArray = np.arange(-self.numberOfSamples + 1, 1, 1, dtype=float)
        self.signalDataArrays = resized_arrays
        self.samplesFilled = keep_count
        self.sampleCursor = keep_count % self.numberOfSamples
        self.plotDirty = True
        self.updatePlot(force=True)
        self.plotWidget.enableAutoRange()

    def orderedSignalData(self, signal_index):
        if self.samplesFilled == 0:
            return np.full(self.numberOfSamples, np.nan, dtype=float)

        signal_data = self.signalDataArrays[signal_index]
        if self.samplesFilled < self.numberOfSamples:
            ordered = np.full(self.numberOfSamples, np.nan, dtype=float)
            ordered[-self.samplesFilled:] = signal_data[:self.samplesFilled]
            return ordered

        return np.concatenate(
            (signal_data[self.sampleCursor:], signal_data[:self.sampleCursor]))

    def refreshPlotIfNeeded(self):
        if self.currentStatus == self.connectedPlottingStartedState and self.plotDirty:
            self.updatePlot()

    def updatePlot(self, force=False):
        if not force and self.currentStatus != self.connectedPlottingStartedState:
            return

        for index, plot_flag in enumerate(self.signalPlotFlags):
            if plot_flag:
                self.signalPlots[index].setData(
                    self.timeArray, self.orderedSignalData(index))

        self.plotDirty = False


class ControlPlotPanel(QtWidgets.QWidget):

    def __init__(self, parent=None, controllerPlotWidget=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()
        self.controlledPlot = controllerPlotWidget

        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.setLayout(self.verticalLayout)

        self.horizontalLayout1 = QtWidgets.QHBoxLayout()
        self.horizontalLayout1.setObjectName('horizontalLayout')

        self.startStopButton = QtWidgets.QPushButton(self)
        self.startStopButton.setText('开始')
        self.startStopButton.setObjectName('Start')
        self.startStopButton.clicked.connect(self.startStoPlotAction)
        self.startStopButton.setIcon(GUIToolKit.getIconByName('start'))
        self.horizontalLayout1.addWidget(self.startStopButton)

        self.pauseContinueButton = QtWidgets.QPushButton(self)
        self.pauseContinueButton.setObjectName('pauseButton')
        self.pauseContinueButton.setText('暂停')
        self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
        self.pauseContinueButton.clicked.connect(self.pauseContinuePlotAction)
        self.horizontalLayout1.addWidget(self.pauseContinueButton)

        self.clearButton = QtWidgets.QPushButton(self)
        self.clearButton.setObjectName('clearPlotButton')
        self.clearButton.setText('清空')
        self.clearButton.setIcon(GUIToolKit.getIconByName('delete'))
        self.clearButton.clicked.connect(self.clearPlotAction)
        self.horizontalLayout1.addWidget(self.clearButton)

        self.zoomAllButton = QtWidgets.QPushButton(self)
        self.zoomAllButton.setObjectName('zoomAllButton')
        self.zoomAllButton.setText('显示所有')
        self.zoomAllButton.setIcon(GUIToolKit.getIconByName('zoomall'))
        self.zoomAllButton.clicked.connect(self.zoomAllPlot)
        self.horizontalLayout1.addWidget(self.zoomAllButton)

        self.signalCheckBox = []
        for index in range(len(self.controlledPlot.signals)):
            check_box = QtWidgets.QCheckBox(self)
            check_box.setObjectName('signalCheckBox' + str(index))
            check_box.setToolTip(self.controlledPlot.signal_tooltip[index])
            check_box.setText(self.controlledPlot.signals[index])
            check_box.setIcon(
                GUIToolKit.getIconByName(self.controlledPlot.signalIcons[index]))
            check_box.setChecked(True)
            self.signalCheckBox.append(check_box)
            self.horizontalLayout1.addWidget(check_box)

        spacer_item = QtWidgets.QSpacerItem(
            100,
            20,
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Maximum)
        self.horizontalLayout1.addItem(spacer_item)

        self.downsampleLabel = QtWidgets.QLabel(self)
        self.downsampleLabel.setText('降采样')
        self.horizontalLayout1.addWidget(self.downsampleLabel)

        self.downampleValue = QtWidgets.QLineEdit(self)
        self.downampleValue.setAlignment(QtCore.Qt.AlignCenter)
        self.downampleValue.setValidator(
            QtGui.QRegExpValidator(QtCore.QRegExp('[1-9][0-9]*')))
        self.downampleValue.setText('100')
        self.downampleValue.editingFinished.connect(self.changeDownsampling)
        self.horizontalLayout1.addWidget(self.downampleValue)

        self.windowLengthLabel = QtWidgets.QLabel(self)
        self.windowLengthLabel.setText('窗口长度')
        self.horizontalLayout1.addWidget(self.windowLengthLabel)

        self.windowLengthValue = QtWidgets.QLineEdit(self)
        self.windowLengthValue.setAlignment(QtCore.Qt.AlignCenter)
        self.windowLengthValue.setValidator(
            QtGui.QRegExpValidator(QtCore.QRegExp('[1-9][0-9]*')))
        self.windowLengthValue.setText(str(self.controlledPlot.numberOfSamples))
        self.windowLengthValue.editingFinished.connect(self.changeWindowLength)
        self.horizontalLayout1.addWidget(self.windowLengthValue)

        self.verticalLayout.addLayout(self.horizontalLayout1)

    def startStoPlotAction(self):
        if self.controlledPlot.currentStatus == self.controlledPlot.initialConnectedState:
            self.startStopButton.setText('停止')
            self.startStopButton.setIcon(GUIToolKit.getIconByName('stop'))
            self.controlledPlot.clearPlotData(reset_view=True)
            self.controlledPlot.currentStatus = (
                self.controlledPlot.connectedPlottingStartedState)
            self.pauseContinueButton.setEnabled(True)
            self.device.sendMonitorDownsample(int(self.downampleValue.text()))
            self.updateMonitorVariables()
        else:
            self.startStopButton.setText('开始')
            self.startStopButton.setIcon(GUIToolKit.getIconByName('start'))
            self.pauseContinueButton.setText('暂停')
            self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
            self.pauseContinueButton.setEnabled(False)
            self.stopAndResetPlot()
            self.device.sendMonitorDownsample(0)
            self.device.sendMonitorClearVariables()

    def pauseContinuePlotAction(self):
        if self.controlledPlot.currentStatus == self.controlledPlot.connectedPausedState:
            self.pauseContinueButton.setText('暂停')
            self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
            self.controlledPlot.currentStatus = (
                self.controlledPlot.connectedPlottingStartedState)
        else:
            self.pauseContinueButton.setText('继续')
            self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('continue'))
            self.controlledPlot.currentStatus = self.controlledPlot.connectedPausedState

    def stopAndResetPlot(self):
        self.controlledPlot.currentStatus = self.controlledPlot.initialConnectedState
        self.controlledPlot.clearPlotData(reset_view=True)

    def clearPlotAction(self):
        self.controlledPlot.clearPlotData(reset_view=True)

    def zoomAllPlot(self):
        self.controlledPlot.plotWidget.enableAutoRange()

    def changeDownsampling(self):
        if (self.controlledPlot.currentStatus ==
                self.controlledPlot.connectedPlottingStartedState):
            self.device.sendMonitorDownsample(int(self.downampleValue.text()))

    def changeWindowLength(self):
        value = self.windowLengthValue.text().strip()
        if value == '':
            value = str(self.controlledPlot.numberOfSamples)
        self.windowLengthValue.setText(value)
        self.controlledPlot.resizeSampleWindow(int(value))

    def updateMonitorVariables(self):
        self.device.sendMonitorVariables([
            check_box.isChecked() for check_box in self.signalCheckBox
        ])
