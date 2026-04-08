#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import ConfigQLineEdit, GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice


class DROGroupBox(QtWidgets.QGroupBox):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()

        self.setTitle('Simple FOC 数字状态显示区')
        self.setObjectName('digitalReadOut')

        self.droGridLayout = QtWidgets.QGridLayout(self)
        self.droGridLayout.setObjectName('droGridLayout')
        self.droGridLayout.setHorizontalSpacing(12)
        self.droGridLayout.setVerticalSpacing(6)

        self.signal0Label = QtWidgets.QLabel('角度', self)
        self.droGridLayout.addWidget(self.signal0Label, 0, 0)

        self.signal1Label = QtWidgets.QLabel('速度', self)
        self.droGridLayout.addWidget(self.signal1Label, 0, 1)

        self.signal2Label = QtWidgets.QLabel('目标', self)
        self.droGridLayout.addWidget(self.signal2Label, 0, 2)

        self.signal3Label = QtWidgets.QLabel('目标', self)
        self.droGridLayout.addWidget(self.signal3Label, 0, 3)

        self.signal0LCDNumber = self._createLcdNumber('signal0LCDNumber')
        self.droGridLayout.addWidget(self.signal0LCDNumber, 1, 0)

        self.signal1LCDNumber = self._createLcdNumber('signal1LCDNumber')
        self.droGridLayout.addWidget(self.signal1LCDNumber, 1, 1)

        self.signal2LCDNumber = self._createLcdNumber('signal2LCDNumber')
        self.droGridLayout.addWidget(self.signal2LCDNumber, 1, 2)

        self.signal3LCDNumber = self._createLcdNumber('signal3LCDNumber')
        self.droGridLayout.addWidget(self.signal3LCDNumber, 1, 3)

        only_int = QtGui.QRegExpValidator(QtCore.QRegExp("[1-9][0-9]*"))

        self.refreshRateLabel = QtWidgets.QLabel('刷新率 [Hz]', self)
        self.droGridLayout.addWidget(self.refreshRateLabel, 2, 0, 1, 2)

        self.refreshRateEdit = ConfigQLineEdit(self)
        self.refreshRateEdit.setObjectName('refreshRateEdit')
        self.refreshRateEdit.setValidator(only_int)
        self.refreshRateEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.refreshRateEdit.setText(str(self.device.stateUpdateRateHz))
        self.refreshRateEdit.updateValue.connect(self.updateRefreshRate)
        self.refreshRateEdit.editingFinished.connect(self.updateRefreshRate)
        self.droGridLayout.addWidget(self.refreshRateEdit, 2, 2, 1, 2)

        for column in range(4):
            self.droGridLayout.setColumnStretch(column, 1)

        self.initDisplay()
        self.disableUI()

        self.device.addConnectionStateListener(self)
        self.device.commProvider.stateMonitorReceived.connect(
            self.commandResponseReceived)
        self.device.commProvider.commandDataReceived.connect(
            self.commandResponseReceived)

        self.connectionStateChanged(self.device.isConnected)

    def _createLcdNumber(self, object_name):
        lcd_number = QtWidgets.QLCDNumber(self)
        lcd_number.setObjectName(object_name)
        lcd_number.setMinimumHeight(30)
        lcd_number.setStyleSheet('QLCDNumber {background-color: white;}')
        palette = self._setColor(lcd_number.palette(), GUIToolKit.RED_COLOR)
        lcd_number.setPalette(palette)
        return lcd_number

    def _setColor(self, palette, color_tuple):
        red, green, blue = color_tuple
        color = QtGui.QColor(red, green, blue)
        palette.setColor(palette.WindowText, color)
        palette.setColor(palette.Background, color)
        palette.setColor(palette.Light, color)
        palette.setColor(palette.Dark, color)
        return palette

    def connectionStateChanged(self, isConnectedFlag):
        if isConnectedFlag is True:
            self.enabeUI()
            self.initDisplay()
        else:
            self.initDisplay()
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)

    def disableUI(self):
        self.setEnabled(False)

    def initDisplay(self):
        self.signal0LCDNumber.display(0.0)
        self.signal1LCDNumber.display(0.0)
        self.signal2LCDNumber.display(0.0)
        self.signal3LCDNumber.display(0.0)

    def updateRefreshRate(self):
        value = self.refreshRateEdit.text().strip()
        if value == '':
            value = '10'
        self.refreshRateEdit.setText(value)
        self.device.setStateUpdateRateHz(value)

    def commandResponseReceived(self, _commandResponse):
        self.signal0LCDNumber.display(self.device.angleNow)
        self.signal1LCDNumber.display(self.device.velocityNow)

        if self.device.torqueType == SimpleFOCDevice.VOLTAGE_TORQUE:
            self.signal2Label.setText('电压')
            self.signal2LCDNumber.display(self.device.voltageQNow)
        else:
            self.signal2Label.setText('电流')
            self.signal2LCDNumber.display(self.device.currentQNow)

        if self.device.passiveTorqueMode:
            self.signal3Label.setText('磁场阻尼角[°]')
            self.signal3LCDNumber.display(self.device.passiveTorqueDampingAngleDeg)
        else:
            self.signal3Label.setText('目标')
            self.signal3LCDNumber.display(self.device.targetNow)
