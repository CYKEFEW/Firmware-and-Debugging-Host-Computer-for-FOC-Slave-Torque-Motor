#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtGui, QtWidgets, QtCore

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit, ConfigQLineEdit
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

        self.signal0Label = QtWidgets.QLabel(self)
        self.signal0Label.setObjectName('angleLabel')
        self.signal0Label.setText('角度')
        self.droGridLayout.addWidget(self.signal0Label, 0, 0)

        self.signal0LCDNumber = QtWidgets.QLCDNumber(self)
        self.putStyleToLCDNumber(self.signal0LCDNumber)
        self.signal0LCDNumber.setObjectName('signal0LCDNumber')
        self.signal0LCDNumber.setMinimumHeight(30)
        self.droGridLayout.addWidget(self.signal0LCDNumber, 1, 0)

        self.signal1Label = QtWidgets.QLabel(self)
        self.signal1Label.setObjectName('velLabel')
        self.signal1Label.setText('速度')
        self.droGridLayout.addWidget(self.signal1Label, 0, 1)

        self.signal1LCDNumber = QtWidgets.QLCDNumber(self)
        self.putStyleToLCDNumber(self.signal1LCDNumber)
        self.signal1LCDNumber.setObjectName('signal1LCDNumber')
        self.signal1LCDNumber.setMinimumHeight(30)
        self.droGridLayout.addWidget(self.signal1LCDNumber, 1, 1)

        self.signal2Label = QtWidgets.QLabel(self)
        self.signal2Label.setObjectName('torqueLabel')
        self.signal2Label.setText('目标')
        self.droGridLayout.addWidget(self.signal2Label, 0, 2)

        self.signal2LCDNumber = QtWidgets.QLCDNumber(self)
        self.putStyleToLCDNumber(self.signal2LCDNumber)
        self.signal2LCDNumber.setObjectName('signal2LCDNumber')
        self.signal2LCDNumber.setMinimumHeight(30)
        self.droGridLayout.addWidget(self.signal2LCDNumber, 1, 2)

        self.signal3Label = QtWidgets.QLabel(self)
        self.signal3Label.setObjectName('targetLabel')
        self.signal3Label.setText('目标')
        self.droGridLayout.addWidget(self.signal3Label, 0, 3)

        self.signal3LCDNumber = QtWidgets.QLCDNumber(self)
        self.putStyleToLCDNumber(self.signal3LCDNumber)
        self.signal3LCDNumber.setObjectName('signal3LCDNumber')
        self.signal3LCDNumber.setMinimumHeight(30)
        self.droGridLayout.addWidget(self.signal3LCDNumber, 1, 3)

        onlyInt = QtGui.QRegExpValidator(QtCore.QRegExp("[1-9][0-9]*"))

        self.refreshRateLabel = QtWidgets.QLabel(self)
        self.refreshRateLabel.setObjectName('refreshRateLabel')
        self.refreshRateLabel.setText('刷新率 [Hz]')
        self.droGridLayout.addWidget(self.refreshRateLabel, 2, 0, 1, 2)

        self.refreshRateEdit = ConfigQLineEdit(self)
        self.refreshRateEdit.setObjectName('refreshRateEdit')
        self.refreshRateEdit.setValidator(onlyInt)
        self.refreshRateEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.refreshRateEdit.setText(str(self.device.stateUpdateRateHz))
        self.refreshRateEdit.updateValue.connect(self.updateRefreshRate)
        self.refreshRateEdit.editingFinished.connect(self.updateRefreshRate)
        self.droGridLayout.addWidget(self.refreshRateEdit, 2, 2, 1, 2)

        for column in range(4):
            self.droGridLayout.setColumnStretch(column, 1)

        self.commandResponseReceived('init')

        self.initDiplay()
        self.disableUI()

        self.device.addConnectionStateListener(self)
        self.device.commProvider.stateMonitorReceived.connect(self.commandResponseReceived)

        self.connectionStateChanged(self.device.isConnected)

    def connectionStateChanged(self, isConnectedFlag):
        if isConnectedFlag is True:
            self.enabeUI()
            self.initDiplay()
        else:
            self.initDiplay()
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)

    def disableUI(self):
        self.setEnabled(False)

    def initDiplay(self):
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

    def putStyleToLCDNumber(self, lcdNumber):
        lcdNumber.setStyleSheet('''QLCDNumber {background-color: white;}''')
        palette = self.setColor(lcdNumber.palette(), GUIToolKit.RED_COLOR)
        lcdNumber.setPalette(palette)

    def setColor(self, palette, colorTouple):
        R = colorTouple[0]
        G = colorTouple[1]
        B = colorTouple[2]
        # foreground color
        palette.setColor(palette.WindowText, QtGui.QColor(R, G, B))
        # background color
        palette.setColor(palette.Background, QtGui.QColor(R, G, B))
        # 'light' border
        palette.setColor(palette.Light, QtGui.QColor(R, G, B))
        # 'dark' border
        palette.setColor(palette.Dark, QtGui.QColor(R, G, B))
        return palette

    def commandResponseReceived(self, cmdRespose):        
        if self.device.torqueType ==  SimpleFOCDevice.VOLTAGE_TORQUE:
            self.signal2Label.setText("电压")
            self.signal2LCDNumber.display(self.device.voltageQNow)
        else: # dc current or FOC current
            self.signal2Label.setText("电流")
            self.signal2LCDNumber.display(self.device.currentQNow)

        self.signal3LCDNumber.display(self.device.targetNow)
        self.signal1LCDNumber.display(self.device.velocityNow)
        self.signal0LCDNumber.display(self.device.angleNow)
