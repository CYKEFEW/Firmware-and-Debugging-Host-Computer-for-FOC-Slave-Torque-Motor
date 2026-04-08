#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

from src.gui.configtool.configureConnectionDialog import \
    ConfigureSerailConnectionDialog
from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit, SerialPortComboBox
from src.simpleFOCConnector import SimpleFOCDevice


class ConnectionControlGroupBox(QtWidgets.QGroupBox):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()

        self.setObjectName('connectionControl')
        self.setTitle('连接')

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setObjectName('generalControlGL')
        self.gridLayout.setHorizontalSpacing(8)
        self.gridLayout.setVerticalSpacing(6)

        self.devCommandIDLabel = QtWidgets.QLabel("命令ID:")
        self.gridLayout.addWidget(self.devCommandIDLabel, 0, 0)

        self.devCommandIDLetter = QtWidgets.QLineEdit()
        self.devCommandIDLetter.setObjectName('devCommandIDLetter')
        self.devCommandIDLetter.editingFinished.connect(self.changeDevicedevCommandID)
        self.gridLayout.addWidget(self.devCommandIDLetter, 0, 1)
        self._applyDefaultConnectionSettings()
        self.devCommandIDLetter.setText(self.device.devCommandID)

        self.pullConfig = QtWidgets.QPushButton()
        self.pullConfig.setObjectName('pullConfig')
        self.pullConfig.setIcon(GUIToolKit.getIconByName('pull'))
        self.pullConfig.setText('获取参数')
        self.pullConfig.clicked.connect(self.device.pullConfiguration)
        
        self.gridLayout.addWidget(self.pullConfig, 1, 0)

        self.connectDisconnectButton = QtWidgets.QPushButton(self)
        self.connectDisconnectButton.setIcon(GUIToolKit.getIconByName('connect'))
        self.connectDisconnectButton.setObjectName('connectDeviceButton')
        self.connectDisconnectButton.setText('连接')
        self.connectDisconnectButton.clicked.connect(self.connectDisconnectDeviceAction)

        self.gridLayout.addWidget(self.connectDisconnectButton, 1, 1)

        self.configureDeviceButton = QtWidgets.QPushButton(self)
        self.configureDeviceButton.setIcon(GUIToolKit.getIconByName('configure'))
        self.configureDeviceButton.setObjectName('configureDeviceButton')
        self.configureDeviceButton.setText('设置')
        self.configureDeviceButton.clicked.connect(self.configureDeviceAction)
        self.gridLayout.addWidget(self.configureDeviceButton, 2, 0, 1, 2)

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)
    
    def changeDevicedevCommandID(self):
        self.device.devCommandID = self.devCommandIDLetter.text()

    def _applyDefaultConnectionSettings(self):
        if not self.device.devCommandID:
            self.device.devCommandID = 'M'
        preferred_port = SerialPortComboBox.getPreferredSerialPortName(
            self.device.serialPortName)
        if preferred_port:
            self.device.serialPortName = preferred_port

    def connectDisconnectDeviceAction(self):
        if self.device.isConnected:
            self.device.disConnect()
        else:
            self._applyDefaultConnectionSettings()
            self.devCommandIDLetter.setText(self.device.devCommandID)
            connectionMode  = SimpleFOCDevice.PULL_CONFIG_ON_CONNECT
            self.device.connect(connectionMode)

    def connectionStateChanged(self, isConnected):
        if isConnected:
            self.connectDisconnectButton.setIcon(
                GUIToolKit.getIconByName('disconnect'))
            self.connectDisconnectButton.setText('断开')
        else:
            self.connectDisconnectButton.setIcon(
                GUIToolKit.getIconByName('connect'))
            self.connectDisconnectButton.setText('连接')

    def configureDeviceAction(self):
        dialog = ConfigureSerailConnectionDialog()
        result = dialog.exec_()
        if result:
            deviceConfig = dialog.getConfigValues()
            self.device.configureConnection(deviceConfig)
            self.devCommandIDLetter.setText(self.device.devCommandID)
