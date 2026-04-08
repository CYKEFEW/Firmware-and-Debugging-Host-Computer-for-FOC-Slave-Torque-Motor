#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

from src.simpleFOCConnector import SimpleFOCDevice


class ControlLoopGroupBox(QtWidgets.QGroupBox):
    PASSIVE_TORQUE_INDEX = 5
    RELEASE_INDEX = 6

    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()

        self.setObjectName('controlLoop')
        self.setTitle('\u8fd0\u52a8\u63a7\u5236\u7c7b\u578b')

        self.controlLoopHorizontalLayout = QtWidgets.QHBoxLayout(self)
        self.controlLoopHorizontalLayout.setObjectName('controlLoopHorizontalLayout')

        self.selectorControlLoop = QtWidgets.QComboBox(self)
        self.selectorControlLoop.setObjectName('selectorControlLoop')
        self.selectorControlLoop.addItems([
            '\u529b\u77e9\u63a7\u5236',
            '\u901f\u5ea6\u63a7\u5236',
            '\u89d2\u5ea6\u63a7\u5236',
            '\u901f\u5ea6\u5f00\u73af\u63a7\u5236',
            '\u89d2\u5ea6\u5f00\u73af\u63a7\u5236',
            '\u4ece\u52a8\u529b\u77e9\u63a7\u5236',
            '\u91ca\u653e',
        ])
        self.selectorControlLoop.currentIndexChanged.connect(self.changeControlLoop)
        self.controlLoopHorizontalLayout.addWidget(self.selectorControlLoop)

        self.setControlLopMode(self.device.controlType)

        self.disableUI()
        self.device.addConnectionStateListener(self)
        self.device.commProvider.commandDataReceived.connect(
            self.commandResponseReceived)

        self.connectionStateChanged(self.device.isConnected)

    def connectionStateChanged(self, deviceConnected):
        if deviceConnected is True:
            self.enabeUI()
        else:
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)

    def disableUI(self):
        self.setEnabled(False)

    def setControlLopMode(self, value):
        self.selectorControlLoop.blockSignals(True)
        if self.device.passiveTorqueMode:
            self.selectorControlLoop.setCurrentIndex(self.PASSIVE_TORQUE_INDEX)
        elif self.device.releaseMode:
            self.selectorControlLoop.setCurrentIndex(self.RELEASE_INDEX)
        elif value == SimpleFOCDevice.TORQUE_CONTROL:
            self.selectorControlLoop.setCurrentIndex(0)
        elif value == SimpleFOCDevice.VELOCITY_CONTROL:
            self.selectorControlLoop.setCurrentIndex(1)
        elif value == SimpleFOCDevice.ANGLE_CONTROL:
            self.selectorControlLoop.setCurrentIndex(2)
        elif value == SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL:
            self.selectorControlLoop.setCurrentIndex(3)
        elif value == SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL:
            self.selectorControlLoop.setCurrentIndex(4)
        self.selectorControlLoop.blockSignals(False)

    def _safeTargetForMode(self, control_type):
        if control_type in (
                SimpleFOCDevice.ANGLE_CONTROL,
                SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL):
            return str(self.device.angleNow)
        return '0'

    def _exitCustomModes(self):
        if self.device.releaseMode:
            self.device.sendReleaseMode(0)
        if self.device.passiveTorqueMode:
            self.device.sendPassiveTorqueMode(0)

    def _resumeStandardControl(self, control_type):
        self._exitCustomModes()
        if self.device.deviceStatus == 0:
            self.device.sendDeviceStatus(1)
        self.device.sendControlType(control_type)
        self.device.sendTargetValue(self._safeTargetForMode(control_type))

    def _enterPassiveTorqueMode(self):
        if self.device.releaseMode:
            self.device.sendReleaseMode(0)
        if self.device.deviceStatus == 0:
            self.device.sendDeviceStatus(1)
        self.device.sendTorqueType(SimpleFOCDevice.FOC_CURRENT_TORQUE)
        self.device.sendPassiveTorqueMode(1)

    def _enterReleaseMode(self):
        if self.device.passiveTorqueMode:
            self.device.sendPassiveTorqueMode(0)
        self.device.sendTargetValue(self._safeTargetForMode(self.device.controlType))
        self.device.sendReleaseMode(1)

    def changeControlLoop(self):
        index = self.selectorControlLoop.currentIndex()
        if index == 0:
            self._resumeStandardControl(SimpleFOCDevice.TORQUE_CONTROL)
        elif index == 1:
            self._resumeStandardControl(SimpleFOCDevice.VELOCITY_CONTROL)
        elif index == 2:
            self._resumeStandardControl(SimpleFOCDevice.ANGLE_CONTROL)
        elif index == 3:
            self._resumeStandardControl(SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL)
        elif index == 4:
            self._resumeStandardControl(SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL)
        elif index == self.PASSIVE_TORQUE_INDEX:
            self._enterPassiveTorqueMode()
        elif index == self.RELEASE_INDEX:
            self._enterReleaseMode()

    def commandResponseReceived(self, cmdRespose):
        self.setControlLopMode(self.device.controlType)
