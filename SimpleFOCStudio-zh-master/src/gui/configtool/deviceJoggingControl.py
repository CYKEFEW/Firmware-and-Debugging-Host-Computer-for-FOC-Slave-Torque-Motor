#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtGui, QtWidgets, QtCore

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice


class DeviceJoggingControl(QtWidgets.QGroupBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.device = SimpleFOCDevice.getInstance()
        self._targetDirty = False
        self._passiveVelOnDirty = False
        self._passiveVelOffDirty = False

        self.setObjectName('joggingControl')
        self.setTitle('\u70b9\u52a8\u63a7\u5236')

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setObjectName('generalControlGL')
        self.gridLayout.setHorizontalSpacing(8)
        self.gridLayout.setVerticalSpacing(6)

        self.fastFordwardButton = QtWidgets.QPushButton()
        self.fastFordwardButton.setObjectName('fastbackward')
        self.fastFordwardButton.setIcon(GUIToolKit.getIconByName('fastbackward'))
        self.fastFordwardButton.clicked.connect(self.joggingFastBackward)
        self.gridLayout.addWidget(self.fastFordwardButton, 0, 0)

        self.backwardButton = QtWidgets.QPushButton()
        self.backwardButton.setObjectName('backward')
        self.backwardButton.setIcon(GUIToolKit.getIconByName('backward'))
        self.backwardButton.clicked.connect(self.joggingBackward)
        self.gridLayout.addWidget(self.backwardButton, 0, 1)

        self.stopButton = QtWidgets.QPushButton()
        self.stopButton.setObjectName('stopbutton')
        self.stopButton.setIcon(GUIToolKit.getIconByName('stopjogging'))
        self.stopButton.clicked.connect(self.joggingStop)
        self.gridLayout.addWidget(self.stopButton, 0, 2)

        self.fordwardButton = QtWidgets.QPushButton()
        self.fordwardButton.setObjectName('fordward')
        self.fordwardButton.setIcon(GUIToolKit.getIconByName('fordward'))
        self.fordwardButton.clicked.connect(self.joggingFordward)
        self.gridLayout.addWidget(self.fordwardButton, 0, 3)

        self.fastBackwardButton = QtWidgets.QPushButton()
        self.fastBackwardButton.setObjectName('fastfordward')
        self.fastBackwardButton.setIcon(GUIToolKit.getIconByName('fastfordward'))
        self.fastBackwardButton.clicked.connect(self.joggingfastFordward)
        self.gridLayout.addWidget(self.fastBackwardButton, 0, 4)

        onlyFloat = QtGui.QRegExpValidator(
            QtCore.QRegExp("[+-]?([0-9]*[.])?[0-9]+"))

        self.incrementLabel = QtWidgets.QLabel()
        self.gridLayout.addWidget(self.incrementLabel, 1, 0, 1, 2)

        self.incrementEdit = QtWidgets.QLineEdit()
        self.incrementEdit.setValidator(onlyFloat)
        self.incrementEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.incrementEdit.setText('1.0')
        self.incrementEdit.setObjectName('incrementEdit')
        self.gridLayout.addWidget(self.incrementEdit, 1, 2, 1, 3)

        self.targetLabel = QtWidgets.QLabel()
        self.gridLayout.addWidget(self.targetLabel, 2, 0, 1, 2)

        self.targetEdit = QtWidgets.QLineEdit()
        self.targetEdit.setValidator(onlyFloat)
        self.targetEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.targetEdit.setText('0.0')
        self.targetEdit.setObjectName('targetEdit')
        self.targetEdit.textEdited.connect(self._markTargetDirty)
        self.targetEdit.returnPressed.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.targetEdit, 2, 2, 1, 2)

        self.applyTargetButton = QtWidgets.QPushButton('\u5e94\u7528')
        self.applyTargetButton.setObjectName('applyTargetButton')
        self.applyTargetButton.clicked.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.applyTargetButton, 2, 4)

        self.passiveVelOnLabel = QtWidgets.QLabel('\u8d77\u63a7\u901f\u5ea6\u9608\u503c[rad/s]:')
        self.gridLayout.addWidget(self.passiveVelOnLabel, 3, 0, 1, 2)

        self.passiveVelOnEdit = QtWidgets.QLineEdit()
        self.passiveVelOnEdit.setValidator(onlyFloat)
        self.passiveVelOnEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.passiveVelOnEdit.setText('0.2')
        self.passiveVelOnEdit.textEdited.connect(self._markPassiveVelOnDirty)
        self.passiveVelOnEdit.returnPressed.connect(self.applyPassiveVelOn)
        self.gridLayout.addWidget(self.passiveVelOnEdit, 3, 2, 1, 2)

        self.applyPassiveVelOnButton = QtWidgets.QPushButton('\u5e94\u7528')
        self.applyPassiveVelOnButton.clicked.connect(self.applyPassiveVelOn)
        self.gridLayout.addWidget(self.applyPassiveVelOnButton, 3, 4)

        self.passiveVelOffLabel = QtWidgets.QLabel('\u91ca\u653e\u901f\u5ea6\u9608\u503c[rad/s]:')
        self.gridLayout.addWidget(self.passiveVelOffLabel, 4, 0, 1, 2)

        self.passiveVelOffEdit = QtWidgets.QLineEdit()
        self.passiveVelOffEdit.setValidator(onlyFloat)
        self.passiveVelOffEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.passiveVelOffEdit.setText('0.1')
        self.passiveVelOffEdit.textEdited.connect(self._markPassiveVelOffDirty)
        self.passiveVelOffEdit.returnPressed.connect(self.applyPassiveVelOff)
        self.gridLayout.addWidget(self.passiveVelOffEdit, 4, 2, 1, 2)

        self.applyPassiveVelOffButton = QtWidgets.QPushButton('\u5e94\u7528')
        self.applyPassiveVelOffButton.clicked.connect(self.applyPassiveVelOff)
        self.gridLayout.addWidget(self.applyPassiveVelOffButton, 4, 4)

        self.disableUI()
        self.refreshModeUi()

        self.device.addConnectionStateListener(self)
        self.device.commProvider.commandDataReceived.connect(
            self.commandResponseReceived)
        self.device.commProvider.stateMonitorReceived.connect(
            self.stateResponseReceived)

    def _isPassiveTorqueMode(self):
        return self.device.passiveTorqueMode

    def _currentValue(self):
        if self._isPassiveTorqueMode():
            return float(self.device.passiveTorqueTargetNm)
        return float(self.device.target)

    def _sendValue(self, value):
        if self._isPassiveTorqueMode():
            self.device.sendPassiveTorqueTarget(str(max(0.0, float(value))))
        else:
            self.device.sendTargetValue(str(value))

    def _setLineEditValue(self, line_edit, value):
        line_edit.blockSignals(True)
        line_edit.setText(str(value))
        line_edit.blockSignals(False)

    def _markTargetDirty(self, _text):
        self._targetDirty = True

    def _markPassiveVelOnDirty(self, _text):
        self._passiveVelOnDirty = True

    def _markPassiveVelOffDirty(self, _text):
        self._passiveVelOffDirty = True

    def refreshModeUi(self):
        if self._isPassiveTorqueMode():
            self.incrementLabel.setText('\u529b\u77e9\u6b65\u8fdb[Nm]:')
            self.targetLabel.setText('\u76ee\u6807\u963b\u5c3c\u529b\u77e9[Nm]:')
            self.passiveVelOnLabel.show()
            self.passiveVelOnEdit.show()
            self.applyPassiveVelOnButton.show()
            self.passiveVelOffLabel.show()
            self.passiveVelOffEdit.show()
            self.applyPassiveVelOffButton.show()
            if not self._passiveVelOnDirty:
                self._setLineEditValue(
                    self.passiveVelOnEdit, self.device.passiveTorqueVelOn)
            if not self._passiveVelOffDirty:
                self._setLineEditValue(
                    self.passiveVelOffEdit, self.device.passiveTorqueVelOff)
        else:
            self.incrementLabel.setText('\u6b65\u8fdb\u503c:')
            self.targetLabel.setText('\u76ee\u6807\u503c:')
            self.passiveVelOnLabel.hide()
            self.passiveVelOnEdit.hide()
            self.applyPassiveVelOnButton.hide()
            self.passiveVelOffLabel.hide()
            self.passiveVelOffEdit.hide()
            self.applyPassiveVelOffButton.hide()

        if not self._targetDirty:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def connectionStateChanged(self, isConnectedFlag):
        if isConnectedFlag is True:
            self.enabeUI()
        else:
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)
        self.refreshModeUi()

    def disableUI(self):
        self.setEnabled(False)

    def commandResponseReceived(self, commandResponse):
        self.refreshModeUi()

    def stateResponseReceived(self, commandResponse):
        if not self._targetDirty:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def applyTargetValue(self):
        target = self.targetEdit.text().strip()
        if target == '':
            return
        self._sendValue(target)
        self._targetDirty = False
        self._setLineEditValue(self.targetEdit, self._currentValue())

    def applyPassiveVelOn(self):
        value = self.passiveVelOnEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueVelOn(value)
        self._passiveVelOnDirty = False
        self._setLineEditValue(
            self.passiveVelOnEdit, self.device.passiveTorqueVelOn)

    def applyPassiveVelOff(self):
        value = self.passiveVelOffEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueVelOff(value)
        self._passiveVelOffDirty = False
        self._setLineEditValue(
            self.passiveVelOffEdit, self.device.passiveTorqueVelOff)

    def _stepCurrentValue(self, delta):
        increment = self.incrementEdit.text().strip()
        if increment == '':
            return
        new_target = self._currentValue() + delta * float(increment)
        if self._isPassiveTorqueMode():
            new_target = max(0.0, new_target)
        self._sendValue(new_target)

    def joggingFastBackward(self):
        self._stepCurrentValue(-2.0)

    def joggingBackward(self):
        self._stepCurrentValue(-1.0)

    def joggingStop(self):
        control_type = self.device.controlType
        if self._isPassiveTorqueMode():
            self.device.sendPassiveTorqueTarget('0')
            return
        if (control_type == SimpleFOCDevice.ANGLE_CONTROL or
                control_type == SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL):
            self.device.sendTargetValue(str(self.device.angleNow))
        elif (control_type == SimpleFOCDevice.VELOCITY_CONTROL or
                control_type == SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL):
            self.device.sendTargetValue('0')

    def joggingFordward(self):
        self._stepCurrentValue(1.0)

    def joggingfastFordward(self):
        self._stepCurrentValue(2.0)
