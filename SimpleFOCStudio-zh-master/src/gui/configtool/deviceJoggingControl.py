#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice


class DeviceJoggingControl(QtWidgets.QGroupBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()
        self._targetDirty = False
        self._deadzoneDirty = False
        self._calculationRateDirty = False
        self._followPidPDirty = False
        self._followPidIDirty = False
        self._followPidDDirty = False

        self.setObjectName('joggingControl')
        self.setTitle('点动控制')

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

        float_validator = QtGui.QRegExpValidator(
            QtCore.QRegExp("[+-]?([0-9]*[.])?[0-9]+"))
        int_validator = QtGui.QRegExpValidator(QtCore.QRegExp("[1-9][0-9]*"))

        self.incrementLabel = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.incrementLabel, 1, 0, 1, 2)

        self.incrementEdit = QtWidgets.QLineEdit(self)
        self.incrementEdit.setValidator(float_validator)
        self.incrementEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.incrementEdit.setText('1.0')
        self.incrementEdit.setObjectName('incrementEdit')
        self.gridLayout.addWidget(self.incrementEdit, 1, 2, 1, 3)

        self.targetLabel = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.targetLabel, 2, 0, 1, 2)

        self.targetEdit = QtWidgets.QLineEdit(self)
        self.targetEdit.setValidator(float_validator)
        self.targetEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.targetEdit.setText('0.0')
        self.targetEdit.setObjectName('targetEdit')
        self.targetEdit.textEdited.connect(self._markTargetDirty)
        self.targetEdit.returnPressed.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.targetEdit, 2, 2, 1, 2)

        self.applyTargetButton = QtWidgets.QPushButton('应用')
        self.applyTargetButton.setObjectName('applyTargetButton')
        self.applyTargetButton.clicked.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.applyTargetButton, 2, 4)

        self.deadzoneLabel = QtWidgets.QLabel('跟随死区[°]:', self)
        self.gridLayout.addWidget(self.deadzoneLabel, 3, 0, 1, 2)

        self.deadzoneEdit = QtWidgets.QLineEdit(self)
        self.deadzoneEdit.setValidator(float_validator)
        self.deadzoneEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.deadzoneEdit.setText('0.8')
        self.deadzoneEdit.textEdited.connect(self._markDeadzoneDirty)
        self.deadzoneEdit.returnPressed.connect(self.applyDeadzone)
        self.gridLayout.addWidget(self.deadzoneEdit, 3, 2, 1, 2)

        self.applyDeadzoneButton = QtWidgets.QPushButton('应用')
        self.applyDeadzoneButton.clicked.connect(self.applyDeadzone)
        self.gridLayout.addWidget(self.applyDeadzoneButton, 3, 4)

        self.calculationRateLabel = QtWidgets.QLabel('计算频率[Hz]:', self)
        self.gridLayout.addWidget(self.calculationRateLabel, 4, 0, 1, 2)

        self.calculationRateEdit = QtWidgets.QLineEdit(self)
        self.calculationRateEdit.setValidator(int_validator)
        self.calculationRateEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.calculationRateEdit.setText('1000')
        self.calculationRateEdit.textEdited.connect(
            self._markCalculationRateDirty)
        self.calculationRateEdit.returnPressed.connect(
            self.applyCalculationRate)
        self.gridLayout.addWidget(self.calculationRateEdit, 4, 2, 1, 2)

        self.applyCalculationRateButton = QtWidgets.QPushButton('应用')
        self.applyCalculationRateButton.clicked.connect(self.applyCalculationRate)
        self.gridLayout.addWidget(self.applyCalculationRateButton, 4, 4)

        self.followPidPLabel = QtWidgets.QLabel('跟随 PID P:', self)
        self.gridLayout.addWidget(self.followPidPLabel, 5, 0, 1, 2)

        self.followPidPEdit = QtWidgets.QLineEdit(self)
        self.followPidPEdit.setValidator(float_validator)
        self.followPidPEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.followPidPEdit.setText('0.25')
        self.followPidPEdit.textEdited.connect(self._markFollowPidPDirty)
        self.followPidPEdit.returnPressed.connect(self.applyFollowPidP)
        self.gridLayout.addWidget(self.followPidPEdit, 5, 2, 1, 2)

        self.applyFollowPidPButton = QtWidgets.QPushButton('应用')
        self.applyFollowPidPButton.clicked.connect(self.applyFollowPidP)
        self.gridLayout.addWidget(self.applyFollowPidPButton, 5, 4)

        self.followPidILabel = QtWidgets.QLabel('跟随 PID I:', self)
        self.gridLayout.addWidget(self.followPidILabel, 6, 0, 1, 2)

        self.followPidIEdit = QtWidgets.QLineEdit(self)
        self.followPidIEdit.setValidator(float_validator)
        self.followPidIEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.followPidIEdit.setText('0.0')
        self.followPidIEdit.textEdited.connect(self._markFollowPidIDirty)
        self.followPidIEdit.returnPressed.connect(self.applyFollowPidI)
        self.gridLayout.addWidget(self.followPidIEdit, 6, 2, 1, 2)

        self.applyFollowPidIButton = QtWidgets.QPushButton('应用')
        self.applyFollowPidIButton.clicked.connect(self.applyFollowPidI)
        self.gridLayout.addWidget(self.applyFollowPidIButton, 6, 4)

        self.followPidDLabel = QtWidgets.QLabel('跟随 PID D:', self)
        self.gridLayout.addWidget(self.followPidDLabel, 7, 0, 1, 2)

        self.followPidDEdit = QtWidgets.QLineEdit(self)
        self.followPidDEdit.setValidator(float_validator)
        self.followPidDEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.followPidDEdit.setText('0.0')
        self.followPidDEdit.textEdited.connect(self._markFollowPidDDirty)
        self.followPidDEdit.returnPressed.connect(self.applyFollowPidD)
        self.gridLayout.addWidget(self.followPidDEdit, 7, 2, 1, 2)

        self.applyFollowPidDButton = QtWidgets.QPushButton('应用')
        self.applyFollowPidDButton.clicked.connect(self.applyFollowPidD)
        self.gridLayout.addWidget(self.applyFollowPidDButton, 7, 4)

        self.disableUI()
        self.refreshModeUi()

        self.device.addConnectionStateListener(self)
        self.device.commProvider.commandDataReceived.connect(
            self.commandResponseReceived)
        self.device.commProvider.stateMonitorReceived.connect(
            self.stateResponseReceived)
        self.connectionStateChanged(self.device.isConnected)

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

    def _markDeadzoneDirty(self, _text):
        self._deadzoneDirty = True

    def _markCalculationRateDirty(self, _text):
        self._calculationRateDirty = True

    def _markFollowPidPDirty(self, _text):
        self._followPidPDirty = True

    def _markFollowPidIDirty(self, _text):
        self._followPidIDirty = True

    def _markFollowPidDDirty(self, _text):
        self._followPidDDirty = True

    def refreshModeUi(self):
        if self._isPassiveTorqueMode():
            self.incrementLabel.setText('力矩步进[Nm]:')
            self.targetLabel.setText('目标阻尼力矩[Nm]:')

            self.deadzoneLabel.show()
            self.deadzoneEdit.show()
            self.applyDeadzoneButton.show()
            self.calculationRateLabel.show()
            self.calculationRateEdit.show()
            self.applyCalculationRateButton.show()
            self.followPidPLabel.show()
            self.followPidPEdit.show()
            self.applyFollowPidPButton.show()
            self.followPidILabel.show()
            self.followPidIEdit.show()
            self.applyFollowPidIButton.show()
            self.followPidDLabel.show()
            self.followPidDEdit.show()
            self.applyFollowPidDButton.show()

            if not self._deadzoneDirty:
                self._setLineEditValue(
                    self.deadzoneEdit,
                    self.device.passiveTorqueFollowDeadzoneDeg)
            if not self._calculationRateDirty:
                self._setLineEditValue(
                    self.calculationRateEdit,
                    self.device.passiveTorqueCalculationHz)
            if not self._followPidPDirty:
                self._setLineEditValue(
                    self.followPidPEdit,
                    self.device.passiveTorqueFollowPidP)
            if not self._followPidIDirty:
                self._setLineEditValue(
                    self.followPidIEdit,
                    self.device.passiveTorqueFollowPidI)
            if not self._followPidDDirty:
                self._setLineEditValue(
                    self.followPidDEdit,
                    self.device.passiveTorqueFollowPidD)
        else:
            self.incrementLabel.setText('步进值:')
            self.targetLabel.setText('目标值:')

            self.deadzoneLabel.hide()
            self.deadzoneEdit.hide()
            self.applyDeadzoneButton.hide()
            self.calculationRateLabel.hide()
            self.calculationRateEdit.hide()
            self.applyCalculationRateButton.hide()
            self.followPidPLabel.hide()
            self.followPidPEdit.hide()
            self.applyFollowPidPButton.hide()
            self.followPidILabel.hide()
            self.followPidIEdit.hide()
            self.applyFollowPidIButton.hide()
            self.followPidDLabel.hide()
            self.followPidDEdit.hide()
            self.applyFollowPidDButton.hide()

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

    def commandResponseReceived(self, _commandResponse):
        self.refreshModeUi()

    def stateResponseReceived(self, _commandResponse):
        if not self._targetDirty:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def applyTargetValue(self):
        target = self.targetEdit.text().strip()
        if target == '':
            return
        self._sendValue(target)
        self._targetDirty = False
        self._setLineEditValue(self.targetEdit, self._currentValue())

    def applyDeadzone(self):
        value = self.deadzoneEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowDeadzone(value)
        self._deadzoneDirty = False
        self._setLineEditValue(
            self.deadzoneEdit,
            self.device.passiveTorqueFollowDeadzoneDeg)

    def applyCalculationRate(self):
        value = self.calculationRateEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueCalculationHz(value)
        self._calculationRateDirty = False
        self._setLineEditValue(
            self.calculationRateEdit,
            self.device.passiveTorqueCalculationHz)

    def applyFollowPidP(self):
        value = self.followPidPEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidP(value)
        self._followPidPDirty = False
        self._setLineEditValue(
            self.followPidPEdit,
            self.device.passiveTorqueFollowPidP)

    def applyFollowPidI(self):
        value = self.followPidIEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidI(value)
        self._followPidIDirty = False
        self._setLineEditValue(
            self.followPidIEdit,
            self.device.passiveTorqueFollowPidI)

    def applyFollowPidD(self):
        value = self.followPidDEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidD(value)
        self._followPidDDirty = False
        self._setLineEditValue(
            self.followPidDEdit,
            self.device.passiveTorqueFollowPidD)

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
        if control_type in (
                SimpleFOCDevice.ANGLE_CONTROL,
                SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL):
            self.device.sendTargetValue(str(self.device.angleNow))
        elif control_type in (
                SimpleFOCDevice.VELOCITY_CONTROL,
                SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL):
            self.device.sendTargetValue('0')

    def joggingFordward(self):
        self._stepCurrentValue(1.0)

    def joggingfastFordward(self):
        self._stepCurrentValue(2.0)
