#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice


class DeviceJoggingControl(QtWidgets.QGroupBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()
        self._dirtyFlags = {
            'target': False,
            'saturation': False,
            'deadzone': False,
            'running_threshold': False,
            'calc': False,
            'low_p': False,
            'low_i': False,
            'low_d': False,
            'run_p': False,
            'run_i': False,
            'run_d': False,
        }

        self.setObjectName('joggingControl')
        self.setTitle('点动控制')

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setObjectName('generalControlGL')
        self.gridLayout.setHorizontalSpacing(8)
        self.gridLayout.setVerticalSpacing(6)

        self._buildJogButtons()
        self._buildEditors()

        self.disableUI()
        self.refreshModeUi()

        self.device.addConnectionStateListener(self)
        self.device.commProvider.commandDataReceived.connect(
            self.commandResponseReceived)
        self.device.commProvider.stateMonitorReceived.connect(
            self.stateResponseReceived)
        self.connectionStateChanged(self.device.isConnected)

    def _buildJogButtons(self):
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

    def _buildEditors(self):
        self.floatValidator = QtGui.QRegExpValidator(
            QtCore.QRegExp("[+-]?([0-9]*[.])?[0-9]+"))
        self.intValidator = QtGui.QRegExpValidator(QtCore.QRegExp("[1-9][0-9]*"))

        self.incrementLabel = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.incrementLabel, 1, 0, 1, 2)

        self.incrementEdit = QtWidgets.QLineEdit(self)
        self.incrementEdit.setValidator(self.floatValidator)
        self.incrementEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.incrementEdit.setText('1.0')
        self.gridLayout.addWidget(self.incrementEdit, 1, 2, 1, 3)

        self.targetLabel = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.targetLabel, 2, 0, 1, 2)

        self.targetEdit = QtWidgets.QLineEdit(self)
        self.targetEdit.setValidator(self.floatValidator)
        self.targetEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.targetEdit.setText('0.0')
        self.targetEdit.textEdited.connect(
            lambda _text: self._markDirty('target'))
        self.targetEdit.returnPressed.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.targetEdit, 2, 2, 1, 2)

        self.applyTargetButton = QtWidgets.QPushButton('应用')
        self.applyTargetButton.clicked.connect(self.applyTargetValue)
        self.gridLayout.addWidget(self.applyTargetButton, 2, 4)

        self.saturationAngleLabel, self.saturationAngleEdit, self.applySaturationAngleButton = (
            self._addEditableRow(
                3,
                '饱和磁场阻尼角[°]:',
                '2.0',
                self.floatValidator,
                'saturation',
                self.applySaturationAngle))

        self.deadzoneLabel, self.deadzoneEdit, self.applyDeadzoneButton = (
            self._addEditableRow(
                4,
                '跟随死区[°]:',
                '0.8',
                self.floatValidator,
                'deadzone',
                self.applyDeadzone))

        self.runningThresholdLabel, self.runningThresholdEdit, self.applyRunningThresholdButton = (
            self._addEditableRow(
                5,
                '旋转工况阈值[rad/s]:',
                '2.0',
                self.floatValidator,
                'running_threshold',
                self.applyRunningThreshold))

        self.calculationRateLabel, self.calculationRateEdit, self.applyCalculationRateButton = (
            self._addEditableRow(
                6,
                '计算频率[Hz]:',
                '1000',
                self.intValidator,
                'calc',
                self.applyCalculationRate))

        self.followLowPidPLabel, self.followLowPidPEdit, self.applyFollowLowPidPButton = (
            self._addEditableRow(
                7,
                '低速/静止跟随 PID P:',
                '0.25',
                self.floatValidator,
                'low_p',
                self.applyFollowLowPidP))

        self.followLowPidILabel, self.followLowPidIEdit, self.applyFollowLowPidIButton = (
            self._addEditableRow(
                8,
                '低速/静止跟随 PID I:',
                '0.0',
                self.floatValidator,
                'low_i',
                self.applyFollowLowPidI))

        self.followLowPidDLabel, self.followLowPidDEdit, self.applyFollowLowPidDButton = (
            self._addEditableRow(
                9,
                '低速/静止跟随 PID D:',
                '0.0',
                self.floatValidator,
                'low_d',
                self.applyFollowLowPidD))

        self.followRunPidPLabel, self.followRunPidPEdit, self.applyFollowRunPidPButton = (
            self._addEditableRow(
                10,
                '旋转工况跟随 PID P:',
                '0.08',
                self.floatValidator,
                'run_p',
                self.applyFollowRunPidP))

        self.followRunPidILabel, self.followRunPidIEdit, self.applyFollowRunPidIButton = (
            self._addEditableRow(
                11,
                '旋转工况跟随 PID I:',
                '0.0',
                self.floatValidator,
                'run_i',
                self.applyFollowRunPidI))

        self.followRunPidDLabel, self.followRunPidDEdit, self.applyFollowRunPidDButton = (
            self._addEditableRow(
                12,
                '旋转工况跟随 PID D:',
                '0.0',
                self.floatValidator,
                'run_d',
                self.applyFollowRunPidD))

    def _addEditableRow(self, row, label_text, default_text, validator, dirty_key, apply_slot):
        label = QtWidgets.QLabel(label_text, self)
        self.gridLayout.addWidget(label, row, 0, 1, 2)

        edit = QtWidgets.QLineEdit(self)
        edit.setValidator(validator)
        edit.setAlignment(QtCore.Qt.AlignCenter)
        edit.setText(default_text)
        edit.textEdited.connect(lambda _text, key=dirty_key: self._markDirty(key))
        edit.returnPressed.connect(apply_slot)
        self.gridLayout.addWidget(edit, row, 2, 1, 2)

        button = QtWidgets.QPushButton('应用')
        button.clicked.connect(apply_slot)
        self.gridLayout.addWidget(button, row, 4)
        return label, edit, button

    def _isPassiveTorqueMode(self):
        return self.device.passiveTorqueMode

    def _markDirty(self, key):
        self._dirtyFlags[key] = True

    def _setLineEditValue(self, line_edit, value):
        line_edit.blockSignals(True)
        line_edit.setText(str(value))
        line_edit.blockSignals(False)

    def _currentValue(self):
        if self._isPassiveTorqueMode():
            return float(self.device.passiveTorqueTargetNm)
        return float(self.device.target)

    def _sendValue(self, value):
        if self._isPassiveTorqueMode():
            self.device.sendPassiveTorqueTarget(str(max(0.0, float(value))))
        else:
            self.device.sendTargetValue(str(value))

    def _syncPassiveTorqueEditors(self):
        bindings = (
            ('saturation', self.saturationAngleEdit,
             self.device.passiveTorqueSaturationAngleDeg),
            ('deadzone', self.deadzoneEdit,
             self.device.passiveTorqueFollowDeadzoneDeg),
            ('running_threshold', self.runningThresholdEdit,
             self.device.passiveTorqueRunningSpeedThresholdRadS),
            ('calc', self.calculationRateEdit,
             self.device.passiveTorqueCalculationHz),
            ('low_p', self.followLowPidPEdit,
             self.device.passiveTorqueFollowLowPidP),
            ('low_i', self.followLowPidIEdit,
             self.device.passiveTorqueFollowLowPidI),
            ('low_d', self.followLowPidDEdit,
             self.device.passiveTorqueFollowLowPidD),
            ('run_p', self.followRunPidPEdit,
             self.device.passiveTorqueFollowRunPidP),
            ('run_i', self.followRunPidIEdit,
             self.device.passiveTorqueFollowRunPidI),
            ('run_d', self.followRunPidDEdit,
             self.device.passiveTorqueFollowRunPidD),
        )
        for dirty_key, line_edit, value in bindings:
            if not self._dirtyFlags[dirty_key]:
                self._setLineEditValue(line_edit, value)

    def refreshModeUi(self):
        passive_widgets = (
            self.saturationAngleLabel,
            self.saturationAngleEdit,
            self.applySaturationAngleButton,
            self.deadzoneLabel,
            self.deadzoneEdit,
            self.applyDeadzoneButton,
            self.runningThresholdLabel,
            self.runningThresholdEdit,
            self.applyRunningThresholdButton,
            self.calculationRateLabel,
            self.calculationRateEdit,
            self.applyCalculationRateButton,
            self.followLowPidPLabel,
            self.followLowPidPEdit,
            self.applyFollowLowPidPButton,
            self.followLowPidILabel,
            self.followLowPidIEdit,
            self.applyFollowLowPidIButton,
            self.followLowPidDLabel,
            self.followLowPidDEdit,
            self.applyFollowLowPidDButton,
            self.followRunPidPLabel,
            self.followRunPidPEdit,
            self.applyFollowRunPidPButton,
            self.followRunPidILabel,
            self.followRunPidIEdit,
            self.applyFollowRunPidIButton,
            self.followRunPidDLabel,
            self.followRunPidDEdit,
            self.applyFollowRunPidDButton,
        )

        if self._isPassiveTorqueMode():
            self.incrementLabel.setText('力矩步进[Nm]:')
            self.targetLabel.setText('目标阻尼力矩[Nm]:')
            for widget in passive_widgets:
                widget.show()
            self._syncPassiveTorqueEditors()
        else:
            self.incrementLabel.setText('步进值:')
            self.targetLabel.setText('目标值:')
            for widget in passive_widgets:
                widget.hide()

        if not self._dirtyFlags['target']:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def connectionStateChanged(self, isConnectedFlag):
        if isConnectedFlag:
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
        if not self._dirtyFlags['target']:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def applyTargetValue(self):
        target = self.targetEdit.text().strip()
        if target == '':
            return
        self._sendValue(target)
        self._dirtyFlags['target'] = False
        self._setLineEditValue(self.targetEdit, self._currentValue())

    def applySaturationAngle(self):
        value = self.saturationAngleEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueSaturationAngle(value)
        self._dirtyFlags['saturation'] = False
        self._setLineEditValue(
            self.saturationAngleEdit,
            self.device.passiveTorqueSaturationAngleDeg)

    def applyDeadzone(self):
        value = self.deadzoneEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowDeadzone(value)
        self._dirtyFlags['deadzone'] = False
        self._setLineEditValue(
            self.deadzoneEdit,
            self.device.passiveTorqueFollowDeadzoneDeg)

    def applyRunningThreshold(self):
        value = self.runningThresholdEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueRunningSpeedThreshold(value)
        self._dirtyFlags['running_threshold'] = False
        self._setLineEditValue(
            self.runningThresholdEdit,
            self.device.passiveTorqueRunningSpeedThresholdRadS)

    def applyCalculationRate(self):
        value = self.calculationRateEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueCalculationHz(value)
        self._dirtyFlags['calc'] = False
        self._setLineEditValue(
            self.calculationRateEdit,
            self.device.passiveTorqueCalculationHz)

    def applyFollowLowPidP(self):
        value = self.followLowPidPEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidP(value)
        self._dirtyFlags['low_p'] = False
        self._setLineEditValue(
            self.followLowPidPEdit,
            self.device.passiveTorqueFollowLowPidP)

    def applyFollowLowPidI(self):
        value = self.followLowPidIEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidI(value)
        self._dirtyFlags['low_i'] = False
        self._setLineEditValue(
            self.followLowPidIEdit,
            self.device.passiveTorqueFollowLowPidI)

    def applyFollowLowPidD(self):
        value = self.followLowPidDEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowPidD(value)
        self._dirtyFlags['low_d'] = False
        self._setLineEditValue(
            self.followLowPidDEdit,
            self.device.passiveTorqueFollowLowPidD)

    def applyFollowRunPidP(self):
        value = self.followRunPidPEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowRunPidP(value)
        self._dirtyFlags['run_p'] = False
        self._setLineEditValue(
            self.followRunPidPEdit,
            self.device.passiveTorqueFollowRunPidP)

    def applyFollowRunPidI(self):
        value = self.followRunPidIEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowRunPidI(value)
        self._dirtyFlags['run_i'] = False
        self._setLineEditValue(
            self.followRunPidIEdit,
            self.device.passiveTorqueFollowRunPidI)

    def applyFollowRunPidD(self):
        value = self.followRunPidDEdit.text().strip()
        if value == '':
            return
        self.device.sendPassiveTorqueFollowRunPidD(value)
        self._dirtyFlags['run_d'] = False
        self._setLineEditValue(
            self.followRunPidDEdit,
            self.device.passiveTorqueFollowRunPidD)

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
