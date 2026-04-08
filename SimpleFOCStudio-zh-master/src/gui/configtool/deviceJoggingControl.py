#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time

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
        self._autoDebugActive = False
        self._autoDebugSamples = []
        self._autoDebugOriginalRateHz = None
        self._autoDebugOriginalTargetNm = None
        self._autoDebugOriginalVelOn = None
        self._autoDebugOriginalVelOff = None
        self._autoDebugStages = []
        self._autoDebugStageIndex = -1
        self._autoDebugStageResults = []
        self._autoDebugActiveTest = None
        self._autoDebugTestId = 0
        self._autoDebugPhase = 'idle'

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

        self.autoDebugStageCountLabel = QtWidgets.QLabel(
            '\u626b\u63cf\u6321\u6570:')
        self.gridLayout.addWidget(self.autoDebugStageCountLabel, 5, 0, 1, 2)

        self.autoDebugStageCountSpin = QtWidgets.QSpinBox(self)
        self.autoDebugStageCountSpin.setRange(4, 50)
        self.autoDebugStageCountSpin.setValue(20)
        self.gridLayout.addWidget(self.autoDebugStageCountSpin, 5, 2, 1, 3)

        self.autoDebugButton = QtWidgets.QPushButton(
            '\u81ea\u52a8\u8c03\u8bd5\u9608\u503c')
        self.autoDebugButton.clicked.connect(self.startPassiveThresholdAutoDebug)
        self.gridLayout.addWidget(self.autoDebugButton, 6, 0, 1, 5)

        self.autoDebugTimer = QtCore.QTimer(self)
        self.autoDebugTimer.setSingleShot(True)
        self.autoDebugTimer.timeout.connect(self.advancePassiveThresholdAutoDebug)

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

    def _resetAutoDebugUi(self):
        self.autoDebugTimer.stop()
        self.autoDebugButton.setEnabled(True)
        self.autoDebugStageCountSpin.setEnabled(True)
        self.autoDebugButton.setText('\u81ea\u52a8\u8c03\u8bd5\u9608\u503c')

    def refreshModeUi(self):
        show_passive_controls = self._isPassiveTorqueMode() or self._autoDebugActive
        if show_passive_controls:
            self.incrementLabel.setText('\u529b\u77e9\u6b65\u8fdb[Nm]:')
            self.targetLabel.setText('\u76ee\u6807\u963b\u5c3c\u529b\u77e9[Nm]:')
            self.passiveVelOnLabel.show()
            self.passiveVelOnEdit.show()
            self.applyPassiveVelOnButton.show()
            self.passiveVelOffLabel.show()
            self.passiveVelOffEdit.show()
            self.applyPassiveVelOffButton.show()
            self.autoDebugStageCountLabel.show()
            self.autoDebugStageCountSpin.show()
            self.autoDebugButton.show()
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
            self.autoDebugStageCountLabel.hide()
            self.autoDebugStageCountSpin.hide()
            self.autoDebugButton.hide()

        if not self._targetDirty:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def connectionStateChanged(self, isConnectedFlag):
        if isConnectedFlag is True:
            self.enabeUI()
        else:
            if self._autoDebugActive:
                self._autoDebugActive = False
                self._autoDebugPhase = 'idle'
                self._autoDebugActiveTest = None
                self._resetAutoDebugUi()
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)
        self.refreshModeUi()

    def disableUI(self):
        self.setEnabled(False)

    def commandResponseReceived(self, commandResponse):
        self.refreshModeUi()

    def stateResponseReceived(self, commandResponse):
        if self._autoDebugActive and self._autoDebugActiveTest is not None:
            self._autoDebugSamples.append((
                time.monotonic(),
                float(self.device.angleNow),
                float(self.device.velocityNow),
                self._autoDebugActiveTest['id'],
            ))
        if not self._targetDirty:
            self._setLineEditValue(self.targetEdit, self._currentValue())

    def _percentile(self, values, q):
        if not values:
            return 0.0
        ordered = sorted(values)
        position = (len(ordered) - 1) * q
        lower_index = int(math.floor(position))
        upper_index = int(math.ceil(position))
        if lower_index == upper_index:
            return ordered[lower_index]
        lower_value = ordered[lower_index]
        upper_value = ordered[upper_index]
        weight = position - lower_index
        return lower_value * (1.0 - weight) + upper_value * weight

    def _unwrapAngles(self, angles):
        if not angles:
            return []
        unwrapped = [angles[0]]
        angle_offset = 0.0
        previous = angles[0]
        for angle in angles[1:]:
            delta = angle - previous
            if delta > math.pi:
                angle_offset -= 2.0 * math.pi
            elif delta < -math.pi:
                angle_offset += 2.0 * math.pi
            unwrapped.append(angle + angle_offset)
            previous = angle
        return unwrapped

    def _candidateVelOn(self, vel_off):
        return round(max(vel_off + 0.4, vel_off * 1.8), 2)

    def _buildTorqueLevels(self, target_torque, stage_count):
        if stage_count <= 1:
            return [round(target_torque, 4)]
        min_torque = max(0.005, target_torque * 0.1)
        if target_torque <= min_torque:
            return [round(target_torque, 4)]
        levels = []
        for index in range(stage_count):
            ratio = index / float(stage_count - 1)
            torque = round(
                min_torque + (target_torque - min_torque) * ratio,
                4)
            if not levels or abs(torque - levels[-1]) > 1e-4:
                levels.append(torque)
        return levels

    def _initialThresholdBounds(self, torque, previous_vel_off):
        low = 0.15 if previous_vel_off is None else max(0.15, previous_vel_off * 0.7)
        high = max(
            low + 0.6,
            1.2 if previous_vel_off is None else previous_vel_off * 1.5,
            torque * 45.0)
        return round(low, 2), round(min(high, 12.0), 2)

    def _angleStabilityLimit(self, torque):
        return 0.025 + torque * 0.30

    def _stageStatusText(self, stage, candidate_vel_off=None):
        if candidate_vel_off is None:
            candidate_vel_off = 0.0
        return '\u8c03\u8bd5 {}/{}  {:.3f}Nm  {:.2f}rad/s'.format(
            self._autoDebugStageIndex + 1,
            len(self._autoDebugStages),
            stage['torque'],
            candidate_vel_off)

    def _analyzeAutoDebugTest(self, test):
        test_samples = [
            sample for sample in self._autoDebugSamples
            if sample[3] == test['id'] and
            sample[0] >= test['sample_start'] and
            sample[0] <= test['sample_end']
        ]
        if len(test_samples) < 8:
            return None

        timestamps = [sample[0] for sample in test_samples]
        angles = [sample[1] for sample in test_samples]
        velocities = [abs(sample[2]) for sample in test_samples]
        unwrapped_angles = self._unwrapAngles(angles)
        derived_velocities = []
        for index in range(1, len(test_samples)):
            dt = timestamps[index] - timestamps[index - 1]
            if dt <= 1e-6:
                continue
            derived_velocities.append(
                abs((unwrapped_angles[index] - unwrapped_angles[index - 1]) / dt))

        angle_span = max(unwrapped_angles) - min(unwrapped_angles)
        disturbance = max(
            self._percentile(velocities, 0.98),
            self._percentile(derived_velocities, 0.98))
        angle_limit = self._angleStabilityLimit(test['torque'])
        stable = (
            angle_span <= angle_limit and
            disturbance <= max(0.20, test['vel_off'] * 0.85))

        return {
            'sample_count': len(test_samples),
            'angle_span': angle_span,
            'disturbance': disturbance,
            'angle_limit': angle_limit,
            'stable': stable,
            'vel_on': test['vel_on'],
            'vel_off': test['vel_off'],
            'torque': test['torque'],
        }

    def _startAutoDebugTest(self, stage):
        candidate_vel_off = round((stage['low'] + stage['high']) * 0.5, 2)
        candidate_vel_on = self._candidateVelOn(candidate_vel_off)
        test_duration_ms = 650
        settle_ms = 220

        self.device.sendPassiveTorqueVelOff(str(candidate_vel_off))
        self.device.sendPassiveTorqueVelOn(str(candidate_vel_on))
        self.device.sendPassiveTorqueTarget(str(stage['torque']))
        self._targetDirty = False
        self._passiveVelOnDirty = False
        self._passiveVelOffDirty = False
        self._setLineEditValue(self.targetEdit, stage['torque'])
        self._setLineEditValue(self.passiveVelOnEdit, candidate_vel_on)
        self._setLineEditValue(self.passiveVelOffEdit, candidate_vel_off)

        start_time = time.monotonic()
        self._autoDebugTestId += 1
        self._autoDebugActiveTest = {
            'id': self._autoDebugTestId,
            'stage_index': self._autoDebugStageIndex,
            'iteration': stage['iteration'],
            'torque': stage['torque'],
            'vel_on': candidate_vel_on,
            'vel_off': candidate_vel_off,
            'sample_start': start_time + settle_ms / 1000.0,
            'sample_end': start_time + test_duration_ms / 1000.0,
        }

        self.autoDebugButton.setText(
            self._stageStatusText(stage, candidate_vel_off))
        self.autoDebugTimer.start(test_duration_ms)

    def _completeAutoDebugTest(self):
        if self._autoDebugActiveTest is None:
            return

        stage = self._autoDebugStages[self._autoDebugStageIndex]
        metrics = self._analyzeAutoDebugTest(self._autoDebugActiveTest)
        if metrics is None:
            metrics = {
                'sample_count': 0,
                'angle_span': float('inf'),
                'disturbance': float('inf'),
                'angle_limit': self._angleStabilityLimit(stage['torque']),
                'stable': False,
                'vel_on': self._autoDebugActiveTest['vel_on'],
                'vel_off': self._autoDebugActiveTest['vel_off'],
                'torque': stage['torque'],
            }

        stage['tests'].append(metrics)
        if metrics['stable']:
            stage['best_vel_off'] = metrics['vel_off']
            stage['best_vel_on'] = metrics['vel_on']
            stage['best_metrics'] = metrics
            stage['high'] = metrics['vel_off']
        else:
            stage['low'] = metrics['vel_off']

        stage['iteration'] += 1
        if abs(stage['high'] - stage['low']) <= 0.10:
            stage['iteration'] = stage['max_iterations']

        self._autoDebugActiveTest = None

    def _finalizeAutoDebugStage(self, stage):
        recommended_vel_off = stage['best_vel_off']
        if recommended_vel_off is None:
            recommended_vel_off = round(stage['high'], 2)
        recommended_vel_on = self._candidateVelOn(recommended_vel_off)
        best_metrics = stage['best_metrics']
        if best_metrics is None:
            best_metrics = {
                'sample_count': 0,
                'angle_span': 0.0,
                'disturbance': 0.0,
            }

        self._autoDebugStageResults.append({
            'torque': stage['torque'],
            'recommended_vel_on': recommended_vel_on,
            'recommended_vel_off': recommended_vel_off,
            'sample_count': best_metrics['sample_count'],
            'angle_span': best_metrics['angle_span'],
            'disturbance': best_metrics['disturbance'],
            'tests': list(stage['tests']),
        })

    def startPassiveThresholdAutoDebug(self):
        if self._autoDebugActive or not self._isPassiveTorqueMode():
            return
        self._autoDebugActive = True
        self._autoDebugSamples = []
        self._autoDebugStages = []
        self._autoDebugStageIndex = -1
        self._autoDebugStageResults = []
        self._autoDebugActiveTest = None
        self._autoDebugTestId = 0
        self._autoDebugPhase = 'release'
        self._autoDebugOriginalRateHz = self.device.stateUpdateRateHz
        self._autoDebugOriginalTargetNm = float(self.device.passiveTorqueTargetNm)
        self._autoDebugOriginalVelOn = float(self.device.passiveTorqueVelOn)
        self._autoDebugOriginalVelOff = float(self.device.passiveTorqueVelOff)
        target_torque = max(0.03, float(self.device.passiveTorqueTargetNm))
        stage_count = int(self.autoDebugStageCountSpin.value())
        torque_levels = self._buildTorqueLevels(target_torque, stage_count)
        self.autoDebugButton.setEnabled(False)
        self.autoDebugStageCountSpin.setEnabled(False)
        self.autoDebugButton.setText('\u91ca\u653e\u7a33\u5b9a\u4e2d...')
        self.device.stateUpdateRateHz = max(60, int(self.device.stateUpdateRateHz))
        for torque in torque_levels:
            self._autoDebugStages.append({
                'torque': torque,
                'iteration': 0,
                'max_iterations': 3,
                'low': 0.0,
                'high': 0.0,
                'best_vel_off': None,
                'best_vel_on': None,
                'best_metrics': None,
                'tests': [],
            })
        QtWidgets.QMessageBox.information(
            self,
            '\u81ea\u52a8\u8c03\u8bd5',
            '\u6d4b\u8bd5\u4f1a\u5148\u91ca\u653e\u7535\u673a 3 \u79d2\uff0c'
            '\u7136\u540e\u6309 {} \u6321\u76ee\u6807\u963b\u5c3c\u529b\u77e9\u9010\u6b65\u6d4b\u8bd5\uff0c'
            '\u6bcf\u4e00\u6321\u90fd\u4f1a\u7528\u8fed\u4ee3\u65b9\u5f0f\u641c\u7d22\u6700\u5c0f\u7a33\u5b9a\u901f\u5ea6\u9608\u503c\u3002'
            '\u8bf7\u5c3d\u91cf\u4fdd\u6301\u7535\u673a\u8f74\u9759\u6b62\u3002'.format(
                len(self._autoDebugStages)))
        self.device.sendPassiveTorqueDebug('1')
        self.device.sendPassiveTorqueTarget('0')
        self.device.sendPassiveTorqueMode(0)
        self.device.sendReleaseMode(1)
        self.autoDebugTimer.start(3000)

    def advancePassiveThresholdAutoDebug(self):
        if not self._autoDebugActive:
            return

        if self._autoDebugPhase == 'release':
            self.device.sendReleaseMode(0)
            self.device.sendPassiveTorqueMode(1)
            self._autoDebugPhase = 'testing'
            self._autoDebugStageIndex = 0
            self.autoDebugButton.setText('\u5efa\u7acb\u6d4b\u8bd5\u72b6\u6001...')
            self.autoDebugTimer.start(250)
            return

        if self._autoDebugActiveTest is not None:
            self._completeAutoDebugTest()

        while self._autoDebugStageIndex < len(self._autoDebugStages):
            stage = self._autoDebugStages[self._autoDebugStageIndex]
            if stage['iteration'] == 0 and stage['low'] == 0.0 and stage['high'] == 0.0:
                previous_vel_off = None
                if self._autoDebugStageResults:
                    previous_vel_off = self._autoDebugStageResults[-1]['recommended_vel_off']
                stage['low'], stage['high'] = self._initialThresholdBounds(
                    stage['torque'], previous_vel_off)

            if stage['iteration'] >= stage['max_iterations']:
                self._finalizeAutoDebugStage(stage)
                self._autoDebugStageIndex += 1
                continue

            self._startAutoDebugTest(stage)
            return

        self.finishPassiveThresholdAutoDebug()

    def finishPassiveThresholdAutoDebug(self):
        if not self._autoDebugActive:
            return

        original_rate_hz = self._autoDebugOriginalRateHz
        original_target_nm = self._autoDebugOriginalTargetNm

        self._autoDebugActive = False
        self._autoDebugPhase = 'idle'
        self._autoDebugActiveTest = None
        self.device.stateUpdateRateHz = original_rate_hz
        self.device.sendPassiveTorqueDebug('0')
        self.device.sendReleaseMode(0)
        self.device.sendPassiveTorqueMode(1)
        if original_target_nm is not None:
            self.device.sendPassiveTorqueTarget(str(original_target_nm))
            self._setLineEditValue(self.targetEdit, original_target_nm)
        self._resetAutoDebugUi()

        if not self._autoDebugStageResults:
            if self._autoDebugOriginalVelOn is not None:
                self.device.sendPassiveTorqueVelOn(str(self._autoDebugOriginalVelOn))
            if self._autoDebugOriginalVelOff is not None:
                self.device.sendPassiveTorqueVelOff(str(self._autoDebugOriginalVelOff))
            QtWidgets.QMessageBox.warning(
                self,
                '\u81ea\u52a8\u8c03\u8bd5',
                '\u91c7\u6837\u6570\u636e\u4e0d\u8db3\uff0c\u8bf7\u4fdd\u6301\u7535\u673a\u9759\u6b62\u540e\u91cd\u8bd5\u3002')
            return

        recommended_vel_off = max(
            stage['recommended_vel_off']
            for stage in self._autoDebugStageResults)
        recommended_vel_on = self._candidateVelOn(recommended_vel_off)

        self.device.sendPassiveTorqueVelOn(str(recommended_vel_on))
        self.device.sendPassiveTorqueVelOff(str(recommended_vel_off))
        self._passiveVelOnDirty = False
        self._passiveVelOffDirty = False
        self._setLineEditValue(self.passiveVelOnEdit, recommended_vel_on)
        self._setLineEditValue(self.passiveVelOffEdit, recommended_vel_off)

        total_duration = 0.0
        if self._autoDebugSamples:
            total_duration = self._autoDebugSamples[-1][0] - self._autoDebugSamples[0][0]
        detail_lines = [
            '\u91c7\u6837\u65f6\u957f: {:.2f}s'.format(total_duration),
            '\u91c7\u6837\u6321\u6570: {}'.format(len(self._autoDebugStageResults)),
            '\u5efa\u8bae\u8d77\u63a7\u9608\u503c: {:.2f} rad/s'.format(recommended_vel_on),
            '\u5efa\u8bae\u91ca\u653e\u9608\u503c: {:.2f} rad/s'.format(recommended_vel_off),
        ]
        for stage_result in self._autoDebugStageResults:
            detail_lines.append(
                '  {:.3f}Nm -> \u9608\u503c {:.2f}/{:.2f} rad/s, \u89d2\u5ea6\u6ce2\u52a8 {:.4f} rad, \u6270\u52a8\u901f\u5ea6 {:.4f} rad/s'.format(
                    stage_result['torque'],
                    stage_result['recommended_vel_on'],
                    stage_result['recommended_vel_off'],
                    stage_result['angle_span'],
                    stage_result['disturbance']))
        QtWidgets.QMessageBox.information(
            self,
            '\u81ea\u52a8\u8c03\u8bd5',
            '\n'.join(detail_lines))

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
