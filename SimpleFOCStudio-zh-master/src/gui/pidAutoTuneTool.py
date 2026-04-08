#!/usr/bin/env python
# -*- coding: utf-8 -*-

import statistics
import threading
import time

from PyQt5 import QtCore, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit, WorkAreaTabWidget
from src.simpleFOCConnector import SimpleFOCDevice


LOOP_DEFINITIONS = (
    ('current_q', '电流 Q PID', True, True, True, False),
    ('current_d', '电流 D PID', False, True, True, False),
    ('velocity', '速度 PID', True, True, True, False),
    ('angle', '角度 PID', False, True, False, False),
    ('passive_follow', '从动力矩跟随 PID', False, True, False, False),
)


class PidAutoTuneWorker(QtCore.QThread):
    logMessage = QtCore.pyqtSignal(str)
    finishedWithStatus = QtCore.pyqtSignal(bool, str)

    def __init__(self, device, selected_options, max_iterations, parent=None):
        super().__init__(parent)
        self.device = device
        self.selected_options = selected_options
        self.max_iterations = max_iterations
        self.measurement_rate_hz = 50
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def _log(self, message):
        self.logMessage.emit(message)

    def _sleep(self, seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while not self.stopped() and time.monotonic() < deadline:
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))

    def _to_float(self, value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _loop_label(self, loop_name):
        for name, label, *_ in LOOP_DEFINITIONS:
            if name == loop_name:
                return label
        return loop_name

    def _loop_enabled(self, loop_name):
        return bool(self.selected_options.get(loop_name, {}).get('enabled'))

    def _loop_terms(self, loop_name):
        options = self.selected_options.get(loop_name, {})
        return {
            'P': bool(options.get('P')),
            'I': bool(options.get('I')),
            'D': bool(options.get('D')),
        }

    def _pid_for_loop(self, loop_name):
        if loop_name == 'current_q':
            return self.device.PIDCurrentQ
        if loop_name == 'current_d':
            return self.device.PIDCurrentD
        if loop_name == 'velocity':
            return self.device.PIDVelocity
        if loop_name == 'angle':
            return self.device.PIDAngle
        return None

    def _snapshot_loop_values(self, loop_name):
        if loop_name == 'passive_follow':
            return {
                'P': self._to_float(self.device.passiveTorqueFollowPidP),
                'I': self._to_float(self.device.passiveTorqueFollowPidI),
                'D': self._to_float(self.device.passiveTorqueFollowPidD),
            }
        pid = self._pid_for_loop(loop_name)
        return {
            'P': self._to_float(pid.P),
            'I': self._to_float(pid.I),
            'D': self._to_float(pid.D),
        }

    def _set_loop_values(self, loop_name, values):
        if loop_name == 'passive_follow':
            self.device.sendPassiveTorqueFollowPidP(str(values['P']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowPidI(str(values['I']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowPidD(str(values['D']))
            self._sleep(0.04)
            return

        pid = self._pid_for_loop(loop_name)
        self.device.sendProportionalGain(pid, str(values['P']))
        self._sleep(0.04)
        self.device.sendIntegralGain(pid, str(values['I']))
        self._sleep(0.04)
        self.device.sendDerivativeGain(pid, str(values['D']))
        self._sleep(0.04)

    def _measurement_attr(self, loop_name):
        if loop_name in ('current_q', 'current_d'):
            return 'currentQNow'
        if loop_name == 'velocity':
            return 'velocityNow'
        if loop_name == 'angle':
            return 'angleNow'
        if loop_name == 'passive_follow':
            return 'passiveTorqueDampingAngleDeg'
        raise ValueError(loop_name)

    def _spec_for_loop(self, loop_name):
        if loop_name in ('current_q', 'current_d'):
            step = min(max(self._to_float(self.device.currentLimit, 1.8) * 0.2, 0.12), 0.5)
            return {'step': step, 'settle': 0.25, 'capture': 0.60}
        if loop_name == 'velocity':
            step = min(max(self._to_float(self.device.velocityLimit, 20.0) * 0.08, 1.0), 6.0)
            return {'step': step, 'settle': 0.30, 'capture': 0.80}
        if loop_name == 'angle':
            return {'step': 0.35, 'settle': 0.35, 'capture': 1.00}
        deadzone = self._to_float(self.device.passiveTorqueFollowDeadzoneDeg, 0.8)
        max_angle = self._to_float(self.device.passiveTorqueMaxDampingAngleDeg, 1.0)
        return {
            'step': max(max_angle * 0.6, deadzone + 0.3, 0.8),
            'settle': 0.25,
            'capture': 0.55,
            'hold': 0.18,
        }

    def _capture_samples(self, loop_name, duration_s):
        attr_name = self._measurement_attr(loop_name)
        interval_s = 1.0 / max(1, self.measurement_rate_hz)
        samples = []
        started = time.monotonic()
        while not self.stopped():
            elapsed = time.monotonic() - started
            if elapsed > duration_s:
                break
            samples.append((elapsed, self._to_float(getattr(self.device, attr_name, 0.0))))
            time.sleep(interval_s)
        return samples

    def _evaluate_step_response(self, samples, target_delta):
        if not samples:
            return {'score': 1e9, 'peak': 0.0, 'steady': 0.0, 'overshoot': 0.0}
        values = [value for _, value in samples]
        head_count = max(2, len(values) // 8)
        tail_count = max(3, len(values) // 5)
        baseline = statistics.fmean(values[:head_count])
        steady = statistics.fmean(values[-tail_count:]) - baseline
        target_abs = max(abs(target_delta), 1e-6)
        direction = 1.0 if target_delta >= 0 else -1.0
        shifted = [(value - baseline) * direction for value in values]
        peak = max(shifted)
        overshoot = max(0.0, peak - target_abs) / target_abs
        final_error = abs(steady - target_delta) / target_abs
        noise = statistics.pstdev(values[-tail_count:]) / target_abs
        band = max(target_abs * 0.12, 1e-4)
        settling_ratio = 1.0
        for index, shifted_value in enumerate(shifted):
            if abs(shifted_value - target_abs) <= band:
                settling_ratio = samples[index][0] / max(samples[-1][0], 1e-6)
                break
        score = final_error + overshoot * 1.8 + noise * 0.5 + settling_ratio * 0.2
        return {
            'score': score,
            'peak': peak,
            'steady': steady,
            'overshoot': overshoot,
        }

    def _evaluate_decay_response(self, samples):
        if not samples:
            return {'score': 1e9, 'peak': 0.0, 'residual': 0.0}
        values = [abs(value) for _, value in samples]
        peak = max(values)
        peak_ref = max(peak, 1e-6)
        tail_count = max(3, len(values) // 4)
        residual = statistics.fmean(values[-tail_count:])
        noise = statistics.pstdev(values[-tail_count:])
        area = statistics.fmean(values)
        score = residual / peak_ref + noise / peak_ref * 0.6 + area / peak_ref * 0.25
        return {
            'score': score,
            'peak': peak,
            'residual': residual,
        }

    def _combine_passive_follow_metrics(self, metrics_list):
        if not metrics_list:
            return {'score': 1e9}
        scores = [metric['score'] for metric in metrics_list]
        return {
            'score': statistics.fmean(scores) + max(scores) * 0.15,
            'peak': max(metric.get('peak', 0.0) for metric in metrics_list),
            'residual': statistics.fmean(
                metric.get('residual', 0.0) for metric in metrics_list),
        }

    def _send_zero_target(self, loop_name):
        if loop_name == 'angle':
            self.device.sendTargetValue(str(self._to_float(self.device.angleNow)))
        else:
            self.device.sendTargetValue('0')

    def _passive_follow_test_target_nm(self):
        value = self._to_float(self.device.passiveTorqueTargetNm, 0.03)
        return value if value >= 0.01 else 0.03

    def _passive_follow_step_count(self):
        return 20

    def _configure_standard_loop(self, loop_name):
        self.device.sendPassiveTorqueMode('0')
        self._sleep(0.05)
        self.device.sendReleaseMode('0')
        self._sleep(0.05)
        self.device.sendDeviceStatus('1')
        self._sleep(0.05)
        self.device.sendTorqueType(SimpleFOCDevice.FOC_CURRENT_TORQUE)
        self._sleep(0.05)
        if loop_name in ('current_q', 'current_d'):
            self.device.sendControlType(SimpleFOCDevice.TORQUE_CONTROL)
        elif loop_name == 'velocity':
            self.device.sendControlType(SimpleFOCDevice.VELOCITY_CONTROL)
        else:
            self.device.sendControlType(SimpleFOCDevice.ANGLE_CONTROL)
        self._sleep(0.08)
        self._send_zero_target(loop_name)
        self._sleep(0.20)

    def _configure_passive_follow_loop(self):
        test_target = self._passive_follow_test_target_nm()
        self.device.sendReleaseMode('0')
        self._sleep(0.05)
        self.device.sendDeviceStatus('1')
        self._sleep(0.05)
        self.device.sendPassiveTorqueTarget(str(test_target))
        self._sleep(0.05)
        self.device.sendPassiveTorqueFieldOffset('0')
        self._sleep(0.05)
        self.device.sendPassiveTorqueMode('1')
        self._sleep(0.20)
        self._log(
            '从动力矩跟随 PID 使用 %.4f Nm 终值，按 %d 挡阻尼力矩步进测试。'
            % (test_target, self._passive_follow_step_count()))

    def _configure_loop(self, loop_name):
        if loop_name == 'passive_follow':
            self._configure_passive_follow_loop()
        else:
            self._configure_standard_loop(loop_name)

    def _collect_standard_metrics(self, loop_name):
        spec = self._spec_for_loop(loop_name)
        base_value = 0.0
        target_value = spec['step']
        if loop_name == 'angle':
            base_value = self._to_float(self.device.angleNow)
            target_value = base_value + spec['step']
        self.device.sendTargetValue(str(target_value))
        samples = self._capture_samples(loop_name, spec['capture'])
        if loop_name == 'angle':
            self.device.sendTargetValue(str(base_value))
        else:
            self.device.sendTargetValue('0')
        self._sleep(spec['settle'])
        return self._evaluate_step_response(samples, spec['step'])

    def _collect_passive_follow_metrics(self):
        spec = self._spec_for_loop('passive_follow')
        final_target_nm = self._passive_follow_test_target_nm()
        metrics = []
        step_count = self._passive_follow_step_count()
        for step_index in range(1, step_count + 1):
            if self.stopped():
                break
            target_nm = final_target_nm * step_index / step_count
            self.device.sendPassiveTorqueTarget(str(target_nm))
            self._sleep(0.12)
            self.device.sendPassiveTorqueFieldOffset('0')
            self._sleep(spec['settle'])
            self.device.sendPassiveTorqueFieldOffset(str(spec['step']))
            hold_samples = self._capture_samples('passive_follow', spec['hold'])
            self.device.sendPassiveTorqueFieldOffset('0')
            decay_samples = self._capture_samples('passive_follow', spec['capture'])
            metrics.append(
                self._evaluate_decay_response(hold_samples + decay_samples))
        self.device.sendPassiveTorqueFieldOffset('0')
        self.device.sendPassiveTorqueTarget(str(final_target_nm))
        self._sleep(spec['settle'])
        return self._combine_passive_follow_metrics(metrics)

    def _collect_metrics(self, loop_name):
        if loop_name == 'passive_follow':
            return self._collect_passive_follow_metrics()
        return self._collect_standard_metrics(loop_name)

    def _candidate_defaults(self, loop_name, term_name):
        defaults = {
            'current_q': {'P': 1.0, 'I': 20.0, 'D': 0.0},
            'current_d': {'P': 1.0, 'I': 20.0, 'D': 0.0},
            'velocity': {'P': 0.2, 'I': 5.0, 'D': 0.0},
            'angle': {'P': 10.0, 'I': 0.0, 'D': 0.0},
            'passive_follow': {'P': 0.25, 'I': 0.0, 'D': 0.0},
        }
        return defaults[loop_name][term_name]

    def _candidate_values(self, loop_name, term_name, current_value):
        current_value = max(0.0, self._to_float(current_value))
        if current_value <= 0.0:
            base = self._candidate_defaults(loop_name, term_name)
            if term_name == 'D' and base <= 0.0:
                base = max(self._candidate_defaults(loop_name, 'P') * 0.05, 0.001)
            return sorted(set([0.0, base * 0.5, base, base * 1.8]))
        if term_name == 'P':
            return sorted(set([current_value * 0.6, current_value, current_value * 1.6]))
        if term_name == 'I':
            return sorted(set([current_value * 0.4, current_value, current_value * 1.8]))
        return sorted(set([0.0, current_value * 0.5, current_value, current_value * 2.0]))

    def _optimize_term(self, loop_name, base_values, term_name):
        best_values = dict(base_values)
        for iteration in range(self.max_iterations):
            if self.stopped():
                break
            candidates = self._candidate_values(loop_name, term_name, best_values[term_name])
            best_score = None
            best_candidate = best_values[term_name]
            for candidate in candidates:
                trial_values = dict(best_values)
                trial_values[term_name] = max(0.0, candidate)
                self._set_loop_values(loop_name, trial_values)
                self._sleep(0.10)
                metrics = self._collect_metrics(loop_name)
                score = metrics['score']
                self._log(
                    '%s | %s 迭代 %d | P=%.5f I=%.5f D=%.5f | score=%.4f'
                    % (
                        self._loop_label(loop_name),
                        term_name,
                        iteration + 1,
                        trial_values['P'],
                        trial_values['I'],
                        trial_values['D'],
                        score))
                if best_score is None or score < best_score:
                    best_score = score
                    best_candidate = trial_values[term_name]
            best_values[term_name] = best_candidate
            self._set_loop_values(loop_name, best_values)
        return best_values

    def _tune_loop(self, loop_name):
        if not self._loop_enabled(loop_name):
            return None
        terms = self._loop_terms(loop_name)
        if not any(terms.values()):
            self._log('%s 已跳过：未勾选任何 PID 项。' % self._loop_label(loop_name))
            return None
        self._configure_loop(loop_name)
        best_values = self._snapshot_loop_values(loop_name)
        for term_name in ('P', 'I', 'D'):
            if terms[term_name]:
                best_values = self._optimize_term(loop_name, best_values, term_name)
        self._set_loop_values(loop_name, best_values)
        if loop_name == 'passive_follow':
            self.device.sendPassiveTorqueFieldOffset('0')
        else:
            self._send_zero_target(loop_name)
        return best_values

    def _restore_runtime_state(self, state):
        self.device.setStateUpdateRateHz(state['state_rate'])
        self.device.sendMonitorDownsample(state['monitor_downsample'])
        if state['monitor_variables']:
            self.device.sendMonitorVariables(state['monitor_variables'])
        else:
            self.device.sendMonitorClearVariables()
        self._sleep(0.05)
        self.device.sendPassiveTorqueFieldOffset('0')
        self._sleep(0.05)
        self.device.sendPassiveTorqueTarget(str(state['passive_target']))
        self._sleep(0.05)
        self.device.sendPassiveTorqueFollowDeadzone(str(state['passive_deadzone']))
        self._sleep(0.05)
        self.device.sendPassiveTorqueMode('1' if state['passive_mode'] else '0')
        self._sleep(0.05)
        self.device.sendReleaseMode('1' if state['release_mode'] else '0')
        self._sleep(0.05)
        self.device.sendTorqueType(state['torque_type'])
        self._sleep(0.05)
        self.device.sendControlType(state['control_type'])
        self._sleep(0.05)
        self.device.sendTargetValue(str(state['target']))
        self._sleep(0.05)
        self.device.sendDeviceStatus(str(state['device_status']))
        self._sleep(0.05)

    def run(self):
        if not self.device.isConnected:
            self.finishedWithStatus.emit(False, '设备未连接。')
            return
        state = {
            'state_rate': self.device.stateUpdateRateHz,
            'monitor_downsample': self.device.monitorDownsample,
            'monitor_variables': self.device.monitorVariables if isinstance(
                self.device.monitorVariables, list) else [],
            'control_type': self.device.controlType,
            'torque_type': self.device.torqueType,
            'target': self._to_float(self.device.target),
            'device_status': int(self._to_float(self.device.deviceStatus, 1)),
            'release_mode': bool(self.device.releaseMode),
            'passive_mode': bool(self.device.passiveTorqueMode),
            'passive_target': self._to_float(self.device.passiveTorqueTargetNm),
            'passive_deadzone': self._to_float(self.device.passiveTorqueFollowDeadzoneDeg),
        }
        results = []
        try:
            with self.device.pullConfigurationLock:
                was_updating_states = (
                    self.device.stateUpdater is not None and
                    not self.device.stateUpdater.stopped())
                if was_updating_states:
                    self.device.stateUpdater.stop()
                    self.device.stateUpdater.wait(500)
                self.device.sendMonitorDownsample(0)
                self.device.sendMonitorClearVariables()
                self.device.setStateUpdateRateHz(self.measurement_rate_hz)
                self._sleep(0.10)
                for loop_name, *_ in LOOP_DEFINITIONS:
                    if self.stopped():
                        break
                    tuned_values = self._tune_loop(loop_name)
                    if tuned_values is not None:
                        results.append(
                            '%s: P=%.5f I=%.5f D=%.5f'
                            % (
                                self._loop_label(loop_name),
                                tuned_values['P'],
                                tuned_values['I'],
                                tuned_values['D']))
                self._restore_runtime_state(state)
                if was_updating_states and self.device.isConnected:
                    self.device.stateUpdater = self.device.stateUpdater.__class__(self.device)
                    self.device.stateUpdater.start()
        except Exception as error:
            try:
                self._restore_runtime_state(state)
            except Exception:
                pass
            self.finishedWithStatus.emit(False, 'PID 自动测量失败：%s' % error)
            return

        if self.stopped():
            self.finishedWithStatus.emit(False, 'PID 自动测量已停止。')
            return
        summary = '；'.join(results) if results else '未执行任何测量。'
        self.finishedWithStatus.emit(True, summary)


class PidAutoTuneTool(WorkAreaTabWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.device = SimpleFOCDevice.getInstance()
        self.worker = None
        self.loopRows = {}

        root_layout = QtWidgets.QVBoxLayout(self)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)

        title_label = QtWidgets.QLabel('PID自动测量')
        title_font = title_label.font()
        title_font.setPointSize(title_font.pointSize() + 2)
        title_font.setBold(True)
        title_label.setFont(title_font)
        root_layout.addWidget(title_label)

        matrix_group = QtWidgets.QGroupBox('测量项')
        matrix_layout = QtWidgets.QGridLayout(matrix_group)
        matrix_layout.setHorizontalSpacing(12)
        matrix_layout.setVerticalSpacing(6)
        headers = ('PID项', '启用', '调P', '调I', '调D')
        for column, header in enumerate(headers):
            label = QtWidgets.QLabel(header)
            label_font = label.font()
            label_font.setBold(True)
            label.setFont(label_font)
            matrix_layout.addWidget(label, 0, column)

        for row_index, (loop_name, label, enable_p, enable_i, enable_d, _) in enumerate(
                LOOP_DEFINITIONS, start=1):
            name_label = QtWidgets.QLabel(label)
            matrix_layout.addWidget(name_label, row_index, 0)

            enabled_box = QtWidgets.QCheckBox()
            enabled_box.setChecked(True)
            matrix_layout.addWidget(enabled_box, row_index, 1)

            p_box = QtWidgets.QCheckBox()
            p_box.setChecked(enable_p)
            matrix_layout.addWidget(p_box, row_index, 2)

            i_box = QtWidgets.QCheckBox()
            i_box.setChecked(enable_i)
            matrix_layout.addWidget(i_box, row_index, 3)

            d_box = QtWidgets.QCheckBox()
            d_box.setChecked(enable_d)
            matrix_layout.addWidget(d_box, row_index, 4)

            self.loopRows[loop_name] = {
                'enabled': enabled_box,
                'P': p_box,
                'I': i_box,
                'D': d_box,
            }

        root_layout.addWidget(matrix_group)

        options_layout = QtWidgets.QHBoxLayout()
        options_layout.addWidget(QtWidgets.QLabel('最大迭代次数:'))
        self.maxIterationsSpin = QtWidgets.QSpinBox()
        self.maxIterationsSpin.setRange(1, 20)
        self.maxIterationsSpin.setValue(6)
        options_layout.addWidget(self.maxIterationsSpin)
        options_layout.addStretch(1)
        root_layout.addLayout(options_layout)

        note_label = QtWidgets.QLabel(
            '说明：自动测量会暂时接管控制模式；从动力矩跟随 PID 使用磁场偏置衰减法，'
            '目标阻尼力矩固定按 20 挡步进扫描；电流 D PID 仍使用 Q 轴响应做代理。')
        note_label.setWordWrap(True)
        root_layout.addWidget(note_label)

        button_layout = QtWidgets.QHBoxLayout()
        self.startButton = QtWidgets.QPushButton('开始测量')
        self.startButton.setIcon(GUIToolKit.getIconByName('start'))
        self.startButton.clicked.connect(self.startAutoTune)
        button_layout.addWidget(self.startButton)

        self.stopButton = QtWidgets.QPushButton('停止')
        self.stopButton.setIcon(GUIToolKit.getIconByName('stop'))
        self.stopButton.setEnabled(False)
        self.stopButton.clicked.connect(self.stopAutoTune)
        button_layout.addWidget(self.stopButton)
        button_layout.addStretch(1)
        root_layout.addLayout(button_layout)

        self.summaryLabel = QtWidgets.QLabel('当前未开始测量。')
        self.summaryLabel.setWordWrap(True)
        root_layout.addWidget(self.summaryLabel)

        self.logOutput = QtWidgets.QPlainTextEdit()
        self.logOutput.setReadOnly(True)
        self.logOutput.setMinimumHeight(260)
        root_layout.addWidget(self.logOutput, 1)

    def getTabIcon(self):
        return GUIToolKit.getIconByName('pid')

    def getTabName(self):
        return 'PID自动测量'

    def appendLog(self, message):
        self.logOutput.appendPlainText(message)
        self.logOutput.verticalScrollBar().setValue(
            self.logOutput.verticalScrollBar().maximum())

    def _buildSelectionOptions(self):
        options = {}
        for loop_name, widgets in self.loopRows.items():
            options[loop_name] = {
                'enabled': widgets['enabled'].isChecked(),
                'P': widgets['P'].isChecked(),
                'I': widgets['I'].isChecked(),
                'D': widgets['D'].isChecked(),
            }
        return options

    def startAutoTune(self):
        if self.worker is not None:
            return
        if not self.device.isConnected:
            QtWidgets.QMessageBox.warning(self, 'PID自动测量', '请先连接设备。')
            return
        selected_options = self._buildSelectionOptions()
        enabled_loops = [
            loop_name for loop_name, options in selected_options.items()
            if options['enabled']
        ]
        if not enabled_loops:
            QtWidgets.QMessageBox.warning(self, 'PID自动测量', '请至少勾选一个 PID 项。')
            return
        for loop_name in enabled_loops:
            terms = selected_options[loop_name]
            if not (terms['P'] or terms['I'] or terms['D']):
                QtWidgets.QMessageBox.warning(
                    self,
                    'PID自动测量',
                    '%s 已启用，但未勾选 P/I/D。' %
                    next(label for name, label, *_ in LOOP_DEFINITIONS if name == loop_name))
                return

        self.logOutput.clear()
        self.summaryLabel.setText('PID 自动测量进行中...')
        self.startButton.setEnabled(False)
        self.stopButton.setEnabled(True)

        self.worker = PidAutoTuneWorker(
            self.device,
            selected_options,
            self.maxIterationsSpin.value(),
            self)
        self.worker.logMessage.connect(self.appendLog)
        self.worker.finishedWithStatus.connect(self.handleFinished)
        self.worker.start()

    def stopAutoTune(self):
        if self.worker is not None:
            self.worker.stop()
            self.summaryLabel.setText('正在停止 PID 自动测量...')

    def handleFinished(self, success, summary):
        self.summaryLabel.setText(summary)
        self.startButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        if self.worker is not None:
            self.worker.deleteLater()
            self.worker = None
        if success:
            self.appendLog('测量完成。')
        else:
            self.appendLog('测量结束：%s' % summary)
