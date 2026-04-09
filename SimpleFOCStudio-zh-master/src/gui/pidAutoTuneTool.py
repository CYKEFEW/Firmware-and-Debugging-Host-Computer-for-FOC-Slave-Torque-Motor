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
    ('passive_follow_static', '从动力矩跟随 PID（低速/静止）', False, True, False, False),
    ('passive_follow_running', '从动力矩跟随 PID（正常旋转）', False, True, False, False),
)


class PidAutoTuneWorker(QtCore.QThread):
    logMessage = QtCore.pyqtSignal(str)
    statusMessage = QtCore.pyqtSignal(str)
    finishedWithStatus = QtCore.pyqtSignal(bool, str)

    def __init__(
            self,
            device,
            selected_options,
            max_iterations,
            passive_follow_final_target_nm,
            passive_follow_step_count,
            parent=None):
        super().__init__(parent)
        self.device = device
        self.selected_options = selected_options
        self.max_iterations = max_iterations
        self.passive_follow_final_target_nm = passive_follow_final_target_nm
        self.passive_follow_step_count = passive_follow_step_count
        self.measurement_rate_hz = 50
        self.rotation_detect_hold_s = 0.30
        self.rotation_detect_timeout_s = 10.0
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def _log(self, message):
        self.logMessage.emit(message)

    def _status(self, message):
        self.statusMessage.emit(message)
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
        if loop_name == 'passive_follow_static':
            return {
                'P': self._to_float(self.device.passiveTorqueFollowLowPidP),
                'I': self._to_float(self.device.passiveTorqueFollowLowPidI),
                'D': self._to_float(self.device.passiveTorqueFollowLowPidD),
            }
        if loop_name == 'passive_follow_running':
            return {
                'P': self._to_float(self.device.passiveTorqueFollowRunPidP),
                'I': self._to_float(self.device.passiveTorqueFollowRunPidI),
                'D': self._to_float(self.device.passiveTorqueFollowRunPidD),
            }
        pid = self._pid_for_loop(loop_name)
        return {
            'P': self._to_float(pid.P),
            'I': self._to_float(pid.I),
            'D': self._to_float(pid.D),
        }

    def _set_loop_values(self, loop_name, values):
        if loop_name == 'passive_follow_static':
            self.device.sendPassiveTorqueFollowPidP(str(values['P']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowPidI(str(values['I']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowPidD(str(values['D']))
            self._sleep(0.04)
            return
        if loop_name == 'passive_follow_running':
            self.device.sendPassiveTorqueFollowRunPidP(str(values['P']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowRunPidI(str(values['I']))
            self._sleep(0.04)
            self.device.sendPassiveTorqueFollowRunPidD(str(values['D']))
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
        if loop_name in ('passive_follow_static', 'passive_follow_running'):
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
        return {
            'step': max(deadzone + 0.4, 0.8),
            'settle': 0.18,
            'capture': 0.45,
            'hold': 0.18,
        }

    def _poll_device_state(self, loop_name):
        self.device.updateStates()
        self._sleep(0.012 if loop_name.startswith('passive_follow') else 0.010)

    def _capture_samples(self, loop_name, duration_s):
        attr_name = self._measurement_attr(loop_name)
        interval_s = 1.0 / max(1, self.measurement_rate_hz)
        samples = []
        started = time.monotonic()
        while not self.stopped():
            elapsed = time.monotonic() - started
            if elapsed > duration_s:
                break
            self._poll_device_state(loop_name)
            samples.append((elapsed, self._to_float(getattr(self.device, attr_name, 0.0))))
            self._sleep(max(0.0, interval_s - 0.012))
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
            'noise': noise,
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
            'noise': noise,
            'area': area,
        }

    def _combine_passive_follow_metrics(self, metrics_list):
        if not metrics_list:
            return {'score': 1e9}
        scores = [metric['score'] for metric in metrics_list]
        return {
            'score': statistics.fmean(scores) + max(scores) * 0.15,
            'peak': max(metric.get('peak', 0.0) for metric in metrics_list),
            'residual': statistics.fmean(metric.get('residual', 0.0) for metric in metrics_list),
            'noise': statistics.fmean(metric.get('noise', 0.0) for metric in metrics_list),
            'score_min': min(scores),
            'score_max': max(scores),
            'sample_count': len(metrics_list),
        }

    def _format_metrics_summary(self, metrics):
        if not metrics:
            return ''
        parts = ['score=%.4f' % metrics.get('score', 0.0)]
        if 'steady' in metrics:
            parts.append('steady=%.4f' % metrics.get('steady', 0.0))
        if 'residual' in metrics:
            parts.append('residual=%.4f' % metrics.get('residual', 0.0))
        if 'peak' in metrics:
            parts.append('peak=%.4f' % metrics.get('peak', 0.0))
        if 'overshoot' in metrics:
            parts.append('overshoot=%.4f' % metrics.get('overshoot', 0.0))
        if 'noise' in metrics:
            parts.append('noise=%.4f' % metrics.get('noise', 0.0))
        if 'score_min' in metrics:
            parts.append('score_min=%.4f' % metrics.get('score_min', 0.0))
        if 'score_max' in metrics:
            parts.append('score_max=%.4f' % metrics.get('score_max', 0.0))
        if 'sample_count' in metrics:
            parts.append('steps=%d' % metrics.get('sample_count', 0))
        return ' | '.join(parts)

    def _send_zero_target(self, loop_name):
        if loop_name == 'angle':
            self.device.sendTargetValue(str(self._to_float(self.device.angleNow)))
        else:
            self.device.sendTargetValue('0')

    def _passive_follow_test_target_nm(self):
        value = self._to_float(self.passive_follow_final_target_nm, 0.05)
        return value if value >= 0.01 else 0.05

    def _passive_follow_step_count(self):
        return max(2, int(self.passive_follow_step_count))

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

    def _configure_passive_follow_loop(self, loop_name):
        test_target = self._passive_follow_test_target_nm()
        spec = self._spec_for_loop(loop_name)
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
        pid_values = self._snapshot_loop_values(loop_name)
        self._log(
            '%s 配置 | final_target=%.4f Nm | steps=%d | deadzone=%.3f deg | calc=%d Hz | '
            'pid=(%.5f, %.5f, %.5f) | offset_step=%.3f deg'
            % (
                self._loop_label(loop_name),
                test_target,
                self._passive_follow_step_count(),
                self._to_float(self.device.passiveTorqueFollowDeadzoneDeg),
                int(self._to_float(self.device.passiveTorqueCalculationHz, 1000)),
                pid_values['P'],
                pid_values['I'],
                pid_values['D'],
                spec['step']))

    def _configure_loop(self, loop_name):
        if loop_name.startswith('passive_follow'):
            self._configure_passive_follow_loop(loop_name)
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

    def _wait_for_manual_rotation(self):
        threshold = max(
            0.1,
            self._to_float(
                getattr(self.device, 'passiveTorqueRunningSpeedThresholdRadS', 2.0),
                2.0))
        self._status(
            '请手动持续旋转电机，检测到速度超过 %.2f rad/s 后开始测量。'
            % threshold)
        deadline = time.monotonic() + self.rotation_detect_timeout_s
        hold_start = None
        while not self.stopped() and time.monotonic() < deadline:
            self.device.updateStates()
            velocity = abs(self._to_float(self.device.velocityNow))
            if velocity >= threshold:
                if hold_start is None:
                    hold_start = time.monotonic()
                if (time.monotonic() - hold_start) >= self.rotation_detect_hold_s:
                    self._log(
                        '已检测到旋转速度 %.4f rad/s，进入正常旋转工况测量。'
                        % velocity)
                    return True
            else:
                hold_start = None
            self._sleep(0.03)
        self._log('未在规定时间内检测到足够旋转速度，本轮按失败计分。')
        return False

    def _collect_passive_follow_metrics(self, loop_name):
        spec = self._spec_for_loop(loop_name)
        final_target_nm = self._passive_follow_test_target_nm()
        step_count = self._passive_follow_step_count()
        metrics = []

        if loop_name == 'passive_follow_running' and not self._wait_for_manual_rotation():
            return {'score': 1e9, 'peak': 0.0, 'residual': 0.0, 'sample_count': 0}

        for step_index in range(1, step_count + 1):
            if self.stopped():
                break
            target_nm = final_target_nm * step_index / step_count
            self._log(
                '%s 挡位 %02d/%02d | target=%.4f Nm | offset_step=%.3f deg'
                % (self._loop_label(loop_name), step_index, step_count, target_nm, spec['step']))
            self.device.sendPassiveTorqueTarget(str(target_nm))
            self._sleep(0.12)
            self.device.sendPassiveTorqueFieldOffset('0')
            self._sleep(spec['settle'])
            self.device.sendPassiveTorqueFieldOffset(str(spec['step']))
            hold_samples = self._capture_samples(loop_name, spec['hold'])
            self.device.sendPassiveTorqueFieldOffset('0')
            decay_samples = self._capture_samples(loop_name, spec['capture'])
            step_metrics = self._evaluate_decay_response(hold_samples + decay_samples)
            metrics.append(step_metrics)
            self._log(
                '%s 挡位 %02d/%02d 结果 | %s'
                % (self._loop_label(loop_name), step_index, step_count,
                   self._format_metrics_summary(step_metrics)))

        self.device.sendPassiveTorqueFieldOffset('0')
        self.device.sendPassiveTorqueTarget(str(final_target_nm))
        self._sleep(spec['settle'])
        combined_metrics = self._combine_passive_follow_metrics(metrics)
        self._log('%s 汇总 | %s' % (
            self._loop_label(loop_name),
            self._format_metrics_summary(combined_metrics)))
        return combined_metrics

    def _collect_metrics(self, loop_name):
        if loop_name.startswith('passive_follow'):
            return self._collect_passive_follow_metrics(loop_name)
        return self._collect_standard_metrics(loop_name)

    def _candidate_defaults(self, loop_name, term_name):
        defaults = {
            'current_q': {'P': 1.0, 'I': 20.0, 'D': 0.0},
            'current_d': {'P': 1.0, 'I': 20.0, 'D': 0.0},
            'velocity': {'P': 0.2, 'I': 5.0, 'D': 0.0},
            'angle': {'P': 10.0, 'I': 0.0, 'D': 0.0},
            'passive_follow_static': {'P': 0.10, 'I': 0.0, 'D': 0.0},
            'passive_follow_running': {'P': 0.05, 'I': 0.0, 'D': 0.0},
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
                    '%s | %s 迭代 %d | P=%.5f I=%.5f D=%.5f | %s'
                    % (
                        self._loop_label(loop_name),
                        term_name,
                        iteration + 1,
                        trial_values['P'],
                        trial_values['I'],
                        trial_values['D'],
                        self._format_metrics_summary(metrics)))
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
        self._status('正在测量 %s...' % self._loop_label(loop_name))
        self._configure_loop(loop_name)
        best_values = self._snapshot_loop_values(loop_name)
        for term_name in ('P', 'I', 'D'):
            if terms[term_name]:
                best_values = self._optimize_term(loop_name, best_values, term_name)
        self._set_loop_values(loop_name, best_values)
        if loop_name.startswith('passive_follow'):
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
            'monitor_variables': self.device.monitorVariables
            if isinstance(self.device.monitorVariables, list) else [],
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

        matrix_group = QtWidgets.QGroupBox('测量项目')
        matrix_layout = QtWidgets.QGridLayout(matrix_group)
        matrix_layout.setHorizontalSpacing(12)
        matrix_layout.setVerticalSpacing(6)
        headers = ('PID项', '启用', '调P', '调I', '调D')
        for column, header in enumerate(headers):
            label = QtWidgets.QLabel(header)
            font = label.font()
            font.setBold(True)
            label.setFont(font)
            matrix_layout.addWidget(label, 0, column)

        for row_index, (loop_name, label, enabled_default, enable_p, enable_i, enable_d) in enumerate(
                LOOP_DEFINITIONS, start=1):
            name_label = QtWidgets.QLabel(label)
            matrix_layout.addWidget(name_label, row_index, 0)

            enabled_box = QtWidgets.QCheckBox()
            enabled_box.setChecked(enabled_default)
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
        options_layout.addWidget(QtWidgets.QLabel('阻尼力矩终值[Nm]:'))
        self.passiveFollowFinalTorqueSpin = QtWidgets.QDoubleSpinBox()
        self.passiveFollowFinalTorqueSpin.setDecimals(3)
        self.passiveFollowFinalTorqueSpin.setRange(0.01, 2.0)
        self.passiveFollowFinalTorqueSpin.setSingleStep(0.01)
        self.passiveFollowFinalTorqueSpin.setValue(0.05)
        options_layout.addWidget(self.passiveFollowFinalTorqueSpin)

        options_layout.addWidget(QtWidgets.QLabel('扫描挡数:'))
        self.passiveFollowStepCountSpin = QtWidgets.QSpinBox()
        self.passiveFollowStepCountSpin.setRange(2, 50)
        self.passiveFollowStepCountSpin.setValue(10)
        options_layout.addWidget(self.passiveFollowStepCountSpin)

        options_layout.addWidget(QtWidgets.QLabel('最大迭代次数:'))
        self.maxIterationsSpin = QtWidgets.QSpinBox()
        self.maxIterationsSpin.setRange(1, 20)
        self.maxIterationsSpin.setValue(6)
        options_layout.addWidget(self.maxIterationsSpin)
        options_layout.addWidget(QtWidgets.QLabel('旋转工况阈值[rad/s]:'))
        self.passiveFollowRunningThresholdSpin = QtWidgets.QDoubleSpinBox()
        self.passiveFollowRunningThresholdSpin.setDecimals(2)
        self.passiveFollowRunningThresholdSpin.setRange(0.1, 100.0)
        self.passiveFollowRunningThresholdSpin.setSingleStep(0.1)
        self.passiveFollowRunningThresholdSpin.setValue(
            max(0.1, float(getattr(self.device, 'passiveTorqueRunningSpeedThresholdRadS', 2.0))))
        options_layout.addWidget(self.passiveFollowRunningThresholdSpin)
        options_layout.addStretch(1)
        root_layout.addLayout(options_layout)

        note_label = QtWidgets.QLabel(
            '说明：自动测量会暂时接管控制模式。'
            '从动力矩跟随 PID 会拆成低速/静止工况与正常旋转工况分别测量；'
            '正常旋转工况开始前会提示你手动旋转电机，检测到速度达标后再开始。'
            '电流 D PID 仍使用 Q 轴响应做代理。')
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

    def updateStatus(self, message):
        self.summaryLabel.setText(message)

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
                label = next(label for name, label, *_ in LOOP_DEFINITIONS if name == loop_name)
                QtWidgets.QMessageBox.warning(
                    self,
                    'PID自动测量',
                    '%s 已启用，但未勾选任何 P/I/D。' % label)
                return

        self.logOutput.clear()
        self.summaryLabel.setText('PID 自动测量进行中...')
        self.startButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        self.device.sendPassiveTorqueRunningSpeedThreshold(
            str(self.passiveFollowRunningThresholdSpin.value()))

        self.worker = PidAutoTuneWorker(
            self.device,
            selected_options,
            self.maxIterationsSpin.value(),
            self.passiveFollowFinalTorqueSpin.value(),
            self.passiveFollowStepCountSpin.value(),
            self)
        self.worker.logMessage.connect(self.appendLog)
        self.worker.statusMessage.connect(self.updateStatus)
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
