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

# 自动测量时的状态采样频率。
DEFAULT_MEASUREMENT_RATE_HZ = 50
# 正常旋转工况等待手动转动的最长时间。
DEFAULT_ROTATION_DETECT_TIMEOUT_S = 12.0
# 正常旋转工况单个挡位的最大重试次数。
DEFAULT_ROTATION_RETRY_LIMIT = 5

# 从动力矩跟随 PID 自动测量的阻尼力矩终值默认值。
DEFAULT_PASSIVE_FOLLOW_FINAL_TORQUE_NM = 0.05
# 速度环 PID 自动测量的目标速度默认值，单位 rpm。
DEFAULT_VELOCITY_TARGET_RPM = 100.0
# 从动力矩跟随 PID 自动测量的阻尼力矩扫描挡数默认值。
DEFAULT_PASSIVE_FOLLOW_STEP_COUNT = 4
# 判定进入正常旋转工况的默认速度阈值。
DEFAULT_PASSIVE_FOLLOW_RUNNING_THRESHOLD_RAD_S = 0.5
# 速度需要连续超过/低于阈值的默认持续时间。
DEFAULT_PASSIVE_FOLLOW_RUNNING_HOLD_S = 1.0
# 上位机自动测量的默认采样时间。
DEFAULT_SAMPLE_DURATION_S = 1.5
# 从释放切回从动力矩时的软启动步数。
DEFAULT_PASSIVE_FOLLOW_SOFTSTART_STEPS = (0.25, 0.5, 1.0)
# 从释放切回从动力矩时每一步软启动的间隔时间。
DEFAULT_PASSIVE_FOLLOW_SOFTSTART_STEP_S = 0.06
# 每个 PID 项的默认最大迭代次数。
DEFAULT_MAX_ITERATIONS = 6

# 各个控制环在自动测量时使用的默认候选 PID 起点。
DEFAULT_PID_CANDIDATES = {
    'current_q': {'P': 1.0, 'I': 20.0, 'D': 0.0},
    'current_d': {'P': 1.0, 'I': 20.0, 'D': 0.0},
    'velocity': {'P': 0.2, 'I': 5.0, 'D': 0.0},
    'angle': {'P': 10.0, 'I': 0.0, 'D': 0.0},
    'passive_follow_static': {'P': 0.10, 'I': 0.0, 'D': 0.0},
    'passive_follow_running': {'P': 0.05, 'I': 0.0, 'D': 0.0},
}


class PidAutoTuneWorker(QtCore.QThread):
    logMessage = QtCore.pyqtSignal(str)
    statusMessage = QtCore.pyqtSignal(str)
    finishedWithStatus = QtCore.pyqtSignal(bool, str)

    def __init__(
            self,
            device,
            selected_options,
            max_iterations,
            velocity_target_rpm,
            passive_follow_final_target_nm,
            passive_follow_step_count,
            passive_follow_running_hold_s,
            sample_duration_s,
            parent=None):
        super().__init__(parent)
        self.device = device
        self.selected_options = selected_options
        self.max_iterations = max_iterations
        self.velocity_target_rpm = velocity_target_rpm
        self.passive_follow_final_target_nm = passive_follow_final_target_nm
        self.passive_follow_step_count_value = passive_follow_step_count
        self.passive_follow_running_hold_s = passive_follow_running_hold_s
        self.sample_duration_s = sample_duration_s
        self.measurement_rate_hz = DEFAULT_MEASUREMENT_RATE_HZ
        self.rotation_detect_timeout_s = DEFAULT_ROTATION_DETECT_TIMEOUT_S
        self.rotation_retry_limit = DEFAULT_ROTATION_RETRY_LIMIT
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
        return 'passiveTorqueDampingAngleDeg'

    def _spec_for_loop(self, loop_name):
        capture_s = self._sample_duration_s()
        if loop_name in ('current_q', 'current_d'):
            step = min(max(self._to_float(self.device.currentLimit, 1.8) * 0.2, 0.12), 0.5)
            return {'step': step, 'settle': 0.25, 'capture': capture_s}
        if loop_name == 'velocity':
            step = self._velocity_target_rad_s()
            return {'step': step, 'settle': 0.30, 'capture': capture_s}
        if loop_name == 'angle':
            return {'step': 0.35, 'settle': 0.35, 'capture': capture_s}
        deadzone = self._to_float(self.device.passiveTorqueFollowDeadzoneDeg, 0.8)
        return {'step': max(deadzone + 0.4, 0.8), 'settle': 0.18, 'capture': capture_s, 'hold': min(0.18, capture_s)}

    def _capture_samples(self, loop_name, duration_s):
        attr_name = self._measurement_attr(loop_name)
        interval_s = 1.0 / max(1, self.measurement_rate_hz)
        samples = []
        started = time.monotonic()
        while not self.stopped():
            elapsed = time.monotonic() - started
            if elapsed > duration_s:
                break
            self.device.updateStates()
            samples.append((elapsed, self._to_float(getattr(self.device, attr_name, 0.0))))
            self._sleep(max(0.0, interval_s - 0.012))
        return samples

    def _capture_samples_with_rotation_guard(self, loop_name, duration_s, threshold, hold_required_s):
        attr_name = self._measurement_attr(loop_name)
        interval_s = 1.0 / max(1, self.measurement_rate_hz)
        samples = []
        started = time.monotonic()
        below_threshold_start = None
        while not self.stopped():
            elapsed = time.monotonic() - started
            if elapsed > duration_s:
                break
            self.device.updateStates()
            value = self._to_float(getattr(self.device, attr_name, 0.0))
            velocity = abs(self._to_float(self.device.velocityNow))
            samples.append((elapsed, value))
            if velocity < threshold:
                if below_threshold_start is None:
                    below_threshold_start = time.monotonic()
                elif (time.monotonic() - below_threshold_start) >= hold_required_s:
                    return samples, False
            else:
                below_threshold_start = None
            self._sleep(max(0.0, interval_s - 0.012))
        return samples, True

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
        for key in ('steady', 'residual', 'peak', 'overshoot', 'noise'):
            if key in metrics:
                parts.append('%s=%.4f' % (key, metrics.get(key, 0.0)))
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
        value = self._to_float(self.passive_follow_final_target_nm, DEFAULT_PASSIVE_FOLLOW_FINAL_TORQUE_NM)
        return value if value >= 0.01 else DEFAULT_PASSIVE_FOLLOW_FINAL_TORQUE_NM

    def _passive_follow_step_count(self):
        return max(2, int(self.passive_follow_step_count_value))

    def _running_threshold(self):
        return max(
            0.1,
            self._to_float(
                getattr(self.device, 'passiveTorqueRunningSpeedThresholdRadS', DEFAULT_PASSIVE_FOLLOW_RUNNING_THRESHOLD_RAD_S),
                DEFAULT_PASSIVE_FOLLOW_RUNNING_THRESHOLD_RAD_S))

    def _rotation_hold_threshold_s(self):
        return max(0.1, self._to_float(self.passive_follow_running_hold_s, DEFAULT_PASSIVE_FOLLOW_RUNNING_HOLD_S))

    def _sample_duration_s(self):
        return max(0.2, self._to_float(self.sample_duration_s, DEFAULT_SAMPLE_DURATION_S))

    def _velocity_target_rad_s(self):
        rpm = max(1.0, self._to_float(self.velocity_target_rpm, DEFAULT_VELOCITY_TARGET_RPM))
        return rpm * 2.0 * 3.141592653589793 / 60.0

    def _stabilize_passive_follow_motor(self, reason_text=''):
        if reason_text:
            self._log('从动力矩切换 | %s | 切到释放态' % reason_text)
        self.device.sendPassiveTorqueTarget('0')
        self._sleep(0.03)
        self.device.sendPassiveTorqueMode('0')
        self._sleep(0.03)
        self.device.sendReleaseMode('1')
        self._sleep(0.05)

    def _engage_passive_follow_target(self, target_nm, reason_text=''):
        if reason_text:
            self._log('从动力矩软接入 | %s | target=%.4f Nm' % (reason_text, target_nm))
        self.device.sendPassiveTorqueTarget('0')
        self._sleep(0.03)
        self.device.sendReleaseMode('0')
        self._sleep(0.05)
        self.device.sendPassiveTorqueMode('1')
        self._sleep(0.08)
        for scale in DEFAULT_PASSIVE_FOLLOW_SOFTSTART_STEPS:
            self.device.sendPassiveTorqueTarget(str(target_nm * scale))
            self._sleep(DEFAULT_PASSIVE_FOLLOW_SOFTSTART_STEP_S)

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
        self._stabilize_passive_follow_motor('进入 %s 测量前' % self._loop_label(loop_name))
        self.device.sendDeviceStatus('1')
        self._sleep(0.05)
        if loop_name == 'passive_follow_static':
            self._engage_passive_follow_target(test_target, '进入低速/静止工况测量')
            self._sleep(0.20)
        else:
            self.device.sendPassiveTorqueTarget('0')
            self._sleep(0.03)
            self.device.sendPassiveTorqueFieldOffset('0')
            self._sleep(0.03)
            self.device.sendPassiveTorqueMode('0')
            self._sleep(0.05)
            self.device.sendReleaseMode('1')
            self._sleep(0.20)
        pid_values = self._snapshot_loop_values(loop_name)
        self._log(
            '%s 配置 | final_target=%.4f Nm | steps=%d | deadzone=%.3f deg | calc=%d Hz | '
            'running_threshold=%.3f rad/s | hold_threshold=%.2f s | sample=%.2f s | '
            'pid=(%.5f, %.5f, %.5f) | offset_step=%.3f deg'
            % (
                self._loop_label(loop_name),
                test_target,
                self._passive_follow_step_count(),
                self._to_float(self.device.passiveTorqueFollowDeadzoneDeg),
                int(self._to_float(self.device.passiveTorqueCalculationHz, 1000)),
                self._running_threshold(),
                self._rotation_hold_threshold_s(),
                self._sample_duration_s(),
                pid_values['P'],
                pid_values['I'],
                pid_values['D'],
                spec['step']))

    def _configure_loop(self, loop_name):
        if loop_name.startswith('passive_follow'):
            self._configure_passive_follow_loop(loop_name)
        else:
            self._configure_standard_loop(loop_name)
            if loop_name == 'velocity':
                self._log(
                    '%s 配置 | target=%.2f rpm (%.4f rad/s) | sample=%.2f s'
                    % (
                        self._loop_label(loop_name),
                        self._to_float(self.velocity_target_rpm, DEFAULT_VELOCITY_TARGET_RPM),
                        self._velocity_target_rad_s(),
                        self._sample_duration_s(),
                    ))

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

    def _wait_for_manual_rotation(self, target_nm):
        threshold = self._running_threshold()
        hold_required_s = self._rotation_hold_threshold_s()
        self.device.sendPassiveTorqueMode('0')
        self._sleep(0.05)
        self.device.sendReleaseMode('1')
        self._sleep(0.10)
        self._status(
            '请手动持续旋转电机。当前已完全释放，速度连续超过 %.2f rad/s 并保持 %.2f s 后开始测量。'
            % (threshold, hold_required_s))
        deadline = time.monotonic() + self.rotation_detect_timeout_s
        hold_start = None
        while not self.stopped() and time.monotonic() < deadline:
            self.device.updateStates()
            velocity = abs(self._to_float(self.device.velocityNow))
            if velocity >= threshold:
                if hold_start is None:
                    hold_start = time.monotonic()
                if (time.monotonic() - hold_start) >= hold_required_s:
                    self._engage_passive_follow_target(target_nm, '检测到持续旋转后接入从动力矩')
                    self._log(
                        '已检测到旋转速度 %.4f rad/s，且持续 %.2f s，进入正常旋转工况测量。'
                        % (velocity, hold_required_s))
                    return True
            else:
                hold_start = None
            self._sleep(0.03)
        self._log('未在规定时间内检测到足够且持续的旋转速度，本轮按失败计分。')
        return False

    def _collect_passive_follow_metrics(self, loop_name):
        spec = self._spec_for_loop(loop_name)
        final_target_nm = self._passive_follow_test_target_nm()
        step_count = self._passive_follow_step_count()
        metrics = []
        threshold = self._running_threshold()
        hold_required_s = self._rotation_hold_threshold_s()

        for step_index in range(1, step_count + 1):
            if self.stopped():
                break
            target_nm = final_target_nm * step_index / step_count
            if loop_name == 'passive_follow_running':
                step_valid = False
                for retry_index in range(1, self.rotation_retry_limit + 1):
                    self._log(
                        '%s 挡位 %02d/%02d | 第 %d 次尝试 | target=%.4f Nm | offset_step=%.3f deg'
                        % (self._loop_label(loop_name), step_index, step_count,
                           retry_index, target_nm, spec['step']))
                    self._stabilize_passive_follow_motor(
                        '%s 挡位 %02d/%02d 设置目标前'
                        % (self._loop_label(loop_name), step_index, step_count))
                    self.device.sendPassiveTorqueTarget('0')
                    self._sleep(0.05)
                    if not self._wait_for_manual_rotation(target_nm):
                        return {'score': 1e9, 'peak': 0.0, 'residual': 0.0, 'sample_count': len(metrics)}
                    self.device.sendPassiveTorqueFieldOffset('0')
                    self._sleep(spec['settle'])
                    self.device.sendPassiveTorqueFieldOffset(str(spec['step']))
                    hold_samples, hold_valid = self._capture_samples_with_rotation_guard(
                        loop_name, spec['hold'], threshold, hold_required_s)
                    self.device.sendPassiveTorqueFieldOffset('0')
                    decay_samples, decay_valid = self._capture_samples_with_rotation_guard(
                        loop_name, spec['capture'], threshold, hold_required_s)
                    if hold_valid and decay_valid:
                        step_metrics = self._evaluate_decay_response(hold_samples + decay_samples)
                        metrics.append(step_metrics)
                        self._log(
                            '%s 挡位 %02d/%02d 结果 | %s'
                            % (self._loop_label(loop_name), step_index, step_count,
                               self._format_metrics_summary(step_metrics)))
                        step_valid = True
                        break
                    self._log(
                        '%s 挡位 %02d/%02d 无效：速度连续低于阈值 %.2f rad/s 超过 %.2f s，重新扫描该挡。'
                        % (self._loop_label(loop_name), step_index, step_count,
                           threshold, hold_required_s))
                    self.device.sendPassiveTorqueMode('0')
                    self._sleep(0.05)
                    self.device.sendReleaseMode('1')
                    self._sleep(0.10)
                if not step_valid:
                    return {'score': 1e9, 'peak': 0.0, 'residual': 0.0, 'sample_count': len(metrics)}
            else:
                self._log(
                    '%s 挡位 %02d/%02d | target=%.4f Nm | offset_step=%.3f deg'
                    % (self._loop_label(loop_name), step_index, step_count,
                       target_nm, spec['step']))
                self._stabilize_passive_follow_motor(
                    '%s 挡位 %02d/%02d 设置目标前'
                    % (self._loop_label(loop_name), step_index, step_count))
                self._engage_passive_follow_target(
                    target_nm,
                    '%s 挡位 %02d/%02d 接入测量'
                    % (self._loop_label(loop_name), step_index, step_count))
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
        return DEFAULT_PID_CANDIDATES.get(loop_name, {}).get(term_name, 0.0)

    def _candidate_values(self, loop_name, term_name, current_value):
        current_value = self._to_float(current_value, self._candidate_defaults(loop_name, term_name))
        if current_value <= 0:
            current_value = self._candidate_defaults(loop_name, term_name)
        if term_name == 'P':
            values = [current_value * 0.6, current_value, current_value * 1.6]
        elif term_name == 'I':
            values = [current_value * 0.4, current_value, current_value * 1.8]
        else:
            base = current_value if current_value > 0 else max(0.0001, self._candidate_defaults(loop_name, term_name))
            values = [0.0, base * 0.5, base, base * 2.0]
        cleaned = []
        for value in values:
            value = max(0.0, float(value))
            rounded = round(value, 8)
            if rounded not in cleaned:
                cleaned.append(rounded)
        return cleaned

    def _optimize_term(self, loop_name, values, term_name):
        best_values = dict(values)
        best_metrics = None
        best_score = float('inf')
        for candidate_value in self._candidate_values(loop_name, term_name, values[term_name]):
            if self.stopped():
                break
            trial_values = dict(values)
            trial_values[term_name] = candidate_value
            if loop_name.startswith('passive_follow'):
                self._stabilize_passive_follow_motor(
                    '%s 调整 %s 前' % (self._loop_label(loop_name), term_name))
                self._set_loop_values(loop_name, trial_values)
                self._configure_loop(loop_name)
            else:
                self._configure_loop(loop_name)
                self._set_loop_values(loop_name, trial_values)
            self._sleep(0.10)
            metrics = self._collect_metrics(loop_name)
            score = metrics.get('score', float('inf'))
            self._log(
                '%s | %s=%.6f | %s'
                % (self._loop_label(loop_name), term_name, candidate_value, self._format_metrics_summary(metrics)))
            if score < best_score:
                best_score = score
                best_values = trial_values
                best_metrics = metrics
        return best_values, best_metrics

    def _tune_loop(self, loop_name):
        if not self._loop_enabled(loop_name):
            return
        current_values = self._snapshot_loop_values(loop_name)
        selected_terms = self._loop_terms(loop_name)
        self._status('开始测量 %s' % self._loop_label(loop_name))
        self._log('当前 PID | P=%.6f I=%.6f D=%.6f' % (
            current_values['P'], current_values['I'], current_values['D']))

        for iteration_index in range(1, self.max_iterations + 1):
            if self.stopped():
                return
            self._log('%s | 第 %d/%d 轮迭代' % (
                self._loop_label(loop_name), iteration_index, self.max_iterations))
            improved = False
            for term_name in ('P', 'I', 'D'):
                if not selected_terms[term_name]:
                    continue
                candidate_values, candidate_metrics = self._optimize_term(loop_name, current_values, term_name)
                if candidate_metrics is None:
                    continue
                if abs(candidate_values[term_name] - current_values[term_name]) > 1e-9:
                    improved = True
                current_values = candidate_values
                self._set_loop_values(loop_name, current_values)
                self._sleep(0.05)
                self._log(
                    '%s | 保留 %s=%.6f | %s'
                    % (self._loop_label(loop_name), term_name, current_values[term_name],
                       self._format_metrics_summary(candidate_metrics)))
            if not improved:
                self._log('%s | 本轮无更优结果，提前结束。' % self._loop_label(loop_name))
                break

        self._set_loop_values(loop_name, current_values)
        self._status(
            '%s 测量完成 | P=%.6f I=%.6f D=%.6f'
            % (self._loop_label(loop_name), current_values['P'], current_values['I'], current_values['D']))

    def _restore_runtime_state(self, snapshot):
        self.device.sendPassiveTorqueFieldOffset('0')
        self._sleep(0.05)
        self.device.sendPassiveTorqueTarget(str(snapshot['passive_torque_target_nm']))
        self._sleep(0.05)
        if snapshot['control_type'] is not None:
            self.device.sendControlType(snapshot['control_type'])
            self._sleep(0.05)
        if snapshot['torque_type'] is not None:
            self.device.sendTorqueType(snapshot['torque_type'])
            self._sleep(0.05)
        self.device.sendDeviceStatus('1' if snapshot['device_status'] else '0')
        self._sleep(0.05)
        self.device.sendReleaseMode('1' if snapshot['release_mode'] else '0')
        self._sleep(0.05)
        self.device.sendPassiveTorqueMode('1' if snapshot['passive_torque_mode'] else '0')
        self._sleep(0.05)
        self.device.sendTargetValue(str(snapshot['target']))
        self._sleep(0.05)

    def run(self):
        if not self.device.isConnected:
            self.finishedWithStatus.emit(False, '设备未连接，无法启动自动测量。')
            return

        snapshot = {
            'device_status': bool(self._to_float(self.device.deviceStatus)),
            'release_mode': bool(self.device.releaseMode),
            'passive_torque_mode': bool(self.device.passiveTorqueMode),
            'control_type': getattr(self.device, 'controlType', None),
            'torque_type': getattr(self.device, 'torqueType', None),
            'target': self._to_float(self.device.target),
            'passive_torque_target_nm': self._to_float(
                self.device.passiveTorqueTargetNm,
                DEFAULT_PASSIVE_FOLLOW_FINAL_TORQUE_NM),
            'state_update_rate_hz': self._to_float(getattr(self.device, 'stateUpdateRateHz', 10), 10),
            'state_updater_running': self.device.stateUpdater is not None and not self.device.stateUpdater.stopped(),
        }

        try:
            if snapshot['state_updater_running']:
                self.device.stateUpdater.stop()
                self.device.stateUpdater.wait(500)
            self.device.setStateUpdateRateHz(self.measurement_rate_hz)
            self._sleep(0.05)

            for loop_name, *_ in LOOP_DEFINITIONS:
                if self.stopped():
                    raise RuntimeError('用户已停止自动测量。')
                if self._loop_enabled(loop_name):
                    self._tune_loop(loop_name)

        except Exception as error:
            self._restore_runtime_state(snapshot)
            if snapshot['state_updater_running'] and self.device.isConnected:
                self.device.stateUpdater = self.device.stateUpdater.__class__(self.device)
                self.device.stateUpdater.start()
            self.device.setStateUpdateRateHz(snapshot['state_update_rate_hz'])
            self.finishedWithStatus.emit(False, str(error))
            return

        self._restore_runtime_state(snapshot)
        self.device.setStateUpdateRateHz(snapshot['state_update_rate_hz'])
        if snapshot['state_updater_running'] and self.device.isConnected:
            self.device.stateUpdater = self.device.stateUpdater.__class__(self.device)
            self.device.stateUpdater.start()
        self.finishedWithStatus.emit(True, 'PID 自动测量完成。')


class PidAutoTuneTool(WorkAreaTabWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.device = SimpleFOCDevice.getInstance()
        self.worker = None
        self.optionWidgets = {}

        self.mainLayout = QtWidgets.QVBoxLayout(self)

        self.optionsGroup = QtWidgets.QGroupBox('测量项目')
        self.optionsLayout = QtWidgets.QGridLayout(self.optionsGroup)
        self.optionsLayout.addWidget(QtWidgets.QLabel('PID 项'), 0, 0)
        self.optionsLayout.addWidget(QtWidgets.QLabel('启用'), 0, 1)
        self.optionsLayout.addWidget(QtWidgets.QLabel('P'), 0, 2)
        self.optionsLayout.addWidget(QtWidgets.QLabel('I'), 0, 3)
        self.optionsLayout.addWidget(QtWidgets.QLabel('D'), 0, 4)

        for row_index, (name, label, enabled_default, p_default, i_default, d_default) in enumerate(LOOP_DEFINITIONS, start=1):
            title_label = QtWidgets.QLabel(label)
            enabled_box = QtWidgets.QCheckBox()
            enabled_box.setChecked(enabled_default)
            p_box = QtWidgets.QCheckBox()
            p_box.setChecked(p_default)
            i_box = QtWidgets.QCheckBox()
            i_box.setChecked(i_default)
            d_box = QtWidgets.QCheckBox()
            d_box.setChecked(d_default)
            self.optionsLayout.addWidget(title_label, row_index, 0)
            self.optionsLayout.addWidget(enabled_box, row_index, 1)
            self.optionsLayout.addWidget(p_box, row_index, 2)
            self.optionsLayout.addWidget(i_box, row_index, 3)
            self.optionsLayout.addWidget(d_box, row_index, 4)
            self.optionWidgets[name] = {'enabled': enabled_box, 'P': p_box, 'I': i_box, 'D': d_box}

        self.mainLayout.addWidget(self.optionsGroup)

        self.settingsGroup = QtWidgets.QGroupBox('测量参数')
        self.settingsLayout = QtWidgets.QGridLayout(self.settingsGroup)

        self.velocityTargetLabel = QtWidgets.QLabel('速度环目标速度[rpm]:')
        self.velocityTargetSpin = QtWidgets.QDoubleSpinBox()
        self.velocityTargetSpin.setRange(1.0, 5000.0)
        self.velocityTargetSpin.setDecimals(1)
        self.velocityTargetSpin.setSingleStep(5.0)
        self.velocityTargetSpin.setValue(DEFAULT_VELOCITY_TARGET_RPM)

        self.passiveFollowFinalTorqueLabel = QtWidgets.QLabel('阻尼力矩终值[Nm]:')
        self.passiveFollowFinalTorqueSpin = QtWidgets.QDoubleSpinBox()
        self.passiveFollowFinalTorqueSpin.setRange(0.01, 1.0)
        self.passiveFollowFinalTorqueSpin.setDecimals(4)
        self.passiveFollowFinalTorqueSpin.setSingleStep(0.01)
        self.passiveFollowFinalTorqueSpin.setValue(DEFAULT_PASSIVE_FOLLOW_FINAL_TORQUE_NM)

        self.passiveFollowStepCountLabel = QtWidgets.QLabel('阻尼力矩扫描挡数:')
        self.passiveFollowStepCountSpin = QtWidgets.QSpinBox()
        self.passiveFollowStepCountSpin.setRange(2, 50)
        self.passiveFollowStepCountSpin.setValue(DEFAULT_PASSIVE_FOLLOW_STEP_COUNT)

        self.passiveFollowRunningThresholdLabel = QtWidgets.QLabel('旋转工况阈值[rad/s]:')
        self.passiveFollowRunningThresholdSpin = QtWidgets.QDoubleSpinBox()
        self.passiveFollowRunningThresholdSpin.setRange(0.1, 50.0)
        self.passiveFollowRunningThresholdSpin.setDecimals(3)
        self.passiveFollowRunningThresholdSpin.setSingleStep(0.1)
        self.passiveFollowRunningThresholdSpin.setValue(
            self._device_value(
                'passiveTorqueRunningSpeedThresholdRadS',
                DEFAULT_PASSIVE_FOLLOW_RUNNING_THRESHOLD_RAD_S))

        self.passiveFollowRunningHoldLabel = QtWidgets.QLabel('手动旋转时间阈值[s]:')
        self.passiveFollowRunningHoldSpin = QtWidgets.QDoubleSpinBox()
        self.passiveFollowRunningHoldSpin.setRange(0.1, 10.0)
        self.passiveFollowRunningHoldSpin.setDecimals(2)
        self.passiveFollowRunningHoldSpin.setSingleStep(0.1)
        self.passiveFollowRunningHoldSpin.setValue(DEFAULT_PASSIVE_FOLLOW_RUNNING_HOLD_S)

        self.sampleDurationLabel = QtWidgets.QLabel('采样时间[s]:')
        self.sampleDurationSpin = QtWidgets.QDoubleSpinBox()
        self.sampleDurationSpin.setRange(0.2, 10.0)
        self.sampleDurationSpin.setDecimals(2)
        self.sampleDurationSpin.setSingleStep(0.1)
        self.sampleDurationSpin.setValue(DEFAULT_SAMPLE_DURATION_S)

        self.maxIterationsLabel = QtWidgets.QLabel('最大迭代次数:')
        self.maxIterationsSpin = QtWidgets.QSpinBox()
        self.maxIterationsSpin.setRange(1, 20)
        self.maxIterationsSpin.setValue(DEFAULT_MAX_ITERATIONS)

        widgets = (
            (self.velocityTargetLabel, self.velocityTargetSpin),
            (self.passiveFollowFinalTorqueLabel, self.passiveFollowFinalTorqueSpin),
            (self.passiveFollowStepCountLabel, self.passiveFollowStepCountSpin),
            (self.passiveFollowRunningThresholdLabel, self.passiveFollowRunningThresholdSpin),
            (self.passiveFollowRunningHoldLabel, self.passiveFollowRunningHoldSpin),
            (self.sampleDurationLabel, self.sampleDurationSpin),
            (self.maxIterationsLabel, self.maxIterationsSpin),
        )
        for index, (label_widget, input_widget) in enumerate(widgets):
            row_index = index // 2
            column_group = index % 2
            base_column = column_group * 2
            self.settingsLayout.addWidget(label_widget, row_index, base_column)
            self.settingsLayout.addWidget(input_widget, row_index, base_column + 1)

        self.settingsLayout.setColumnStretch(0, 0)
        self.settingsLayout.setColumnStretch(1, 1)
        self.settingsLayout.setColumnStretch(2, 0)
        self.settingsLayout.setColumnStretch(3, 1)

        self.mainLayout.addWidget(self.settingsGroup)

        self.noteLabel = QtWidgets.QLabel(
            '说明：\n'
            '1. 从动力矩跟随 PID 的释放缓冲已经下沉到固件，上位机不再额外缓冲。\n'
            '2. 正常旋转工况只有在转速连续高于“旋转工况阈值”并保持“手动旋转时间阈值”后，才开始该挡测量。\n'
            '3. 测量中如果转速连续低于阈值超过“手动旋转时间阈值”，该挡结果判无效并重扫。\n'
            '4. 采样时间[s]用于设置上位机每轮测量的采样窗口。\n'
            '5. 速度 PID 使用“速度环目标速度[rpm]”作为测量目标。\n'
            '6. 电流 D PID 仍然基于当前工程可用链路做代理测量。')
        self.noteLabel.setWordWrap(True)
        self.mainLayout.addWidget(self.noteLabel)

        self.buttonLayout = QtWidgets.QHBoxLayout()
        self.startButton = QtWidgets.QPushButton('开始测量')
        self.startButton.setIcon(GUIToolKit.getIconByName('start'))
        self.startButton.clicked.connect(self.startAutoTune)
        self.stopButton = QtWidgets.QPushButton('停止')
        self.stopButton.setIcon(GUIToolKit.getIconByName('stop'))
        self.stopButton.clicked.connect(self.stopAutoTune)
        self.stopButton.setEnabled(False)
        self.statusLabel = QtWidgets.QLabel('就绪')
        self.buttonLayout.addWidget(self.startButton)
        self.buttonLayout.addWidget(self.stopButton)
        self.buttonLayout.addWidget(self.statusLabel, 1)
        self.mainLayout.addLayout(self.buttonLayout)

        self.logOutput = QtWidgets.QPlainTextEdit()
        self.logOutput.setReadOnly(True)
        self.mainLayout.addWidget(self.logOutput, 1)

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)

    def getTabIcon(self):
        return GUIToolKit.getIconByName('pid')

    def getTabName(self):
        return 'PID自动测量'

    def _device_value(self, attr_name, default):
        try:
            return float(getattr(self.device, attr_name))
        except (TypeError, ValueError, AttributeError):
            return float(default)

    def _selected_options(self):
        selected = {}
        for name, widgets in self.optionWidgets.items():
            selected[name] = {
                'enabled': widgets['enabled'].isChecked(),
                'P': widgets['P'].isChecked(),
                'I': widgets['I'].isChecked(),
                'D': widgets['D'].isChecked(),
            }
        return selected

    def _has_enabled_loop(self):
        for widgets in self.optionWidgets.values():
            if widgets['enabled'].isChecked():
                return True
        return False

    def _appendLog(self, message):
        self.logOutput.appendPlainText(message)
        scrollbar = self.logOutput.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def connectionStateChanged(self, is_connected):
        self.startButton.setEnabled(bool(is_connected) and self.worker is None)

    def startAutoTune(self):
        if self.worker is not None:
            return
        if not self.device.isConnected:
            QtWidgets.QMessageBox.warning(self, 'PID自动测量', '设备未连接。')
            return
        if not self._has_enabled_loop():
            QtWidgets.QMessageBox.warning(self, 'PID自动测量', '请至少勾选一个 PID 项。')
            return

        self.device.sendPassiveTorqueRunningSpeedThreshold(str(self.passiveFollowRunningThresholdSpin.value()))
        self.logOutput.clear()
        self._appendLog('开始 PID 自动测量。')
        self.worker = PidAutoTuneWorker(
            self.device,
            self._selected_options(),
            self.maxIterationsSpin.value(),
            self.velocityTargetSpin.value(),
            self.passiveFollowFinalTorqueSpin.value(),
            self.passiveFollowStepCountSpin.value(),
            self.passiveFollowRunningHoldSpin.value(),
            self.sampleDurationSpin.value(),
            self)
        self.worker.logMessage.connect(self._appendLog)
        self.worker.statusMessage.connect(self.statusLabel.setText)
        self.worker.finishedWithStatus.connect(self._workerFinished)
        self.startButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        self.worker.start()

    def stopAutoTune(self):
        if self.worker is not None:
            self.worker.stop()
            self.statusLabel.setText('正在停止...')

    def _workerFinished(self, success, message):
        self.stopButton.setEnabled(False)
        self.worker = None
        self.statusLabel.setText(message)
        self.startButton.setEnabled(self.device.isConnected)
        self._appendLog(message)
        if not success:
            QtWidgets.QMessageBox.warning(self, 'PID自动测量', message)
