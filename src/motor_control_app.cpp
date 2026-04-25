#include "motor_control_app.h"

MotorControlApp* MotorControlApp::active_instance_ = nullptr;

namespace {
// 自定义命令也遵循 Commander 的空参数规则，避免查询命令被误判成设置命令。
bool IsCommandSentinel(char ch) {
  return ch == '\0' || ch == '\n' || ch == '\r';
}

char kMotorCommandLabel[] = "motor";
}  // namespace

MotorControlApp::MotorControlApp(const MotorAppConfig& config)
    : config_(config),
      motor_(config.pole_pairs),
      driver_(config.pwm_u_pin,
              config.pwm_v_pin,
              config.pwm_w_pin,
              config.driver_enable_pin),
      wire_(0),
      sensor_(&wire_,
              config.i2c_address,
              config.angle_reg_msb,
              config.cpr,
              config.i2c_sda_pin,
              config.i2c_scl_pin,
              config.i2c_clock_hz,
              config.i2c_timeout_ms),
      current_sense_(config.shunt_resistor_ohm,
                     config.current_sense_gain,
                     config.current_a_pin,
                     config.current_b_pin),
      serial0_command_(Serial),
      serial1_command_(Serial1),
      serial2_command_(Serial2),
      motion_downsample_(config.motion_downsample),
      passive_torque_target_nm_(config.passive_torque_target_nm),
      passive_torque_saturation_angle_deg_(
          config.passive_torque_saturation_angle_deg),
      passive_torque_follow_deadzone_deg_(
          config.passive_torque_follow_deadzone_deg),
      passive_torque_running_speed_threshold_rad_s_(
          config.passive_torque_running_speed_threshold_rad_s),
      passive_torque_calculation_hz_(config.passive_torque_calculation_hz),
      passive_torque_transition_buffer_ms_(
          config.passive_torque_transition_buffer_ms),
      passive_torque_follow_pid_low_p_(config.passive_torque_follow_pid_low_p),
      passive_torque_follow_pid_low_i_(config.passive_torque_follow_pid_low_i),
      passive_torque_follow_pid_low_d_(config.passive_torque_follow_pid_low_d),
      passive_torque_follow_pid_run_p_(config.passive_torque_follow_pid_run_p),
      passive_torque_follow_pid_run_i_(config.passive_torque_follow_pid_run_i),
      passive_torque_follow_pid_run_d_(config.passive_torque_follow_pid_run_d) {
  active_instance_ = this;
}

void MotorControlApp::setup() {
  initializeSerialPorts();

  initializeI2C();
  initializeMotor();
  initializeCurrentSense();

  motor_.init();
  motor_.initFOC();
  motor_.target = 0.0f;
  resetPassiveTorqueFieldReference();

  initializeCommanders();

  Serial.println(F("Motor self-check finished, target set to 0."));
  Serial.printf("[I2C] SDA=%d SCL=%d clk=%luHz timeout=%ums\n",
                config_.i2c_sda_pin,
                config_.i2c_scl_pin,
                static_cast<unsigned long>(config_.i2c_clock_hz),
                static_cast<unsigned>(config_.i2c_timeout_ms));
  Serial.printf(
      "[PassiveTorque] target=%.4fNm saturation=%.3fdeg deadzone=%.3fdeg calc=%uHz speed_threshold=%.3frad/s low_pid=(%.4f, %.4f, %.4f) run_pid=(%.4f, %.4f, %.4f) current_limit=%.2fA\n",
      passive_torque_target_nm_,
      passive_torque_saturation_angle_deg_,
      passive_torque_follow_deadzone_deg_,
      passive_torque_calculation_hz_,
      passive_torque_running_speed_threshold_rad_s_,
      passive_torque_follow_pid_low_p_,
      passive_torque_follow_pid_low_i_,
      passive_torque_follow_pid_low_d_,
      passive_torque_follow_pid_run_p_,
      passive_torque_follow_pid_run_i_,
      passive_torque_follow_pid_run_d_,
      motor_.current_limit);

  _delay(1000);
}

void MotorControlApp::loop() {
  if (release_mode_) {
    updateReleasedState();
  } else if (passive_torque_mode_) {
    motor_.loopFOC();
    updatePassiveTorqueControl();
    motor_.move();
  } else {
    motor_.loopFOC();
    motor_.move();
  }

  motor_.monitor();
  runCommander(serial0_command_, config_.serial0_command_enabled);
  runCommander(serial1_command_, config_.serial1_command_enabled);
  runCommander(serial2_command_, config_.serial2_command_enabled);
}

void MotorControlApp::handleMotorCommandStatic(char* cmd) {
  if (active_instance_ != nullptr) {
    active_instance_->handleMotorCommand(cmd);
  }
}

Stream& MotorControlApp::activeCommandPort() {
  if (active_command_ != nullptr && active_command_->com_port != nullptr) {
    return *active_command_->com_port;
  }
  return Serial;
}

void MotorControlApp::handleMotorCommand(char* cmd) {
  Commander& active_command =
      active_command_ != nullptr ? *active_command_ : serial0_command_;

  if (cmd && cmd[0] == 'X') {
    // 释放模式只接受明确的MX0/MX1，避免畸形命令误触发释放。
    if (IsCommandSentinel(cmd[1])) {
      reportReleaseMode();
    } else if ((cmd[1] == '0' || cmd[1] == '1') && IsCommandSentinel(cmd[2])) {
      setReleaseMode(cmd[1] == '1');
      reportReleaseMode();
    }
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'M') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueMode(cmd[2] != '0');
    }
    reportPassiveTorqueMode();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'T') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueTargetNm(atof(cmd + 2));
    }
    reportPassiveTorqueTargetNm();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'A') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueSaturationAngleDeg(atof(cmd + 2));
    }
    reportPassiveTorqueSaturationAngleDeg();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'H') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowDeadzoneDeg(atof(cmd + 2));
    }
    reportPassiveTorqueFollowDeadzoneDeg();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'R') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueRunningSpeedThresholdRadS(atof(cmd + 2));
    }
    reportPassiveTorqueRunningSpeedThresholdRadS();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'F') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueCalculationHz(
          static_cast<unsigned int>(max(1, atoi(cmd + 2))));
    }
    reportPassiveTorqueCalculationHz();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'P') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidLowP(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidLowP();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'I') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidLowI(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidLowI();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'D') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidLowD(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidLowD();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'J') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidRunP(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidRunP();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'K') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidRunI(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidRunI();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'L') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueFollowPidRunD(atof(cmd + 2));
    }
    reportPassiveTorqueFollowPidRunD();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'G') {
    reportPassiveTorqueDampingAngleDeg();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'O') {
    if (!IsCommandSentinel(cmd[2])) {
      injectPassiveTorqueFieldOffsetDeg(atof(cmd + 2));
    }
    reportPassiveTorqueDampingAngleDeg();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'Y') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueDebug(cmd[2] != '0');
    }
    reportPassiveTorqueDebugEnabled();
    return;
  }

  if (cmd && cmd[0] == 'C' && cmd[1] == 'D') {
    if (!IsCommandSentinel(cmd[2])) {
      setMotionDownsample(static_cast<unsigned int>(max(0, atoi(cmd + 2))));
    }
    reportMotionDownsample();
    return;
  }

  if (cmd && cmd[0] == 'E' && !IsCommandSentinel(cmd[1])) {
    release_mode_ = false;
    passive_torque_mode_ = false;
    passive_torque_transition_pending_ = false;
    passive_torque_damping_angle_rad_ = 0.0f;
    passive_follow_pid_low_integral_ = 0.0f;
    passive_follow_pid_low_prev_error_ = 0.0f;
    passive_follow_pid_run_integral_ = 0.0f;
    passive_follow_pid_run_prev_error_ = 0.0f;
    motor_.target = 0.0f;
  }

  active_command.motor(&motor_, cmd);
}

void MotorControlApp::initializeI2C() {
  pinMode(config_.gpio_pullup_32, INPUT_PULLUP);
  pinMode(config_.gpio_pullup_33, INPUT_PULLUP);
  pinMode(config_.gpio_pullup_25, INPUT_PULLUP);
  pinMode(config_.gpio_pullup_26, INPUT_PULLUP);
  pinMode(config_.gpio_pullup_27, INPUT_PULLUP);
  pinMode(config_.gpio_pullup_14, INPUT_PULLUP);

  sensor_.begin();
  motor_.linkSensor(&sensor_);
}

void MotorControlApp::initializeSerialPorts() {
  Serial.begin(config_.serial_baud);
  delay(50);

  if (config_.serial1_command_enabled) {
    Serial1.begin(config_.serial_baud,
                  SERIAL_8N1,
                  config_.serial1_rx_pin,
                  config_.serial1_tx_pin);
  }

  if (config_.serial2_command_enabled) {
    Serial2.begin(config_.serial_baud,
                  SERIAL_8N1,
                  config_.serial2_rx_pin,
                  config_.serial2_tx_pin);
  }
}

void MotorControlApp::initializeMotor() {
  driver_.voltage_power_supply = config_.power_supply_voltage;
  driver_.init();
  motor_.linkDriver(&driver_);

  motor_.torque_controller = TorqueControlType::foc_current;
  motor_.controller = MotionControlType::torque;
  motor_.PID_velocity.P = config_.pid_velocity_p;
  motor_.PID_velocity.I = config_.pid_velocity_i;
  motor_.PID_velocity.D = config_.pid_velocity_d;
  motor_.PID_current_q.P = config_.pid_current_q_p;
  motor_.PID_current_q.I = config_.pid_current_q_i;
  motor_.PID_current_q.D = config_.pid_current_q_d;
  motor_.PID_current_d.P = config_.pid_current_d_p;
  motor_.PID_current_d.I = config_.pid_current_d_i;
  motor_.PID_current_d.D = config_.pid_current_d_d;
  motor_.current_limit = config_.current_limit_a;
  motor_.voltage_limit = config_.voltage_limit_v;
  motor_.LPF_velocity.Tf = config_.lpf_velocity_tf;
  motor_.LPF_current_q.Tf = config_.lpf_current_q_tf;
  motor_.LPF_current_d.Tf = config_.lpf_current_d_tf;
  motor_.P_angle.P = config_.p_angle_p;
  motor_.velocity_limit = config_.velocity_limit_rad_s;
  motor_.motion_downsample = motion_downsample_;
  motor_.useMonitoring(Serial);
  motor_.monitor_downsample = config_.monitor_downsample;
  motor_.monitor_variables = config_.monitor_variables;
}

void MotorControlApp::initializeCurrentSense() {
  current_sense_.init();
  current_sense_.gain_b *= -1;
  current_sense_.gain_a *= -1;
  current_sense_.skip_align = true;
  motor_.linkCurrentSense(&current_sense_);
}

void MotorControlApp::initializeCommanders() {
  serial0_command_.add(config_.command_id,
                       handleMotorCommandStatic,
                       kMotorCommandLabel);
  serial1_command_.add(config_.command_id,
                       handleMotorCommandStatic,
                       kMotorCommandLabel);
  serial2_command_.add(config_.command_id,
                       handleMotorCommandStatic,
                       kMotorCommandLabel);
}

void MotorControlApp::runCommander(Commander& commander, bool enabled) {
  if (!enabled) {
    return;
  }

  active_command_ = &commander;
  commander.run();
  active_command_ = nullptr;
}

void MotorControlApp::setReleaseMode(bool enabled) {
  release_mode_ = enabled;
  passive_torque_transition_pending_ = false;
  passive_torque_damping_angle_rad_ = 0.0f;
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
  motor_.target = 0.0f;

  if (release_mode_) {
    passive_torque_mode_ = false;
    driver_.disable();
  } else if (motor_.enabled) {
    driver_.enable();
  }
}

void MotorControlApp::setPassiveTorqueMode(bool enabled) {
  passive_torque_mode_ = enabled;
  motor_.target = 0.0f;
  passive_torque_transition_pending_ = false;
  passive_torque_damping_angle_rad_ = 0.0f;
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
  passive_follow_using_run_pid_ = false;

  if (passive_torque_mode_) {
    release_mode_ = true;
    passive_torque_transition_pending_ = true;
    passive_torque_transition_start_us_ = micros();
    motor_.controller = MotionControlType::torque;
    motor_.torque_controller = TorqueControlType::foc_current;
    if (!motor_.enabled) {
      motor_.enable();
    }
    driver_.disable();
    reportPassiveTorqueDebug("pending", 0.0f);
  } else {
    release_mode_ = false;
    resetPassiveTorqueFieldReference();
    reportPassiveTorqueDebug("disable", 0.0f);
  }
}

void MotorControlApp::setPassiveTorqueDebug(bool enabled) {
  passive_torque_debug_enabled_ = enabled;
}

void MotorControlApp::setMotionDownsample(unsigned int downsample) {
  motion_downsample_ = downsample;
  motor_.motion_downsample = motion_downsample_;
}

void MotorControlApp::setPassiveTorqueTargetNm(float target_nm) {
  passive_torque_target_nm_ = clampPassiveTorqueTargetNm(target_nm);
}

void MotorControlApp::setPassiveTorqueSaturationAngleDeg(float angle_deg) {
  passive_torque_saturation_angle_deg_ =
      constrain(fabsf(angle_deg), 0.05f, 45.0f);
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowDeadzoneDeg(float angle_deg) {
  passive_torque_follow_deadzone_deg_ = constrain(fabsf(angle_deg), 0.0f, 20.0f);
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueRunningSpeedThresholdRadS(float value) {
  passive_torque_running_speed_threshold_rad_s_ = constrain(fabsf(value), 0.1f, 100.0f);
}

void MotorControlApp::setPassiveTorqueCalculationHz(unsigned int calculation_hz) {
  passive_torque_calculation_hz_ = constrain(calculation_hz, 1u, 5000u);
}

void MotorControlApp::setPassiveTorqueFollowPidLowP(float value) {
  passive_torque_follow_pid_low_p_ = max(0.0f, value);
  passive_follow_pid_low_prev_error_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowPidLowI(float value) {
  passive_torque_follow_pid_low_i_ = max(0.0f, value);
  passive_follow_pid_low_integral_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowPidLowD(float value) {
  passive_torque_follow_pid_low_d_ = max(0.0f, value);
  passive_follow_pid_low_prev_error_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowPidRunP(float value) {
  passive_torque_follow_pid_run_p_ = max(0.0f, value);
  passive_follow_pid_run_prev_error_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowPidRunI(float value) {
  passive_torque_follow_pid_run_i_ = max(0.0f, value);
  passive_follow_pid_run_integral_ = 0.0f;
}

void MotorControlApp::setPassiveTorqueFollowPidRunD(float value) {
  passive_torque_follow_pid_run_d_ = max(0.0f, value);
  passive_follow_pid_run_prev_error_ = 0.0f;
}

void MotorControlApp::injectPassiveTorqueFieldOffsetDeg(float offset_deg) {
  const float current_angle = motor_.shaftAngle();
  const float offset_rad = constrain(offset_deg, -180.0f, 180.0f) * DEG_TO_RAD;
  passive_field_angle_ref_rad_ = current_angle - offset_rad;
  passive_torque_damping_angle_rad_ = offset_rad;
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
  passive_torque_last_update_us_ = micros();
}

void MotorControlApp::resetPassiveTorqueFieldReference() {
  passive_field_angle_ref_rad_ = motor_.shaftAngle();
  passive_torque_damping_angle_rad_ = 0.0f;
  passive_follow_pid_low_integral_ = 0.0f;
  passive_follow_pid_low_prev_error_ = 0.0f;
  passive_follow_pid_run_integral_ = 0.0f;
  passive_follow_pid_run_prev_error_ = 0.0f;
  passive_follow_using_run_pid_ = false;
  passive_torque_last_update_us_ = micros();
}

void MotorControlApp::updateReleasedState() {
  sensor_.update();
  motor_.shaft_angle = motor_.shaftAngle();
  motor_.shaft_velocity = motor_.shaftVelocity();
  motor_.electrical_angle = motor_.electricalAngle();
  motor_.voltage.q = 0.0f;
  motor_.voltage.d = 0.0f;
  passive_torque_damping_angle_rad_ = 0.0f;

  if (motor_.current_sense) {
    if (motor_.torque_controller == TorqueControlType::dc_current) {
      motor_.current.q =
          motor_.current_sense->getDCCurrent(motor_.electrical_angle);
      motor_.current.q = motor_.LPF_current_q(motor_.current.q);
      motor_.current.d = 0.0f;
    } else {
      motor_.current =
          motor_.current_sense->getFOCCurrents(motor_.electrical_angle);
      motor_.current.q = motor_.LPF_current_q(motor_.current.q);
      motor_.current.d = motor_.LPF_current_d(motor_.current.d);
    }
  }

  if (passive_torque_transition_pending_) {
    const unsigned long now_us = micros();
    const unsigned long buffer_us =
        static_cast<unsigned long>(passive_torque_transition_buffer_ms_) * 1000UL;
    if ((now_us - passive_torque_transition_start_us_) >= buffer_us) {
      resetPassiveTorqueFieldReference();
      passive_torque_transition_pending_ = false;
      release_mode_ = false;
      motor_.target = 0.0f;
      if (!motor_.enabled) {
        motor_.enable();
      }
      driver_.enable();
      passive_torque_last_update_us_ = now_us;
      reportPassiveTorqueDebug("align", 0.0f);
      return;
    }
  }

  driver_.disable();
}

void MotorControlApp::updatePassiveTorqueControl() {
  const unsigned long now_us = micros();
  const unsigned long update_interval_us =
      max(1UL, 1000000UL / max(1u, passive_torque_calculation_hz_));
  if ((now_us - passive_torque_last_update_us_) < update_interval_us) {
    return;
  }

  const float dt = max(
      1e-6f, (now_us - passive_torque_last_update_us_) / 1000000.0f);
  passive_torque_last_update_us_ = now_us;

  const float current_angle = motor_.shaft_angle;
  const float current_velocity = motor_.shaft_velocity;
  const bool use_run_pid =
      fabsf(current_velocity) >= passiveTorqueRunningSpeedThresholdRadS();
  if (use_run_pid != passive_follow_using_run_pid_) {
    passive_follow_using_run_pid_ = use_run_pid;
    if (use_run_pid) {
      passive_follow_pid_run_integral_ = 0.0f;
      passive_follow_pid_run_prev_error_ = 0.0f;
    } else {
      passive_follow_pid_low_integral_ = 0.0f;
      passive_follow_pid_low_prev_error_ = 0.0f;
    }
  }

  float& pid_integral = use_run_pid ? passive_follow_pid_run_integral_
                                    : passive_follow_pid_low_integral_;
  float& pid_prev_error = use_run_pid ? passive_follow_pid_run_prev_error_
                                      : passive_follow_pid_low_prev_error_;
  const float pid_p = use_run_pid ? passive_torque_follow_pid_run_p_
                                  : passive_torque_follow_pid_low_p_;
  const float pid_i = use_run_pid ? passive_torque_follow_pid_run_i_
                                  : passive_torque_follow_pid_low_i_;
  const float pid_d = use_run_pid ? passive_torque_follow_pid_run_d_
                                  : passive_torque_follow_pid_low_d_;

  const float saturation_angle_rad = passiveTorqueSaturationAngleRad();
  const float raw_follow_error = current_angle - passive_field_angle_ref_rad_;
  const float follow_deadzone_rad = passiveFieldFollowDeadzoneRad();

  float follow_error = 0.0f;
  if (fabsf(raw_follow_error) > follow_deadzone_rad) {
    follow_error = copysignf(
        fabsf(raw_follow_error) - follow_deadzone_rad, raw_follow_error);
  } else {
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
  }

  const float integral_limit =
      pid_i > 1e-6f ? saturation_angle_rad / pid_i : 0.0f;
  if (fabsf(follow_error) > 1e-6f) {
    pid_integral += follow_error * dt;
    if (integral_limit > 0.0f) {
      pid_integral = constrain(pid_integral, -integral_limit, integral_limit);
    }
  }

  const float derivative =
      dt > 1e-6f ? (follow_error - pid_prev_error) / dt : 0.0f;
  float follow_step = pid_p * follow_error + pid_i * pid_integral +
                      pid_d * derivative;
  follow_step =
      constrain(follow_step, -fabsf(raw_follow_error), fabsf(raw_follow_error));

  passive_field_angle_ref_rad_ += follow_step;
  pid_prev_error = follow_error;

  const float damping_angle = current_angle - passive_field_angle_ref_rad_;
  passive_torque_damping_angle_rad_ = damping_angle;

  float torque_nm = 0.0f;
  if (saturation_angle_rad > 1e-6f) {
    const float normalized_angle =
        constrain(damping_angle / saturation_angle_rad, -1.0f, 1.0f);
    // 磁场阻尼角与输出电流采用二阶线性关系，同时保留方向符号。
    const float torque_scale = normalized_angle * fabsf(normalized_angle);
    torque_nm = -torque_scale * passive_torque_target_nm_;
  }

  const float iq_target = constrain(torque_nm / motorKtNmPerAmp(),
                                    -motor_.current_limit,
                                    motor_.current_limit);
  motor_.target = iq_target;
}

void MotorControlApp::reportReleaseMode() {
  Stream& port = activeCommandPort();
  port.print(F("Release:"));
  port.println(release_mode_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueMode() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueMode:"));
  port.println(passive_torque_mode_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueDebugEnabled() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueDebug:"));
  port.println(passive_torque_debug_enabled_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueTargetNm() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueTarget:"));
  port.println(passive_torque_target_nm_, 4);
}

void MotorControlApp::reportPassiveTorqueSaturationAngleDeg() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueSaturationAngle:"));
  port.println(passive_torque_saturation_angle_deg_, 4);
}

void MotorControlApp::reportPassiveTorqueFollowDeadzoneDeg() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowDeadzone:"));
  port.println(passive_torque_follow_deadzone_deg_, 4);
}

void MotorControlApp::reportPassiveTorqueRunningSpeedThresholdRadS() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueRunningSpeedThreshold:"));
  port.println(passive_torque_running_speed_threshold_rad_s_, 4);
}

void MotorControlApp::reportPassiveTorqueCalculationHz() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueCalculationHz:"));
  port.println(passive_torque_calculation_hz_);
}

void MotorControlApp::reportPassiveTorqueFollowPidLowP() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidP:"));
  port.println(passive_torque_follow_pid_low_p_, 6);
}

void MotorControlApp::reportPassiveTorqueFollowPidLowI() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidI:"));
  port.println(passive_torque_follow_pid_low_i_, 6);
}

void MotorControlApp::reportPassiveTorqueFollowPidLowD() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidD:"));
  port.println(passive_torque_follow_pid_low_d_, 6);
}

void MotorControlApp::reportPassiveTorqueFollowPidRunP() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidRunP:"));
  port.println(passive_torque_follow_pid_run_p_, 6);
}

void MotorControlApp::reportPassiveTorqueFollowPidRunI() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidRunI:"));
  port.println(passive_torque_follow_pid_run_i_, 6);
}

void MotorControlApp::reportPassiveTorqueFollowPidRunD() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueFollowPidRunD:"));
  port.println(passive_torque_follow_pid_run_d_, 6);
}

void MotorControlApp::reportPassiveTorqueDampingAngleDeg() {
  Stream& port = activeCommandPort();
  port.print(F("PassiveTorqueDampingAngle:"));
  port.println(passive_torque_damping_angle_rad_ * RAD_TO_DEG, 4);
}

void MotorControlApp::reportMotionDownsample() {
  Stream& port = activeCommandPort();
  port.print(F("Motion: downsample: "));
  port.println(motion_downsample_);
}

void MotorControlApp::reportPassiveTorqueDebug(const char* phase,
                                               float iq_target) const {
  if (!passive_torque_debug_enabled_) {
    return;
  }

  Stream& port = active_command_ != nullptr && active_command_->com_port != nullptr
                     ? *active_command_->com_port
                     : Serial;

  const bool use_run_pid =
      fabsf(motor_.shaft_velocity) >= passiveTorqueRunningSpeedThresholdRadS();
  const float pid_p =
      use_run_pid ? passive_torque_follow_pid_run_p_ : passive_torque_follow_pid_low_p_;
  const float pid_i =
      use_run_pid ? passive_torque_follow_pid_run_i_ : passive_torque_follow_pid_low_i_;
  const float pid_d =
      use_run_pid ? passive_torque_follow_pid_run_d_ : passive_torque_follow_pid_low_d_;

  port.print(F("[PassiveTorque] "));
  port.print(phase);
  port.print(F(" | mode="));
  port.print(use_run_pid ? F("run") : F("low"));
  port.print(F(" | mech="));
  port.print(motor_.shaft_angle, 4);
  port.print(F(" rad | vel="));
  port.print(motor_.shaft_velocity, 4);
  port.print(F(" rad/s | field="));
  port.print(passive_field_angle_ref_rad_, 4);
  port.print(F(" rad | damping="));
  port.print(passive_torque_damping_angle_rad_ * RAD_TO_DEG, 4);
  port.print(F(" deg | saturation="));
  port.print(passive_torque_saturation_angle_deg_, 4);
  port.print(F(" deg | deadzone="));
  port.print(passive_torque_follow_deadzone_deg_, 4);
  port.print(F(" deg | calc="));
  port.print(passive_torque_calculation_hz_);
  port.print(F(" Hz | speed_threshold="));
  port.print(passive_torque_running_speed_threshold_rad_s_, 4);
  port.print(F(" rad/s | pid=("));
  port.print(pid_p, 4);
  port.print(F(","));
  port.print(pid_i, 4);
  port.print(F(","));
  port.print(pid_d, 4);
  port.print(F(") | torque="));
  port.print(passive_torque_target_nm_, 4);
  port.print(F(" Nm | iq="));
  port.println(iq_target, 4);
}

float MotorControlApp::clampPassiveTorqueTargetNm(float target_nm) const {
  const float max_safe_torque_nm = motor_.current_limit * motorKtNmPerAmp();
  return constrain(fabsf(target_nm), 0.0f, max_safe_torque_nm);
}

float MotorControlApp::motorKtNmPerAmp() const {
  return 60.0f / (_2PI * max(config_.kv_rpm_per_volt, 1e-3f));
}

float MotorControlApp::passiveTorqueSaturationAngleRad() const {
  return passive_torque_saturation_angle_deg_ * DEG_TO_RAD;
}

float MotorControlApp::passiveFieldFollowDeadzoneRad() const {
  return passive_torque_follow_deadzone_deg_ * DEG_TO_RAD;
}

float MotorControlApp::passiveTorqueRunningSpeedThresholdRadS() const {
  return max(0.1f, passive_torque_running_speed_threshold_rad_s_);
}
