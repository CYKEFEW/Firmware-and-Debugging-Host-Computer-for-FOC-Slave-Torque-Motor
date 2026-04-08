#include "motor_control_app.h"

MotorControlApp* MotorControlApp::active_instance_ = nullptr;

namespace {
// 自定义命令也按 Commander 的哨兵规则处理，避免查询命令被误当成设置命令
bool IsCommandSentinel(char ch) {
  return ch == '\0' || ch == '\n' || ch == '\r';
}
}

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
      command_(Serial),
      motion_downsample_(config.motion_downsample),
      passive_torque_target_nm_(config.passive_torque_target_nm),
      passive_torque_vel_on_rad_s_(config.passive_torque_vel_on_rad_s),
      passive_torque_vel_off_rad_s_(config.passive_torque_vel_off_rad_s) {
  active_instance_ = this;
}

void MotorControlApp::setup() {
  Serial.begin(config_.serial_baud);
  delay(50);

  initializeI2C();
  initializeMotor();
  initializeCurrentSense();

  motor_.init();
  motor_.initFOC();
  motor_.target = 0.0f;

  command_.add(config_.command_id, handleMotorCommandStatic, "motor");

  Serial.println(F("电机自检完成，目标值已设为0。"));
  Serial.printf("[I2C] AS5600总线: SDA=%d SCL=%d 频率=%luHz 超时=%ums\n",
                config_.i2c_sda_pin,
                config_.i2c_scl_pin,
                (unsigned long)config_.i2c_clock_hz,
                (unsigned)config_.i2c_timeout_ms);
  Serial.printf("[CTRL] 电流限制=%.2fA, Kt=%.4fNm/A, 从动力矩=%.3fNm, 起控阈值=%.3frad/s, 释放阈值=%.3frad/s\n",
                motor_.current_limit,
                motorKtNmPerAmp(),
                passive_torque_target_nm_,
                passive_torque_vel_on_rad_s_,
                passive_torque_vel_off_rad_s_);

  _delay(1000);
}

void MotorControlApp::loop() {
  if (release_mode_) {
    updateReleasedState();
  } else if (passive_torque_mode_) {
    motor_.loopFOC();
    updatePassiveTorqueTarget();
    motor_.move();
  } else {
    motor_.loopFOC();
    motor_.move();
  }

  motor_.monitor();
  command_.run();
}

void MotorControlApp::handleMotorCommandStatic(char* cmd) {
  if (active_instance_ != nullptr) {
    active_instance_->handleMotorCommand(cmd);
  }
}

void MotorControlApp::handleMotorCommand(char* cmd) {
  if (cmd && cmd[0] == 'X') {
    if (!IsCommandSentinel(cmd[1])) {
      setReleaseMode(cmd[1] != '0');
    }
    reportReleaseMode();
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

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'V') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueVelOn(atof(cmd + 2));
    }
    reportPassiveTorqueVelOn();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'W') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueVelOff(atof(cmd + 2));
    }
    reportPassiveTorqueVelOff();
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
    passive_torque_active_ = false;
  }

  command_.motor(&motor_, cmd);
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

void MotorControlApp::initializeMotor() {
  driver_.voltage_power_supply = config_.power_supply_voltage;
  driver_.init();
  motor_.linkDriver(&driver_);

  motor_.torque_controller = TorqueControlType::foc_current;
  motor_.controller = MotionControlType::torque;
  motor_.PID_velocity.P = config_.pid_velocity_p;
  motor_.PID_velocity.I = config_.pid_velocity_i;
  motor_.PID_velocity.D = config_.pid_velocity_d;
  motor_.current_limit = config_.current_limit_a;
  motor_.voltage_limit = config_.voltage_limit_v;
  motor_.LPF_velocity.Tf = config_.lpf_velocity_tf;
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

void MotorControlApp::setReleaseMode(bool enabled) {
  release_mode_ = enabled;
  if (release_mode_) {
    passive_torque_mode_ = false;
    passive_torque_active_ = false;
    passive_torque_direction_ = 0;
    driver_.disable();
  } else if (motor_.enabled) {
    driver_.enable();
  }
}

void MotorControlApp::setPassiveTorqueMode(bool enabled) {
  passive_torque_mode_ = enabled;
  passive_torque_active_ = false;
  passive_torque_direction_ = 0;
  if (passive_torque_mode_) {
    release_mode_ = false;
    motor_.controller = MotionControlType::torque;
    motor_.torque_controller = TorqueControlType::foc_current;
    motor_.target = 0.0f;
    if (!motor_.enabled) {
      motor_.enable();
    }
    driver_.enable();
    reportPassiveTorqueDebug("enable", 0.0f, 0.0f);
  } else {
    motor_.target = 0.0f;
    reportPassiveTorqueDebug("disable", 0.0f, 0.0f);
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

void MotorControlApp::setPassiveTorqueVelOn(float velocity_threshold) {
  passive_torque_vel_on_rad_s_ = max(0.0f, velocity_threshold);
}

void MotorControlApp::setPassiveTorqueVelOff(float velocity_threshold) {
  passive_torque_vel_off_rad_s_ = max(0.0f, velocity_threshold);
}

void MotorControlApp::updateReleasedState() {
  sensor_.update();
  motor_.shaft_angle = motor_.shaftAngle();
  motor_.shaft_velocity = motor_.shaftVelocity();
  motor_.electrical_angle = motor_.electricalAngle();
  motor_.voltage.q = 0.0f;
  motor_.voltage.d = 0.0f;

  if (motor_.current_sense) {
    if (motor_.torque_controller == TorqueControlType::dc_current) {
      motor_.current.q = motor_.current_sense->getDCCurrent(motor_.electrical_angle);
      motor_.current.q = motor_.LPF_current_q(motor_.current.q);
      motor_.current.d = 0.0f;
    } else {
      motor_.current = motor_.current_sense->getFOCCurrents(motor_.electrical_angle);
      motor_.current.q = motor_.LPF_current_q(motor_.current.q);
      motor_.current.d = motor_.LPF_current_d(motor_.current.d);
    }
  }

  driver_.disable();
}

void MotorControlApp::updatePassiveTorqueTarget() {
  motor_.shaft_angle = motor_.shaftAngle();
  motor_.shaft_velocity = motor_.shaftVelocity();
  const float velocity = motor_.shaft_velocity;
  const float abs_velocity = fabsf(velocity);
  const float enter_threshold = max(passive_torque_vel_on_rad_s_, passive_torque_vel_off_rad_s_);
  const float exit_threshold = min(passive_torque_vel_on_rad_s_, passive_torque_vel_off_rad_s_);
  const bool was_active = passive_torque_active_;

  if (passive_torque_active_) {
    if (abs_velocity <= exit_threshold) {
      passive_torque_active_ = false;
    }
  } else if (abs_velocity >= enter_threshold) {
    passive_torque_active_ = true;
  }

  if (!passive_torque_active_) {
    motor_.target = 0.0f;
    if (was_active) {
      passive_torque_direction_ = 0;
      reportPassiveTorqueDebug("exit", velocity, 0.0f);
    }
    return;
  }

  const float torque_nm = velocity > 0.0f ? -passive_torque_target_nm_ : passive_torque_target_nm_;
  const float iq_target = constrain(
      torque_nm / motorKtNmPerAmp(),
      -motor_.current_limit,
      motor_.current_limit);
  const int direction = velocity > 0.0f ? 1 : -1;

  if (!was_active) {
    reportPassiveTorqueDebug("enter", velocity, iq_target);
  } else if (direction != passive_torque_direction_) {
    reportPassiveTorqueDebug("reverse", velocity, iq_target);
  }

  passive_torque_direction_ = direction;
  motor_.target = iq_target;
}

void MotorControlApp::reportReleaseMode() {
  Serial.print(F("Release:"));
  Serial.println(release_mode_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueMode() {
  Serial.print(F("PassiveTorqueMode:"));
  Serial.println(passive_torque_mode_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueDebugEnabled() {
  Serial.print(F("PassiveTorqueDebug:"));
  Serial.println(passive_torque_debug_enabled_ ? 1 : 0);
}

void MotorControlApp::reportPassiveTorqueTargetNm() {
  Serial.print(F("PassiveTorqueTarget:"));
  Serial.println(passive_torque_target_nm_, 4);
}

void MotorControlApp::reportPassiveTorqueVelOn() {
  Serial.print(F("PassiveTorqueVelOn:"));
  Serial.println(passive_torque_vel_on_rad_s_, 4);
}

void MotorControlApp::reportPassiveTorqueVelOff() {
  Serial.print(F("PassiveTorqueVelOff:"));
  Serial.println(passive_torque_vel_off_rad_s_, 4);
}

void MotorControlApp::reportPassiveTorqueDebug(const char* phase,
                                               float velocity,
                                               float iq_target) const {
  if (!passive_torque_debug_enabled_) {
    return;
  }
  Serial.print(F("[PassiveTorque] "));
  Serial.print(phase);
  Serial.print(F(" | angle="));
  Serial.print(motor_.shaft_angle, 4);
  Serial.print(F(" rad"));
  Serial.print(F(" | vel="));
  Serial.print(velocity, 4);
  Serial.print(F(" rad/s | enter="));
  Serial.print(max(passive_torque_vel_on_rad_s_, passive_torque_vel_off_rad_s_), 4);
  Serial.print(F(" | exit="));
  Serial.print(min(passive_torque_vel_on_rad_s_, passive_torque_vel_off_rad_s_), 4);
  Serial.print(F(" | torque="));
  Serial.print(passive_torque_target_nm_, 4);
  Serial.print(F(" Nm | iq="));
  Serial.println(iq_target, 4);
}

void MotorControlApp::reportMotionDownsample() {
  Serial.print(F("Motion: downsample: "));
  Serial.println(motion_downsample_);
}

float MotorControlApp::clampPassiveTorqueTargetNm(float target_nm) const {
  const float max_safe_torque_nm = motor_.current_limit * motorKtNmPerAmp();
  return constrain(fabsf(target_nm), 0.0f, max_safe_torque_nm);
}

float MotorControlApp::motorKtNmPerAmp() const {
  return 60.0f / (_2PI * config_.kv_rpm_per_volt);
}
