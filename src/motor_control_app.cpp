#include "motor_control_app.h"

MotorControlApp* MotorControlApp::active_instance_ = nullptr;

namespace {
// 自定义命令也遵循 Commander 的空参数约定，避免查询命令被误判成设置命令。
bool IsCommandSentinel(char ch) {
  return ch == '\0' || ch == '\n' || ch == '\r';
}
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
      command_(Serial),
      motion_downsample_(config.motion_downsample),
      passive_torque_target_nm_(config.passive_torque_target_nm),
      passive_torque_max_damping_angle_deg_(
          config.passive_torque_max_damping_angle_deg),
      passive_torque_update_hz_(config.passive_torque_update_hz) {
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
  resetPassiveTorqueFieldReference();

  command_.add(config_.command_id, handleMotorCommandStatic, "motor");

  Serial.println(F("Motor self-check finished, target set to 0."));
  Serial.printf("[I2C] SDA=%d SCL=%d clk=%luHz timeout=%ums\n",
                config_.i2c_sda_pin,
                config_.i2c_scl_pin,
                (unsigned long)config_.i2c_clock_hz,
                (unsigned)config_.i2c_timeout_ms);
  Serial.printf(
      "[PassiveTorque] target=%.4fNm max_angle=%.3fdeg logic=%uHz current_limit=%.2fA\n",
      passive_torque_target_nm_,
      passive_torque_max_damping_angle_deg_,
      passive_torque_update_hz_,
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

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'A') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueMaxDampingAngleDeg(atof(cmd + 2));
    }
    reportPassiveTorqueMaxDampingAngleDeg();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'F') {
    if (!IsCommandSentinel(cmd[2])) {
      setPassiveTorqueUpdateHz(
          static_cast<unsigned int>(max(1, atoi(cmd + 2))));
    }
    reportPassiveTorqueUpdateHz();
    return;
  }

  if (cmd && cmd[0] == 'Z' && cmd[1] == 'G') {
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
    passive_torque_damping_angle_rad_ = 0.0f;
    motor_.target = 0.0f;
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
  passive_torque_damping_angle_rad_ = 0.0f;
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
  passive_torque_damping_angle_rad_ = 0.0f;

  if (passive_torque_mode_) {
    release_mode_ = false;
    motor_.controller = MotionControlType::torque;
    motor_.torque_controller = TorqueControlType::foc_current;
    if (!motor_.enabled) {
      motor_.enable();
    }
    driver_.enable();
    resetPassiveTorqueFieldReference();
    reportPassiveTorqueDebug("enable", 0.0f);
  } else {
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

void MotorControlApp::setPassiveTorqueMaxDampingAngleDeg(float angle_deg) {
  passive_torque_max_damping_angle_deg_ = constrain(fabsf(angle_deg), 0.05f, 45.0f);
}

void MotorControlApp::setPassiveTorqueUpdateHz(unsigned int update_hz) {
  passive_torque_update_hz_ = constrain(update_hz, 1u, 5000u);
}

void MotorControlApp::resetPassiveTorqueFieldReference() {
  passive_field_angle_ref_rad_ = motor_.shaftAngle();
  passive_torque_damping_angle_rad_ = 0.0f;
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

void MotorControlApp::updatePassiveTorqueControl() {
  const unsigned long now_us = micros();
  const unsigned long update_interval_us =
      max(1UL, 1000000UL / max(1u, passive_torque_update_hz_));
  if ((now_us - passive_torque_last_update_us_) < update_interval_us) {
    return;
  }
  passive_torque_last_update_us_ = now_us;

  const float current_angle = motor_.shaft_angle;
  const float max_damping_angle_rad = maxPassiveTorqueDampingAngleRad();
  float damping_angle = current_angle - passive_field_angle_ref_rad_;
  bool field_followed = false;

  if (damping_angle > max_damping_angle_rad) {
    passive_field_angle_ref_rad_ = current_angle - max_damping_angle_rad;
    damping_angle = max_damping_angle_rad;
    field_followed = true;
  } else if (damping_angle < -max_damping_angle_rad) {
    passive_field_angle_ref_rad_ = current_angle + max_damping_angle_rad;
    damping_angle = -max_damping_angle_rad;
    field_followed = true;
  }

  passive_torque_damping_angle_rad_ = damping_angle;

  const float torque_scale =
      max_damping_angle_rad > 1e-6f ? damping_angle / max_damping_angle_rad : 0.0f;
  const float torque_nm = -torque_scale * passive_torque_target_nm_;
  const float iq_target = constrain(
      torque_nm / motorKtNmPerAmp(),
      -motor_.current_limit,
      motor_.current_limit);

  motor_.target = iq_target;

  if (field_followed) {
    reportPassiveTorqueDebug("follow", iq_target);
  }
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

void MotorControlApp::reportPassiveTorqueMaxDampingAngleDeg() {
  Serial.print(F("PassiveTorqueMaxAngle:"));
  Serial.println(passive_torque_max_damping_angle_deg_, 4);
}

void MotorControlApp::reportPassiveTorqueUpdateHz() {
  Serial.print(F("PassiveTorqueUpdateHz:"));
  Serial.println(passive_torque_update_hz_);
}

void MotorControlApp::reportPassiveTorqueDampingAngleDeg() {
  Serial.print(F("PassiveTorqueDampingAngle:"));
  Serial.println(passive_torque_damping_angle_rad_ * RAD_TO_DEG, 4);
}

void MotorControlApp::reportMotionDownsample() {
  Serial.print(F("Motion: downsample: "));
  Serial.println(motion_downsample_);
}

void MotorControlApp::reportPassiveTorqueDebug(const char* phase,
                                               float iq_target) const {
  if (!passive_torque_debug_enabled_) {
    return;
  }

  Serial.print(F("[PassiveTorque] "));
  Serial.print(phase);
  Serial.print(F(" | mech="));
  Serial.print(motor_.shaft_angle, 4);
  Serial.print(F(" rad | field="));
  Serial.print(passive_field_angle_ref_rad_, 4);
  Serial.print(F(" rad | damping="));
  Serial.print(passive_torque_damping_angle_rad_ * RAD_TO_DEG, 4);
  Serial.print(F(" deg | max="));
  Serial.print(passive_torque_max_damping_angle_deg_, 4);
  Serial.print(F(" deg | torque="));
  Serial.print(passive_torque_target_nm_, 4);
  Serial.print(F(" Nm | iq="));
  Serial.println(iq_target, 4);
}

float MotorControlApp::clampPassiveTorqueTargetNm(float target_nm) const {
  const float max_safe_torque_nm = motor_.current_limit * motorKtNmPerAmp();
  return constrain(fabsf(target_nm), 0.0f, max_safe_torque_nm);
}

float MotorControlApp::motorKtNmPerAmp() const {
  return 60.0f / (_2PI * config_.kv_rpm_per_volt);
}

float MotorControlApp::maxPassiveTorqueDampingAngleRad() const {
  return passive_torque_max_damping_angle_deg_ * DEG_TO_RAD;
}
