#pragma once

#include <SimpleFOC.h>

#include "recovering_as5600_sensor.h"

struct MotorAppConfig {
  int pole_pairs;
  int pwm_u_pin;
  int pwm_v_pin;
  int pwm_w_pin;
  int driver_enable_pin;
  int current_a_pin;
  int current_b_pin;
  int i2c_sda_pin;
  int i2c_scl_pin;
  int gpio_pullup_32;
  int gpio_pullup_33;
  int gpio_pullup_25;
  int gpio_pullup_26;
  int gpio_pullup_27;
  int gpio_pullup_14;
  uint8_t i2c_address;
  uint8_t angle_reg_msb;
  uint16_t cpr;
  uint32_t i2c_clock_hz;
  uint16_t i2c_timeout_ms;
  float power_supply_voltage;
  float shunt_resistor_ohm;
  float current_sense_gain;
  float current_limit_a;
  float voltage_limit_v;
  float velocity_limit_rad_s;
  float kv_rpm_per_volt;
  float passive_torque_target_nm;
  float passive_torque_max_damping_angle_deg;
  unsigned int passive_torque_update_hz;
  float pid_velocity_p;
  float pid_velocity_i;
  float pid_velocity_d;
  float lpf_velocity_tf;
  float p_angle_p;
  unsigned int motion_downsample;
  uint8_t monitor_variables;
  unsigned int monitor_downsample;
  unsigned long serial_baud;
  char command_id;
};

// 电机应用层：封装初始化、运行循环和自定义 Commander 命令。
class MotorControlApp {
 public:
  explicit MotorControlApp(const MotorAppConfig& config);

  void setup();
  void loop();

 private:
  static void handleMotorCommandStatic(char* cmd);

  void handleMotorCommand(char* cmd);
  void initializeI2C();
  void initializeMotor();
  void initializeCurrentSense();
  void setReleaseMode(bool enabled);
  void setPassiveTorqueMode(bool enabled);
  void setPassiveTorqueDebug(bool enabled);
  void setMotionDownsample(unsigned int downsample);
  void setPassiveTorqueTargetNm(float target_nm);
  void setPassiveTorqueMaxDampingAngleDeg(float angle_deg);
  void setPassiveTorqueUpdateHz(unsigned int update_hz);
  void resetPassiveTorqueFieldReference();
  void updateReleasedState();
  void updatePassiveTorqueControl();
  void reportReleaseMode();
  void reportPassiveTorqueMode();
  void reportPassiveTorqueDebugEnabled();
  void reportPassiveTorqueTargetNm();
  void reportPassiveTorqueMaxDampingAngleDeg();
  void reportPassiveTorqueUpdateHz();
  void reportPassiveTorqueDampingAngleDeg();
  void reportMotionDownsample();
  void reportPassiveTorqueDebug(const char* phase, float iq_target) const;
  float clampPassiveTorqueTargetNm(float target_nm) const;
  float motorKtNmPerAmp() const;
  float maxPassiveTorqueDampingAngleRad() const;

  static MotorControlApp* active_instance_;

  MotorAppConfig config_;
  BLDCMotor motor_;
  BLDCDriver3PWM driver_;
  TwoWire wire_;
  RecoveringAS5600Sensor sensor_;
  InlineCurrentSense current_sense_;
  Commander command_;
  bool release_mode_ = false;
  bool passive_torque_mode_ = false;
  bool passive_torque_debug_enabled_ = false;
  unsigned int motion_downsample_ = 0;
  float passive_torque_target_nm_;
  float passive_torque_max_damping_angle_deg_;
  unsigned int passive_torque_update_hz_;
  unsigned long passive_torque_last_update_us_ = 0;
  float passive_field_angle_ref_rad_ = 0.0f;
  float passive_torque_damping_angle_rad_ = 0.0f;
};
