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
  float passive_torque_saturation_angle_deg;
  float passive_torque_follow_deadzone_deg;
  float passive_torque_running_speed_threshold_rad_s;
  unsigned int passive_torque_calculation_hz;
  unsigned int passive_torque_transition_buffer_ms;
  float passive_torque_follow_pid_low_p;
  float passive_torque_follow_pid_low_i;
  float passive_torque_follow_pid_low_d;
  float passive_torque_follow_pid_run_p;
  float passive_torque_follow_pid_run_i;
  float passive_torque_follow_pid_run_d;
  float pid_velocity_p;
  float pid_velocity_i;
  float pid_velocity_d;
  float lpf_velocity_tf;
  float p_angle_p;
  float pid_current_q_p;
  float pid_current_q_i;
  float pid_current_q_d;
  float lpf_current_q_tf;
  float pid_current_d_p;
  float pid_current_d_i;
  float pid_current_d_d;
  float lpf_current_d_tf;
  unsigned int motion_downsample;
  uint8_t monitor_variables;
  unsigned int monitor_downsample;
  unsigned long serial_baud;
  bool serial0_command_enabled;
  bool serial1_command_enabled;
  bool serial2_command_enabled;
  int serial1_rx_pin;
  int serial1_tx_pin;
  int serial2_rx_pin;
  int serial2_tx_pin;
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

  Stream& activeCommandPort();
  void handleMotorCommand(char* cmd);
  void initializeI2C();
  void initializeSerialPorts();
  void initializeMotor();
  void initializeCurrentSense();
  void initializeCommanders();
  void runCommander(Commander& commander, bool enabled);
  void setReleaseMode(bool enabled);
  void setPassiveTorqueMode(bool enabled);
  void setPassiveTorqueDebug(bool enabled);
  void setMotionDownsample(unsigned int downsample);
  void setPassiveTorqueTargetNm(float target_nm);
  void setPassiveTorqueSaturationAngleDeg(float angle_deg);
  void setPassiveTorqueFollowDeadzoneDeg(float angle_deg);
  void setPassiveTorqueRunningSpeedThresholdRadS(float value);
  void setPassiveTorqueCalculationHz(unsigned int calculation_hz);
  void setPassiveTorqueFollowPidLowP(float value);
  void setPassiveTorqueFollowPidLowI(float value);
  void setPassiveTorqueFollowPidLowD(float value);
  void setPassiveTorqueFollowPidRunP(float value);
  void setPassiveTorqueFollowPidRunI(float value);
  void setPassiveTorqueFollowPidRunD(float value);
  void injectPassiveTorqueFieldOffsetDeg(float offset_deg);
  void resetPassiveTorqueFieldReference();
  void updateReleasedState();
  void updatePassiveTorqueControl();
  void reportReleaseMode();
  void reportPassiveTorqueMode();
  void reportPassiveTorqueDebugEnabled();
  void reportPassiveTorqueTargetNm();
  void reportPassiveTorqueSaturationAngleDeg();
  void reportPassiveTorqueFollowDeadzoneDeg();
  void reportPassiveTorqueRunningSpeedThresholdRadS();
  void reportPassiveTorqueCalculationHz();
  void reportPassiveTorqueFollowPidLowP();
  void reportPassiveTorqueFollowPidLowI();
  void reportPassiveTorqueFollowPidLowD();
  void reportPassiveTorqueFollowPidRunP();
  void reportPassiveTorqueFollowPidRunI();
  void reportPassiveTorqueFollowPidRunD();
  void reportPassiveTorqueDampingAngleDeg();
  void reportMotionDownsample();
  void reportPassiveTorqueDebug(const char* phase, float iq_target) const;
  float clampPassiveTorqueTargetNm(float target_nm) const;
  float motorKtNmPerAmp() const;
  float passiveTorqueSaturationAngleRad() const;
  float passiveFieldFollowDeadzoneRad() const;
  float passiveTorqueRunningSpeedThresholdRadS() const;

  static MotorControlApp* active_instance_;

  MotorAppConfig config_;
  BLDCMotor motor_;
  BLDCDriver3PWM driver_;
  TwoWire wire_;
  RecoveringAS5600Sensor sensor_;
  InlineCurrentSense current_sense_;
  Commander serial0_command_;
  Commander serial1_command_;
  Commander serial2_command_;
  Commander* active_command_ = nullptr;
  bool release_mode_ = false;
  bool passive_torque_mode_ = false;
  bool passive_torque_debug_enabled_ = false;
  unsigned int motion_downsample_ = 0;
  float passive_torque_target_nm_;
  float passive_torque_saturation_angle_deg_;
  float passive_torque_follow_deadzone_deg_;
  float passive_torque_running_speed_threshold_rad_s_;
  unsigned int passive_torque_calculation_hz_;
  unsigned int passive_torque_transition_buffer_ms_;
  float passive_torque_follow_pid_low_p_;
  float passive_torque_follow_pid_low_i_;
  float passive_torque_follow_pid_low_d_;
  float passive_torque_follow_pid_run_p_;
  float passive_torque_follow_pid_run_i_;
  float passive_torque_follow_pid_run_d_;
  unsigned long passive_torque_last_update_us_ = 0;
  float passive_field_angle_ref_rad_ = 0.0f;
  float passive_torque_damping_angle_rad_ = 0.0f;
  float passive_follow_pid_low_integral_ = 0.0f;
  float passive_follow_pid_low_prev_error_ = 0.0f;
  float passive_follow_pid_run_integral_ = 0.0f;
  float passive_follow_pid_run_prev_error_ = 0.0f;
  bool passive_follow_using_run_pid_ = false;
  bool passive_torque_transition_pending_ = false;
  unsigned long passive_torque_transition_start_us_ = 0;
};
