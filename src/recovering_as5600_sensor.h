#pragma once

#include <SimpleFOC.h>

// 带自动恢复能力的 AS5600 I2C 传感器封装
class RecoveringAS5600Sensor : public Sensor {
 public:
  RecoveringAS5600Sensor(TwoWire* wire_bus,
                         uint8_t i2c_address,
                         uint8_t angle_reg_msb,
                         uint16_t cpr,
                         int sda_pin,
                         int scl_pin,
                         uint32_t clock_hz,
                         uint16_t timeout_ms);

  void begin();

 protected:
  float getSensorAngle() override;

 private:
  bool readRaw(uint16_t& raw, int& tx_error, size_t& rx_len);
  void handleReadFailure(int tx_error, size_t rx_len);
  int clearBus();
  bool reinitializeBus(const char* reason, int tx_error = -1, size_t rx_len = 0);

  TwoWire* wire_;
  uint8_t i2c_address_;
  uint8_t angle_reg_msb_;
  uint16_t cpr_;
  int sda_pin_;
  int scl_pin_;
  uint32_t clock_hz_;
  uint16_t timeout_ms_;
  uint16_t last_good_raw_ = 0;
  uint16_t consecutive_failures_ = 0;
  uint32_t last_error_log_ms_ = 0;
  uint32_t last_recovery_ms_ = 0;
  uint32_t recovery_count_ = 0;
};
