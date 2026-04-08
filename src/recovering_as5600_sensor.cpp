#include "recovering_as5600_sensor.h"

RecoveringAS5600Sensor::RecoveringAS5600Sensor(TwoWire* wire_bus,
                                               uint8_t i2c_address,
                                               uint8_t angle_reg_msb,
                                               uint16_t cpr,
                                               int sda_pin,
                                               int scl_pin,
                                               uint32_t clock_hz,
                                               uint16_t timeout_ms)
    : wire_(wire_bus),
      i2c_address_(i2c_address),
      angle_reg_msb_(angle_reg_msb),
      cpr_(cpr),
      sda_pin_(sda_pin),
      scl_pin_(scl_pin),
      clock_hz_(clock_hz),
      timeout_ms_(timeout_ms) {}

void RecoveringAS5600Sensor::begin() {
  reinitializeBus("startup");
  Sensor::init();
}

float RecoveringAS5600Sensor::getSensorAngle() {
  uint16_t raw = 0;
  int tx_error = 0;
  size_t rx_len = 0;

  if (readRaw(raw, tx_error, rx_len)) {
    consecutive_failures_ = 0;
    last_good_raw_ = raw;
    return (raw / (float)cpr_) * _2PI;
  }

  handleReadFailure(tx_error, rx_len);
  return (last_good_raw_ / (float)cpr_) * _2PI;
}

bool RecoveringAS5600Sensor::readRaw(uint16_t& raw, int& tx_error, size_t& rx_len) {
  wire_->beginTransmission(i2c_address_);
  wire_->write(angle_reg_msb_);
  tx_error = wire_->endTransmission(false);
  if (tx_error != 0) {
    rx_len = 0;
    return false;
  }

  rx_len = wire_->requestFrom((uint16_t)i2c_address_, (size_t)2, true);
  if (rx_len != 2 || wire_->available() < 2) {
    return false;
  }

  const int msb = wire_->read();
  const int lsb = wire_->read();
  if (msb < 0 || lsb < 0) {
    return false;
  }

  raw = (((uint16_t)msb & 0x0F) << 8) | ((uint16_t)lsb & 0xFF);
  return true;
}

void RecoveringAS5600Sensor::handleReadFailure(int tx_error, size_t rx_len) {
  consecutive_failures_++;
  const uint32_t now = millis();

  if (now - last_error_log_ms_ >= 500) {
    Serial.printf(
        "[I2C] AS5600读失败: tx_err=%d rx_len=%u consecutive=%u\n",
        tx_error,
        (unsigned)rx_len,
        consecutive_failures_);
    last_error_log_ms_ = now;
  }

  if (now - last_recovery_ms_ >= 100) {
    reinitializeBus("read failure", tx_error, rx_len);
    last_recovery_ms_ = now;
  }
}

int RecoveringAS5600Sensor::clearBus() {
  pinMode(scl_pin_, INPUT_PULLUP);
  pinMode(sda_pin_, INPUT_PULLUP);
  delay(1);

  if (digitalRead(scl_pin_) == LOW) {
    return 1;
  }

  if (digitalRead(sda_pin_) == LOW) {
    pinMode(scl_pin_, OUTPUT);
    digitalWrite(scl_pin_, HIGH);
    for (uint8_t i = 0; i < 16; i++) {
      digitalWrite(scl_pin_, LOW);
      delayMicroseconds(20);
      digitalWrite(scl_pin_, HIGH);
      delayMicroseconds(20);
    }

    pinMode(sda_pin_, INPUT_PULLUP);
    delayMicroseconds(20);
    if (digitalRead(sda_pin_) == LOW) {
      return 2;
    }
  }

  pinMode(scl_pin_, INPUT_PULLUP);
  pinMode(sda_pin_, INPUT_PULLUP);
  return 0;
}

bool RecoveringAS5600Sensor::reinitializeBus(const char* reason,
                                             int tx_error,
                                             size_t rx_len) {
  recovery_count_++;

  wire_->end();
  delay(2);
  const int bus_state = clearBus();
  const bool begin_ok = wire_->begin(sda_pin_, scl_pin_, clock_hz_);
  wire_->setTimeOut(timeout_ms_);
  delay(2);

  uint16_t probe_raw = 0;
  int probe_tx_error = 0;
  size_t probe_rx_len = 0;
  const bool probe_ok = readRaw(probe_raw, probe_tx_error, probe_rx_len);

  Serial.printf(
      "[I2C] Recovery #%lu (%s): prev_tx=%d prev_rx=%u bus_state=%d begin=%d probe=%d probe_tx=%d probe_rx=%u raw=%u clk=%luHz timeout=%ums\n",
      (unsigned long)recovery_count_,
      reason,
      tx_error,
      (unsigned)rx_len,
      bus_state,
      begin_ok,
      probe_ok,
      probe_tx_error,
      (unsigned)probe_rx_len,
      probe_raw,
      (unsigned long)clock_hz_,
      (unsigned)timeout_ms_);

  if (probe_ok) {
    last_good_raw_ = probe_raw;
    consecutive_failures_ = 0;
  }

  return begin_ok && probe_ok;
}
