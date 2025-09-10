#include "gl_r01_i2c.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gl_r01_i2c {

static const char *const TAG = "gl_r01_i2c";

static const uint8_t REG_VERSION  = 0x00;  // 2B, MSB first
static const uint8_t REG_DISTANCE = 0x02;  // 2B, MSB first (mm)
static const uint8_t REG_CTRL     = 0x10;  // write-only

static const uint8_t CMD_TRIGGER  = 0xB0;  // single-shot measure
static const uint8_t CMD_RST1     = 0x5A;  // part of restart sequence
static const uint8_t CMD_RST2     = 0xA5;  // part of restart sequence

void GLR01I2CComponent::setup() {
  uint16_t ver{0};
  if (this->read_u16_(REG_VERSION, ver)) {
    this->version_ = ver;
  } else {
    ESP_LOGW(TAG, "FW version read failed (device may still work)");
  }
}

void GLR01I2CComponent::dump_config() {
  LOG_SENSOR(TAG, "Gauselink GL-R01 (I2C) Distance", this);
  LOG_I2C_DEVICE(this);
  ESP_LOGI(TAG, "FW version: 0x%04X", this->version_);
  ESP_LOGI(TAG, "Trigger delay: %u ms, restart after %u failures",
           this->trigger_delay_ms_, this->restart_after_n_failures_);
}

void GLR01I2CComponent::update() {
  if (!this->write_u8_(REG_CTRL, CMD_TRIGGER)) {
    ESP_LOGW(TAG, "Trigger write failed");
    this->status_set_warning();
    this->publish_state(NAN);
    return;
  }
  this->set_timeout("gl_r01_read", this->trigger_delay_ms_, [this] { this->read_distance_(); });
}

void GLR01I2CComponent::read_distance_() {
  uint16_t dist{0};
  if (!this->read_u16_(REG_DISTANCE, dist)) {
    ESP_LOGW(TAG, "Distance read failed");
    this->status_set_warning();
    this->publish_state(NAN);
    if (++this->consecutive_bad_ >= this->restart_after_n_failures_) {
      ESP_LOGW(TAG, "Restarting sensor after %u consecutive failures", this->consecutive_bad_);
      (void) this->write_u8x2_(REG_CTRL, CMD_RST1, CMD_RST2);
      this->consecutive_bad_ = 0;
    }
    return;
  }

  if (dist == 0xFFFF || dist == 0x0000) {
    ESP_LOGW(TAG, "Invalid measurement: 0x%04X", dist);
    this->status_set_warning();
    this->publish_state(NAN);
    if (++this->consecutive_bad_ >= this->restart_after_n_failures_) {
      ESP_LOGW(TAG, "Restarting sensor after %u consecutive invalids", this->consecutive_bad_);
      (void) this->write_u8x2_(REG_CTRL, CMD_RST1, CMD_RST2);
      this->consecutive_bad_ = 0;
    }
    return;
  }

  this->consecutive_bad_ = 0;
  this->status_clear_warning();
  this->publish_state(static_cast<float>(dist));  // mm
  ESP_LOGV(TAG, "Distance: %u mm", dist);
}

bool GLR01I2CComponent::read_u16_(uint8_t reg, uint16_t &out) {
  uint8_t buf[2] = {0};
  if (this->read_register(reg, buf, 2) != i2c::ERROR_OK)
    return false;
  out = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
  return true;
}

bool GLR01I2CComponent::write_u8_(uint8_t reg, uint8_t value) {
  uint8_t buf[1] = {value};
  return this->write_register(reg, buf, 1) == i2c::ERROR_OK;
}

bool GLR01I2CComponent::write_u8x2_(uint8_t reg, uint8_t a, uint8_t b) {
  uint8_t buf[2] = {a, b};
  return this->write_register(reg, buf, 2) == i2c::ERROR_OK;
}

}  // namespace gl_r01_i2c
}  // namespace esphome
