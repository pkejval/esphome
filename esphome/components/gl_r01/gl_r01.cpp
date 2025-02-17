#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "gl_r01.h"

namespace esphome {
namespace gl_r01 {

static const char *const TAG = "gl_r01";

// Register definitions from datasheet.
static const uint8_t REG_VERSION = 0x00;
static const uint8_t REG_DISTANCE = 0x02;
static const uint8_t REG_SLAVE_ADDR = 0x05;
static const uint8_t REG_TRIGGER = 0x10;
static const uint8_t CMD_TRIGGER = 0xB0;
static const uint8_t RESTART_CMD1 = 0x5A;
static const uint8_t RESTART_CMD2 = 0xA5;

// Try reading the software version register to verify the sensor responds.
bool GLR01Component::verify_device_(uint8_t address) {
  uint16_t version;
  this->address_ = address;
  if (this->read_byte_16(REG_VERSION, &version)) {
    ESP_LOGD(TAG, "Found GL-R01 at address 0x%02X with version 0x%04X", address, version);
    return true;
  }
  return false;
}

optional<uint8_t> GLR01Component::scan_for_device() {
  for (size_t i = 0; i < NUM_VALID_ADDRESSES; i++) {
    if (verify_device_(VALID_ADDRESSES[i])) {
      return {VALID_ADDRESSES[i]};
    }
  }
  return {};
}

bool GLR01Component::detect_address_() {
  ESP_LOGI(TAG, "Auto-detecting GL-R01 address...");
  auto addr = scan_for_device();
  if (addr.has_value()) {
    this->address_ = addr.value();
    ESP_LOGI(TAG, "Successfully detected GL-R01 at address 0x%02X", addr.value());
    return true;
  }
  ESP_LOGE(TAG, "No GL-R01 device found on any valid address!");
  return false;
}

void GLR01Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up GL-R01...");
  if (this->auto_detect_) {
    if (!this->detect_address_()) {
      this->mark_failed();
      return;
    }
  } else {
    // Validate the configured I2C address.
    bool valid = false;
    for (size_t i = 0; i < NUM_VALID_ADDRESSES; i++) {
      if (VALID_ADDRESSES[i] == this->address_) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      ESP_LOGE(TAG, "Invalid I2C address 0x%02X configured!", this->address_);
      this->mark_failed();
      return;
    }
    // Verify sensor presence at the configured address.
    if (!verify_device_(this->address_)) {
      ESP_LOGE(TAG, "No GL-R01 found at configured address 0x%02X!", this->address_);
      this->mark_failed();
      return;
    }
  }
  dump_config();
}

void GLR01Component::dump_config() {
  ESP_LOGCONFIG(TAG, "GL-R01:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with sensor failed! Sensor will not be polled.");
  }
}

void GLR01Component::update() {
  if (this->is_failed())
    ESP_LOGVV(TAG, "Skipping update - sensor marked as failed!");
  return;
  if (state_ != GLR01State::IDLE) {
    ESP_LOGVV(TAG, "Previous measurement still in progress");
    return;
  }
  // Trigger a new measurement.
  if (!this->write_byte(REG_TRIGGER, CMD_TRIGGER)) {
    ESP_LOGE(TAG, "Failed to trigger measurement!");
    this->mark_failed();
    this->status_set_warning();
    return;
  }
  state_ = GLR01State::TRIGGERED;
  trigger_time_ = millis();
}

void GLR01Component::loop() {
  if (this->is_failed())
    return;

  // Wait for result after measurement was triggered
  if (state_ == GLR01State::TRIGGERED) {
    if (millis() - trigger_time_ >= MIN_READ_INTERVAL) {
      state_ = GLR01State::READY;
    }
  }
  if (state_ == GLR01State::READY) {
    uint16_t distance = 0;
    if (!this->read_byte_16(REG_DISTANCE, &distance)) {
      ESP_LOGE(TAG, "Failed to read distance value!");
      this->mark_failed();
      this->status_set_warning();
      state_ = GLR01State::IDLE;
      return;
    }
    if (distance == 0xFFFF) {
      ESP_LOGW(TAG, "Invalid measurement received!");
      this->status_set_warning();
    } else {
      ESP_LOGV(TAG, "Distance: %umm", distance);
      this->publish_state(distance);
      this->status_clear_warning();
    }
    state_ = GLR01State::IDLE;
  }
}

float GLR01Component::get_setup_priority() const { return setup_priority::DATA; }

bool GLR01Component::set_i2c_address(uint8_t address) {
  bool valid = false;
  for (size_t i = 0; i < NUM_VALID_ADDRESSES; i++) {
    if (VALID_ADDRESSES[i] == address) {
      valid = true;
      break;
    }
  }
  if (!valid) {
    ESP_LOGE(TAG, "Invalid I2C address 0x%02X", address);
    return false;
  }
  ESP_LOGI(TAG, "Changing I2C address from 0x%02X to 0x%02X", this->address_, address);
  // Write new address to sensor.
  if (!this->write_byte(REG_SLAVE_ADDR, address)) {
    ESP_LOGE(TAG, "Failed to set new I2C address!");
    return false;
  }
  delay(10);
  // Update the internal address.
  this->address_ = address;
  if (!verify_device_(address)) {
    ESP_LOGE(TAG, "Failed to verify new I2C address!");
    return false;
  }
  ESP_LOGI(TAG, "Successfully changed and verified new I2C address");
  return true;
}

bool GLR01Component::restart_sensor() {
  if (!this->write_byte(REG_TRIGGER, RESTART_CMD1)) {
    ESP_LOGE(TAG, "Failed to send restart command part 1!");
    return false;
  }
  if (!this->write_byte(REG_TRIGGER, RESTART_CMD2)) {
    ESP_LOGE(TAG, "Failed to send restart command part 2!");
    return false;
  }
  ESP_LOGI(TAG, "Restart command issued successfully. Sensor restarting...");
  return true;
}

}  // namespace gl_r01
}  // namespace esphome
