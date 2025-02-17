#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace gl_r01 {

enum class GLR01State { IDLE, TRIGGERED, READY };

class GLR01Component : public sensor::Sensor, public i2c::I2CDevice, public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override;
  void set_auto_detect(bool auto_detect) { auto_detect_ = auto_detect; }
  bool set_i2c_address(uint8_t address);
  optional<uint8_t> scan_for_device();
  bool restart_sensor();

 protected:
  GLR01State state_{GLR01State::IDLE};
  uint32_t trigger_time_{0};
  static const uint32_t MIN_READ_INTERVAL = 40;  // minimum milliseconds from datasheet
  bool auto_detect_{false};

  bool detect_address_();
  bool verify_device_(uint8_t address);

  // List of valid I2C addresses.
  static constexpr uint8_t VALID_ADDRESSES[] = {0xE8, 0xD0, 0xD2, 0xD4, 0xD6, 0xD8, 0xDA, 0xDC, 0xDE, 0xE0,
                                                0xE2, 0xE4, 0xE6, 0xEA, 0xEC, 0xEE, 0xF8, 0xFA, 0xFC, 0xFE};
  static constexpr size_t NUM_VALID_ADDRESSES = sizeof(VALID_ADDRESSES) / sizeof(VALID_ADDRESSES[0]);
};

class ChangeAddressAction : public Action<> {
 public:
  explicit ChangeAddressAction(GLR01Component *parent) : parent_(parent) {}
  void set_address(uint8_t address) { address_ = address; }
  void play() override {
    if (!this->parent_->set_i2c_address(this->address_)) {
      ESP_LOGE("gl_r01", "Failed to change I2C address!");
    }
  }

 protected:
  GLR01Component *parent_;
  uint8_t address_;
};

class RestartSensorAction : public Action<> {
 public:
  explicit RestartSensorAction(GLR01Component *parent) : parent_(parent) {}
  void play() override {
    if (!this->parent_->restart_sensor()) {
      ESP_LOGE("gl_r01", "Failed to restart sensor!");
    }
  }

 protected:
  GLR01Component *parent_;
};

}  // namespace gl_r01
}  // namespace esphome
