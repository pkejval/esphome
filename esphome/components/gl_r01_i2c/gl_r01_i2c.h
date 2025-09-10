#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace gl_r01_i2c {

class GLR01I2CComponent : public sensor::Sensor, public i2c::I2CDevice, public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_trigger_delay_ms(uint16_t v) { this->trigger_delay_ms_ = v; }
  void set_restart_after_n_failures(uint8_t v) { this->restart_after_n_failures_ = v; }

 protected:
  void read_distance_();
  bool read_u16_(uint8_t reg, uint16_t &out);
  bool write_u8_(uint8_t reg, uint8_t value);
  bool write_u8x2_(uint8_t reg, uint8_t a, uint8_t b);

  uint16_t version_{0};
  uint16_t trigger_delay_ms_{40};
  uint8_t  consecutive_bad_{0};
  uint8_t  restart_after_n_failures_{3};
};

}  // namespace gl_r01_i2c
}  // namespace esphome
