#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

#include <stdint.h>
#include <esp_timer.h>

namespace esphome {
namespace hw_pulse_meter {

enum CountMode : uint8_t {
  RISING = 0,
  FALLING = 1,
  BOTH   = 2,
};

class HWPulseMeter : public sensor::Sensor, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_count_mode(CountMode m) { count_mode_ = m; }
  void set_internal_filter_us(uint32_t us) { internal_filter_us_ = us; }
  void set_pulses_per_revolution(uint32_t ppr) { pulses_per_revolution_ = ppr == 0 ? 1u : ppr; }

  void set_publish_total(bool v) { publish_total_ = v; }
  void set_publish_pps(bool v) { publish_pps_ = v; }
  void set_publish_revolutions(bool v) { publish_revolutions_ = v; }
  void set_total_sensor(sensor::Sensor *s) { total_sensor_ = s; }
  void set_pps_sensor(sensor::Sensor *s) { pps_sensor_ = s; }
  void set_revolutions_sensor(sensor::Sensor *s) { revolutions_sensor_ = s; }
  void set_idle_timeout_us(uint32_t us) { idle_timeout_us_ = us; }

  uint32_t get_pulses_per_revolution() const { return pulses_per_revolution_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  bool init_pcnt_();
  void configure_pcnt_internal_filter_();
  bool read_pcnt_total_(int32_t &out);

  InternalGPIOPin *pin_{nullptr};
  CountMode count_mode_{RISING};
  uint32_t internal_filter_us_{13};
  uint32_t pulses_per_revolution_{1};

  void *unit_{nullptr};
  void *channel_{nullptr};

  uint64_t last_change_us_{0};
  uint64_t last_pub_us_{0};
  int32_t last_pcnt_total_raw_{0};
  uint64_t cumulative_total_{0};
  uint64_t last_published_total_{0};
  uint64_t last_revolutions_pub_{0};
  uint32_t idle_timeout_us_{0};
  bool idle_zero_published_{false};
  bool publish_total_{false}, publish_pps_{false}, publish_revolutions_{false};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *pps_sensor_{nullptr};
  sensor::Sensor *revolutions_sensor_{nullptr};
};

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
