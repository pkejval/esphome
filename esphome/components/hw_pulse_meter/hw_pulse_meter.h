#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

#include <cstdint>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace esphome {
namespace hw_pulse_meter {

enum CountMode : uint8_t {
  RISING = 0,
  FALLING = 1,
  BOTH = 2,
};

class HWPulseMeter : public sensor::Sensor, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_count_mode(CountMode m) { count_mode_ = m; }
  void set_internal_filter_us(uint32_t us_req, uint32_t us_applied) {
    internal_filter_us_requested_ = us_req;
    internal_filter_us_applied_ = us_applied;
  }
  void set_pulses_per_revolution(uint32_t ppr) { pulses_per_revolution_ = ppr == 0 ? 1u : ppr; }
  void set_idle_timeout_us(uint32_t us) { idle_timeout_us_ = us; }

  void set_publish_total(bool v) { publish_total_ = v; }
  void set_publish_pps(bool v) { publish_pps_ = v; }
  void set_publish_revolutions(bool v) { publish_revolutions_ = v; }
  void set_total_sensor(sensor::Sensor *s) { total_sensor_ = s; }
  void set_pps_sensor(sensor::Sensor *s) { pps_sensor_ = s; }
  void set_revolutions_sensor(sensor::Sensor *s) { revolutions_sensor_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  ~HWPulseMeter();

 protected:
  bool init_pcnt_();
  void apply_glitch_filter_();
  static pcnt_channel_edge_action_t map_edge_rising_(CountMode m);
  static pcnt_channel_edge_action_t map_edge_falling_(CountMode m);

  static bool IRAM_ATTR on_reach_isr_(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_data);

  InternalGPIOPin *pin_{nullptr};
  CountMode count_mode_{RISING};
  uint32_t internal_filter_us_requested_{12};
  uint32_t internal_filter_us_applied_{12};
  uint32_t pulses_per_revolution_{1};
  uint32_t idle_timeout_us_{0};

  bool publish_total_{false}, publish_pps_{false}, publish_revolutions_{false};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *pps_sensor_{nullptr};
  sensor::Sensor *revolutions_sensor_{nullptr};

  pcnt_unit_handle_t unit_{nullptr};
  pcnt_channel_handle_t channel_{nullptr};

  QueueHandle_t evt_queue_{nullptr};  // fronta timestampů (µs) pro celé otáčky

  uint64_t last_rev_time_us_{0};
  uint64_t last_event_time_us_{0};
  bool idle_zero_sent_{false};

  uint64_t current_total_pulses_{0};
  uint64_t current_total_revs_{0};
};

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
