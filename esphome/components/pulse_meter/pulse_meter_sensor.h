#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>
#include <stdint.h>

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#if __has_include("driver/gpio_filter.h")
#include "driver/gpio_filter.h"
#endif

#ifndef LIKELY
#define LIKELY(x) (__builtin_expect(!!(x), 1))
#endif
#ifndef UNLIKELY
#define UNLIKELY(x) (__builtin_expect(!!(x), 0))
#endif

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  enum InternalFilterMode {
    FILTER_EDGE = 0,
    FILTER_PULSE,
  };

  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }

  void set_filter_us(uint32_t filter) {
    this->filter_us_ = filter;
    this->update_hysteresis_defaults_();
  }

  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { this->filter_mode_ = mode; }

  void set_pulse_hysteresis_us(uint32_t min_low_us, uint32_t min_high_us) {
    this->min_low_us_ = min_low_us;
    this->min_high_us_ = min_high_us;
  }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  static void IRAM_ATTR edge_intr(PulseMeterSensor *sensor);
  static void IRAM_ATTR pulse_intr(PulseMeterSensor *sensor);

  void update_hysteresis_defaults_() {
    this->min_low_us_ = (this->filter_us_ * 4U) / 5U;   // 0.8x
    this->min_high_us_ = (this->filter_us_ * 6U) / 5U;  // 1.2x
  }

  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};

  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;
  bool peeked_edge_ = false;
  uint32_t total_pulses_ = 0;
  uint32_t last_processed_edge_us_ = 0;

  struct State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t last_rising_edge_us_ = 0;
    uint32_t count_ = 0;
  } __attribute__((packed, aligned(4)));

  State state_[2];
  volatile State *set_ = state_;
  volatile State *get_ = state_ + 1;

  ISRInternalGPIOPin isr_pin_;

  struct EdgeState {
    uint32_t last_sent_edge_us_ = 0;
  };
  EdgeState edge_state_{};

  struct PulseState {
    uint32_t last_intr_ = 0;
    bool latched_ = false;
    bool last_pin_val_ = false;
  };
  PulseState pulse_state_{};

  volatile bool new_event_ = false;
  uint32_t next_timeout_check_us_ = 0;

  uint32_t min_low_us_ = 0;
  uint32_t min_high_us_ = 0;

#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
  gpio_glitch_filter_handle_t glitch_filter_{nullptr};
#endif
};

}  // namespace pulse_meter
}  // namespace esphome
