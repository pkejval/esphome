#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>

#ifndef LIKELY
#define LIKELY(x) (__builtin_expect(!!(x), 1))
#endif
#ifndef UNLIKELY
#define UNLIKELY(x) (__builtin_expect(!!(x), 0))
#endif

// ESP-IDF capability flags (PCNT / GPIO glitch filter)
#if __has_include("soc/soc_caps.h")
#include "soc/soc_caps.h"
#else
// starší IDF fallback
#define SOC_PCNT_SUPPORTED 1
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
  void set_filter_us(uint32_t filter) { this->filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { this->filter_mode_ = mode; }
  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  static inline uint32_t us_since(uint32_t now, uint32_t then) { return static_cast<uint32_t>(now - then); }

  // --- Fast ISR backend (fallback when PCNT not used) ---
  static void IRAM_ATTR edge_intr(PulseMeterSensor *sensor);
  static void IRAM_ATTR pulse_intr(PulseMeterSensor *sensor);

  inline void IRAM_ATTR record_edge_(uint32_t now) {
    if (UNLIKELY(!this->hw_filter_active_ && (us_since(now, this->edge_state_.last_sent_edge_us_) < this->filter_us_)))
      return;
    this->edge_state_.last_sent_edge_us_ = now;
    auto &set = *this->set_;
    set.last_detected_edge_us_ = now;
    set.last_rising_edge_us_ = now;
    set.count_++;
  }

  // --- Common config/state ---
  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};

  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;

  uint32_t total_pulses_ = 0;
  uint32_t last_processed_edge_us_ = 0;

  struct State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t last_rising_edge_us_ = 0;
    uint32_t count_ = 0;
  };
  State state_[2];
  volatile State *set_ = state_;
  volatile State *get_ = state_ + 1;

  ISRInternalGPIOPin isr_pin_;
  bool last_pin_val_ = false;

  struct EdgeState {
    uint32_t last_sent_edge_us_ = 0;
  };
  EdgeState edge_state_{};

  struct PulseState {
    uint32_t last_intr_ = 0;
    bool latched_ = false;
  };
  PulseState pulse_state_{};

#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
  void *glitch_filter_handle_ = nullptr;
#endif
  bool hw_filter_active_ = false;

  // --- PCNT backend (EDGE mode) ---
#if defined(SOC_PCNT_SUPPORTED)
  bool pcnt_active_ = false;
  int pcnt_unit_ = -1;               // přidělená PCNT jednotka
  int16_t pcnt_last_count_ = 0;      // poslední hodnoty 16b čítače
  uint32_t last_count_time_us_ = 0;  // čas posledního odečtu pro rychlost/timeout
  uint32_t last_pulse_time_us_ = 0;  // čas posledního zjištěného přírůstku

  bool setup_pcnt_edge_mode_();
  void loop_pcnt_edge_mode_();
  static inline uint16_t pcnt_filter_ticks_(uint32_t us) {
    // APB 80 MHz → 1 tick = 12.5 ns, max 1023 → ~12.8 µs
    uint64_t t = (uint64_t) us * 80ULL;
    if (t > 1023ULL)
      t = 1023ULL;
    return (uint16_t) t;
  }
#endif
};

}  // namespace pulse_meter
}  // namespace esphome
