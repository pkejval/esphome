#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

#include <stdint.h>

namespace esphome {
namespace hw_pulse_meter {

// Unscoped enum (ESPHome codegen z YAML očekává RISING/FALLING/BOTH v tomto namespace)
enum CountMode : uint8_t {
  RISING = 0,
  FALLING = 1,
  BOTH   = 2,
};

// LPM = hlavní senzor; TOTAL a PPS jsou podsenzory
class HWPulseMeter : public sensor::Sensor, public Component {
 public:
  HWPulseMeter() = default;

  // YAML konfigurace
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_count_mode(CountMode m) { count_mode_ = m; }
  void set_glitch_filter_us(uint32_t us) { glitch_filter_us_ = us; }  // µs (v .cpp převádím na ns)
  void set_min_interval_us(uint32_t us) { min_interval_us_ = us; }    // soft debounce (µs)
  void set_pulses_per_revolution(uint32_t ppr) { pulses_per_revolution_ = (ppr == 0) ? 1u : ppr; }

  // Podsenzory
  void set_publish_total(bool v) { publish_total_ = v; }
  void set_publish_pps(bool v) { publish_pps_ = v; }
  void set_total_sensor(sensor::Sensor *s) { total_sensor_ = s; }
  void set_pps_sensor(sensor::Sensor *s) { pps_sensor_ = s; }

  // Runtime API
  uint32_t get_pulses_per_revolution() const { return pulses_per_revolution_; }

  // Component API
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  // ISR na hranu GPIO – jen probuzení loopu (debounce soft)
  static void IRAM_ATTR gpio_isr_trampoline(void *arg) {
    reinterpret_cast<HWPulseMeter *>(arg)->on_edge_isr_();
  }
  inline void IRAM_ATTR on_edge_isr_() {
    const uint64_t now = esp_timer_get_time();  // us
    if (min_interval_us_ > 0 && (now - last_edge_us_) < min_interval_us_) return;
    last_edge_us_ = now;
    edge_flag_ = true;
  }

  // Pulse Counter (nový driver) – pomocné (definováno v .cpp, kde includuju pulse_cnt.h)
  bool init_pcnt_();
  void configure_pcnt_glitch_filter_();
  bool read_pcnt_total_(int32_t &out);

  // Konfigurace
  InternalGPIOPin *pin_{nullptr};
  CountMode count_mode_{RISING};
  uint32_t glitch_filter_us_{0};  // HW filtr (µs)
  uint32_t min_interval_us_{0};   // soft debounce (µs)
  uint32_t pulses_per_revolution_{1};

  // Stav
  void *unit_{nullptr};    // pcnt_unit_handle_t (opaque, kvůli konfliktu s legacy pcnt.h)
  void *channel_{nullptr}; // pcnt_channel_handle_t (opaque)
  volatile bool edge_flag_{false};
  uint64_t last_edge_us_{0};
  uint64_t last_pub_us_{0};
  int32_t last_pcnt_total_{0};
  int32_t last_published_total_{0};

  // Podsenzory
  bool publish_total_{false}, publish_pps_{false};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *pps_sensor_{nullptr};
};

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
