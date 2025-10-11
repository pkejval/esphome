#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

#include <driver/pcnt.h>
#include <driver/gpio.h>
#include <esp_timer.h>

namespace esphome {
namespace hw_pulse_meter {

enum class CountMode : uint8_t {
  RISING = 0,
  FALLING = 1,
  BOTH   = 2,
};

class HWPulseMeter : public sensor::Sensor, public Component {
 public:
  HWPulseMeter() = default;

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_count_mode(CountMode m) { count_mode_ = m; }
  void set_glitch_filter_us(uint32_t us) { glitch_filter_us_ = us; }
  void set_min_interval_us(uint32_t us) { min_interval_us_ = us; }
  void set_pulses_per_revolution(uint32_t ppr) {
    if (ppr == 0) ppr = 1;
    this->pulses_per_revolution_ = ppr;
    ESP_LOGD("hw_pulse_meter", "Pulses per revolution set to %u", ppr);
  }
  uint32_t get_pulses_per_revolution() const { return this->pulses_per_revolution_; }

  // Podsenzory
  void set_publish_total(bool v) { publish_total_ = v; }
  void set_publish_pps(bool v) { publish_pps_ = v; }
  void set_total_sensor(sensor::Sensor *s) { total_sensor_ = s; }
  void set_pps_sensor(sensor::Sensor *s) { pps_sensor_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  static void IRAM_ATTR gpio_isr_trampoline(void *arg) {
    reinterpret_cast<HWPulseMeter *>(arg)->on_edge_isr_();
  }
  inline void IRAM_ATTR on_edge_isr_() {
    const uint64_t now = esp_timer_get_time();  // us
    if (min_interval_us_ > 0 && (now - last_edge_us_) < min_interval_us_) return;
    last_edge_us_ = now;
    edge_flag_ = true;
  }

  bool init_pcnt_();
  void configure_pcnt_glitch_filter_();
  bool read_pcnt_total_(int32_t &out);

  InternalGPIOPin *pin_{nullptr};
  CountMode count_mode_{CountMode::RISING};
  uint32_t glitch_filter_us_{0};
  uint32_t min_interval_us_{0};
  uint32_t pulses_per_revolution_{1};  // nově přidané

  int pcnt_unit_{-1};
  pcnt_channel_t pcnt_channel_{PCNT_CHANNEL_0};
  volatile bool edge_flag_{false};
  uint64_t last_edge_us_{0};
  uint64_t last_pub_us_{0};
  int32_t last_pcnt_total_{0};
  int32_t last_published_total_{0};  // poslední stav, kdy proběhl publish

  bool publish_total_{false}, publish_pps_{false};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *pps_sensor_{nullptr};
};

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif
