// pulse_meter_sensor.h
#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"
#include "esp_timer.h"
#include "freertos/timers.h"

namespace esphome {
namespace pulse_meter {

// Forward declaration for mutex
class PulseMeterSensor;

struct __attribute__((packed)) ESP32State {
  uint32_t last_edge_time;
  int16_t pulse_count;
  uint8_t pcnt_unit;
  uint8_t initialized : 1;
  uint8_t reserved : 7;  // For future use
};

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  PulseMeterSensor();
  ~PulseMeterSensor();

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_filter_us(uint32_t filter) { filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { total_sensor_ = sensor; }
  void set_total_pulses(uint32_t pulses);

  // Override Component methods
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;

 protected:
  // Hardware-specific methods
  bool setup_pcnt();
  void cleanup_pcnt();
  static void IRAM_ATTR pcnt_intr_handler(void *arg);
  
  // ESP32 hardware resources
  InternalGPIOPin *pin_{nullptr};
  hw_timer_t *timer_{nullptr};
  portMUX_TYPE timer_mux_ = portMUX_INITIALIZER_UNLOCKED;
  
  // Configuration
  uint32_t filter_us_{0};
  uint32_t timeout_us_{1000000UL * 60UL * 5UL};  // 5 minutes default
  sensor::Sensor *total_sensor_{nullptr};
  
  // State tracking
  ESP32State state_{};
  volatile uint32_t total_pulses_{0};
  
  // Static resource management
  static PulseMeterSensor *pcnt_unit_mutex_[PCNT_UNIT_MAX];
  
  // Internal methods
  void IRAM_ATTR handle_interrupt();
  void update_total_pulses(int16_t count);
  void calculate_frequency();
  
  // Configuration methods
  bool configure_pcnt();
  bool configure_timer();
  
  // Helper methods
  static uint32_t get_time_us() { return esp_timer_get_time(); }
  static uint32_t time_diff(uint32_t newer, uint32_t older) {
    return (newer >= older) ? (newer - older) : (0xFFFFFFFF - older + newer + 1);
  }
  
  // Friend declaration for test access
  friend class TestPulseMeterSensor;
};

} // namespace pulse_meter
} // namespace esphome
