#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  enum InternalFilterMode {
    FILTER_EDGE = 0,
    FILTER_PULSE,
  };

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_filter_us(uint32_t filter) { filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { filter_mode_ = mode; }
  void set_total_pulses(uint32_t pulses);

  // Component interface
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;

 protected:
  static constexpr uint32_t INTERRUPT_ALLOCATION_FLAGS = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1;
  static constexpr size_t PULSE_BUFFER_SIZE = 32;
  static constexpr size_t BUFFER_MASK = PULSE_BUFFER_SIZE - 1;
  
  struct alignas(4) PulseBuffer {
      uint32_t timestamps[PULSE_BUFFER_SIZE];
      volatile uint32_t write_idx;
      uint32_t read_idx;
  };
  
  // Interrupt handlers
  static void IRAM_ATTR edge_intr(PulseMeterSensor *sensor);
  static void IRAM_ATTR pulse_intr(PulseMeterSensor *sensor);

  // Critical data structures - cache aligned
  alignas(32) PulseBuffer pulse_buffer_;
  alignas(4) volatile uint32_t missed_pulses_;
  
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  // Configuration
  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};
  
  // GPIO state
  uint32_t gpio_mask_;
  volatile uint32_t *gpio_input_reg_;
  ISRInternalGPIOPin isr_pin_;

  // Meter state
  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;
  bool peeked_edge_ = false;
  uint32_t total_pulses_ = 0;
  uint32_t last_processed_edge_us_ = 0;

  // Double-buffered state for ISR communication
  struct alignas(4) State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t last_rising_edge_us_ = 0;
    uint32_t count_ = 0;
  };

  alignas(32) State state_[2];
  volatile State *set_ = &state_[0];
  volatile State *get_ = &state_[1];

  // Mode-specific state
  union {
    struct {
      uint32_t last_sent_edge_us_;
    } edge;
    struct {
      uint32_t last_intr_;
      bool latched_;
      bool last_pin_val_;
    } pulse;
  };
};

}  // namespace pulse_meter
}  // namespace esphome
