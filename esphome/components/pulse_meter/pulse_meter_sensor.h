#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <atomic>

#ifdef USE_ESP32
#include "driver/pcnt.h"
#endif

namespace esphome {
namespace pulse_meter {

#define PULSE_METER_FIXED_POINT_SCALE 1000  // Scale factor for fixed-point math

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  enum InternalFilterMode {
    FILTER_EDGE = 0,
    FILTER_PULSE,
    #ifdef USE_ESP32
    FILTER_PCNT,  // Use ESP32 hardware pulse counter
    #endif
  };

  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_filter_us(uint32_t filter) { this->filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { this->filter_mode_ = mode; }
  void set_total_pulses(uint32_t pulses);
  void set_core_pinning(bool enable) { this->use_core_pinning_ = enable; }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  // Attribute ensures proper memory alignment for ISR accessed data
  struct __attribute__((aligned(4))) State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t last_rising_edge_us_ = 0;
    uint32_t count_ = 0;
  };
  
  struct __attribute__((aligned(4))) EdgeState {
    uint32_t last_sent_edge_us_ = 0;
  };
  
  struct __attribute__((aligned(4))) PulseState {
    uint32_t last_intr_ = 0;
    bool latched_ = false;
    bool last_pin_val_ = false;
  };

  // Static ISR handlers with hot attribute for compiler optimization
  static void IRAM_ATTR edge_intr(PulseMeterSensor *sensor) __attribute__((hot));
  static void IRAM_ATTR pulse_intr(PulseMeterSensor *sensor) __attribute__((hot));
  
  #ifdef USE_ESP32
  void setup_pcnt();
  static void IRAM_ATTR pcnt_intr(void *arg);
  #endif

  esp_err_t atomic_update_pulses(uint32_t new_pulses);
  
  // Main processing task for decoupling from network/other tasks
  #ifdef USE_ESP32
  static void processing_task(void *arg);
  TaskHandle_t task_handle_ = nullptr;
  bool use_core_pinning_ = true;  // Pin to a specific core by default
  #endif

  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};

  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;
  bool peeked_edge_ = false;

  std::atomic<uint32_t> total_pulses_{0};
  uint32_t last_processed_edge_us_ = 0;

  State state_[2] __attribute__((aligned(4)));
  volatile State *set_ = state_;
  volatile State *get_ = state_ + 1;
  
  ISRInternalGPIOPin isr_pin_;
  EdgeState edge_state_{};
  PulseState pulse_state_{};
  
  #ifdef USE_ESP32
  pcnt_unit_t pcnt_unit_ = PCNT_UNIT_0;
  bool using_pcnt_ = false;
  SemaphoreHandle_t pulse_mutex_ = nullptr;
  #endif
};

} // namespace pulse_meter
} // namespace esphome
