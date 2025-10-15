#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

#include <cinttypes>
#include <memory>

#if defined(USE_ESP32)
#include <driver/pulse_cnt.h>
#define HAS_PCNT
#endif

namespace esphome {
namespace pulse_counter {

enum PulseCounterCountMode {
  PULSE_COUNTER_DISABLE = 0,
  PULSE_COUNTER_INCREMENT,
  PULSE_COUNTER_DECREMENT,
};

using pulse_counter_t = int32_t;

struct PulseCounterStorageBase {
  virtual ~PulseCounterStorageBase() = default;
  virtual bool pulse_counter_setup(InternalGPIOPin *pin) = 0;
  virtual pulse_counter_t read_raw_value() = 0;

  InternalGPIOPin *pin{nullptr};
  PulseCounterCountMode rising_edge_mode{PULSE_COUNTER_INCREMENT};
  PulseCounterCountMode falling_edge_mode{PULSE_COUNTER_DISABLE};
  uint32_t filter_us{0};
  pulse_counter_t last_value{0};
};

struct BasicPulseCounterStorage : public PulseCounterStorageBase {
  static void gpio_intr(BasicPulseCounterStorage *arg);

  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_raw_value() override;
  ~BasicPulseCounterStorage() override;

  volatile pulse_counter_t counter{0};
  volatile uint32_t last_pulse{0};

  ISRInternalGPIOPin isr_pin;
};

#ifdef HAS_PCNT
struct HwPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_raw_value() override;
  ~HwPulseCounterStorage() override;

  // PCNT v2 driver handles
  pcnt_unit_handle_t unit{nullptr};
  pcnt_channel_handle_t channel{nullptr};

  // Watch-pointy pro limity signed 16b
  const int high_watch_{32767};
  const int low_watch_{-32768};

  // Počet průchodů limitem (rozšíření šířky)
  volatile int32_t wraps_{0};
  volatile bool cb_registered_{false};

  // Poslední extended hodnota (pro delta)
  int64_t last_ext_{0};
  bool first_read_{true};

  // ISR callback pro watch-pointy
  static bool IRAM_ATTR on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
};
#endif

std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt = false);

class PulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  explicit PulseCounterSensor(bool hw_pcnt = false) : storage_(get_storage(hw_pcnt)) {}

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_->rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_->falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_->filter_us = filter; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void update() override;
  void dump_config() override;

 protected:
  InternalGPIOPin *pin_{nullptr};
  std::unique_ptr<PulseCounterStorageBase> storage_;
  uint64_t last_time_us_{0};
  uint64_t current_total_{0};
  sensor::Sensor *total_sensor_{nullptr};
};

}  // namespace pulse_counter
}  // namespace esphome
