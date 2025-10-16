#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

#include <cinttypes>
#include <memory>
#include <atomic>

#if defined(USE_ESP32)
#include <driver/pulse_cnt.h>
#include <esp_timer.h>
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
// HW PCNT – čtení & nulování oknem
struct HwPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_raw_value() override;
  ~HwPulseCounterStorage() override;

  pcnt_unit_handle_t unit{nullptr};
  pcnt_channel_handle_t channel{nullptr};
};
#endif

std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt = false);

class PulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  explicit PulseCounterSensor(bool hw_pcnt = false) : storage_(get_storage(hw_pcnt)) {}
  ~PulseCounterSensor();

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_->rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_->falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_->filter_us = filter; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  // Volitelné vyhlazení EMA (0 = vypnuto)
  void set_ema_alpha(float alpha) { ema_alpha_ = alpha; }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void update() override;
  void dump_config() override;

  // Při změně intervalu restartujeme timer (ESP32)
  void set_update_interval(uint32_t update_interval) override;

 protected:
  static void timer_callback(void *arg);  // periodické vzorkování mimo main loop

  InternalGPIOPin *pin_{nullptr};
  std::unique_ptr<PulseCounterStorageBase> storage_;
  uint64_t current_total_{0};
  sensor::Sensor *total_sensor_{nullptr};

  // EMA konfigurace (0 => vypnuto)
  float ema_alpha_{0.0f};
  float ema_state_{NAN};

#if defined(USE_ESP32)
  esp_timer_handle_t timer_handle_{nullptr};
  std::atomic<float> last_calculated_ppm_{NAN};  // poslední vypočtená (příp. EMA) hodnota
  std::atomic<bool> new_value_ready_{false};     // flag pro publish
  std::atomic<int32_t> pending_total_delta_{0};  // bezpečný přenos přírůstku "total"
  uint64_t last_tick_us_{0};                     // čas posledního vzorku (pro reálné dt)
#else
  // Fallback pro non-ESP32
  uint64_t last_time_us_{0};
#endif
};

}  // namespace pulse_counter
}  // namespace esphome
