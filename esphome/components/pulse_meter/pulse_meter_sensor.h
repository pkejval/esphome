#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <atomic>

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
public:
  void set_pin(InternalGPIOPin *pin);
  void configure(uint32_t filter_us, uint32_t timeout_us, bool use_pulse_mode);

  void setup() override;
  void dump_config() override;

private:
  static void IRAM_ATTR hw_timer_isr(void *arg);
  static void IRAM_ATTR gpio_isr(void *arg);
  void process_edges();

  gpio_num_t gpio_num_;
  timer_group_t timer_group_ = TIMER_GROUP_0;
  timer_idx_t timer_idx_ = TIMER_0;
  intr_handle_t timer_isr_handle_;
  
  struct alignas(64) {
    std::atomic<uint32_t> edge_count{0};
    std::atomic<uint32_t> last_edge_us{0};
    std::atomic<uint32_t> pulse_width_us{0};
  } atomic_state_;

  uint32_t filter_us_;
  uint32_t timeout_us_;
  bool pulse_mode_;
  TaskHandle_t processing_task_;
};

} // namespace pulse_meter
} // namespace esphome
