#include "pulse_meter_sensor.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_pin(InternalGPIOPin *pin) {
  gpio_num_ = static_cast<gpio_num_t>(pin->get_pin());
}

void PulseMeterSensor::configure(uint32_t filter_us, uint32_t timeout_us, bool use_pulse_mode) {
  filter_us_ = filter_us;
  timeout_us_ = timeout_us;
  pulse_mode_ = use_pulse_mode;
}

void PulseMeterSensor::setup() {
  // Configure hardware timer
  timer_config_t timer_cfg = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = 80  // 1MHz timer (80MHz / 80)
  };
  timer_init(timer_group_, timer_idx_, &timer_cfg);
  timer_set_counter_value(timer_group_, timer_idx_, 0);
  timer_set_alarm_value(timer_group_, timer_idx_, filter_us_);
  timer_enable_intr(timer_group_, timer_idx_);
  timer_isr_register(timer_group_, timer_idx_, hw_timer_isr, this, ESP_INTR_FLAG_IRAM, &timer_isr_handle_);

  // Configure GPIO with hardware filtering
  gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << gpio_num_,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_ANYEDGE};
  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(gpio_num_, gpio_isr, this);

  // Create high-priority processing task
  xTaskCreatePinnedToCore(
      [](void *arg) {
        static_cast<PulseMeterSensor *>(arg)->process_edges();
      }, "pulse_proc", 2048, this, configMAX_PRIORITIES - 1, &processing_task_, xPortGetCoreID());
  
  timer_start(timer_group_, timer_idx_);
}

void IRAM_ATTR PulseMeterSensor::hw_timer_isr(void *arg) {
  PulseMeterSensor *sensor = static_cast<PulseMeterSensor *>(arg);
  uint64_t cnt;
  timer_get_counter_value(sensor->timer_group_, sensor->timer_idx_, &cnt);
  timer_set_alarm(sensor->timer_group_, sensor->timer_idx_, cnt + sensor->filter_us_);
  timer_group_clr_intr_status_in_isr(sensor->timer_group_, sensor->timer_idx_);
  timer_group_enable_alarm_in_isr(sensor->timer_group_, sensor->timer_idx_);
}

void IRAM_ATTR PulseMeterSensor::gpio_isr(void *arg) {
  PulseMeterSensor *sensor = static_cast<PulseMeterSensor *>(arg);
  const uint32_t now = esp_timer_get_time();
  const bool state = gpio_get_level(sensor->gpio_num_);
  
  if(sensor->pulse_mode_) {
    static uint32_t last_rising = 0;
    if(state && (now - last_rising) > sensor->filter_us_) {
      sensor->atomic_state_.last_edge_us.store(now, std::memory_order_relaxed);
      sensor->atomic_state_.edge_count.fetch_add(1, std::memory_order_relaxed);
      last_rising = now;
    }
  } else {
    sensor->atomic_state_.last_edge_us.store(now, std::memory_order_relaxed);
    sensor->atomic_state_.edge_count.fetch_add(1, std::memory_order_relaxed);
  }
}

void PulseMeterSensor::process_edges() {
  uint32_t last_count = 0;
  uint32_t last_edge = 0;
  
  while(true) {
    const uint32_t current_count = atomic_state_.edge_count.load(std::memory_order_acquire);
    const uint32_t current_edge = atomic_state_.last_edge_us.load(std::memory_order_relaxed);
    
    if(current_count != last_count) {
      const uint32_t delta_us = current_edge - last_edge;
      const float rate = (current_count - last_count) * 1e6f * 60.0f / delta_us;
      publish_state(rate);
      last_count = current_count;
      last_edge = current_edge;
    } else if((esp_timer_get_time() - current_edge) > timeout_us_) {
      publish_state(0.0f);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void PulseMeterSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Pulse Meter:");
  ESP_LOGCONFIG(TAG, "  GPIO: %d", gpio_num_);
  ESP_LOGCONFIG(TAG, "  Filter: %uus", filter_us_);
  ESP_LOGCONFIG(TAG, "  Timeout: %us", timeout_us_ / 1000000);
}
} // namespace pulse_meter
} // namespace esphome
