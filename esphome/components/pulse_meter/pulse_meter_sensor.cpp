#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <inttypes.h>
#include <utility>

#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
#include "driver/gpio.h"
#include "esp_idf_version.h"
#endif

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  this->total_pulses_ = pulses;
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();

  this->last_pin_val_ = this->pin_->digital_read();
  this->last_processed_edge_us_ = micros();

#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
  // HW glitch filtr – pokud jej SoC/IDF podporuje a máme nenulový filtr
  if (this->filter_mode_ == FILTER_EDGE && this->filter_us_ > 0) {
    const int raw_pin = this->pin_->get_pin();  // zjednodušeno dle návrhu
#if ESP_IDF_VERSION_MAJOR >= 5
    gpio_glitch_filter_handle_t h = nullptr;
    gpio_glitch_filter_config_t cfg = {};
    cfg.gpio_num = static_cast<gpio_num_t>(raw_pin);
    cfg.clk_src = GPIO_GLITCH_FILTER_CLK_SRC_DEFAULT;
    cfg.window_thres_ns = static_cast<uint32_t>(this->filter_us_) * 1000U;
    cfg.window_width_ns = 0;
    if (gpio_new_glitch_filter(&cfg, &h) == ESP_OK) {
      if (gpio_glitch_filter_enable(h) == ESP_OK) {
        this->glitch_filter_handle_ = h;
        this->hw_filter_active_ = true;  // HW filtr aktivní → SW filtr v ISR se vypne
      }
    }
#endif
  }
#endif

  if (this->filter_mode_ == FILTER_EDGE) {
    this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
  } else {
    this->pulse_state_.latched_ = this->last_pin_val_;
    this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }

  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::loop() {
  {
    InterruptLock lock;

    // Optimalizovaný swap: nejdřív přepnout, pak vynulovat nový write buffer
    std::swap(this->set_, this->get_);
    this->set_->count_ = 0;
  }

  const uint32_t now = micros();

  if (LIKELY(this->get_->count_ > 0)) {
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += this->get_->count_;
      this->total_sensor_->publish_state(this->total_pulses_);
    }

    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;

      case MeterState::RUNNING: {
        const uint32_t delta_us = us_since(this->get_->last_detected_edge_us_, this->last_processed_edge_us_);
        const float pulse_width_us = delta_us / static_cast<float>(this->get_->count_);
        ESP_LOGV(TAG, "New pulse, delta: %" PRIu32 " µs, count: %" PRIu32 ", width: %.5f µs", delta_us,
                 this->get_->count_, pulse_width_us);
        this->publish_state((60.0f * 1000000.0f) / pulse_width_us);
      } break;
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else {
    const uint32_t idle_us = us_since(now, this->last_processed_edge_us_);
    if (UNLIKELY(this->meter_state_ == MeterState::INITIAL || this->meter_state_ == MeterState::RUNNING)) {
      if (idle_us > this->timeout_us_) {
        this->meter_state_ = MeterState::TIMED_OUT;
        ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min", idle_us / 1000000U);
        this->publish_state(0.0f);
      }
    }
  }
}

float PulseMeterSensor::get_setup_priority() const { return setup_priority::DATA; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %" PRIu32 " µs apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %" PRIu32 " µs", this->filter_us_);
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %" PRIu32 "s",
                this->timeout_us_ / 1000000U);
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  sensor->record_edge_(now);
  sensor->last_pin_val_ = true;  // rising -> high
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();

  auto &ps = sensor->pulse_state_;
  auto &set = *sensor->set_;

  const bool enough = us_since(now, ps.last_intr_) >= sensor->filter_us_;

  if (enough && ps.latched_ && !sensor->last_pin_val_) {
    ps.latched_ = false;
  } else if (enough && !ps.latched_ && sensor->last_pin_val_) {
    ps.latched_ = true;
    set.last_detected_edge_us_ = ps.last_intr_;
    set.count_++;
  }

  set.last_rising_edge_us_ = (!ps.latched_ && pin_val) ? now : set.last_detected_edge_us_;

  ps.last_intr_ = now;
  sensor->last_pin_val_ = pin_val;
}

}  // namespace pulse_meter
}  // namespace esphome
