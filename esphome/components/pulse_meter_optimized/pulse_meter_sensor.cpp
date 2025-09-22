#include "pulse_meter_optimized_sensor.h"
#include "esphome/core/log.h"
#include <inttypes.h>

namespace esphome {
namespace pulse_meter_optimized {

static const char *const TAG = "pulse_meter_optimized";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  atomic_update_pulses(pulses);

  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_.load(std::memory_order_acquire));
  }
}

void PulseMeterSensor::atomic_update_pulses(uint32_t new_pulses) {
  uint32_t expected = total_pulses_.load(std::memory_order_relaxed);
  while (!total_pulses_.compare_exchange_weak(expected, new_pulses,
                                              std::memory_order_release,
                                              std::memory_order_relaxed)) {
  }
}

void PulseMeterSensor::atomic_increment_pulses(uint32_t increment) {
  total_pulses_.fetch_add(increment, std::memory_order_acq_rel);
}

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();
  esp_task_wdt_add(nullptr);
  this->last_processed_edge_us_ = (uint32_t)esp_timer_get_time();

  if (this->filter_mode_ == FILTER_EDGE) {
    this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
  } else if (this->filter_mode_ == FILTER_PULSE) {
    this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
    this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
    this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = (uint32_t)esp_timer_get_time();

  this->get_->count_ = 0;
  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

  if (this->peeked_edge_ && this->get_->count_ > 0) {
    this->peeked_edge_ = false;
    this->get_->count_ = this->get_->count_ - 1;
  }

  if ((this->get_->last_rising_edge_us_ != this->get_->last_detected_edge_us_) &&
      (now - this->get_->last_rising_edge_us_ >= this->filter_us_)) {
    this->peeked_edge_ = true;
    this->get_->last_detected_edge_us_ = this->get_->last_rising_edge_us_;
    this->get_->count_ = this->get_->count_ + 1;
  }

  if (this->get_->count_ > 0) {
    if (this->total_sensor_ != nullptr) {
      this->atomic_increment_pulses(this->get_->count_);
      uint32_t total = this->total_pulses_.load(std::memory_order_acquire);
      this->total_sensor_->publish_state(total);
    }
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;
      case MeterState::RUNNING: {
        uint32_t delta_us = this->get_->last_detected_edge_us_ - this->last_processed_edge_us_;
        float pulse_width_us = delta_us / float(this->get_->count_);
        ESP_LOGV(TAG, "New pulse, delta: %" PRIu32 " µs, count: %" PRIu32 ", width: %.5f µs",
                 delta_us, this->get_->count_, pulse_width_us);
        float pulses_per_minute = (60.0f * 1000000.0f) / pulse_width_us;
        this->publish_state(pulses_per_minute);
      } break;
    }
    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else {
    uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::RUNNING:
        if (time_since_valid_edge_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %" PRIu32 " s, assuming 0 pulses/min", time_since_valid_edge_us / 1000000);
          this->publish_state(0.0f);
        }
        break;
      default:
        break;
    }
  }
  esp_task_wdt_reset();
}

float PulseMeterSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %" PRIu32 " µs apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %" PRIu32 " µs", this->filter_us_);
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %" PRIu32 " s", this->timeout_us_ / 1000000);
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  auto &state = sensor->edge_state_;
  auto &set = *sensor->set_;
  if ((now - state.last_sent_edge_us_) >= sensor->filter_us_) {
    state.last_sent_edge_us_ = now;
    set.last_detected_edge_us_ = now;
    set.last_rising_edge_us_ = now;
    set.count_ = set.count_ + 1;
  }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &state = sensor->pulse_state_;
  auto &set = *sensor->set_;
  bool length = (now - state.last_intr_) >= sensor->filter_us_;
  if (length && state.latched_ && !state.last_pin_val_) {
    state.latched_ = false;
  } else if (length && !state.latched_ && state.last_pin_val_) {
    state.latched_ = true;
    set.last_detected_edge_us_ = state.last_intr_;
    set.count_ = set.count_ + 1;
  }
  set.last_rising_edge_us_ = (!state.latched_ && pin_val) ? now : set.last_detected_edge_us_;
  state.last_intr_ = now;
  state.last_pin_val_ = pin_val;
}

}  // namespace pulse_meter_optimized
}  // namespace esphome
