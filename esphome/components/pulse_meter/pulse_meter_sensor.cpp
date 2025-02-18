#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <inttypes.h>

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";
static constexpr float kPulsesPerMinuteScaling = 60000000.0f;

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
    // Check for overflow before adding
    if (this->total_pulses_ > UINT32_MAX - pulses) {
        ESP_LOGW(TAG, "Total pulse counter overflow - resetting to 0");
        this->total_pulses_ = 0;
    } else {
        this->total_pulses_ += pulses;
    }
    
    if (this->total_sensor_ != nullptr) {
        this->total_sensor_->publish_state(this->total_pulses_);
    }
}

void PulseMeterSensor::setup() {
    // Initialize all fields first
    this->pin_->setup();
    this->isr_pin_ = pin_->to_isr();
    this->last_processed_edge_us_ = (uint32_t)esp_timer_get_time();
    
    // Clear both state buffers
    memset(&state_[0], 0, sizeof(State));
    memset(&state_[1], 0, sizeof(State));
    
    // Set up interrupt handling last
    if (this->filter_mode_ == FILTER_EDGE) {
        this->edge.last_sent_edge_us_ = 0;
        this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    } else {
        this->pulse.last_pin_val_ = this->isr_pin_.digital_read();
        this->pulse.latched_ = this->pulse.last_pin_val_;
        this->pulse.last_intr_ = 0;
        this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
    
    esp_task_wdt_add(nullptr);
}

void PulseMeterSensor::loop() {
  const uint32_t now = (uint32_t)esp_timer_get_time();

  this->get_->count_ = 0;
  portENTER_CRITICAL(&this->mux_);
  volatile State *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;
  portEXIT_CRITICAL(&this->mux_);

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
    if (this->total_sensor_) {
      this->total_pulses_ += this->get_->count_;
      this->total_sensor_->publish_state(this->total_pulses_);
    }
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;
      case MeterState::RUNNING: {
        uint32_t delta_us;
        if (this->get_->last_detected_edge_us_ >= this->last_processed_edge_us_) {
            delta_us = this->get_->last_detected_edge_us_ - this->last_processed_edge_us_;
        } else {
            // Handle wraparound
            delta_us = (UINT32_MAX - this->last_processed_edge_us_) + this->get_->last_detected_edge_us_ + 1;
        }
        float pulses_per_minute = (kPulsesPerMinuteScaling * this->get_->count_) / delta_us;
        ESP_LOGV(TAG, "New pulse, delta: %" PRIu32 " µs, count: %" PRIu32 ", rate: %.5f/min",
                 delta_us, this->get_->count_, pulses_per_minute);
        this->publish_state(pulses_per_minute);
      } break;
    }
    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else {
    uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
    if ((this->meter_state_ == MeterState::INITIAL || this->meter_state_ == MeterState::RUNNING) &&
        (time_since_valid_edge_us > this->timeout_us_)) {
      this->meter_state_ = MeterState::TIMED_OUT;
      ESP_LOGD(TAG, "Timeout: %" PRIu32 " s, 0 pulses/min", time_since_valid_edge_us / 1000000);
      this->publish_state(0.0f);
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
  const char *filter_mode = this->filter_mode_ == FILTER_EDGE ? "edges" : "pulses";
  ESP_LOGCONFIG(TAG, "  Filtering %s shorter than %" PRIu32 " µs", filter_mode, this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " s", this->timeout_us_ / 1000000);
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
    // Load volatile state pointer once
    volatile State* const current_set = sensor->set_;
    const uint32_t now = (uint32_t)esp_timer_get_time();
    
    // Single time delta calculation
    const uint32_t time_since_last = now - sensor->edge.last_sent_edge_us_;
    
    if (time_since_last >= sensor->filter_us_) {
        sensor->edge.last_sent_edge_us_ = now;
        // Separate volatile writes to avoid deprecated warning
        current_set->last_rising_edge_us_ = now;
        current_set->last_detected_edge_us_ = now;
        // Use relaxed memory ordering since this is just a counter
        __atomic_add_fetch(&current_set->count_, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
    const uint32_t now = (uint32_t)esp_timer_get_time();
    
    // Cache frequently accessed members
    auto &state = sensor->pulse;
    volatile State* const current_set = sensor->set_;
    const bool pin_val = sensor->isr_pin_.digital_read();
    
    // Calculate time delta once
    const uint32_t pulse_length = now - state.last_intr_;
    
    if (pulse_length >= sensor->filter_us_) {
        const bool was_latched = state.latched_;
        const bool was_last_low = !state.last_pin_val_;
        
        if (was_latched && was_last_low) {
            // Falling edge after minimum pulse width
            state.latched_ = false;
        } else if (!was_latched && !was_last_low) {
            // Rising edge after minimum pulse width
            state.latched_ = true;
            current_set->last_detected_edge_us_ = state.last_intr_;
            // Use relaxed memory ordering for counter
            __atomic_add_fetch(&current_set->count_, 1, __ATOMIC_RELAXED);
        }
    }

    // Combine conditional with edge timing update
    if (!state.latched_ && pin_val) {
        current_set->last_rising_edge_us_ = now;
    }

    // Update state for next interrupt
    state.last_intr_ = now;
    state.last_pin_val_ = pin_val;
}

}  // namespace pulse_meter
}  // namespace esphome
