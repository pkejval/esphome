#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <inttypes.h>

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";
static constexpr float kPulsesPerMinuteScaling = 60000000.0f;

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
    if (total_pulses_ > UINT32_MAX - pulses) {
        total_pulses_ = 0;
    } else {
        total_pulses_ += pulses;
    }
    
    if (total_sensor_ != nullptr) {
        total_sensor_->publish_state(total_pulses_);
    }
}

void PulseMeterSensor::setup() {
    pin_->setup();
    isr_pin_ = pin_->to_isr();
    last_processed_edge_us_ = (uint32_t)esp_timer_get_time();
    
    // Precompute GPIO mask and register for ISR
    const uint8_t pin_number = pin_->get_pin();
    gpio_mask_ = (1ULL << (pin_number & 31));
    gpio_input_reg_ = (volatile uint32_t*)((pin_number < 32) ? 
        &GPIO.in : &GPIO.in1.val);
    
    // Clear state buffers
    memset(state_, 0, sizeof(state_));
    
    if (filter_mode_ == FILTER_EDGE) {
        edge.last_sent_edge_us_ = 0;
        pin_->attach_interrupt(edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    } else {
        pulse.last_pin_val_ = (*gpio_input_reg_ & gpio_mask_) != 0;
        pulse.latched_ = pulse.last_pin_val_;
        pulse.last_intr_ = 0;
        pin_->attach_interrupt(pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
    
    esp_task_wdt_add(nullptr);
}

void PulseMeterSensor::loop() {
    const uint32_t now = (uint32_t)esp_timer_get_time();
    
    // Fast path - swap buffers
    State *current_get;
    {
        portENTER_CRITICAL(&mux_);
        current_get = const_cast<State*>(get_);
        get_ = set_;
        set_ = current_get;
        portEXIT_CRITICAL(&mux_);
    }
    
    // Reset count for new buffer
    current_get->count_ = 0;

    // Process peeked edge if any
    if (peeked_edge_ && current_get->count_ > 0) {
        peeked_edge_ = false;
        current_get->count_--;
    }

    // Check for new valid edge
    const bool new_edge = (current_get->last_rising_edge_us_ != current_get->last_detected_edge_us_) &&
                         (now - current_get->last_rising_edge_us_ >= filter_us_);
    
    if (new_edge) {
        peeked_edge_ = true;
        current_get->last_detected_edge_us_ = current_get->last_rising_edge_us_;
        current_get->count_++;
    }

    // Fast path - no pulses
    if (current_get->count_ == 0) {
        const uint32_t time_since_valid_edge_us = now - last_processed_edge_us_;
        if ((meter_state_ != MeterState::TIMED_OUT) && (time_since_valid_edge_us > timeout_us_)) {
            meter_state_ = MeterState::TIMED_OUT;
            publish_state(0.0f);
        }
        esp_task_wdt_reset();
        return;
    }

    // Update total pulses if needed
    if (total_sensor_) {
        total_pulses_ += current_get->count_;
        total_sensor_->publish_state(total_pulses_);
    }

    // Handle meter state transitions
    switch (meter_state_) {
        case MeterState::INITIAL:
        case MeterState::TIMED_OUT:
            meter_state_ = MeterState::RUNNING;
            break;
            
        case MeterState::RUNNING: {
            uint32_t delta_us;
            if (current_get->last_detected_edge_us_ >= last_processed_edge_us_) {
                delta_us = current_get->last_detected_edge_us_ - last_processed_edge_us_;
            } else {
                delta_us = (UINT32_MAX - last_processed_edge_us_) + current_get->last_detected_edge_us_ + 1;
            }
            
            const float pulses_per_minute = (kPulsesPerMinuteScaling * current_get->count_) / delta_us;
            publish_state(pulses_per_minute);
            break;
        }
    }
    
    last_processed_edge_us_ = current_get->last_detected_edge_us_;
    esp_task_wdt_reset();
}

void PulseMeterSensor::dump_config() {
    LOG_SENSOR("", "Pulse Meter", this);
    LOG_PIN("  Pin: ", this->pin_);
    const char *filter_mode = this->filter_mode_ == FILTER_EDGE ? "edges" : "pulses";
    ESP_LOGCONFIG(TAG, "  Filtering %s shorter than %" PRIu32 " µs", filter_mode, this->filter_us_);
    ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " s", this->timeout_us_ / 1000000);
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
    const uint32_t now = (uint32_t)esp_timer_get_time();
    const uint32_t last_edge = sensor->edge.last_sent_edge_us_;
    
    if (now - last_edge < sensor->filter_us_)
        return;
    
    sensor->edge.last_sent_edge_us_ = now;
    
    const uint32_t write_idx = sensor->pulse_buffer_.write_idx;
    const uint32_t next_write = (write_idx + 1) & BUFFER_MASK;
    
    if (next_write == sensor->pulse_buffer_.read_idx) {
        __atomic_add_fetch(&sensor->missed_pulses_, 1, __ATOMIC_RELAXED);
        return;
    }
    
    sensor->pulse_buffer_.timestamps[write_idx] = now;
    __atomic_store_n(&sensor->pulse_buffer_.write_idx, next_write, __ATOMIC_RELEASE);
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
    const uint32_t now = (uint32_t)esp_timer_get_time();
    auto &state = sensor->pulse;
    const uint32_t pulse_length = now - state.last_intr_;
    
    if (pulse_length < sensor->filter_us_) {
        state.last_intr_ = now;
        return;
    }
    
    const bool pin_val = (*sensor->gpio_input_reg_ & sensor->gpio_mask_) != 0;
    volatile State* const current_set = sensor->set_;
    
    const bool was_latched = state.latched_;
    const bool rising_edge = !state.last_pin_val_ && pin_val;
    const bool falling_edge = state.last_pin_val_ && !pin_val;
    
    if (was_latched && falling_edge) {
        state.latched_ = false;
    } else if (!was_latched && rising_edge) {
        state.latched_ = true;
        current_set->last_detected_edge_us_ = state.last_intr_;
        __atomic_add_fetch(&current_set->count_, 1, __ATOMIC_RELAXED);
    }
    
    if (rising_edge) {
        current_set->last_rising_edge_us_ = now;
    }
    
    state.last_intr_ = now;
    state.last_pin_val_ = pin_val;
}

}  // namespace pulse_meter
}  // namespace esphome
