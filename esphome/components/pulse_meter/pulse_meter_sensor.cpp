#include "pulse_meter_sensor.h"
#include <utility>
#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  this->total_pulses_ = pulses;
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
void PulseMeterSensor::attach_isr_task_(void *arg) {
  auto *self = static_cast<PulseMeterSensor *>(arg);
  if (self->filter_mode_ == FILTER_EDGE) {
    self->pin_->attach_interrupt(PulseMeterSensor::edge_intr, self, gpio::INTERRUPT_RISING_EDGE);
  } else {
    self->pulse_state_.last_pin_val_ = self->isr_pin_.digital_read();
    self->pulse_state_.latched_ = self->pulse_state_.last_pin_val_;
    self->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, self, gpio::INTERRUPT_ANY_EDGE);
  }
  vTaskDelete(nullptr);
}
#endif

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();

  // Set the last processed edge to now for the first timeout
  this->last_processed_edge_us_ = micros();

  if (this->filter_mode_ == FILTER_EDGE) {
    this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
  } else if (this->filter_mode_ == FILTER_PULSE) {
    // Set the pin value to the current value to avoid a false edge
    this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
    this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
    this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = micros();

  // Reset the count in get before we pass it back to the ISR as set
  this->get_->count_ = 0;

  // Swap out set and get to get the latest state from the ISR
  // The ISR could interrupt on any of these lines and the results would be consistent
  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (this->use_rmt_) {
    const rmt_symbol_word_t *sym = (const rmt_symbol_word_t *) this->rmt_recv_symbols_;
    size_t count = (size_t) this->rmt_recv_count_;
    if (sym != nullptr && count > 0) {
      bool latched = this->pulse_state_.latched_;
      bool last_pin = this->pulse_state_.last_pin_val_;
      uint32_t last_detected_edge_us = this->get_->last_detected_edge_us_;
      uint32_t local_count = 0;

      for (size_t i = 0; i < count; ++i) {
        const auto &w = sym[i];

        const bool lvl0 = w.level0;
        const uint32_t dur0_us = w.duration0;
        if (lvl0 != last_pin) {
          if (last_pin == 0) {
            if (dur0_us >= this->min_low_us_)
              latched = false;
          } else {
            if (dur0_us >= this->min_high_us_) {
              latched = true;
              last_detected_edge_us = now;
              ++local_count;
            }
          }
          last_pin = lvl0;
        }

        const bool lvl1 = w.level1;
        const uint32_t dur1_us = w.duration1;
        if (lvl1 != last_pin) {
          if (last_pin == 0) {
            if (dur1_us >= this->min_low_us_)
              latched = false;
          } else {
            if (dur1_us >= this->min_high_us_) {
              latched = true;
              last_detected_edge_us = now;
              ++local_count;
            }
          }
          last_pin = lvl1;
        }
      }

      if (local_count > 0) {
        this->get_->last_detected_edge_us_ = last_detected_edge_us;
        this->get_->last_rising_edge_us_ = last_detected_edge_us;
        this->get_->count_ += local_count;
        this->new_event_ = true;
        this->pulse_state_.latched_ = latched;
        this->pulse_state_.last_pin_val_ = last_pin;
      }

      this->rmt_recv_symbols_ = nullptr;
      this->rmt_recv_count_ = 0;
      (void) rmt_receive(this->rmt_rx_channel_, nullptr, 0, &this->rmt_rx_cfg_);
    }
  }
#endif

  if (LIKELY(!this->new_event_) && LIKELY(now < this->next_timeout_check_us_)) {
    return;
  }

  this->get_->count_ = 0;

  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

  // snapshot z volatile do lokálů (méně readů)
  const uint32_t cnt = this->get_->count_;
  const uint32_t tdet = this->get_->last_detected_edge_us_;
  const uint32_t trise = this->get_->last_rising_edge_us_;
  bool had_event = this->new_event_;
  this->new_event_ = false;

  if (this->peeked_edge_ && cnt > 0) {
    this->peeked_edge_ = false;
    this->get_->count_ = this->get_->count_ - 1;
  }

  if (trise != tdet && (now - trise) >= this->filter_us_) {
    this->peeked_edge_ = true;
    this->get_->last_detected_edge_us_ = this->get_->last_rising_edge_us_;
    this->get_->count_ = this->get_->count_ + 1;
  }

  if (LIKELY(this->get_->count_ > 0)) {
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += this->get_->count_;
      const uint32_t total = this->total_pulses_;
      this->total_sensor_->publish_state(total);
    }

    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;
      case MeterState::RUNNING: {
        const uint32_t delta_us = this->get_->last_detected_edge_us_ - this->last_processed_edge_us_;
        if (delta_us > 0) {
          const float rpm = (60000000.0f * float(this->get_->count_)) / float(delta_us);
          this->publish_state(rpm);
          this->update_period_estimate_(delta_us, this->get_->count_);
        }
      } break;
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
    this->plan_next_check_(now);
    return;
  }

  if (UNLIKELY(!had_event)) {
    const uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::RUNNING:
        if (time_since_valid_edge_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min",
                   time_since_valid_edge_us / 1000000);
          this->publish_state(0.0f);
          this->next_timeout_check_us_ = now + this->timeout_us_;
        } else {
          this->plan_next_check_(now);
        }
        break;
      default:
        break;
    }
  }
}

float PulseMeterSensor::get_setup_priority() const { return setup_priority::DATA; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %" PRIu32 " us apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %" PRIu32 " us (low>=%" PRIu32 " us, high>=%" PRIu32 " us)",
                  this->filter_us_, this->min_low_us_, this->min_high_us_);
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %" PRIu32 " s",
                this->timeout_us_ / 1000000);
#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
#if (ESP_IDF_VERSION_MAJOR >= 5)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6)
  ESP_LOGCONFIG(TAG, "  GPIO glitch filter: %s", this->glitch_filter_ ? "enabled" : "not available");
#endif
#endif
#endif
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (this->filter_mode_ == FILTER_PULSE)
    ESP_LOGCONFIG(TAG, "  RMT backend: %s", this->use_rmt_ ? "enabled" : "not available");
#endif
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  if (UNLIKELY(now < sensor->coalesce_until_us_))
    return;

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
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &state = sensor->pulse_state_;
  auto &set = *sensor->set_;

  // Filter length has passed since the last interrupt
  const bool length = now - state.last_intr_ >= sensor->filter_us_;

  if (length && state.latched_ && !state.last_pin_val_) {  // Long enough low edge
    state.latched_ = false;
  } else if (length && !state.latched_ && state.last_pin_val_) {  // Long enough high edge
    state.latched_ = true;
    set.last_detected_edge_us_ = state.last_intr_;
    set.count_ = set.count_ + 1;
  }

  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &st = sensor->pulse_state_;
  auto &set = *sensor->set_;

  state.last_intr_ = now;
  state.last_pin_val_ = pin_val;
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
bool IRAM_ATTR PulseMeterSensor::rmt_rx_done_cb_(rmt_channel_handle_t, const rmt_rx_done_event_data_t *edata,
                                                 void *user_ctx) {
  auto *self = static_cast<PulseMeterSensor *>(user_ctx);
  self->rmt_recv_symbols_ = edata->received_symbols;
  self->rmt_recv_count_ = edata->num_symbols;
  self->new_event_ = true;
  return false;
}
#endif

}  // namespace pulse_meter
}  // namespace esphome
