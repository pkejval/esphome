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

  this->last_processed_edge_us_ = micros();
  this->next_timeout_check_us_ = this->last_processed_edge_us_ + this->timeout_us_;
  this->new_event_ = false;

  if (this->min_low_us_ == 0 && this->min_high_us_ == 0) {
    this->update_hysteresis_defaults_();
  }

#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
#if (ESP_IDF_VERSION_MAJOR >= 5)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6)
  {
    gpio_glitch_filter_config_t cfg{};
    cfg.gpio_num = static_cast<gpio_num_t>(this->pin_->get_pin());
    cfg.clk_src = GPIO_GLITCH_FILTER_CLK_SRC_DEFAULT;
    cfg.window_thres_ns = (uint32_t) (this->filter_us_ * 1000ULL);
    if (gpio_new_glitch_filter(&cfg, &this->glitch_filter_) == ESP_OK) {
      gpio_glitch_filter_enable(this->glitch_filter_);
    } else {
      this->glitch_filter_ = nullptr;
    }
  }
#endif
#endif
#endif

  if (this->filter_mode_ == FILTER_EDGE) {
    this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
  } else {
    this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
    this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
    this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = micros();

  if (LIKELY(!this->new_event_) && LIKELY(now < this->next_timeout_check_us_)) {
    return;
  }

  this->get_->count_ = 0;

  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

  bool had_event = this->new_event_;
  this->new_event_ = false;

  if (this->peeked_edge_ && this->get_->count_ > 0) {
    this->peeked_edge_ = false;
    this->get_->count_ = this->get_->count_ - 1;
  }

  if (this->get_->last_rising_edge_us_ != this->get_->last_detected_edge_us_ &&
      (now - this->get_->last_rising_edge_us_) >= this->filter_us_) {
    this->peeked_edge_ = true;
    this->get_->last_detected_edge_us_ = this->get_->last_rising_edge_us_;
    this->get_->count_ = this->get_->count_ + 1;
    had_event = true;
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
        const float pulse_width_us = delta_us / float(this->get_->count_);
#if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE)
        ESP_LOGV(TAG, "New pulse, delta: %" PRIu32 " us, count: %" PRIu32 ", width: %.5f us", delta_us,
                 this->get_->count_, pulse_width_us);
#endif
        this->publish_state((60.0f * 1000000.0f) / pulse_width_us);
      } break;
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
    this->next_timeout_check_us_ = this->last_processed_edge_us_ + this->timeout_us_;
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
    sensor->new_event_ = true;
  }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &st = sensor->pulse_state_;
  auto &set = *sensor->set_;

  const bool long_enough = (now - st.last_intr_) >= sensor->filter_us_;

  if (long_enough && st.latched_ && !st.last_pin_val_) {
    if ((now - st.last_intr_) >= sensor->min_low_us_) {
      st.latched_ = false;
    }
  } else if (long_enough && !st.latched_ && st.last_pin_val_) {
    if ((now - st.last_intr_) >= sensor->min_high_us_) {
      st.latched_ = true;
      set.last_detected_edge_us_ = st.last_intr_;
      set.count_ = set.count_ + 1;
      sensor->new_event_ = true;
    }
  }

  set.last_rising_edge_us_ = (!st.latched_ && pin_val) ? now : set.last_detected_edge_us_;

  st.last_intr_ = now;
  st.last_pin_val_ = pin_val;
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
