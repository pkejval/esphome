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

  const uint32_t now = micros();
  this->last_processed_edge_us_ = now;
  this->next_timeout_check_us_ = now + this->timeout_us_;
  this->new_event_ = false;
  this->period_estimate_us_ = 0.0f;

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
    const uint32_t ns = (uint32_t) (this->filter_us_ * 1000ULL);
    cfg.window_thres_ns = ns > 0 ? ns : 0;
    if (ns > 0 && gpio_new_glitch_filter(&cfg, &this->glitch_filter_) == ESP_OK) {
      gpio_glitch_filter_enable(this->glitch_filter_);
    } else {
      this->glitch_filter_ = nullptr;
    }
  }
#endif
#endif
#endif

// RMT backend (PULSE)
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (this->filter_mode_ == FILTER_PULSE) {
    rmt_rx_channel_config_t ch_cfg{};
    ch_cfg.gpio_num = (gpio_num_t) this->pin_->get_pin();
    ch_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    ch_cfg.resolution_hz = this->rmt_resolution_hz_;
    ch_cfg.mem_block_symbols = 512;
    if (rmt_new_rx_channel(&ch_cfg, &this->rmt_rx_channel_) == ESP_OK && this->rmt_rx_channel_) {
      rmt_rx_event_callbacks_t cbs{};
      cbs.on_recv_done = &PulseMeterSensor::rmt_rx_done_cb_;
      if (rmt_rx_register_event_callbacks(this->rmt_rx_channel_, &cbs, this) == ESP_OK) {
        rmt_receive_config_t rx_cfg{};
        rx_cfg.signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL);
        const uint32_t max_ns = (this->timeout_us_ > 0 ? this->timeout_us_ : 1000000UL) * 1000UL;
        rx_cfg.signal_range_max_ns = max_ns;
        this->rmt_rx_cfg_ = rx_cfg;
        if (rmt_enable(this->rmt_rx_channel_) == ESP_OK &&
            rmt_receive(this->rmt_rx_channel_, nullptr, 0, &this->rmt_rx_cfg_) == ESP_OK) {
          this->use_rmt_ = true;
        }
      }
    }
  }
#endif

// MCPWM Capture backend (fallback za RMT)
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/mcpwm_cap.h")
  if (!this->use_rmt_ && this->filter_mode_ == FILTER_PULSE) {
    mcpwm_capture_timer_config_t tcfg{};
    tcfg.group_id = 0;
    tcfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = this->cap_resolution_hz_;
    if (mcpwm_new_capture_timer(&tcfg, &this->cap_timer_) == ESP_OK && this->cap_timer_) {
      mcpwm_capture_channel_config_t ccfg{};
      ccfg.gpio_num = (gpio_num_t) this->pin_->get_pin();
      ccfg.prescale = 1;
      ccfg.flags.pull_up = 1;
      ccfg.flags.invert_cap_signal = 0;
      if (mcpwm_new_capture_channel(this->cap_timer_, &ccfg, &this->cap_chan_) == ESP_OK && this->cap_chan_) {
        mcpwm_capture_event_callbacks_t cbs{};
        cbs.on_cap = &PulseMeterSensor::mcpwm_cap_cb_;
        if (mcpwm_capture_channel_register_event_callbacks(this->cap_chan_, &cbs, this) == ESP_OK &&
            mcpwm_capture_channel_enable(this->cap_chan_) == ESP_OK &&
            mcpwm_capture_timer_enable(this->cap_timer_) == ESP_OK) {
          mcpwm_capture_timer_start(this->cap_timer_);
          mcpwm_capture_edge_t edges = (mcpwm_capture_edge_t) (MCPWM_CAPTURE_EDGE_POS | MCPWM_CAPTURE_EDGE_NEG);
          mcpwm_capture_channel_start(this->cap_chan_, edges);
          this->use_mcpwm_ = true;
          this->cap_last_ts_us_ = 0;
          this->cap_last_level_high_ = this->isr_pin_.digital_read();
        }
      }
    }
  }
#endif

  // Fallback na GPIO ISR
  if (
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
      !this->use_rmt_ &&
#endif
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/mcpwm_cap.h")
      !this->use_mcpwm_ &&
#endif
      true) {
    if (this->filter_mode_ == FILTER_EDGE) {
      this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    } else {
      this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
      this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
      this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = micros();

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
        if (delta_us > 0) {
          const float pulse_width_us = delta_us / float(this->get_->count_);
          this->publish_state((60.0f * 1000000.0f) / pulse_width_us);
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
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/mcpwm_cap.h")
  if (this->filter_mode_ == FILTER_PULSE)
    ESP_LOGCONFIG(TAG, "  MCPWM capture backend: %s", this->use_mcpwm_ ? "enabled" : "not available");
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
    if ((now - st.last_intr_) >= self->min_low_us_) {
      st.latched_ = false;
    }
  } else if (long_enough && !st.latched_ && st.last_pin_val_) {
    if ((now - st.last_intr_) >= self->min_high_us_) {
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

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/mcpwm_cap.h")
bool IRAM_ATTR PulseMeterSensor::mcpwm_cap_cb_(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *edata,
                                               void *user_ctx) {
  auto *self = static_cast<PulseMeterSensor *>(user_ctx);

  const uint32_t ts_us = (uint32_t) (edata->cap_value);

  if (self->cap_last_ts_us_ != 0) {
    const uint32_t dur_us = ts_us - self->cap_last_ts_us_;
    const bool rising = (edata->cap_edge & MCPWM_CAPTURE_EDGE_POS) != 0;
    const bool falling = (edata->cap_edge & MCPWM_CAPTURE_EDGE_NEG) != 0;

    if (rising) {
      if (dur_us >= self->min_low_us_) {
        self->pulse_state_.latched_ = false;
      }
    } else if (falling) {
      if (dur_us >= self->min_high_us_) {
        self->pulse_state_.latched_ = true;
        self->set_->last_detected_edge_us_ = ts_us;
        self->set_->last_rising_edge_us_ = ts_us;
        self->set_->count_ = self->set_->count_ + 1;
        self->new_event_ = true;
      }
    }
  }

  self->cap_last_ts_us_ = ts_us;

  return true;
}
#endif

}  // namespace pulse_meter
}  // namespace esphome
