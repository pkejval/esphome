#include "pulse_counter_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <limits>
#include <cmath>

#if !defined(USE_ESP32)
#include <Arduino.h>
#endif

namespace esphome {
namespace pulse_counter {

static const char *const TAG = "pulse_counter";

static const char *const EDGE_MODE_TO_STRING[] = {"DISABLE", "INCREMENT", "DECREMENT"};

#ifdef HAS_PCNT
std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt) {
  if (hw_pcnt)
    return std::make_unique<HwPulseCounterStorage>();
  return std::make_unique<BasicPulseCounterStorage>();
}
#else
std::unique_ptr<PulseCounterStorageBase> get_storage(bool) { return std::make_unique<BasicPulseCounterStorage>(); }
#endif

// -------------------- Software counter --------------------

void IRAM_ATTR BasicPulseCounterStorage::gpio_intr(BasicPulseCounterStorage *arg) {
  const uint32_t t = micros();
  const bool discard = t - arg->last_pulse < arg->filter_us;
  arg->last_pulse = t;
  if (discard)
    return;

  const bool level = arg->isr_pin.digital_read();
  const auto mode = level ? arg->rising_edge_mode : arg->falling_edge_mode;

  switch (mode) {
    case PULSE_COUNTER_DISABLE:
      break;
    case PULSE_COUNTER_INCREMENT:
      arg->counter = arg->counter + 1;
      break;
    case PULSE_COUNTER_DECREMENT:
      arg->counter = arg->counter - 1;
      break;
  }
}

bool BasicPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();
  this->isr_pin = this->pin->to_isr();
  this->pin->attach_interrupt(BasicPulseCounterStorage::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  this->last_value = 0;
  this->counter = 0;
  return true;
}

pulse_counter_t BasicPulseCounterStorage::read_raw_value() {
  pulse_counter_t current;
  pulse_counter_t delta;
  {
    InterruptLock lk;
    current = this->counter;
    delta = current - this->last_value;
    this->last_value = current;
  }
  return delta;
}

BasicPulseCounterStorage::~BasicPulseCounterStorage() {
  if (this->pin != nullptr)
    this->pin->detach_interrupt();
}

// -------------------- Hardware PCNT (read & clear) --------------------

#ifdef HAS_PCNT
static pcnt_channel_edge_action_t map_edge(PulseCounterCountMode m) {
  switch (m) {
    case PULSE_COUNTER_INCREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case PULSE_COUNTER_DECREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}

bool HwPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();

  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = std::numeric_limits<int16_t>::min();
  unit_cfg.high_limit = std::numeric_limits<int16_t>::max();

  esp_err_t err = pcnt_new_unit(&unit_cfg, &this->unit);
  if (err != ESP_OK || this->unit == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  pcnt_chan_config_t chan_cfg = {};
  chan_cfg.edge_gpio_num = this->pin->get_pin();
  chan_cfg.level_gpio_num = -1;

  err = pcnt_new_channel(this->unit, &chan_cfg, &this->channel);
  if (err != ESP_OK || this->channel == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT channel failed: %s", esp_err_to_name(err));
    return false;
  }

  err =
      pcnt_channel_set_edge_action(this->channel, map_edge(this->rising_edge_mode), map_edge(this->falling_edge_mode));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting edge action failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_channel_set_level_action(this->channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting level action failed: %s", esp_err_to_name(err));
    return false;
  }

  if (this->filter_us != 0) {
    pcnt_glitch_filter_config_t gf = {};
    gf.max_glitch_ns = static_cast<uint64_t>(this->filter_us) * 1000ULL;
    err = pcnt_unit_set_glitch_filter(this->unit, &gf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Setting glitch filter failed: %s", esp_err_to_name(err));
      return false;
    }
  }

  err = pcnt_unit_enable(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Enabling PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_clear_count(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_start(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Starting PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  return true;
}

pulse_counter_t HwPulseCounterStorage::read_raw_value() {
  int value = 0;
  esp_err_t err = pcnt_unit_get_count(this->unit, &value);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Getting PCNT count failed: %s", esp_err_to_name(err));
    return 0;
  }

  err = pcnt_unit_clear_count(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
    // vrátíme hodnotu i při chybě clear
  }

  return static_cast<pulse_counter_t>(value);
}

HwPulseCounterStorage::~HwPulseCounterStorage() {
  if (this->unit != nullptr)
    pcnt_unit_stop(this->unit);
  if (this->channel != nullptr) {
    pcnt_del_channel(this->channel);
    this->channel = nullptr;
  }
  if (this->unit != nullptr) {
    pcnt_del_unit(this->unit);
    this->unit = nullptr;
  }
}
#endif

// -------------------- Sensor wrapper --------------------

PulseCounterSensor::~PulseCounterSensor() {
#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    esp_timer_stop(this->timer_handle_);
    esp_timer_delete(this->timer_handle_);
    this->timer_handle_ = nullptr;
  }
#endif
}

#if defined(USE_ESP32)
void PulseCounterSensor::timer_callback(void *arg) {
  auto *self = static_cast<PulseCounterSensor *>(arg);

  // Reálný čas v µs
  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());
  const pulse_counter_t raw = self->storage_->read_raw_value();

  if (self->last_tick_us_ == 0) {
    self->last_tick_us_ = t;
    // zapíšeme případný total delta, ale bez výpočtu ppm (chybí dt)
    if (raw != 0)
      self->pending_total_delta_.fetch_add(raw, std::memory_order_relaxed);
    return;
  }

  const uint64_t dt_us = t - self->last_tick_us_;
  self->last_tick_us_ = t;

  if (raw != 0) {
    self->pending_total_delta_.fetch_add(raw, std::memory_order_relaxed);
  }

  if (dt_us == 0) {
    return;
  }

  // Pulses per minute z reálného dt
  const double ppm = (static_cast<double>(raw) * 60000000.0) / static_cast<double>(dt_us);

  if (std::isfinite(ppm)) {
    float out = static_cast<float>(ppm);
    // Volitelné vyhlazení EMA (pevná alfa, časově nekorigovaná – jednoduché a stabilní)
    if (self->ema_alpha_ > 0.0f && self->ema_alpha_ <= 1.0f) {
      if (!std::isfinite(self->ema_state_))
        self->ema_state_ = out;
      else
        self->ema_state_ = self->ema_state_ + self->ema_alpha_ * (out - self->ema_state_);
      out = self->ema_state_;
    }

    self->last_calculated_ppm_.store(out, std::memory_order_relaxed);
    self->new_value_ready_.store(true, std::memory_order_release);
  }
}
#endif

void PulseCounterSensor::setup() {
  if (!this->storage_->pulse_counter_setup(this->pin_)) {
    this->mark_failed();
    return;
  }

#if defined(USE_ESP32)
  esp_timer_create_args_t timer_args = {
      .callback = &PulseCounterSensor::timer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,  // běh v timer tasku
      .name = "pulse_counter",
  };
  esp_err_t err = esp_timer_create(&timer_args, &this->timer_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  uint64_t period_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
  if (period_us == 0)
    period_us = 10000ULL;  // bezpečné minimum (10 ms), kdyby někdo nastavil 0

  err = esp_timer_start_periodic(this->timer_handle_, period_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
#endif
}

void PulseCounterSensor::set_update_interval(uint32_t update_interval) {
  PollingComponent::set_update_interval(update_interval);
#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    esp_timer_stop(this->timer_handle_);
    uint64_t period_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    if (period_us == 0)
      period_us = 10000ULL;
    esp_err_t err = esp_timer_start_periodic(this->timer_handle_, period_us);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_timer_start_periodic (restart) failed: %s", esp_err_to_name(err));
      this->mark_failed();
    }
    // reset vzorkovací reference
    this->last_tick_us_ = 0;
  }
#endif
}

void PulseCounterSensor::set_total_pulses(uint32_t pulses) {
  this->current_total_ = static_cast<uint64_t>(pulses);
  if (this->total_sensor_ != nullptr)
    this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
}

void PulseCounterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Counter", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG,
                "  Rising Edge: %s\n"
                "  Falling Edge: %s\n"
                "  Filtering pulses shorter than %" PRIu32 " us\n"
                "  EMA alpha: %.3f (0=off)",
                EDGE_MODE_TO_STRING[this->storage_->rising_edge_mode],
                EDGE_MODE_TO_STRING[this->storage_->falling_edge_mode], this->storage_->filter_us, this->ema_alpha_);
  LOG_UPDATE_INTERVAL(this);
}

void PulseCounterSensor::update() {
#if defined(USE_ESP32)
  // Publikace poslední hodnoty z timeru (nejnovější dostupná)
  if (this->new_value_ready_.load(std::memory_order_acquire)) {
    this->new_value_ready_.store(false, std::memory_order_release);
    const float ppm = this->last_calculated_ppm_.load(std::memory_order_relaxed);
    if (std::isfinite(ppm)) {
      ESP_LOGD(TAG, "'%s': timer ppm=%.6f", this->get_name().c_str(), ppm);
      this->publish_state(ppm);
    }
  }

  // Bezpečné převzetí přírůstku total a publikace
  if (this->total_sensor_ != nullptr) {
    const int32_t delta = this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);
    if (delta != 0) {
      if (delta > 0) {
        this->current_total_ += static_cast<uint64_t>(delta);
        this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
        ESP_LOGV(TAG, "'%s': Total += %" PRIi32 " -> %" PRIu64, this->get_name().c_str(), delta, this->current_total_);
      } else {
        ESP_LOGV(TAG, "'%s': Negative delta (%" PRIi32 ") ignored for total.", this->get_name().c_str(), delta);
      }
    }
  }
#else
  // Fallback: výpočet z dt mezi dvěma update() (loop jitter může ovlivnit přesnost)
  const pulse_counter_t raw = this->storage_->read_raw_value();
  const uint64_t t = static_cast<uint64_t>(micros());

  if (this->last_time_us_ != 0) {
    const uint64_t dt = t - this->last_time_us_;
    if (dt > 0) {
      const double ppm = (static_cast<double>(raw) * 60000000.0) / static_cast<double>(dt);
      if (std::isfinite(ppm)) {
        float out = static_cast<float>(ppm);
        if (this->ema_alpha_ > 0.0f && this->ema_alpha_ <= 1.0f) {
          if (!std::isfinite(this->ema_state_))
            this->ema_state_ = out;
          else
            this->ema_state_ = this->ema_state_ + this->ema_alpha_ * (out - this->ema_state_);
          out = this->ema_state_;
        }
        this->publish_state(out);
      }
    }
  }
  this->last_time_us_ = t;

  if (this->total_sensor_ != nullptr) {
    if (raw > 0) {
      this->current_total_ += static_cast<uint64_t>(raw);
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
    }
  }
#endif
}

}  // namespace pulse_counter
}  // namespace esphome
