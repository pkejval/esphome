#include "pulse_counter_sensor.h"
#include "esphome/core/log.h"
#include <limits>

#if defined(USE_ESP32)
#include <esp_timer.h>
#endif

namespace esphome {
namespace pulse_counter {

static const char *const TAG = "pulse_counter";

const char *const EDGE_MODE_TO_STRING[] = {"DISABLE", "INCREMENT", "DECREMENT"};

static inline uint64_t now_us() {
#if defined(USE_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time());
#else
  return static_cast<uint64_t>(micros());
#endif
}

#ifdef HAS_PCNT
PulseCounterStorageBase *get_storage(bool hw_pcnt) {
  return (hw_pcnt ? (PulseCounterStorageBase *) (new HwPulseCounterStorage)
                  : (PulseCounterStorageBase *) (new BasicPulseCounterStorage));
}
#else
PulseCounterStorageBase *get_storage(bool) { return new BasicPulseCounterStorage; }
#endif

void IRAM_ATTR BasicPulseCounterStorage::gpio_intr(BasicPulseCounterStorage *arg) {
  const uint32_t now = micros();
  const bool discard = now - arg->last_pulse < arg->filter_us;
  arg->last_pulse = now;
  if (discard)
    return;

  PulseCounterCountMode mode = arg->isr_pin.digital_read() ? arg->rising_edge_mode : arg->falling_edge_mode;
  switch (mode) {
    case PULSE_COUNTER_DISABLE:
      break;
    case PULSE_COUNTER_INCREMENT: {
      auto x = arg->counter + 1;
      arg->counter = x;
    } break;
    case PULSE_COUNTER_DECREMENT: {
      auto x = arg->counter - 1;
      arg->counter = x;
    } break;
  }
}

bool BasicPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();
  this->isr_pin = this->pin->to_isr();
  this->pin->attach_interrupt(BasicPulseCounterStorage::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  return true;
}

pulse_counter_t BasicPulseCounterStorage::read_raw_value() {
  pulse_counter_t counter = this->counter;
  pulse_counter_t ret = counter - this->last_value;
  this->last_value = counter;
  return ret;
}

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
    uint64_t ns = static_cast<uint64_t>(this->filter_us) * 1000ULL;
    gf.max_glitch_ns = ns;
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
    return 0;
  }
  pulse_counter_t ret = static_cast<pulse_counter_t>(value);
  return ret;
}
#endif

void PulseCounterSensor::setup() {
  if (!this->storage_.pulse_counter_setup(this->pin_)) {
    this->mark_failed();
    return;
  }
}

void PulseCounterSensor::set_total_pulses(uint32_t pulses) {
  this->current_total_ = pulses;
  this->total_sensor_->publish_state(pulses);
}

void PulseCounterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Counter", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG,
                "  Rising Edge: %s\n"
                "  Falling Edge: %s\n"
                "  Filtering pulses shorter than %" PRIu32 " µs",
                EDGE_MODE_TO_STRING[this->storage_.rising_edge_mode],
                EDGE_MODE_TO_STRING[this->storage_.falling_edge_mode], this->storage_.filter_us);
  LOG_UPDATE_INTERVAL(this);
}

void PulseCounterSensor::update() {
  pulse_counter_t raw = this->storage_.read_raw_value();
  uint64_t now = now_us();
  if (this->last_time_us_ != 0) {
    uint64_t interval_us = now - this->last_time_us_;
    if (interval_us > 0) {
      float value = (60000000.0f * raw) / static_cast<float>(interval_us);
      ESP_LOGD(TAG, "'%s': Retrieved counter: %0.2f pulses/min", this->get_name().c_str(), value);
      this->publish_state(value);
    }
  }

  if (this->total_sensor_ != nullptr) {
    current_total_ += raw;
    ESP_LOGD(TAG, "'%s': Total : %" PRIu32 " pulses", this->get_name().c_str(), current_total_);
    this->total_sensor_->publish_state(current_total_);
  }
  this->last_time_us_ = now;
}

}  // namespace pulse_counter
}  // namespace esphome
