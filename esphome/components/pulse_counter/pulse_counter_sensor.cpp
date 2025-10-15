#include "pulse_counter_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"      // InterruptLock pro SW čítač
#include "esphome/core/application.h"  // App.feed_wdt()
#include <limits>
#include <cmath>

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
std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt) {
  if (hw_pcnt)
    return std::make_unique<HwPulseCounterStorage>();
  return std::make_unique<BasicPulseCounterStorage>();
}
#else
std::unique_ptr<PulseCounterStorageBase> get_storage(bool) { return std::make_unique<BasicPulseCounterStorage>(); }
#endif

// -------------------- BASIC (SW) --------------------

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
  pulse_counter_t ret;
  {
    InterruptLock lk;
    current = this->counter;
    ret = current - this->last_value;
    this->last_value = current;
  }
  return ret;
}

BasicPulseCounterStorage::~BasicPulseCounterStorage() {
  if (this->pin != nullptr) {
    this->pin->detach_interrupt();
  }
}

// -------------------- HW PCNT (read & clear) --------------------

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
  unit_cfg.low_limit = std::numeric_limits<int16_t>::min();   // -32768
  unit_cfg.high_limit = std::numeric_limits<int16_t>::max();  //  32767
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

  this->first_read_ = true;

  err = pcnt_unit_start(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Starting PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  return true;
}

pulse_counter_t HwPulseCounterStorage::read_raw_value() {
  // Krátké "krmení" WDT před a po volání driveru
  App.feed_wdt();

  int value = 0;
  esp_err_t err = pcnt_unit_get_count(this->unit, &value);

  App.feed_wdt();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Getting PCNT count failed: %s", esp_err_to_name(err));
    return 0;
  }

  // Read & clear: delta = aktuální count
  err = pcnt_unit_clear_count(this->unit);

  App.feed_wdt();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
    // i při selhání clearu vrátíme přečtenou hodnotu
  }

  this->first_read_ = false;
  return static_cast<pulse_counter_t>(value);
}

HwPulseCounterStorage::~HwPulseCounterStorage() {
  if (this->unit != nullptr) {
    pcnt_unit_stop(this->unit);
  }
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

// -------------------- SENSOR --------------------

void PulseCounterSensor::setup() {
  if (!this->storage_->pulse_counter_setup(this->pin_)) {
    this->mark_failed();
    return;
  }
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
                "  Filtering pulses shorter than %" PRIu32 " µs\n"
                "  Total counter mode: monotonic (increments only)",
                EDGE_MODE_TO_STRING[this->storage_->rising_edge_mode],
                EDGE_MODE_TO_STRING[this->storage_->falling_edge_mode], this->storage_->filter_us);
  LOG_UPDATE_INTERVAL(this);
}

void PulseCounterSensor::update() {
  const pulse_counter_t raw = this->storage_->read_raw_value();
  const uint64_t now = now_us();
  if (this->last_time_us_ != 0) {
    const uint64_t interval_us = now - this->last_time_us_;
    if (interval_us > 0) {
      const double value_ppm = (static_cast<double>(raw) * 60000000.0) / static_cast<double>(interval_us);
      if (std::isfinite(value_ppm)) {
        ESP_LOGD(TAG, "'%s': Retrieved counter: %.6f pulses/min", this->get_name().c_str(), value_ppm);
        this->publish_state(static_cast<float>(value_ppm));
      } else {
        ESP_LOGW(TAG, "'%s': Computed non-finite value (raw=%" PRIi32 ", dt=%" PRIu64 " us) — skipping publish",
                 this->get_name().c_str(), raw, interval_us);
      }
    }
  }

  if (this->total_sensor_ != nullptr) {
    if (raw > 0) {
      this->current_total_ += static_cast<uint64_t>(raw);
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
      ESP_LOGD(TAG, "'%s': Total +%" PRIi32 " -> %" PRIu64 " pulses", this->get_name().c_str(), raw,
               this->current_total_);
    } else if (raw < 0) {
      ESP_LOGV(TAG, "'%s': Negative delta (%" PRIi32 ") ignored for total (monotonic).", this->get_name().c_str(), raw);
    }
  }

  this->last_time_us_ = now;
}

}  // namespace pulse_counter
}  // namespace esphome
