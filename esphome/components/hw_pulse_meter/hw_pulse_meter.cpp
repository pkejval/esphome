#include "hw_pulse_meter.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <limits>
#include <cmath>

namespace esphome {
namespace hw_pulse_meter {

static const char *const TAG = "hw_pulse_meter";

// Mapování hran
pcnt_channel_edge_action_t HWPulseMeter::map_edge_rising_(CountMode m) {
  switch (m) {
    case RISING:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case FALLING:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
    case BOTH:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}
pcnt_channel_edge_action_t HWPulseMeter::map_edge_falling_(CountMode m) {
  switch (m) {
    case RISING:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
    case FALLING:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case BOTH:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}

void HWPulseMeter::setup() {
  if (this->pin_ == nullptr) {
    ESP_LOGE(TAG, "No pin configured");
    this->mark_failed();
    return;
  }
  this->pin_->setup();

  if (!this->init_pcnt_()) {
    this->mark_failed();
    return;
  }
  this->apply_glitch_filter_();

  this->last_pulse_time_us_ = 0;
  this->last_publish_time_us_ = 0;
  this->current_total_ = 0;
  this->last_revolutions_pub_ = 0;
  this->idle_zero_armed_ = false;
  this->ever_published_ = false;

  this->pending_total_delta_.store(0, std::memory_order_relaxed);
  this->pending_total_since_boot_.store(0, std::memory_order_relaxed);
  this->pending_pulses_since_pub_.store(0, std::memory_order_relaxed);

  if (!this->start_timer_(TIMER_PERIOD_US)) {
    this->mark_failed();
    return;
  }
}

bool HWPulseMeter::init_pcnt_() {
  pcnt_unit_config_t unit_cfg{};
  unit_cfg.low_limit = std::numeric_limits<int16_t>::min();
  unit_cfg.high_limit = std::numeric_limits<int16_t>::max();

  if (pcnt_new_unit(&unit_cfg, &this->unit_) != ESP_OK || this->unit_ == nullptr) {
    ESP_LOGE(TAG, "pcnt_new_unit failed");
    return false;
  }

  pcnt_chan_config_t ch_cfg{};
  ch_cfg.edge_gpio_num = this->pin_->get_pin();
  ch_cfg.level_gpio_num = -1;

  if (pcnt_new_channel(this->unit_, &ch_cfg, &this->channel_) != ESP_OK || this->channel_ == nullptr) {
    ESP_LOGE(TAG, "pcnt_new_channel failed");
    return false;
  }

  if (pcnt_channel_set_edge_action(this->channel_, map_edge_rising_(this->count_mode_),
                                   map_edge_falling_(this->count_mode_)) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_channel_set_edge_action failed");
    return false;
  }

  if (pcnt_channel_set_level_action(this->channel_, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP) !=
      ESP_OK) {
    ESP_LOGE(TAG, "pcnt_channel_set_level_action failed");
    return false;
  }

  if (pcnt_unit_enable(this->unit_) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_unit_enable failed");
    return false;
  }
  if (pcnt_unit_clear_count(this->unit_) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_unit_clear_count failed");
    return false;
  }
  if (pcnt_unit_start(this->unit_) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_unit_start failed");
    return false;
  }
  return true;
}

void HWPulseMeter::apply_glitch_filter_() {
  if (this->unit_ == nullptr)
    return;
  pcnt_glitch_filter_config_t gf{};
  gf.max_glitch_ns = static_cast<uint64_t>(this->internal_filter_us_applied_) * 1000ULL;
  esp_err_t err = pcnt_unit_set_glitch_filter(this->unit_, &gf);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "pcnt_unit_set_glitch_filter failed: %s", esp_err_to_name(err));
  }
}

void HWPulseMeter::timer_callback_(void *arg) {
  auto *self = static_cast<HWPulseMeter *>(arg);
  if (self == nullptr)
    return;

  int32_t raw = 0;
  if (!self->read_and_clear_pcnt_(raw))
    return;

  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

  if (raw > 0) {
    self->pending_total_delta_.fetch_add(static_cast<int64_t>(raw), std::memory_order_relaxed);
    self->pending_total_since_boot_.fetch_add(static_cast<int64_t>(raw), std::memory_order_relaxed);
    self->pending_pulses_since_pub_.fetch_add(static_cast<int64_t>(raw), std::memory_order_relaxed);

    self->last_pulse_time_us_ = now_us;
    self->idle_zero_armed_ = true;  // po vypršení timeoutu připravíme 0, ale odešleme až v try_publish_throttled_
  }
}

bool HWPulseMeter::start_timer_(uint64_t period_us) {
  if (this->timer_ != nullptr) {
    esp_timer_stop(this->timer_);
    esp_timer_delete(this->timer_);
    this->timer_ = nullptr;
  }
  esp_timer_create_args_t args{};
  args.callback = &HWPulseMeter::timer_callback_;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "hw_pulse_meter";

  if (esp_timer_create(&args, &this->timer_) != ESP_OK || this->timer_ == nullptr) {
    ESP_LOGE(TAG, "esp_timer_create failed");
    return false;
  }
  if (esp_timer_start_periodic(this->timer_, period_us == 0 ? TIMER_PERIOD_US : period_us) != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed");
    esp_timer_delete(this->timer_);
    this->timer_ = nullptr;
    return false;
  }
  return true;
}

void HWPulseMeter::stop_timer_() {
  if (this->timer_ != nullptr) {
    esp_timer_stop(this->timer_);
    esp_timer_delete(this->timer_);
    this->timer_ = nullptr;
  }
}

bool HWPulseMeter::read_and_clear_pcnt_(int32_t &out) {
  if (this->unit_ == nullptr)
    return false;
  int value = 0;
  if (pcnt_unit_get_count(this->unit_, &value) != ESP_OK)
    return false;
  out = static_cast<int32_t>(value);
  (void) pcnt_unit_clear_count(this->unit_);
  return true;
}

// Throttlovaná publikace všech senzorů
void HWPulseMeter::try_publish_throttled_(uint64_t now_us) {
  const uint64_t since_last_pub =
      this->ever_published_ ? (now_us - this->last_publish_time_us_) : this->min_publish_interval_us_;

  if (since_last_pub < this->min_publish_interval_us_)
    return;

  const int64_t pulses = this->pending_pulses_since_pub_.exchange(0, std::memory_order_acq_rel);
  const int64_t total_delta = this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);

  float ppm_to_pub = NAN;
  bool should_pub_zero = false;

  if (pulses > 0) {
    const double dt_us = static_cast<double>(since_last_pub);
    if (dt_us > 0.0) {
      const double ppm = (static_cast<double>(pulses) * 60000000.0) / dt_us;
      if (std::isfinite(ppm))
        ppm_to_pub = static_cast<float>(ppm);
    }
  } else {
    if (this->idle_timeout_us_ > 0 && this->idle_zero_armed_ && this->last_pulse_time_us_ > 0) {
      const uint64_t since_last_pulse = now_us - this->last_pulse_time_us_;
      if (since_last_pulse >= this->idle_timeout_us_) {
        should_pub_zero = true;
      }
    }
  }

  if (std::isfinite(ppm_to_pub)) {
    this->publish_state(ppm_to_pub);
    if (this->publish_pps_ && this->pps_sensor_ != nullptr) {
      this->pps_sensor_->publish_state(ppm_to_pub / 60.0f);
    }
    this->idle_zero_armed_ = true;  // po publikaci platné hodnoty znovu povolíme idle nulu
  } else if (should_pub_zero) {
    this->publish_state(0.0f);
    if (this->publish_pps_ && this->pps_sensor_ != nullptr) {
      this->pps_sensor_->publish_state(0.0f);
    }
    this->idle_zero_armed_ = false;  // nulu nebudeme spamovat každým intervalem
  }

  if (this->publish_total_ && this->total_sensor_ != nullptr) {
    if (total_delta > 0) {
      this->current_total_ += static_cast<uint64_t>(total_delta);
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
    }
  } else {
    // pokud total nepoužíváme, přesto udržuj current_total_ pro revolutions
    if (total_delta > 0) {
      this->current_total_ += static_cast<uint64_t>(total_delta);
    }
  }

  if (this->publish_revolutions_ && this->revolutions_sensor_ != nullptr) {
    const uint32_t ppr = this->pulses_per_revolution_;
    if (ppr > 0) {
      const uint64_t revs_now = this->current_total_ / static_cast<uint64_t>(ppr);
      if (revs_now != this->last_revolutions_pub_) {
        this->last_revolutions_pub_ = revs_now;
        this->revolutions_sensor_->publish_state(static_cast<float>(revs_now));
      }
    }
  }

  this->last_publish_time_us_ = now_us;
  this->ever_published_ = true;
}

void HWPulseMeter::loop() {
  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  this->try_publish_throttled_(now_us);
}

void HWPulseMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "HW Pulse Meter (PCNT)");
  if (this->pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", this->pin_->get_pin());
  }
  const char *mode_str = (this->count_mode_ == RISING) ? "RISING" : (this->count_mode_ == FALLING) ? "FALLING" : "BOTH";
  ESP_LOGCONFIG(TAG, "  Count mode: %s", mode_str);
  ESP_LOGCONFIG(TAG, "  Internal filter (requested/applied): %u us / %u us",
                (unsigned) this->internal_filter_us_requested_, (unsigned) this->internal_filter_us_applied_);
  ESP_LOGCONFIG(TAG, "  Idle timeout: %u us", (unsigned) this->idle_timeout_us_);
  ESP_LOGCONFIG(TAG, "  Min publish interval: %u us", (unsigned) this->min_publish_interval_us_);
  ESP_LOGCONFIG(TAG, "  PPR: %u", (unsigned) this->pulses_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Subsensors: total=%s, pps=%s, revolutions=%s", this->publish_total_ ? "yes" : "no",
                this->publish_pps_ ? "yes" : "no", this->publish_revolutions_ ? "yes" : "no");
}

HWPulseMeter::~HWPulseMeter() {
  this->stop_timer_();

  if (this->unit_ != nullptr) {
    (void) pcnt_unit_stop(this->unit_);
  }
  if (this->channel_ != nullptr) {
    (void) pcnt_del_channel(this->channel_);
    this->channel_ = nullptr;
  }
  if (this->unit_ != nullptr) {
    (void) pcnt_del_unit(this->unit_);
    this->unit_ = nullptr;
  }
}

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
