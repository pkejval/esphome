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
  this->last_rev_time_us_ = 0;
  this->current_total_ = 0;
  this->last_revolutions_pub_ = 0;
  this->idle_zero_published_ = false;

  this->pending_total_delta_.store(0, std::memory_order_relaxed);
  this->pending_pulses_since_rev_.store(0, std::memory_order_relaxed);
  this->last_calculated_rpm_.store(NAN, std::memory_order_relaxed);
  this->last_calculated_pps_.store(NAN, std::memory_order_relaxed);
  this->new_value_ready_.store(false, std::memory_order_relaxed);

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
    // akumuluj pro TOTAL a REV publikaci
    self->pending_total_delta_.fetch_add(static_cast<int64_t>(raw), std::memory_order_relaxed);

    // idle tracking
    self->last_pulse_time_us_ = now_us;
    self->idle_zero_published_ = false;

    // akumuluj pro "celé otáčky"
    const uint32_t added = static_cast<uint32_t>(raw);
    const uint32_t prev = self->pending_pulses_since_rev_.load(std::memory_order_relaxed);
    const uint32_t now = prev + added;
    self->pending_pulses_since_rev_.store(now, std::memory_order_relaxed);

    const uint32_t ppr = self->pulses_per_revolution_;
    if (ppr > 0 && now >= ppr) {
      // Kolik celých otáček spadlo v tomto kroku
      const uint32_t revs = now / ppr;

      // Pokud je to první publikace po bootu, jen "nastartuj" čas základny
      if (self->last_rev_time_us_ == 0) {
        self->last_rev_time_us_ = now_us;
      } else {
        const uint64_t dt_us = (now_us > self->last_rev_time_us_) ? (now_us - self->last_rev_time_us_) : 0ULL;
        if (dt_us > 0) {
          // RPM = (revs / dt[s]) * 60 = (revs * 60e6) / dt_us
          const double rpm = (static_cast<double>(revs) * 60000000.0) / static_cast<double>(dt_us);
          if (std::isfinite(rpm)) {
            self->last_calculated_rpm_.store(static_cast<float>(rpm), std::memory_order_relaxed);
            // PPS = pulzy / s = (revs * ppr) / dt[s] = (revs * ppr * 1e6) / dt_us
            const double pps =
                (static_cast<double>(revs) * static_cast<double>(ppr) * 1000000.0) / static_cast<double>(dt_us);
            self->last_calculated_pps_.store(static_cast<float>(pps), std::memory_order_relaxed);
            self->new_value_ready_.store(true, std::memory_order_release);
          }
        }
      }
      // posuň základní čas a nech si zbytek pulsů
      self->last_rev_time_us_ = now_us;
      const uint32_t leftover = now % ppr;
      self->pending_pulses_since_rev_.store(leftover, std::memory_order_relaxed);
    }
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

void HWPulseMeter::loop() {
  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

  // Idle: pokud dlouho nic nepřišlo, publikuj 0 (jen jednou, dokud nepřijde další pulz)
  if (this->idle_timeout_us_ > 0 && this->last_pulse_time_us_ != 0 && !this->idle_zero_published_) {
    const uint64_t dt = now_us - this->last_pulse_time_us_;
    if (dt >= this->idle_timeout_us_) {
      this->publish_state(0.0f);
      if (this->publish_pps_ && this->pps_sensor_ != nullptr) {
        this->pps_sensor_->publish_state(0.0f);
      }
      this->idle_zero_published_ = true;
    }
  }

  // Hlavní senzor (RPM) + PPS: publikuj, když timer spočítal novou hodnotu
  if (this->new_value_ready_.load(std::memory_order_acquire)) {
    this->new_value_ready_.store(false, std::memory_order_release);
    const float rpm = this->last_calculated_rpm_.load(std::memory_order_relaxed);
    if (std::isfinite(rpm)) {
      this->publish_state(rpm);
      if (this->publish_pps_ && this->pps_sensor_ != nullptr) {
        const float pps = this->last_calculated_pps_.load(std::memory_order_relaxed);
        if (std::isfinite(pps))
          this->pps_sensor_->publish_state(pps);
      }
    }
  }

  // TOTAL: publikuj při každém přírůstku pulzů
  if (this->publish_total_ && this->total_sensor_ != nullptr) {
    const int64_t delta = this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);
    if (delta > 0) {
      this->current_total_ += static_cast<uint64_t>(delta);
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
    }
  } else {
    (void) this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);
  }

  // REVOLUTIONS: publikuj při každé nové celé otáčce (odvozeno z current_total_/PPR)
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
  ESP_LOGCONFIG(TAG, "  PPR: %u", (unsigned) this->pulses_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Publishes: main=RPM, pps=%s, revolutions=%s, total=%s", this->publish_pps_ ? "yes" : "no",
                this->publish_revolutions_ ? "yes" : "no", this->publish_total_ ? "yes" : "no");
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
