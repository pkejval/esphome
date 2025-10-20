#include "hw_pulse_meter.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <limits>
#include <cmath>

namespace esphome {
namespace hw_pulse_meter {

static const char *const TAG = "hw_pulse_meter";

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
      return PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_HOLD,
             PCNT_CHANNEL_EDGE_ACTION_HOLD;  // never used
    case FALLING:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case BOTH:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}

void HWPulseMeter::setup() {
  if (pin_ == nullptr) {
    this->mark_failed();
    return;
  }
  pin_->setup();

  if (!this->init_pcnt_()) {
    this->mark_failed();
    return;
  }
  this->apply_glitch_filter_();

  // fronta pro časové značky celých otáček
  evt_queue_ = xQueueCreate(16, sizeof(uint64_t));
  if (evt_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event queue");
    this->mark_failed();
    return;
  }

  // watchpoint = PPR
  const int watch_val = static_cast<int>(this->pulses_per_revolution_);
  if (pcnt_unit_add_watch_point(this->unit_, watch_val) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_unit_add_watch_point(%d) failed", watch_val);
    this->mark_failed();
    return;
  }
  (void) pcnt_unit_enable_event(this->unit_, PCNT_EVENT_REACH);

  pcnt_event_callbacks_t cbs{};
  cbs.on_reach = &HWPulseMeter::on_reach_isr_;
  if (pcnt_unit_register_event_callbacks(this->unit_, &cbs, this) != ESP_OK) {
    ESP_LOGE(TAG, "pcnt_unit_register_event_callbacks failed");
    this->mark_failed();
    return;
  }

  last_rev_time_us_ = 0;
  last_event_time_us_ = 0;
  idle_zero_sent_ = false;
  current_total_pulses_ = 0;
  current_total_revs_ = 0;
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
  pcnt_glitch_filter_config_t gf{};
  gf.max_glitch_ns = static_cast<uint64_t>(this->internal_filter_us_applied_) * 1000ULL;
  (void) pcnt_unit_set_glitch_filter(this->unit_, &gf);
}

bool IRAM_ATTR HWPulseMeter::on_reach_isr_(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t * /*edata*/,
                                           void *user_data) {
  auto *self = static_cast<HWPulseMeter *>(user_data);
  if (self == nullptr)
    return false;

  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());
  BaseType_t hpw = pdFALSE;
  (void) xQueueSendFromISR(self->evt_queue_, &t, &hpw);

  // rearm – začneme další otáčku od nuly
  (void) pcnt_unit_clear_count(unit);

  return hpw == pdTRUE;
}

bool HWPulseMeter::read_and_clear_pcnt_(int32_t &out) {
  // V event módu PCNT nečteme periodicky, ale ponecháme utilitu pro případné rozšíření.
  if (this->unit_ == nullptr)
    return false;
  int v = 0;
  if (pcnt_unit_get_count(this->unit_, &v) != ESP_OK)
    return false;
  out = static_cast<int32_t>(v);
  (void) pcnt_unit_clear_count(this->unit_);
  return true;
}

void HWPulseMeter::loop() {
  // Zpracuj všechny doručené celé otáčky
  uint64_t t_us = 0;
  bool any = false;
  while (xQueueReceive(this->evt_queue_, &t_us, 0) == pdTRUE) {
    any = true;
    const uint64_t now_us = t_us;

    // RPM/PPS z rozdílu času dvou po sobě jdoucích otáček
    if (this->last_rev_time_us_ != 0 && now_us > this->last_rev_time_us_) {
      const double dt_s = static_cast<double>(now_us - this->last_rev_time_us_) / 1e6;
      if (dt_s > 0.0) {
        const double rpm = (1.0 / dt_s) * 60.0;
        this->publish_state(static_cast<float>(rpm));

        if (this->publish_pps_ && this->pps_sensor_ != nullptr) {
          const double pps = static_cast<double>(this->pulses_per_revolution_) / dt_s;
          this->pps_sensor_->publish_state(static_cast<float>(pps));
        }
      }
    }

    this->last_rev_time_us_ = now_us;
    this->last_event_time_us_ = now_us;
    this->idle_zero_sent_ = false;

    // Revoluce a total
    this->current_total_revs_ += 1;
    this->current_total_pulses_ += this->pulses_per_revolution_;

    if (this->publish_revolutions_ && this->revolutions_sensor_ != nullptr) {
      this->revolutions_sensor_->publish_state(static_cast<float>(this->current_total_revs_));
    }
    if (this->publish_total_ && this->total_sensor_ != nullptr) {
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_pulses_));
    }
  }

  // Idle: pokud dlouho nepřišla otáčka, publikuj 0 (jen jednou)
  if (!any && this->idle_timeout_us_ > 0 && this->last_event_time_us_ != 0 && !this->idle_zero_sent_) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    if ((now - this->last_event_time_us_) >= this->idle_timeout_us_) {
      this->publish_state(0.0f);
      if (this->publish_pps_ && this->pps_sensor_ != nullptr)
        this->pps_sensor_->publish_state(0.0f);
      this->idle_zero_sent_ = true;
    }
  }
}

void HWPulseMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "HW Pulse Meter (PCNT watchpoint)");
  if (this->pin_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", this->pin_->get_pin());
  const char *mode_str = (this->count_mode_ == RISING) ? "RISING" : (this->count_mode_ == FALLING) ? "FALLING" : "BOTH";
  ESP_LOGCONFIG(TAG, "  Count mode: %s", mode_str);
  ESP_LOGCONFIG(TAG, "  Internal filter (requested/applied): %u us / %u us",
                (unsigned) this->internal_filter_us_requested_, (unsigned) this->internal_filter_us_applied_);
  ESP_LOGCONFIG(TAG, "  PPR: %u", (unsigned) this->pulses_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Idle timeout: %u us", (unsigned) this->idle_timeout_us_);
  ESP_LOGCONFIG(TAG, "  Subsensors: total=%s, pps=%s, revolutions=%s", this->publish_total_ ? "yes" : "no",
                this->publish_pps_ ? "yes" : "no", this->publish_revolutions_ ? "yes" : "no");
}

HWPulseMeter::~HWPulseMeter() {
  if (this->unit_ != nullptr)
    (void) pcnt_unit_stop(this->unit_);
  if (this->channel_ != nullptr) {
    (void) pcnt_del_channel(this->channel_);
    this->channel_ = nullptr;
  }
  if (this->unit_ != nullptr) {
    (void) pcnt_del_unit(this->unit_);
    this->unit_ = nullptr;
  }
  if (this->evt_queue_ != nullptr) {
    vQueueDelete(this->evt_queue_);
    this->evt_queue_ = nullptr;
  }
}

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
