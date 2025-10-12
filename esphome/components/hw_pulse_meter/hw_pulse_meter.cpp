#include "hw_pulse_meter.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <driver/pulse_cnt.h>
#include <limits.h>

namespace esphome {
namespace hw_pulse_meter {

static const char *const TAG = "hw_pulse_meter";

void HWPulseMeter::setup() {
  if (pin_ == nullptr) { this->mark_failed(); return; }

  pin_->setup();

  if (!this->init_pcnt_()) { this->mark_failed(); return; }
  this->configure_pcnt_internal_filter_();

  last_change_us_ = 0;
  last_pub_us_ = 0;
  cumulative_total_ = 0;
  last_published_total_ = 0;
  last_revolutions_pub_ = 0;
  idle_zero_published_ = false;

  int32_t start_raw = 0;
  if (this->read_pcnt_total_(start_raw)) last_pcnt_total_raw_ = start_raw;
  else last_pcnt_total_raw_ = 0;
}

bool HWPulseMeter::init_pcnt_() {
  pcnt_unit_config_t unit_cfg{};
  unit_cfg.low_limit  = INT16_MIN;
  unit_cfg.high_limit = INT16_MAX;

  pcnt_unit_handle_t unit_h = nullptr;
  if (pcnt_new_unit(&unit_cfg, &unit_h) != ESP_OK || unit_h == nullptr) return false;
  this->unit_ = unit_h;

  const int gpio_num = (int) pin_->get_pin();

  pcnt_chan_config_t ch_cfg{};
  ch_cfg.edge_gpio_num  = gpio_num;
  ch_cfg.level_gpio_num = -1;

  pcnt_channel_handle_t ch_h = nullptr;
  if (pcnt_new_channel(unit_h, &ch_cfg, &ch_h) != ESP_OK || ch_h == nullptr) return false;
  this->channel_ = ch_h;

  pcnt_channel_edge_action_t pos_act =
      (count_mode_ != FALLING) ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_HOLD;
  pcnt_channel_edge_action_t neg_act =
      (count_mode_ != RISING)  ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_HOLD;

  if (pcnt_channel_set_edge_action(ch_h, pos_act, neg_act) != ESP_OK) return false;

  if (pcnt_unit_enable(unit_h) != ESP_OK) return false;
  if (pcnt_unit_clear_count(unit_h) != ESP_OK) return false;
  if (pcnt_unit_start(unit_h) != ESP_OK) return false;

  return true;
}

void HWPulseMeter::configure_pcnt_internal_filter_() {
  pcnt_unit_handle_t unit_h = reinterpret_cast<pcnt_unit_handle_t>(this->unit_);
  if (unit_h == nullptr) return;

  uint32_t clamped_us = internal_filter_us_;
  if (clamped_us > 13) clamped_us = 13;
  pcnt_glitch_filter_config_t gf{};
  gf.max_glitch_ns = (uint32_t)((uint64_t) clamped_us * 1000ULL);
  (void) pcnt_unit_set_glitch_filter(unit_h, &gf);
}

bool HWPulseMeter::read_pcnt_total_(int32_t &out) {
  pcnt_unit_handle_t unit_h = reinterpret_cast<pcnt_unit_handle_t>(this->unit_);
  if (unit_h == nullptr) return false;
  int value = 0;
  if (pcnt_unit_get_count(unit_h, &value) != ESP_OK) return false;
  out = (int32_t) value;
  return true;
}

void HWPulseMeter::loop() {
  const uint64_t now_us = esp_timer_get_time();

  if (idle_timeout_us_ > 0 && last_change_us_ != 0) {
    if ((now_us - last_change_us_) >= idle_timeout_us_ && !idle_zero_published_) {
      if (publish_pps_ && pps_sensor_) pps_sensor_->publish_state(0.0f);
      this->publish_state(0.0f);
      idle_zero_published_ = true;
      last_pub_us_ = now_us;
      last_published_total_ = cumulative_total_;
    }
  }

  int32_t total_now_raw = 0;
  if (!this->read_pcnt_total_(total_now_raw)) return;

  const uint16_t delta_u16 =
      static_cast<uint16_t>(static_cast<uint16_t>(total_now_raw) - static_cast<uint16_t>(last_pcnt_total_raw_));
  if (delta_u16 == 0) return;

  last_pcnt_total_raw_ = total_now_raw;
  cumulative_total_ += static_cast<uint32_t>(delta_u16);
  idle_zero_published_ = false;
  last_change_us_ = now_us;

  if (publish_total_ && total_sensor_) total_sensor_->publish_state(static_cast<float>(cumulative_total_));

  if (publish_revolutions_ && revolutions_sensor_) {
    const uint64_t revs_now = cumulative_total_ / pulses_per_revolution_;
    if (revs_now != last_revolutions_pub_) {
      last_revolutions_pub_ = revs_now;
      revolutions_sensor_->publish_state(static_cast<float>(revs_now));
    }
  }

  const uint64_t since_pub = cumulative_total_ - last_published_total_;
  if (since_pub < pulses_per_revolution_) return;

  if (last_pub_us_ != 0) {
    const float dt_s = float(now_us - last_pub_us_) / 1e6f;
    if (dt_s > 0.0f) {
      const float pps = float(since_pub) / dt_s;
      if (publish_pps_ && pps_sensor_) pps_sensor_->publish_state(pps);

      const float rps = (float(since_pub) / (float) pulses_per_revolution_) / dt_s;
      this->publish_state(rps * 60.0f);
    }
  }

  last_pub_us_ = now_us;
  last_published_total_ = cumulative_total_;
}

void HWPulseMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "HW Pulse Meter (pulse_cnt)");
  if (pin_) ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  Count mode: %s",
                count_mode_ == RISING ? "RISING" : (count_mode_ == FALLING ? "FALLING" : "BOTH"));
  ESP_LOGCONFIG(TAG, "  Internal filter: %u us", (unsigned) internal_filter_us_);
  ESP_LOGCONFIG(TAG, "  Idle timeout: %u us", (unsigned) idle_timeout_us_);
  ESP_LOGCONFIG(TAG, "  PPR: %u", (unsigned) pulses_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Subsensors: total=%s, pps=%s, revolutions=%s",
                publish_total_ ? "yes" : "no",
                publish_pps_ ? "yes" : "no",
                publish_revolutions_ ? "yes" : "no");
}

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
