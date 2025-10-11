#include "hw_pulse_meter.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

// <<< Import nového driveru pouze v .cpp, aby se nebil s legacy pcnt.h v jiných TU >>>
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <limits.h>

namespace esphome {
namespace hw_pulse_meter {

static const char *const TAG = "hw_pulse_meter";

void HWPulseMeter::setup() {
  ESP_LOGI(TAG, "Setting up HW Pulse Meter (pulse_cnt + GPIO ISR)...");
  if (pin_ == nullptr) {
    ESP_LOGE(TAG, "No pin configured");
    this->mark_failed();
    return;
  }

  // Respektuj YAML (input/pull/inverted)
  pin_->setup();
  const auto pin_num = pin_->get_pin();

  // ISR typ podle režimu
  gpio_set_intr_type(
      (gpio_num_t) pin_num,
      count_mode_ == BOTH ? GPIO_INTR_ANYEDGE
                          : (count_mode_ == RISING ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE));

  static bool isr_svc_installed = false;
  if (!isr_svc_installed) {
    gpio_install_isr_service(0);
    isr_svc_installed = true;
  }
  gpio_isr_handler_add((gpio_num_t) pin_num, &HWPulseMeter::gpio_isr_trampoline, this);

  if (!this->init_pcnt_()) {
    ESP_LOGE(TAG, "pulse_cnt init failed");
    this->mark_failed();
    return;
  }
  this->configure_pcnt_glitch_filter_();

  last_edge_us_ = 0;
  last_pub_us_ = 0;
  last_published_total_ = 0;

  int32_t start_total = 0;
  this->read_pcnt_total_(start_total);
  last_pcnt_total_ = start_total;

  ESP_LOGI(TAG, "Pulse counter unit created on GPIO %d, PPR=%u", pin_num, pulses_per_revolution_);
}

bool HWPulseMeter::init_pcnt_() {
  // 1) Vytvoř unit
  pcnt_unit_config_t unit_cfg{};
  // U nového driveru je pořadí polí low_limit, high_limit (vyplníme explicitně, bez designátorů):
  unit_cfg.low_limit  = INT16_MIN;
  unit_cfg.high_limit = INT16_MAX;

  pcnt_unit_handle_t unit_h = nullptr;
  if (pcnt_new_unit(&unit_cfg, &unit_h) != ESP_OK || unit_h == nullptr) {
    return false;
  }
  this->unit_ = unit_h;

  // 2) Vytvoř channel
  const int gpio_num = (int) pin_->get_pin();

  pcnt_chan_config_t ch_cfg{};
  ch_cfg.edge_gpio_num  = gpio_num;
  ch_cfg.level_gpio_num = -1; // nepoužíváme dir/ctrl pin
  // default actions; přepíšeme níže:
  ch_cfg.pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  ch_cfg.neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  ch_cfg.level_action    = PCNT_CHANNEL_LEVEL_ACTION_KEEP;

  pcnt_channel_handle_t ch_h = nullptr;
  if (pcnt_new_channel(unit_h, &ch_cfg, &ch_h) != ESP_OK || ch_h == nullptr) {
    return false;
  }
  this->channel_ = ch_h;

  // 3) Nastav akce podle režimu
  pcnt_chan_edge_action_t pos_act = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  pcnt_chan_edge_action_t neg_act = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  switch (count_mode_) {
    case RISING:
      pos_act = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
      neg_act = PCNT_CHANNEL_EDGE_ACTION_HOLD;
      break;
    case FALLING:
      pos_act = PCNT_CHANNEL_EDGE_ACTION_HOLD;
      neg_act = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
      break;
    case BOTH:
      pos_act = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
      neg_act = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
      break;
  }
  if (pcnt_channel_set_edge_action(ch_h, pos_act, neg_act) != ESP_OK) {
    return false;
  }
  // level_action zůstává KEEP

  // 4) Enable/Clear/Start
  if (pcnt_unit_enable(unit_h) != ESP_OK) return false;
  if (pcnt_unit_clear_count(unit_h) != ESP_OK) return false;
  if (pcnt_unit_start(unit_h) != ESP_OK) return false;

  return true;
}

void HWPulseMeter::configure_pcnt_glitch_filter_() {
  pcnt_unit_handle_t unit_h = reinterpret_cast<pcnt_unit_handle_t>(this->unit_);
  if (unit_h == nullptr) return;

  pcnt_glitch_filter_config_t gf{};
  // nový driver bere přímo ns; 0 = vypnuto
  gf.max_glitch_ns = (uint32_t)((uint64_t) glitch_filter_us_ * 1000ULL);

  esp_err_t err = pcnt_unit_set_glitch_filter(unit_h, &gf);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set glitch filter (err=%d)", (int) err);
  }
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
  if (!edge_flag_) return;
  edge_flag_ = false;

  int32_t total_now = 0;
  if (!this->read_pcnt_total_(total_now)) {
    ESP_LOGW(TAG, "pulse_cnt read failed");
    return;
  }

  int32_t delta_pulses = total_now - last_pcnt_total_;
  if (delta_pulses <= 0) return;
  last_pcnt_total_ = total_now;

  // Publish TOTAL podsenzor (kumulativně)
  if (publish_total_ && total_sensor_) total_sensor_->publish_state((float) total_now);

  // Publish až po celé otáčce (násobek PPR)
  if ((total_now - last_published_total_) < (int32_t) pulses_per_revolution_) return;

  const uint64_t now_us = esp_timer_get_time();
  const int32_t delta_since_pub = total_now - last_published_total_;
  last_published_total_ = total_now;

  float pps = NAN;
  if (last_pub_us_ != 0 && delta_since_pub > 0) {
    const float dt_s = (float) (now_us - last_pub_us_) / 1e6f;
    if (dt_s > 0.0f) pps = (float) delta_since_pub / dt_s;
  }
  last_pub_us_ = now_us;

  if (publish_pps_ && pps_sensor_ && !std::isnan(pps)) pps_sensor_->publish_state(pps);

  if (!std::isnan(pps)) {
    // Hlavní senzor = LPM
    this->publish_state(pps * 60.0f);
  }
}

void HWPulseMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "HW Pulse Meter (LPM primary, pulse_cnt driver; legacy-safe header):");
  if (pin_) ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  Count mode: %s",
                count_mode_ == RISING ? "RISING" :
                (count_mode_ == FALLING ? "FALLING" : "BOTH"));
  ESP_LOGCONFIG(TAG, "  Glitch filter (HW): %u us (~%u ns)",
                (unsigned) glitch_filter_us_, (unsigned) (glitch_filter_us_ * 1000u));
  ESP_LOGCONFIG(TAG, "  Min interval (soft): %u us", (unsigned) min_interval_us_);
  ESP_LOGCONFIG(TAG, "  Pulses per revolution (PPR): %u", (unsigned) pulses_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Subsensors: total=%s, pps=%s",
                publish_total_ ? "yes" : "no",
                publish_pps_ ? "yes" : "no");
}

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
