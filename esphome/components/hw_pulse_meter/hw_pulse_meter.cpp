#include "hw_pulse_meter.h"

#ifdef USE_ESP32
#include "esphome/core/log.h"

namespace esphome {
namespace hw_pulse_meter {

static const char *const TAG = "hw_pulse_meter";

void HWPulseMeter::setup() {
  ESP_LOGI(TAG, "Setting up HW Pulse Meter (PCNT + GPIO ISR)...");
  if (pin_ == nullptr) {
    ESP_LOGE(TAG, "No pin configured");
    this->mark_failed();
    return;
  }

  // Respektuj YAML pin nastavení (input/pull/inverted)
  pin_->setup();
  const auto pin_num = pin_->get_pin();

  // ISR typ podle režimu
  gpio_set_intr_type(
      (gpio_num_t) pin_num,
      count_mode_ == CountMode::BOTH ? GPIO_INTR_ANYEDGE
                                     : (count_mode_ == CountMode::RISING ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE));

  static bool isr_svc_installed = false;
  if (!isr_svc_installed) {
    gpio_install_isr_service(0);
    isr_svc_installed = true;
  }
  gpio_isr_handler_add((gpio_num_t) pin_num, &HWPulseMeter::gpio_isr_trampoline, this);

  if (!this->init_pcnt_()) {
    ESP_LOGE(TAG, "PCNT init failed");
    this->mark_failed();
    return;
  }
  this->configure_pcnt_glitch_filter_();

  last_edge_us_ = 0;
  last_pub_us_ = 0;

  int32_t start_total = 0;
  this->read_pcnt_total_(start_total);
  last_pcnt_total_ = start_total;

  ESP_LOGI(TAG, "Ready on GPIO %d (unit %d, channel %d)", pin_num, pcnt_unit_, (int) pcnt_channel_);
}

bool HWPulseMeter::init_pcnt_() {
#if defined(PCNT_UNIT_MAX)
  const int UNIT_MAX = PCNT_UNIT_MAX;
#else
  const int UNIT_MAX = 8;
#endif

  for (int unit = 0; unit < UNIT_MAX; unit++) {
    pcnt_config_t cfg{};
    cfg.pulse_gpio_num = (int) pin_->get_pin();
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;

    switch (count_mode_) {
      case CountMode::RISING:
        cfg.pos_mode = PCNT_COUNT_INC;
        cfg.neg_mode = PCNT_COUNT_DIS;
        break;
      case CountMode::FALLING:
        cfg.pos_mode = PCNT_COUNT_DIS;
        cfg.neg_mode = PCNT_COUNT_INC;
        break;
      case CountMode::BOTH:
        cfg.pos_mode = PCNT_COUNT_INC;
        cfg.neg_mode = PCNT_COUNT_INC;
        break;
    }
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;

    cfg.unit = (pcnt_unit_t) unit;
    cfg.channel = PCNT_CHANNEL_0;

    if (pcnt_unit_config(&cfg) == ESP_OK) {
      pcnt_unit_ = unit;
      pcnt_channel_ = PCNT_CHANNEL_0;

      // Vypnout eventy – čteme jen ručně
      pcnt_event_disable((pcnt_unit_t) pcnt_unit_, PCNT_EVT_THRES_0);
      pcnt_event_disable((pcnt_unit_t) pcnt_unit_, PCNT_EVT_THRES_1);
      pcnt_event_disable((pcnt_unit_t) pcnt_unit_, PCNT_EVT_L_LIM);
      pcnt_event_disable((pcnt_unit_t) pcnt_unit_, PCNT_EVT_H_LIM);
      pcnt_event_disable((pcnt_unit_t) pcnt_unit_, PCNT_EVT_ZERO);

      pcnt_counter_pause((pcnt_unit_t) pcnt_unit_);
      pcnt_counter_clear((pcnt_unit_t) pcnt_unit_);
      pcnt_counter_resume((pcnt_unit_t) pcnt_unit_);
      return true;
    }
  }
  return false;
}

void HWPulseMeter::configure_pcnt_glitch_filter_() {
  // PCNT filtr: hodnota je v APB tickách (~80 MHz). Často je rozsah 0..1023 (≈ 0..12.8us).
  if (glitch_filter_us_ == 0) {
    pcnt_filter_disable((pcnt_unit_t) pcnt_unit_);
    return;
  }
  uint64_t ticks64 = (uint64_t) glitch_filter_us_ * 80ULL;  // 1 us ~ 80 ticků
  if (ticks64 > 1023ULL) ticks64 = 1023ULL;                 // HW limit ≈ 1023
  uint16_t ticks = (uint16_t) ticks64;
  if (ticks == 0) {
    pcnt_filter_disable((pcnt_unit_t) pcnt_unit_);
  } else {
    pcnt_set_filter_value((pcnt_unit_t) pcnt_unit_, ticks);
    pcnt_filter_enable((pcnt_unit_t) pcnt_unit_);
  }
}

bool HWPulseMeter::read_pcnt_total_(int32_t &out) {
  int16_t cnt = 0;
  if (pcnt_get_counter_value((pcnt_unit_t) pcnt_unit_, &cnt) != ESP_OK) return false;
  out = (int32_t) cnt;
  return true;
}

void HWPulseMeter::loop() {
  if (!edge_flag_) return;
  edge_flag_ = false;

  int32_t total_now = 0;
  if (!this->read_pcnt_total_(total_now)) {
    ESP_LOGW(TAG, "PCNT read failed");
    return;
  }

  int32_t delta = total_now - last_pcnt_total_;
  if (delta > 0) {
    last_pcnt_total_ = total_now;
  }

  const uint64_t now = esp_timer_get_time();  // us
  float pps = NAN;
  if (last_pub_us_ != 0) {
    const uint64_t dt = now - last_pub_us_;
    if (dt > 0 && delta > 0) {
      pps = (float) delta * (1000000.0f / (float) dt);
    }
  }
  last_pub_us_ = now;

  // Publikace podsenzorů
  if (publish_total_ && total_sensor_) total_sensor_->publish_state((float) last_pcnt_total_);
  if (publish_pps_ && pps_sensor_ && !std::isnan(pps)) pps_sensor_->publish_state(pps);

  // Hlavní senzor = LPM
  if (!std::isnan(pps)) {
    this->publish_state(pps * 60.0f);
  }
}

void HWPulseMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "HW Pulse Meter (LPM primary):");
  if (pin_) ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  Count mode: %s",
                count_mode_ == CountMode::RISING ? "RISING" :
                (count_mode_ == CountMode::FALLING ? "FALLING" : "BOTH"));
  ESP_LOGCONFIG(TAG, "  Glitch filter (HW): %u us (capped ~12.8us)", (unsigned) glitch_filter_us_);
  ESP_LOGCONFIG(TAG, "  Min interval (soft debounce): %u us", (unsigned) min_interval_us_);
  ESP_LOGCONFIG(TAG, "  Subsensors: total=%s, pps=%s",
                publish_total_ ? "yes" : "no",
                publish_pps_ ? "yes" : "no");
}

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
