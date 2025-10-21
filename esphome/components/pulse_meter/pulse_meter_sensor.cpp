#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <inttypes.h>
#include <utility>

#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
#include "driver/gpio.h"
#include "esp_idf_version.h"
#endif

#if defined(SOC_PCNT_SUPPORTED)
#include "driver/pcnt.h"
#endif

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  this->total_pulses_ = pulses;
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();

  this->last_pin_val_ = this->pin_->digital_read();
  const uint32_t now = micros();
  this->last_processed_edge_us_ = now;

#if defined(SOC_PCNT_SUPPORTED)
  if (this->filter_mode_ == FILTER_EDGE) {
    if (this->setup_pcnt_edge_mode_()) {
      this->pcnt_active_ = true;
      this->hw_filter_active_ = (this->filter_us_ > 0);  // PCNT filtr je HW
      this->last_count_time_us_ = now;
      this->last_pulse_time_us_ = now;
    }
  }
#endif

  if (!this->pcnt_active_) {
    // GPIO glitch filter (pokud k dispozici), jen pro EDGE
#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
    if (this->filter_mode_ == FILTER_EDGE && this->filter_us_ > 0) {
      const int raw_pin = this->pin_->get_pin();
#if ESP_IDF_VERSION_MAJOR >= 5
      gpio_glitch_filter_handle_t h = nullptr;
      gpio_glitch_filter_config_t cfg = {};
      cfg.gpio_num = static_cast<gpio_num_t>(raw_pin);
      cfg.clk_src = GPIO_GLITCH_FILTER_CLK_SRC_DEFAULT;
      cfg.window_thres_ns = static_cast<uint32_t>(this->filter_us_) * 1000U;
      cfg.window_width_ns = 0;
      if (gpio_new_glitch_filter(&cfg, &h) == ESP_OK) {
        if (gpio_glitch_filter_enable(h) == ESP_OK) {
          this->glitch_filter_handle_ = h;
          this->hw_filter_active_ = true;
        }
      }
#endif
    }
#endif

    if (this->filter_mode_ == FILTER_EDGE) {
      this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    } else {
      this->pulse_state_.latched_ = this->last_pin_val_;
      this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
  }

  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::loop() {
#if defined(SOC_PCNT_SUPPORTED)
  if (this->pcnt_active_ && this->filter_mode_ == FILTER_EDGE) {
    this->loop_pcnt_edge_mode_();
    return;
  }
#endif

  {
    InterruptLock lock;
    std::swap(this->set_, this->get_);
    this->set_->count_ = 0;
  }

  const uint32_t now = micros();

  if (LIKELY(this->get_->count_ > 0)) {
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += this->get_->count_;
      this->total_sensor_->publish_state(this->total_pulses_);
    }

    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;

      case MeterState::RUNNING: {
        const uint32_t delta_us = us_since(this->get_->last_detected_edge_us_, this->last_processed_edge_us_);
        const float pulse_width_us = delta_us / static_cast<float>(this->get_->count_);
        ESP_LOGV(TAG, "New pulse, delta: %" PRIu32 " µs, count: %" PRIu32 ", width: %.5f µs", delta_us,
                 this->get_->count_, pulse_width_us);
        this->publish_state((60.0f * 1000000.0f) / pulse_width_us);
      } break;
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else {
    const uint32_t idle_us = us_since(now, this->last_processed_edge_us_);
    if (UNLIKELY(this->meter_state_ == MeterState::INITIAL || this->meter_state_ == MeterState::RUNNING)) {
      if (idle_us > this->timeout_us_) {
        this->meter_state_ = MeterState::TIMED_OUT;
        ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min", idle_us / 1000000U);
        this->publish_state(0.0f);
      }
    }
  }
}

float PulseMeterSensor::get_setup_priority() const { return setup_priority::DATA; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  const char *mode_str = (this->filter_mode_ == FILTER_EDGE) ? "EDGE" : "PULSE";
  ESP_LOGCONFIG(TAG, "  Mode: %s", mode_str);

#if defined(SOC_PCNT_SUPPORTED)
  if (this->pcnt_active_ && this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Backend: PCNT (HW)");
    ESP_LOGCONFIG(TAG, "  Glitch filter: HW (PCNT, %" PRIu32 " µs)", this->filter_us_);
  } else
#endif
  {
    const char *backend = (this->filter_mode_ == FILTER_EDGE) ? "GPIO IRQ" : "ANY_EDGE IRQ";
    ESP_LOGCONFIG(TAG, "  Backend: %s", backend);
    const char *ftype = (this->filter_mode_ == FILTER_EDGE && this->hw_filter_active_) ? "HW" : "SW";
    ESP_LOGCONFIG(TAG, "  Glitch filter: %s (%" PRIu32 " µs)", ftype, this->filter_us_);
  }

  ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " s", this->timeout_us_ / 1000000U);
  if (this->total_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Total pulses sensor linked");
  }
}

// ---------- ISR backend (fallback) ----------
void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  sensor->record_edge_(now);
  sensor->last_pin_val_ = true;
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();

  auto &ps = sensor->pulse_state_;
  auto &set = *sensor->set_;

  const bool enough = us_since(now, ps.last_intr_) >= sensor->filter_us_;
  if (enough && ps.latched_ && !sensor->last_pin_val_) {
    ps.latched_ = false;
  } else if (enough && !ps.latched_ && sensor->last_pin_val_) {
    ps.latched_ = true;
    set.last_detected_edge_us_ = ps.last_intr_;
    set.count_++;
  }

  set.last_rising_edge_us_ = (!ps.latched_ && pin_val) ? now : set.last_detected_edge_us_;
  ps.last_intr_ = now;
  sensor->last_pin_val_ = pin_val;
}

// ---------- PCNT backend (EDGE mode) ----------
#if defined(SOC_PCNT_SUPPORTED)

bool PulseMeterSensor::setup_pcnt_edge_mode_() {
  // najdi volnou PCNT jednotku
  int unit = -1;
  for (int u = 0; u < PCNT_UNIT_MAX; ++u) {
    // Neexistuje veřejné API na "is unit free", zkusíme prostě config a bereme první ESP_OK
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = this->pin_->get_pin();
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.unit = (pcnt_unit_t) u;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.pos_mode = PCNT_COUNT_INC;  // počítat RISING
    cfg.neg_mode = PCNT_COUNT_DIS;  // nepočítat FALLING
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = -32768;
    esp_err_t err = pcnt_unit_config(&cfg);
    if (err == ESP_OK) {
      unit = u;
      break;
    }
  }
  if (unit < 0) {
    ESP_LOGE(TAG, "PCNT: failed to allocate unit");
    return false;
  }

  this->pcnt_unit_ = unit;

  // inicializace čítače
  pcnt_counter_pause((pcnt_unit_t) this->pcnt_unit_);
  pcnt_counter_clear((pcnt_unit_t) this->pcnt_unit_);

  // HW glitch filtr (PCNT)
  if (this->filter_us_ > 0) {
    uint16_t ticks = pcnt_filter_ticks_(this->filter_us_);
    pcnt_set_filter_value((pcnt_unit_t) this->pcnt_unit_, ticks);
    pcnt_filter_enable((pcnt_unit_t) this->pcnt_unit_);
  } else {
    pcnt_filter_disable((pcnt_unit_t) this->pcnt_unit_);
  }

  // start
  pcnt_counter_resume((pcnt_unit_t) this->pcnt_unit_);

  // načti počáteční stav
  int16_t v = 0;
  pcnt_get_counter_value((pcnt_unit_t) this->pcnt_unit_, &v);
  this->pcnt_last_count_ = v;

  ESP_LOGD(TAG, "PCNT unit %d initialized on GPIO %d", this->pcnt_unit_, this->pin_->get_pin());
  return true;
}

void PulseMeterSensor::loop_pcnt_edge_mode_() {
  const uint32_t now = micros();

  int16_t v = 0;
  pcnt_get_counter_value((pcnt_unit_t) this->pcnt_unit_, &v);

  // delta s ošetřením wrapu 16b signed čítače
  int32_t delta = (int32_t) v - (int32_t) this->pcnt_last_count_;
  if (delta > 16384)
    delta -= 32768;  // wrap dolů -> reálný přírůstek
  if (delta < -16384)
    delta += 32768;  // wrap nahoru (neměl by nastat s INC only)

  if (delta > 0) {
    // total
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += (uint32_t) delta;
      this->total_sensor_->publish_state(this->total_pulses_);
    }

    // rychlost z delta/čas
    const uint32_t dt = us_since(now, this->last_count_time_us_);
    if (dt > 0) {
      const float ppm = (float) delta * (60.0f * 1000000.0f) / (float) dt;
      this->publish_state(ppm);
      this->meter_state_ = MeterState::RUNNING;
      this->last_processed_edge_us_ = now;
      this->last_pulse_time_us_ = now;
      this->last_count_time_us_ = now;
    }
  } else {
    // žádné nové pulzy → timeout?
    const uint32_t idle_us = us_since(now, this->last_pulse_time_us_);
    if (UNLIKELY(this->meter_state_ == MeterState::INITIAL || this->meter_state_ == MeterState::RUNNING)) {
      if (idle_us > this->timeout_us_) {
        this->meter_state_ = MeterState::TIMED_OUT;
        ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min", idle_us / 1000000U);
        this->publish_state(0.0f);
        this->last_count_time_us_ = now;  // reset měřicího intervalu
      }
    }
  }

  this->pcnt_last_count_ = v;
}

#endif  // SOC_PCNT_SUPPORTED

}  // namespace pulse_meter
}  // namespace esphome
