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
  if (this->total_sensor_ != nullptr)
    this->total_sensor_->publish_state(this->total_pulses_);
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
      this->last_count_time_us_ = now;
      this->last_pulse_time_us_ = now;
#if defined(SOC_GPIO_SUPPORT_GLITCH_FILTER)
      if (this->filter_us_ > 0) {
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
    }
  }
#endif

  if (!this->pcnt_active_) {
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
    const uint32_t now = micros();

    int16_t v = 0;
    pcnt_get_counter_value((pcnt_unit_t) this->pcnt_unit_, &v);
    int32_t delta = (int32_t) v - (int32_t) this->pcnt_last_count_;
    if (delta > 16384)
      delta -= 32768;
    if (delta < -16384)
      delta += 32768;

    if (delta > 0) {
      if (this->total_sensor_ != nullptr) {
        this->total_pulses_ += (uint32_t) delta;
        this->total_sensor_->publish_state(this->total_pulses_);
      }
      this->last_pulse_time_us_ = now;
    }

    // Odhad frekvence pro rozhodnutí o režimu (při delta>0 použij delta/dt, jinak drž poslední rozhodnutí)
    float est_hz = -1.0f;
    const uint32_t dt_est = us_since(now, this->last_count_time_us_);
    if (delta > 0 && dt_est > 0) {
      est_hz = (float) delta * (1000000.0f / (float) dt_est);
      this->last_count_time_us_ = now;
    }

    // Přepínání režimu s hysterezí
    if (!this->period_mode_active_) {
      if ((est_hz > 0.0f && est_hz <= this->switch_off_hz_) || delta == 0) {
        this->enable_period_mode_isr_();
      }
    } else {
      if (est_hz > this->switch_on_hz_) {
        this->disable_period_mode_isr_();
      }
    }

    // PERIOD režim: publikace z přesné periody měřené ISR
    if (this->period_mode_active_) {
      bool publish = false;
      uint32_t per = 0;
      {
        InterruptLock lk;
        if (this->period_new_) {
          per = this->period_us_;
          this->period_new_ = false;
          publish = true;
        }
      }
      if (publish && per > 0) {
        const float ppm = (60.0f * 1000000.0f) / (float) per;
        this->publish_state(ppm);
        this->meter_state_ = MeterState::RUNNING;
        this->last_processed_edge_us_ = now;
      }
      // Timeout (když dlouho žádná hrana)
      const uint32_t idle_us = us_since(now, this->last_pulse_time_us_);
      if (UNLIKELY(this->meter_state_ == MeterState::INITIAL || this->meter_state_ == MeterState::RUNNING)) {
        if (idle_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min", idle_us / 1000000U);
          this->publish_state(0.0f);
        }
      }

      // Ulož poslední hodnotu čítače
      this->pcnt_last_count_ = v;
      return;
    }

    // RATE režim: delta/dt
    if (delta > 0)
      this->loop_pcnt_edge_mode_rate_(now, delta);

    this->pcnt_last_count_ = v;
    return;
  }
#endif

  // Fallback (bez PCNT): ISR vede hrany do double-bufferu, loop jen čte snapshot
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
    ESP_LOGCONFIG(TAG, "  Backend: PCNT (HW), hybrid with PERIOD for low Hz");
    const char *ftype = this->hw_filter_active_ ? "HW" : "SW";
    ESP_LOGCONFIG(TAG, "  Glitch filter: %s (%" PRIu32 " µs)", ftype, this->filter_us_);
    ESP_LOGCONFIG(TAG, "  Switch: PERIOD if <= %.1f Hz, RATE if >= %.1f Hz", this->switch_off_hz_, this->switch_on_hz_);
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

// ---------- PCNT: RATE větev (delta/dt pro vyšší Hz) ----------
#if defined(SOC_PCNT_SUPPORTED)
void PulseMeterSensor::loop_pcnt_edge_mode_rate_(uint32_t now, int32_t delta) {
  const uint32_t dt = us_since(now, this->last_processed_edge_us_);
  if (dt == 0)
    return;
  const float ppm = (float) delta * (60.0f * 1000000.0f) / (float) dt;
  this->publish_state(ppm);
  this->meter_state_ = MeterState::RUNNING;
  this->last_processed_edge_us_ = now;
}
#endif

// ---------- ISR fallback / PULSE mód ----------
void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();

  // PERIOD mód pro nízké frekvence (aktivní jen když period_mode_active_ = true)
  if (sensor->period_mode_active_) {
    uint32_t last = sensor->last_rise_us_;
    sensor->last_rise_us_ = now;
    if (last != 0) {
      uint32_t per = us_since(now, last);
      sensor->period_us_ = per;
      sensor->period_new_ = true;
    }
    sensor->last_pin_val_ = true;
    return;
  }

  // Jinak fallback EDGE: zaznamenej hranu do bufferu
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

// ---------- PCNT init ----------
#if defined(SOC_PCNT_SUPPORTED)
bool PulseMeterSensor::setup_pcnt_edge_mode_() {
  int unit = -1;
  for (int u = 0; u < PCNT_UNIT_MAX; ++u) {
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = this->pin_->get_pin();
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.unit = (pcnt_unit_t) u;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.pos_mode = PCNT_COUNT_INC;
    cfg.neg_mode = PCNT_COUNT_DIS;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = -32768;
    if (pcnt_unit_config(&cfg) == ESP_OK) {
      unit = u;
      break;
    }
  }
  if (unit < 0) {
    ESP_LOGE(TAG, "PCNT: failed to allocate unit");
    return false;
  }

  this->pcnt_unit_ = unit;

  pcnt_counter_pause((pcnt_unit_t) this->pcnt_unit_);
  pcnt_counter_clear((pcnt_unit_t) this->pcnt_unit_);

  if (this->filter_us_ > 0) {
    uint64_t t = (uint64_t) this->filter_us_ * 80ULL;
    uint16_t ticks = (t > 1023ULL) ? 1023U : (uint16_t) t;
    pcnt_set_filter_value((pcnt_unit_t) this->pcnt_unit_, ticks);
    pcnt_filter_enable((pcnt_unit_t) this->pcnt_unit_);
  } else {
    pcnt_filter_disable((pcnt_unit_t) this->pcnt_unit_);
  }

  pcnt_counter_resume((pcnt_unit_t) this->pcnt_unit_);

  int16_t v = 0;
  pcnt_get_counter_value((pcnt_unit_t) this->pcnt_unit_, &v);
  this->pcnt_last_count_ = v;

  ESP_LOGD(TAG, "PCNT unit %d initialized on GPIO %d", this->pcnt_unit_, this->pin_->get_pin());
  return true;
}
#endif

// ---------- PERIOD mód pomocné ----------
void PulseMeterSensor::enable_period_mode_isr_() {
  if (this->period_mode_active_)
    return;
  this->period_mode_active_ = true;
  this->last_rise_us_ = 0;
  this->period_new_ = false;
  this->period_us_ = 0;
  this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
}

void PulseMeterSensor::disable_period_mode_isr_() {
  if (!this->period_mode_active_)
    return;
  this->period_mode_active_ = false;
  // V PCNT režimu ISR nepotřebujeme – zrušíme
  this->pin_->detach_interrupt();
  // Připojíme ISR jen pokud běží fallback (bez PCNT) nebo PULSE (to řeší setup/loop)
}

inline void PulseMeterSensor::pulse_mode_edge_(uint32_t now, bool pin_val) {
  auto &ps = this->pulse_state_;
  auto &set = *this->set_;
  const bool enough = us_since(now, ps.last_intr_) >= this->filter_us_;
  if (enough && ps.latched_ && !this->last_pin_val_) {
    ps.latched_ = false;
  } else if (enough && !ps.latched_ && this->last_pin_val_) {
    ps.latched_ = true;
    set.last_detected_edge_us_ = ps.last_intr_;
    set.count_++;
  }
  set.last_rising_edge_us_ = (!ps.latched_ && pin_val) ? now : set.last_detected_edge_us_;
}
}  // namespace pulse_meter
}  // namespace esphome
