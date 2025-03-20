#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <atomic>

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

#ifdef USE_ESP32
// Define appropriate PCNT unit based on availability
pcnt_unit_t get_next_pcnt_unit() {
  static uint8_t next_pcnt_unit = 0;
  return static_cast<pcnt_unit_t>(next_pcnt_unit++ % PCNT_UNIT_MAX);
}
#endif

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  atomic_update_pulses(pulses);
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_.load(std::memory_order_acquire));
  }
}

esp_err_t PulseMeterSensor::atomic_update_pulses(uint32_t new_pulses) {
  uint32_t expected = total_pulses_.load(std::memory_order_relaxed);
  while (!total_pulses_.compare_exchange_weak(expected, new_pulses,
                                             std::memory_order_release,
                                             std::memory_order_relaxed)) {
    // Keep trying until successful
  }
  return ESP_OK;
}

void PulseMeterSensor::setup() {
  esp_err_t err = ESP_OK;
  
  // Initialize pulse mutex
  #ifdef USE_ESP32
  this->pulse_mutex_ = xSemaphoreCreateMutex();
  if (this->pulse_mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create pulse mutex");
    this->mark_failed();
    return;
  }
  #endif
  
  // Configure pin
  err = this->pin_->setup();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "GPIO setup failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  
  this->isr_pin_ = pin_->to_isr();
  esp_task_wdt_add(nullptr);
  this->last_processed_edge_us_ = (uint32_t)esp_timer_get_time();

  #ifdef USE_ESP32
  if (this->filter_mode_ == FILTER_PCNT) {
    this->setup_pcnt();
  } else 
  #endif
  if (this->filter_mode_ == FILTER_EDGE) {
    err = this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to attach edge interrupt: %s", esp_err_to_name(err));
      this->mark_failed();
      return;
    }
  } else if (this->filter_mode_ == FILTER_PULSE) {
    this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
    this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
    err = this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to attach pulse interrupt: %s", esp_err_to_name(err));
      this->mark_failed();
      return;
    }
  }
  
  #ifdef USE_ESP32
  // Create dedicated processing task
  if (this->use_core_pinning_) {
    // Pin pulse processing to core 1, leaving core 0 for networking
    xTaskCreatePinnedToCore(
        PulseMeterSensor::processing_task,
        "pulse_proc",
        4096,
        this,
        5,  // Higher priority
        &this->task_handle_,
        1    // Core 1 for processing
    );
    
    // Optimize WiFi for reduced interference with pulse measurement
    esp_wifi_set_ps(WIFI_PS_NONE);
  }
  #endif
}

#ifdef USE_ESP32
void PulseMeterSensor::setup_pcnt() {
  this->pcnt_unit_ = get_next_pcnt_unit();
  this->using_pcnt_ = true;
  
  // Configure PCNT unit
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = static_cast<int>(this->pin_->get_pin()),
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,   // Count rising edges
    .neg_mode = PCNT_COUNT_DIS,   // Ignore falling edges
    .counter_h_lim = 32767,
    .counter_l_lim = 0,
    .unit = this->pcnt_unit_,
    .channel = PCNT_CHANNEL_0,
  };
  
  esp_err_t err = pcnt_unit_config(&pcnt_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "PCNT config failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  
  // Set up filter
  if (this->filter_us_ > 0) {
    // PCNT filter is in APB cycles (80MHz)
    uint16_t filter_val = this->filter_us_ * 80; // Convert μs to APB cycles
    if (filter_val > 1023) filter_val = 1023;    // Max filter value
    err = pcnt_set_filter_value(this->pcnt_unit_, filter_val);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "PCNT filter config failed: %s", esp_err_to_name(err));
    }
    err = pcnt_filter_enable(this->pcnt_unit_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "PCNT filter enable failed: %s", esp_err_to_name(err));
    }
  }
  
  // Clear counter
  err = pcnt_counter_clear(this->pcnt_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "PCNT counter clear failed: %s", esp_err_to_name(err));
  }
  
  // Start counting
  err = pcnt_counter_resume(this->pcnt_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "PCNT counter resume failed: %s", esp_err_to_name(err));
  }
}

void IRAM_ATTR PulseMeterSensor::pcnt_intr(void *arg) {
  // This implementation is placeholder - PCNT interrupts would be used
  // for overflow handling in a full implementation
}

void PulseMeterSensor::processing_task(void *arg) {
  PulseMeterSensor *sensor = static_cast<PulseMeterSensor*>(arg);
  
  // Configure high-priority task for pulse processing
  esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
  
  while (true) {
    if (xSemaphoreTake(sensor->pulse_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Process pulses from ISR
      // (Most processing remains in loop() for compatibility)
      xSemaphoreGive(sensor->pulse_mutex_);
    }
    
    // Yield to allow other tasks to run
    vTaskDelay(1);
  }
}
#endif

void PulseMeterSensor::loop() {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  
  #ifdef USE_ESP32
  // If we're using hardware pulse counter, read and process it
  if (this->using_pcnt_) {
    int16_t count;
    if (pcnt_get_counter_value(this->pcnt_unit_, &count) == ESP_OK) {
      if (count > 0) {
        State new_state;
        new_state.count_ = count;
        new_state.last_detected_edge_us_ = now;
        new_state.last_rising_edge_us_ = now;
        
        // Clear the counter for next reading
        pcnt_counter_clear(this->pcnt_unit_);
        
        // Update total pulse count
        if (this->total_sensor_ != nullptr) {
          uint32_t current_total = this->total_pulses_.load(std::memory_order_relaxed);
          atomic_update_pulses(current_total + count);
          this->total_sensor_->publish_state(this->total_pulses_.load(std::memory_order_acquire));
        }
        
        if (this->meter_state_ == MeterState::INITIAL || 
            this->meter_state_ == MeterState::TIMED_OUT) {
          this->meter_state_ = MeterState::RUNNING;
        } else if (this->meter_state_ == MeterState::RUNNING) {
          uint32_t delta_us = now - this->last_processed_edge_us_;
          
          // Use fixed-point math for better performance
          uint32_t scaled_width = (delta_us * PULSE_METER_FIXED_POINT_SCALE) / count;
          uint32_t scaled_ppm = (60ULL * 1000000ULL * PULSE_METER_FIXED_POINT_SCALE) / scaled_width;
          float pulses_per_minute = (float)scaled_ppm / PULSE_METER_FIXED_POINT_SCALE;
          
          ESP_LOGV(TAG, "PCNT pulses: %d, delta: %u µs, ppm: %.2f", 
                  count, delta_us, pulses_per_minute);
          
          this->publish_state(pulses_per_minute);
        }
        
        this->last_processed_edge_us_ = now;
      } else {
        // Check for timeout
        uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
        if ((this->meter_state_ == MeterState::INITIAL || 
             this->meter_state_ == MeterState::RUNNING) &&
            time_since_valid_edge_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %u s, assuming 0 pulses/min", 
                  time_since_valid_edge_us / 1000000);
          this->publish_state(0.0f);
        }
      }
    }
    
    esp_task_wdt_reset();
    return;
  }
  #endif
  
  // Software-based pulse counting (legacy approach)
  this->get_->count_ = 0;
  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

  if (this->peeked_edge_ && this->get_->count_ > 0) {
    this->peeked_edge_ = false;
    this->get_->count_ = this->get_->count_ - 1;
  }

  if ((this->get_->last_rising_edge_us_ != this->get_->last_detected_edge_us_) &&
      (now - this->get_->last_rising_edge_us_ >= this->filter_us_)) {
    this->peeked_edge_ = true;
    this->get_->last_detected_edge_us_ = this->get_->last_rising_edge_us_;
    this->get_->count_ = this->get_->count_ + 1;
  }

  if (this->get_->count_ > 0) {
    if (this->total_sensor_ != nullptr) {
      uint32_t current_total = this->total_pulses_.load(std::memory_order_relaxed);
      atomic_update_pulses(current_total + this->get_->count_);
      uint32_t total = this->total_pulses_.load(std::memory_order_acquire);
      this->total_sensor_->publish_state(total);
    }

    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;
      case MeterState::RUNNING: {
        uint32_t delta_us = this->get_->last_detected_edge_us_ - this->last_processed_edge_us_;
        
        // Use fixed-point math for better performance
        uint32_t scaled_width = (delta_us * PULSE_METER_FIXED_POINT_SCALE) / this->get_->count_;
        uint32_t scaled_ppm = (60ULL * 1000000ULL * PULSE_METER_FIXED_POINT_SCALE) / scaled_width;
        float pulses_per_minute = (float)scaled_ppm / PULSE_METER_FIXED_POINT_SCALE;
        
        ESP_LOGV(TAG, "New pulse, delta: %u µs, count: %u, width: %.5f µs",
                delta_us, this->get_->count_, (float)delta_us / this->get_->count_);
        
        this->publish_state(pulses_per_minute);
      } break;
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else {
    uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::RUNNING:
        if (time_since_valid_edge_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %u s, assuming 0 pulses/min", 
                  time_since_valid_edge_us / 1000000);
          this->publish_state(0.0f);
        }
        break;
      default:
        break;
    }
  }

  esp_task_wdt_reset();
}

float PulseMeterSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  
  #ifdef USE_ESP32
  if (this->filter_mode_ == FILTER_PCNT) {
    ESP_LOGCONFIG(TAG, "  Using ESP32 hardware pulse counter (PCNT Unit %d)", this->pcnt_unit_);
    ESP_LOGCONFIG(TAG, "  Filter: %u µs", this->filter_us_);
  } else 
  #endif
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %u µs apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %u µs", this->filter_us_);
  }
  
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %u s", 
                this->timeout_us_ / 1000000);
  
  #ifdef USE_ESP32
  ESP_LOGCONFIG(TAG, "  Core pinning: %s", this->use_core_pinning_ ? "enabled" : "disabled");
  #endif
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  auto &state = sensor->edge_state_;
  auto &set = *sensor->set_;
  
  if ((now - state.last_sent_edge_us_) >= sensor->filter_us_) {
    state.last_sent_edge_us_ = now;
    set.last_detected_edge_us_ = now;
    set.last_rising_edge_us_ = now;
    set.count_ = set.count_ + 1;
  }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &state = sensor->pulse_state_;
  auto &set = *sensor->set_;
  
  bool length = (now - state.last_intr_) >= sensor->filter_us_;
  
  if (length && state.latched_ && !state.last_pin_val_) {
    state.latched_ = false;
  } else if (length && !state.latched_ && state.last_pin_val_) {
    state.latched_ = true;
    set.last_detected_edge_us_ = state.last_intr_;
    set.count_ = set.count_ + 1;
  }
  
  set.last_rising_edge_us_ = (!state.latched_ && pin_val) ? now : set.last_detected_edge_us_;
  state.last_intr_ = now;
  state.last_pin_val_ = pin_val;
}

} // namespace pulse_meter
} // namespace esphome
