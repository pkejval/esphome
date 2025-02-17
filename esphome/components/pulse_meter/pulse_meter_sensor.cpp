// pulse_meter_sensor.cpp
#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

// Initialize static member
PulseMeterSensor *PulseMeterSensor::pcnt_unit_mutex_[PCNT_UNIT_MAX] = {};

PulseMeterSensor::PulseMeterSensor() {
  // Allocate PCNT unit
  for (uint8_t i = 0; i < PCNT_UNIT_MAX; i++) {
    if (pcnt_unit_mutex_[i] == nullptr) {
      state_.pcnt_unit = i;
      pcnt_unit_mutex_[i] = this;
      break;
    }
  }
}

PulseMeterSensor::~PulseMeterSensor() {
  cleanup_pcnt();
}

void PulseMeterSensor::setup() {
  if (!setup_pcnt()) {
    ESP_LOGE(TAG, "Failed to initialize PCNT unit");
    return;
  }
  
  state_.initialized = true;
  state_.last_edge_time = get_time_us();
  ESP_LOGD(TAG, "PulseMeterSensor initialized on PCNT unit %d", state_.pcnt_unit);
}

bool PulseMeterSensor::setup_pcnt() {
  pcnt_unit_config_t unit_config = {
      .high_limit = 32767,
      .low_limit = -32768,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_));
  
  pcnt_chan_config_t chan_config = {
      .edge_gpio_num = pin_->get_pin(),
      .level_gpio_num = -1,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_, &chan_config, &pcnt_chan_));
  
  pcnt_event_callbacks_t cbs = {
      .on_reach = pcnt_intr_handler
  };
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_, &cbs, this));
  
  return true;
}

void PulseMeterSensor::cleanup_pcnt() {
  if (state_.initialized) {
    pcnt_unit_mutex_[state_.pcnt_unit] = nullptr;
    state_.initialized = false;
  }
}

void PulseMeterSensor::loop() {
  if (!state_.initialized) return;
  
  int16_t count;
  if (pcnt_get_counter_value(static_cast<pcnt_unit_t>(state_.pcnt_unit), &count) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read PCNT counter");
    return;
  }
  
  if (count > 0) {
    uint32_t now = get_time_us();
    uint32_t delta = time_diff(now, state_.last_edge_time);
    
    if (delta > 0) {
      float rate = (count * 60.0f * 1000000.0f) / delta;
      publish_state(rate);
      
      if (total_sensor_ != nullptr) {
        total_pulses_ += count;
        total_sensor_->publish_state(total_pulses_);
      }
      
      state_.last_edge_time = now;
      pcnt_counter_clear(static_cast<pcnt_unit_t>(state_.pcnt_unit));
    }
  } else if (time_diff(get_time_us(), state_.last_edge_time) > timeout_us_) {
    publish_state(0.0f);
  }
}

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  total_pulses_ = pulses;
  if (total_sensor_ != nullptr) {
    total_sensor_->publish_state(total_pulses_);
  }
}

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  PCNT Unit: %d", state_.pcnt_unit);
  ESP_LOGCONFIG(TAG, "  Filter: %uµs", filter_us_);
  ESP_LOGCONFIG(TAG, "  Timeout: %us", timeout_us_ / 1000000);
}

} // namespace pulse_meter
} // namespace esphome
