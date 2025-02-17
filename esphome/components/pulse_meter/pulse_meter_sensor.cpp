#include "pulse_meter_sensor.h"
#include <utility>
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include "pulse_meter_sensor.h"
#include "esp32/rom/ets_sys.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

// ESP32-specific optimizations:
// 1. Use PCNT (Pulse Counter) hardware
// 2. Use high-resolution timer (APB_CLK)
// 3. Use RMT for precise timing measurements
// 4. Utilize ESP32's atomic operations
// 5. Place critical code in IRAM

struct __attribute__((packed)) ESP32State {
  uint32_t last_edge_time;
  int16_t pulse_count;
  uint8_t pcnt_unit;
  uint8_t rmt_channel;
};

class PulseMeterSensor : public Component, public sensor::Sensor {
 private:
  ESP32State state_;
  pcnt_config_t pcnt_config_;
  volatile uint32_t total_pulses_{0};
  uint32_t filter_us_;
  uint32_t timeout_us_;
  sensor::Sensor *total_sensor_{nullptr};
  
  // Use ESP32's hardware timer for precise timing
  hw_timer_t *timer_ = nullptr;
  portMUX_TYPE timer_mux_ = portMUX_INITIALIZER_UNLOCKED;

  static void IRAM_ATTR timer_isr(void *arg) {
    auto *sensor = static_cast<PulseMeterSensor *>(arg);
    portENTER_CRITICAL_ISR(&sensor->timer_mux_);
    sensor->handle_timeout();
    portEXIT_CRITICAL_ISR(&sensor->timer_mux_);
  }

  void IRAM_ATTR handle_timeout() {
    if (total_sensor_ != nullptr) {
      int16_t count;
      pcnt_get_counter_value(static_cast<pcnt_unit_t>(state_.pcnt_unit), &count);
      if (count > 0) {
        total_pulses_ += count;
        pcnt_counter_clear(static_cast<pcnt_unit_t>(state_.pcnt_unit));
      }
    }
  }

 public:
  PulseMeterSensor() {
    // Allocate PCNT unit
    for (uint8_t i = 0; i < PCNT_UNIT_MAX; i++) {
      if (pcnt_unit_mutex[i] == nullptr) {
        state_.pcnt_unit = i;
        pcnt_unit_mutex[i] = this;
        break;
      }
    }
  }

  void setup() override {
    // Configure PCNT unit
    pcnt_config_.pulse_gpio_num = pin_->get_pin();
    pcnt_config_.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcnt_config_.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_.pos_mode = PCNT_COUNT_INC;
    pcnt_config_.neg_mode = PCNT_COUNT_DIS;
    pcnt_config_.counter_h_lim = 32767;
    pcnt_config_.counter_l_lim = -32768;
    pcnt_config_.unit = static_cast<pcnt_unit_t>(state_.pcnt_unit);
    pcnt_config_.channel = PCNT_CHANNEL_0;

    pcnt_unit_config(&pcnt_config_);

    // Configure hardware filter
    uint16_t filter_val = filter_us_ * APB_CLK_FREQ / 1000000;
    pcnt_set_filter_value(static_cast<pcnt_unit_t>(state_.pcnt_unit), filter_val);
    pcnt_filter_enable(static_cast<pcnt_unit_t>(state_.pcnt_unit));

    // Configure and start timer for timeout detection
    timer_ = timerBegin(0, APB_CLK_FREQ / 1000000, true);
    timerAttachInterrupt(timer_, timer_isr, true);
    timerAlarmWrite(timer_, timeout_us_, true);
    timerAlarmEnable(timer_);

    // Start PCNT
    pcnt_counter_pause(static_cast<pcnt_unit_t>(state_.pcnt_unit));
    pcnt_counter_clear(static_cast<pcnt_unit_t>(state_.pcnt_unit));
    pcnt_counter_resume(static_cast<pcnt_unit_t>(state_.pcnt_unit));
  }

  void loop() override {
    int16_t count;
    pcnt_get_counter_value(static_cast<pcnt_unit_t>(state_.pcnt_unit), &count);
    
    if (count > 0) {
      // Calculate rate using high-precision timer
      uint32_t now = esp_timer_get_time();
      uint32_t delta = now - state_.last_edge_time;
      
      if (delta > 0) {
        float rate = (count * 60.0f * 1000000.0f) / delta;
        publish_state(rate);
        
        if (total_sensor_ != nullptr) {
          total_pulses_ += count;
          total_sensor_->publish_state(total_pulses_);
        }
      }
      
      state_.last_edge_time = now;
      pcnt_counter_clear(static_cast<pcnt_unit_t>(state_.pcnt_unit));
    } else if (esp_timer_get_time() - state_.last_edge_time > timeout_us_) {
      publish_state(0.0f);
    }
  }

  void set_filter_us(uint32_t filter) { 
    filter_us_ = filter; 
  }
  
  void set_timeout_us(uint32_t timeout) { 
    timeout_us_ = timeout; 
  }
  
  void set_total_sensor(sensor::Sensor *sensor) { 
    total_sensor_ = sensor; 
  }

  float get_setup_priority() const override { 
    return setup_priority::HARDWARE; 
  }

  ~PulseMeterSensor() {
    if (timer_) {
      timerEnd(timer_);
    }
    pcnt_unit_mutex[state_.pcnt_unit] = nullptr;
  }

 private:
  static PulseMeterSensor *pcnt_unit_mutex[PCNT_UNIT_MAX];
};

PulseMeterSensor *PulseMeterSensor::pcnt_unit_mutex[PCNT_UNIT_MAX] = {};

} // namespace pulse_meter
} // namespace esphome

#else
namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

// Handle micros() overflow
static inline uint32_t time_diff(uint32_t newer, uint32_t older) {
  return (newer >= older) ? newer - older : (0xFFFFFFFF - older) + newer + 1;
}

// Ensure thread-safe state transitions
enum class MeterState : uint8_t { 
  INITIAL = 0, 
  RUNNING = 1, 
  TIMED_OUT = 2 
};

struct __attribute__((packed)) PackedState {
  uint32_t last_detected_edge_us;
  uint32_t last_rising_edge_us;
  uint16_t count;
  
  // Track sequence number to detect missed updates
  uint16_t sequence;
};

class PulseMeterSensor : public sensor::Sensor, public Component {
 protected:
  // Use ring buffer for pulse history
  static constexpr size_t HISTORY_SIZE = 4;
  struct PulseHistory {
    uint32_t timestamps[HISTORY_SIZE];
    uint8_t write_idx;
    uint8_t count;
  } pulse_history_;

  // Track running statistics for better accuracy
  struct RunningStats {
    float avg_pulse_width;
    float variance;
    uint32_t sample_count;
  } stats_;

  // Add state validation
  bool validate_state_transition(MeterState new_state) {
    switch (meter_state_) {
      case MeterState::INITIAL:
        return new_state == MeterState::RUNNING || new_state == MeterState::TIMED_OUT;
      case MeterState::RUNNING:
        return new_state == MeterState::TIMED_OUT;
      case MeterState::TIMED_OUT:
        return new_state == MeterState::RUNNING;
      default:
        return false;
    }
  }

  // Improved pulse filtering
  bool is_valid_pulse(uint32_t pulse_width) {
    if (stats_.sample_count < 2) return true;
    
    // Use running statistics to detect outliers
    float z_score = (pulse_width - stats_.avg_pulse_width) / 
                    sqrtf(stats_.variance);
    return fabsf(z_score) <= 3.0f; // Within 3 standard deviations
  }

  void update_statistics(uint32_t pulse_width) {
    stats_.sample_count++;
    float delta = pulse_width - stats_.avg_pulse_width;
    stats_.avg_pulse_width += delta / stats_.sample_count;
    float delta2 = pulse_width - stats_.avg_pulse_width;
    stats_.variance = ((stats_.sample_count - 1) * stats_.variance + 
                      delta * delta2) / stats_.sample_count;
  }

  void add_to_history(uint32_t timestamp) {
    pulse_history_.timestamps[pulse_history_.write_idx] = timestamp;
    pulse_history_.write_idx = (pulse_history_.write_idx + 1) % HISTORY_SIZE;
    if (pulse_history_.count < HISTORY_SIZE)
      pulse_history_.count++;
  }

  // More accurate rate calculation using history
  float calculate_pulse_rate() {
    if (pulse_history_.count < 2) return 0.0f;
    
    uint32_t total_time = 0;
    uint8_t read_idx = pulse_history_.write_idx;
    
    for (uint8_t i = 0; i < pulse_history_.count - 1; i++) {
      uint8_t prev_idx = (read_idx - 1 + HISTORY_SIZE) % HISTORY_SIZE;
      total_time += time_diff(pulse_history_.timestamps[read_idx],
                            pulse_history_.timestamps[prev_idx]);
      read_idx = prev_idx;
    }
    
    float avg_period = total_time / float(pulse_history_.count - 1);
    return (60.0f * 1000000.0f) / avg_period;
  }

 public:
  void IRAM_ATTR pulse_intr(void) {
    const uint32_t now = micros();
    const bool pin_val = isr_pin_.digital_read();
    auto &state = pulse_state_;
    
    // More robust edge detection
    const uint32_t time_since_last = time_diff(now, state.last_intr);
    const bool is_stable = time_since_last >= filter_us_;
    
    if (is_stable) {
      bool valid_edge = false;
      
      if (state.latched && !state.last_pin_val && !pin_val) {
        // Falling edge after stable high
        state.latched = false;
        valid_edge = true;
      } else if (!state.latched && state.last_pin_val && pin_val) {
        // Rising edge after stable low
        state.latched = true;
        valid_edge = true;
        
        PackedState new_state = set_state_;
        new_state.last_detected_edge_us = state.last_intr;
        new_state.count++;
        new_state.sequence++;
        
        __atomic_store(&set_state_, &new_state, __ATOMIC_RELEASE);
      }
      
      if (valid_edge) {
        add_to_history(now);
      }
    }
    
    state.last_intr = now;
    state.last_pin_val = pin_val;
  }

  void loop() override {
    const uint32_t now = micros();
    
    PackedState current_state;
    static uint16_t last_sequence = 0;
    
    __atomic_load(&set_state_, &current_state, __ATOMIC_ACQUIRE);
    
    // Detect missed updates
    if (current_state.sequence != last_sequence) {
      last_sequence = current_state.sequence;
      
      if (current_state.count > 0) {
        const uint32_t pulse_width = time_diff(
          current_state.last_detected_edge_us,
          last_processed_edge_us_);
          
        if (is_valid_pulse(pulse_width)) {
          update_statistics(pulse_width);
          
          if (total_sensor_ != nullptr) {
            total_pulses_ += current_state.count;
            total_sensor_->publish_state(total_pulses_);
          }
          
          const float rate = calculate_pulse_rate();
          if (validate_state_transition(MeterState::RUNNING)) {
            meter_state_ = MeterState::RUNNING;
            publish_state(rate);
          }
        }
        
        last_processed_edge_us_ = current_state.last_detected_edge_us;
      }
    }
    
    // Check for timeout
    if (meter_state_ != MeterState::TIMED_OUT) {
      if (time_diff(now, last_processed_edge_us_) > timeout_us_) {
        if (validate_state_transition(MeterState::TIMED_OUT)) {
          meter_state_ = MeterState::TIMED_OUT;
          stats_ = {}; // Reset statistics
          publish_state(0.0f);
        }
      }
    }
  }
};

} // namespace pulse_meter
} // namespace esphome
#endif
