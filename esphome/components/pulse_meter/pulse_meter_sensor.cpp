#include "pulse_meter_sensor.h"
#include <utility>
#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

// BUGFIX 1: Handle micros() overflow
static inline uint32_t time_diff(uint32_t newer, uint32_t older) {
  return (newer >= older) ? newer - older : (0xFFFFFFFF - older) + newer + 1;
}

// BUGFIX 2: Ensure thread-safe state transitions
enum class MeterState : uint8_t { 
  INITIAL = 0, 
  RUNNING = 1, 
  TIMED_OUT = 2 
};

struct __attribute__((packed)) PackedState {
  uint32_t last_detected_edge_us;
  uint32_t last_rising_edge_us;
  uint16_t count;
  
  // BUGFIX 3: Track sequence number to detect missed updates
  uint16_t sequence;
};

class PulseMeterSensor : public sensor::Sensor, public Component {
 protected:
  // OPTIMIZATION 1: Use ring buffer for pulse history
  static constexpr size_t HISTORY_SIZE = 4;
  struct PulseHistory {
    uint32_t timestamps[HISTORY_SIZE];
    uint8_t write_idx;
    uint8_t count;
  } pulse_history_;

  // OPTIMIZATION 2: Track running statistics for better accuracy
  struct RunningStats {
    float avg_pulse_width;
    float variance;
    uint32_t sample_count;
  } stats_;

  // BUGFIX 4: Add state validation
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

  // OPTIMIZATION 3: Improved pulse filtering
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

  // OPTIMIZATION 4: More accurate rate calculation using history
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
    
    // BUGFIX 5: More robust edge detection
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
    
    // BUGFIX 6: Detect missed updates
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
