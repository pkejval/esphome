#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

#include <cstdint>
#include <atomic>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>

namespace esphome {
namespace hw_pulse_meter {

enum CountMode : uint8_t {
  RISING = 0,
  FALLING = 1,
  BOTH = 2,
};

class HWPulseMeter : public sensor::Sensor, public Component {
 public:
  // Konfigurace
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_count_mode(CountMode m) { count_mode_ = m; }
  void set_internal_filter_us(uint32_t us_req, uint32_t us_applied) {
    internal_filter_us_requested_ = us_req;
    internal_filter_us_applied_ = us_applied;
  }
  void set_pulses_per_revolution(uint32_t ppr) { pulses_per_revolution_ = (ppr == 0) ? 1u : ppr; }
  void set_idle_timeout_us(uint32_t us) { idle_timeout_us_ = us; }

  void set_publish_total(bool v) { publish_total_ = v; }
  void set_publish_pps(bool v) { publish_pps_ = v; }
  void set_publish_revolutions(bool v) { publish_revolutions_ = v; }
  void set_total_sensor(sensor::Sensor *s) { total_sensor_ = s; }
  void set_pps_sensor(sensor::Sensor *s) { pps_sensor_ = s; }
  void set_revolutions_sensor(sensor::Sensor *s) { revolutions_sensor_ = s; }

  uint32_t get_pulses_per_revolution() const { return pulses_per_revolution_; }

  // Lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  ~HWPulseMeter();

 protected:
  // PCNT
  bool init_pcnt_();
  void apply_glitch_filter_();
  static pcnt_channel_edge_action_t map_edge_rising_(CountMode m);
  static pcnt_channel_edge_action_t map_edge_falling_(CountMode m);

  // Timer
  static void timer_callback_(void *arg);
  bool start_timer_(uint64_t period_us);
  void stop_timer_();

  // Helpers
  bool read_and_clear_pcnt_(int32_t &out);

  // Konfig
  InternalGPIOPin *pin_{nullptr};
  CountMode count_mode_{RISING};
  uint32_t internal_filter_us_requested_{12};
  uint32_t internal_filter_us_applied_{12};
  uint32_t pulses_per_revolution_{1};
  uint32_t idle_timeout_us_{0};

  bool publish_total_{false}, publish_pps_{false}, publish_revolutions_{false};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *pps_sensor_{nullptr};
  sensor::Sensor *revolutions_sensor_{nullptr};

  // PCNT handles
  pcnt_unit_handle_t unit_{nullptr};
  pcnt_channel_handle_t channel_{nullptr};

  // Periodický interní timer (10 ms jen pro read&clear z PCNT)
  esp_timer_handle_t timer_{nullptr};
  static constexpr uint64_t TIMER_PERIOD_US = 10000ULL;

  // Sdílené akumulátory mezi timerem a loopem
  std::atomic<int64_t> pending_total_delta_{0};        // pulzy od posledního loopu (pro total/revs)
  std::atomic<uint32_t> pending_pulses_since_rev_{0};  // pulzy od poslední publikované otáčky

  // Poslední spočítané hodnoty (z timeru)
  std::atomic<float> last_calculated_rpm_{NAN};
  std::atomic<float> last_calculated_pps_{NAN};
  std::atomic<bool> new_value_ready_{false};

  // Časy
  uint64_t last_pulse_time_us_{0};  // poslední reálný pulz (pro idle)
  uint64_t last_rev_time_us_{0};    // čas poslední publikované otáčky

  // Stav publikovaných counters
  uint64_t current_total_{0};         // publikovaný total
  uint64_t last_revolutions_pub_{0};  // publikované whole revs = floor(current_total_/PPR)

  // Idle publikace nuly
  bool idle_zero_published_{false};
};

}  // namespace hw_pulse_meter
}  // namespace esphome

#endif  // USE_ESP32
