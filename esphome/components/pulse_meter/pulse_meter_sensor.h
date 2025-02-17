// pulse_meter_sensor.h
#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <atomic>
#include <driver/gpio.h>
#include <driver/pcnt.h>

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
public:
    enum InternalFilterMode {
        FILTER_EDGE = 0,
        FILTER_PULSE,
    };

    void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
    void set_filter_us(uint32_t filter) { filter_us_ = filter; }
    void set_timeout_us(uint32_t timeout) { timeout_us_ = timeout; }
    void set_total_sensor(sensor::Sensor *sensor) { total_sensor_ = sensor; }
    void set_filter_mode(InternalFilterMode mode) { filter_mode_ = mode; }
    void set_total_pulses(uint32_t pulses);
    
    void setup() override;
    void loop() override;
    float get_setup_priority() const override { return setup_priority::DATA; }
    void dump_config() override;

protected:
    static void IRAM_ATTR isr_handler(void *arg);
    void process_pulses();

    // ESP32 hardware pulse counter configuration
    pcnt_unit_t pcnt_unit_ = PCNT_UNIT_0;
    pcnt_channel_t pcnt_channel_ = PCNT_CHANNEL_0;
    
    InternalGPIOPin *pin_{nullptr};
    uint32_t filter_us_{0};
    uint32_t timeout_us_{30000000}; // 30s default timeout
    sensor::Sensor *total_sensor_{nullptr};
    
    // Atomic counters for lock-free access
    std::atomic<uint32_t> pulse_count_{0};
    std::atomic<uint32_t> last_edge_us_{0};
    std::atomic<uint32_t> total_pulses_{0};
    
    // Task handle for core affinity
    TaskHandle_t task_handle_{nullptr};
    QueueHandle_t event_queue_{nullptr};
    
    // Hardware timer for timeout detection
    esp_timer_handle_t timer_handle_{nullptr};
};

} // namespace pulse_meter
} // namespace esphome
