// pulse_meter_sensor.cpp
#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"
#include <esp_timer.h>
#include <freertos/queue.h>

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
    total_pulses_ = pulses;
    if(total_sensor_) total_sensor_->publish_state(total_pulses_);
}

void PulseMeterSensor::setup() {
    // Configure hardware pulse counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pin_->get_pin(),
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 10000,
        .counter_l_lim = 0,
        .unit = pcnt_unit_,
        .channel = pcnt_channel_,
    };
    
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));
    ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_unit_, filter_us_ * 80)); // APB_CLK is 80MHz
    ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit_, PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_unit_));

    // Create event queue and task
    event_queue_ = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreatePinnedToCore(
        [](void *arg) {
            PulseMeterSensor *sensor = static_cast<PulseMeterSensor*>(arg);
            while(true) {
                uint32_t count;
                if(xQueueReceive(sensor->event_queue_, &count, portMAX_DELAY)) {
                    sensor->process_pulses();
                }
            }
        }, 
        "pulse_task", 4096, this, 5, &task_handle_, PRO_CPU_NUM);

    // Configure timeout timer
    esp_timer_create_args_t timer_args = {
        .callback = [](void *arg) {
            PulseMeterSensor *sensor = static_cast<PulseMeterSensor*>(arg);
            sensor->publish_state(0.0f);
        },
        .arg = this,
        .name = "pulse_timeout"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle_));
}

void IRAM_ATTR PulseMeterSensor::isr_handler(void *arg) {
    PulseMeterSensor *sensor = static_cast<PulseMeterSensor*>(arg);
    uint32_t count;
    pcnt_get_counter_value(sensor->pcnt_unit_, &count);
    sensor->pulse_count_ += count;
    sensor->last_edge_us_ = esp_timer_get_time();
    pcnt_counter_clear(sensor->pcnt_unit_);
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(sensor->event_queue_, &count, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void PulseMeterSensor::process_pulses() {
    const uint32_t count = pulse_count_.exchange(0);
    if(count > 0) {
        total_pulses_ += count;
        const uint32_t now = esp_timer_get_time();
        const uint32_t period = now - last_edge_us_;
        
        if(total_sensor_) total_sensor_->publish_state(total_pulses_);
        publish_state((60.0f * 1000000.0f * count) / period);
        
        // Reset timeout timer
        esp_timer_stop(timer_handle_);
        ESP_ERROR_CHECK(esp_timer_start_once(timer_handle_, timeout_us_));
    }
}

void PulseMeterSensor::loop() {
    // Watchdog reset for long operations
    esp_task_wdt_reset();
}

void PulseMeterSensor::dump_config() {
    LOG_SENSOR("", "Pulse Meter", this);
    LOG_PIN(" Pin: ", pin_);
    ESP_LOGCONFIG(TAG, " Filter: %" PRIu32 " µs", filter_us_);
    ESP_LOGCONFIG(TAG, " Timeout: %" PRIu32 "s", timeout_us_ / 1000000);
}

} // namespace pulse_meter
} // namespace esphome
