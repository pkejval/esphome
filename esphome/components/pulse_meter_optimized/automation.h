#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/pulse_meter_optimized/pulse_meter_optimized_sensor.h"

namespace esphome {

namespace pulse_meter_optimized {

template<typename... Ts> class SetTotalPulsesAction : public Action<Ts...> {
 public:
  SetTotalPulsesAction(PulseMeterSensor *pulse_meter_optimized) : pulse_meter_optimized_(pulse_meter_optimized) {}

  TEMPLATABLE_VALUE(uint32_t, total_pulses)

  void play(Ts... x) override { this->pulse_meter_optimized_->set_total_pulses(this->total_pulses_.value(x...)); }

 protected:
  PulseMeterSensor *pulse_meter_optimized_;
};

}  // namespace pulse_meter_optimized
}  // namespace esphome
