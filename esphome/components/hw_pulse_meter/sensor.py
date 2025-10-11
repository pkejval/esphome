import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, gpio
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    ICON_PULSE,
)

hw_pulse_meter_ns = cg.esphome_ns.namespace("hw_pulse_meter")
HWPulseMeter = hw_pulse_meter_ns.class_("HWPulseMeter", sensor.Sensor, cg.Component)
CountMode = hw_pulse_meter_ns.enum("CountMode")

COUNT_MODE = {
    "RISING": CountMode.RISING,
    "FALLING": CountMode.FALLING,
    "BOTH": CountMode.BOTH,
}

CONF_COUNT_MODE = "count_mode"
CONF_GLITCH_FILTER = "glitch_filter"
CONF_MIN_INTERVAL = "min_interval"
CONF_PPR = "pulses_per_revolution"
CONF_TOTAL = "total"
CONF_PPS = "pps"

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="lpm",
    icon=ICON_PULSE,
    accuracy_decimals=2,
).extend(
    {
        cv.GenerateID(): cv.declare_id(HWPulseMeter),
        cv.Required(CONF_PIN): cv.All(gpio.gpio_input_pin_schema),
        cv.Optional(CONF_COUNT_MODE, default="RISING"): cv.enum(COUNT_MODE, upper=True),
        cv.Optional(CONF_GLITCH_FILTER, default="0us"): cv.positive_time_period_microseconds,
        cv.Optional(CONF_MIN_INTERVAL, default="0us"): cv.positive_time_period_microseconds,
        cv.Optional(CONF_PPR, default=1): cv.positive_int,

        cv.Optional(CONF_TOTAL): sensor.sensor_schema(),
        cv.Optional(CONF_PPS): sensor.sensor_schema(unit_of_measurement="pps", icon=ICON_PULSE),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_count_mode(config[CONF_COUNT_MODE]))

    cg.add(var.set_glitch_filter_us(config[CONF_GLITCH_FILTER].total_microseconds))
    cg.add(var.set_min_interval_us(config[CONF_MIN_INTERVAL].total_microseconds))
    cg.add(var.set_pulses_per_revolution(config[CONF_PPR]))

    publish_total = CONF_TOTAL in config
    publish_pps = CONF_PPS in config
    cg.add(var.set_publish_total(publish_total))
    cg.add(var.set_publish_pps(publish_pps))

    if publish_total:
        sens = await sensor.new_sensor(config[CONF_TOTAL])
        cg.add(var.set_total_sensor(sens))

    if publish_pps:
        sens = await sensor.new_sensor(config[CONF_PPS])
        cg.add(var.set_pps_sensor(sens))
