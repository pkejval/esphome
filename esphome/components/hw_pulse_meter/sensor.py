import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_PIN,
    ICON_PULSE,
    UNIT_PULSES,
    UNIT_REVOLUTIONS_PER_MINUTE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
)
from esphome import pins
from esphome.core import CORE

hw_pulse_meter_ns = cg.esphome_ns.namespace("hw_pulse_meter")
HWPulseMeter = hw_pulse_meter_ns.class_("HWPulseMeter", sensor.Sensor, cg.Component)

CountMode = hw_pulse_meter_ns.enum("CountMode")
COUNT_MODE = {
    "RISING": CountMode.RISING,
    "FALLING": CountMode.FALLING,
    "BOTH": CountMode.BOTH,
}

CONF_COUNT_MODE = "count_mode"
CONF_INTERNAL_FILTER = "internal_filter"
CONF_TIMEOUT = "timeout"
CONF_PPR = "pulses_per_revolution"
CONF_TOTAL = "total"
CONF_PPS = "pps"
CONF_REVS = "revolutions"
CONF_POLL_INTERVAL = "poll_interval"

LEGACY_ESP32_PCNT_FILTER_LIMIT_US = 12.0  # ESP32, ESP32-S2
MODERN_ESP32_PCNT_FILTER_LIMIT_US = 819.0  # ESP32-S3, C3, C6, H2


def _get_esp32_variant():
    esp32_data = CORE.data.get("esp32", {})
    return esp32_data.get("variant", "ESP32")


def _validate_internal_filter(v):
    t = cv.positive_time_period_microseconds(v)
    if CORE.is_esp8266:
        raise cv.Invalid("HWPulseMeter is ESP32-only")
    if CORE.is_esp32:
        variant = _get_esp32_variant()
        is_legacy = variant in ("ESP32", "ESP32S2")
        limit = (
            LEGACY_ESP32_PCNT_FILTER_LIMIT_US
            if is_legacy
            else MODERN_ESP32_PCNT_FILTER_LIMIT_US
        )
        applied = min(float(t.total_microseconds), limit)
        t._applied_microseconds = int(applied)
    return t


CONFIG_SCHEMA = sensor.sensor_schema(
    HWPulseMeter,
    unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
    icon=ICON_PULSE,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.GenerateID(): cv.declare_id(HWPulseMeter),
        cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_COUNT_MODE, default="RISING"): cv.enum(COUNT_MODE, upper=True),
        cv.Optional(CONF_INTERNAL_FILTER, default="12us"): _validate_internal_filter,
        cv.Optional(CONF_TIMEOUT, default="0s"): cv.positive_time_period_microseconds,
        cv.Optional(CONF_PPR, default=1): cv.positive_int,
        cv.Optional(CONF_POLL_INTERVAL, default="0s"): cv.time_period_microseconds,
        cv.Optional(CONF_TOTAL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PULSES,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon=ICON_PULSE,
        ),
        cv.Optional(CONF_PPS): sensor.sensor_schema(
            unit_of_measurement="pps",
            accuracy_decimals=2,
            icon=ICON_PULSE,
        ),
        cv.Optional(CONF_REVS): sensor.sensor_schema(
            unit_of_measurement="rev",
            accuracy_decimals=0,
            icon=ICON_PULSE,
        ),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_count_mode(config[CONF_COUNT_MODE]))

    filt = config[CONF_INTERNAL_FILTER]
    requested_us = int(filt.total_microseconds)
    applied_us = getattr(filt, "_applied_microseconds", requested_us)
    cg.add(var.set_internal_filter_us(requested_us, applied_us))

    cg.add(var.set_idle_timeout_us(config[CONF_TIMEOUT].total_microseconds))
    cg.add(var.set_pulses_per_revolution(config[CONF_PPR]))

    poll_us = int(config[CONF_POLL_INTERVAL].total_microseconds)
    cg.add(var.set_poll_interval_us(poll_us))

    publish_total = CONF_TOTAL in config
    publish_pps = CONF_PPS in config
    publish_revs = CONF_REVS in config
    cg.add(var.set_publish_total(publish_total))
    cg.add(var.set_publish_pps(publish_pps))
    cg.add(var.set_publish_revolutions(publish_revs))

    if publish_total:
        s_total = await sensor.new_sensor(config[CONF_TOTAL])
        cg.add(var.set_total_sensor(s_total))
    if publish_pps:
        s_pps = await sensor.new_sensor(config[CONF_PPS])
        cg.add(var.set_pps_sensor(s_pps))
    if publish_revs:
        s_revs = await sensor.new_sensor(config[CONF_REVS])
        cg.add(var.set_revolutions_sensor(s_revs))
