import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
)

CODEOWNERS = ["@pkejval"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = False

ns = cg.esphome_ns.namespace("gl_r01_i2c")
GLR01I2CComponent = ns.class_("GLR01I2CComponent", sensor.Sensor, i2c.I2CDevice, cg.PollingComponent)

CONF_TRIGGER_DELAY_MS = "trigger_delay_ms"
CONF_RESTART_AFTER_N = "restart_after_n_failures"

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        GLR01I2CComponent,
        unit_of_measurement=UNIT_MILLIMETER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DISTANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x74))
    .extend(
        {
            cv.Optional(CONF_TRIGGER_DELAY_MS, default=40): cv.int_range(min=10, max=1000),
            cv.Optional(CONF_RESTART_AFTER_N, default=3): cv.int_range(min=1, max=10),
        }
    )
)

async def to_code(config):
    var = cg.new_Pvariable(config[cv.CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_trigger_delay_ms(config[CONF_TRIGGER_DELAY_MS]))
    cg.add(var.set_restart_after_n_failures(config[CONF_RESTART_AFTER_N]))
