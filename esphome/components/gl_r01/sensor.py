import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.automation as automation
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
)

DEPENDENCIES = ["i2c"]

CONF_AUTO_DETECT_ADDRESS = "auto_detect_address"
CONF_GL_R01_ID = "gl_r01_id"
CONF_NEW_ADDRESS = "new_address"

# Valid I2C addresses for the sensor.
VALID_ADDRESSES = [
    0xD0,
    0xD2,
    0xD4,
    0xD6,
    0xD8,
    0xDA,
    0xDC,
    0xDE,
    0xE0,
    0xE2,
    0xE4,
    0xE6,
    0xE8,
    0xEA,
    0xEC,
    0xEE,
    0xF8,
    0xFA,
    0xFC,
    0xFE,
]

gl_r01_ns = cg.esphome_ns.namespace("gl_r01")
GLR01Component = gl_r01_ns.class_("GLR01Component", i2c.I2CDevice, cg.PollingComponent)
ChangeAddressAction = gl_r01_ns.class_("ChangeAddressAction", automation.Action)
RestartSensorAction = gl_r01_ns.class_("RestartSensorAction", automation.Action)


def validate_gl_r01_address(value):
    if isinstance(value, str) and value.lower() == "auto":
        return "auto"
    value = cv.hex_int(value)
    if value not in VALID_ADDRESSES:
        raise cv.Invalid(
            f"Invalid I2C address {hex(value)}. Valid addresses are: {[hex(x) for x in VALID_ADDRESSES]}"
        )
    return value


gl_r01_base_schema = cv.Schema(
    {
        cv.Optional(CONF_ADDRESS, default=0xE8): validate_gl_r01_address,
    }
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        GLR01Component,
        unit_of_measurement=UNIT_MILLIMETER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DISTANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0xE8))
    .extend(gl_r01_base_schema)
)


@automation.register_action(
    "gl_r01.set_address",
    ChangeAddressAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(GLR01Component),
            cv.Required(CONF_NEW_ADDRESS): validate_gl_r01_address,
        }
    ),
)
async def gl_r01_set_address_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_address(config[CONF_NEW_ADDRESS]))
    return var


@automation.register_action(
    "gl_r01.restart",
    RestartSensorAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(GLR01Component),
        }
    ),
)
async def gl_r01_restart_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if config[i2c.CONF_ADDRESS] == "auto":
        cg.add(var.set_auto_detect(True))
    else:
        await i2c.register_i2c_device(var, config)
    await sensor.register_sensor(var, config)
