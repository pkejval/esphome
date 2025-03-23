import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID, CONF_LAMBDA, CONF_ON_BOOT, CONF_TRIGGER_ID,
    CONF_VALUE, CONF_PAGE, CONF_FORMAT, CONF_COLOR,
    CONF_ARGS,
)

from . import NextionIntelligent, nextion_intelligent_ns

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["uart"]
CODEOWNERS = ["@pkejval"]

# Define component namespace
nextion_intelligent_ns = cg.esphome_ns.namespace("nextion_intelligent")
NextionIntelligent = nextion_intelligent_ns.class_(
    "NextionIntelligent", cg.Component, uart.UARTDevice
)

# Config schema for the component
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NextionIntelligent),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)


# Define configuration constants
CONF_NEXTION_INTELLIGENT_ID = "nextion_intelligent_id"
CONF_COMPONENT_NAME = "component_name"
CONF_PICTURE_ID = "picture_id"


# Action classes
SetComponentValueAction = nextion_intelligent_ns.class_(
    "SetComponentValueAction", automation.Action
)
SetComponentTextAction = nextion_intelligent_ns.class_(
    "SetComponentTextAction", automation.Action
)
SetComponentTextPrintfAction = nextion_intelligent_ns.class_(
    "SetComponentTextPrintfAction", automation.Action
)
SetComponentPictureAction = nextion_intelligent_ns.class_(
    "SetComponentPictureAction", automation.Action
)
SetComponentPicture1Action = nextion_intelligent_ns.class_(
    "SetComponentPicture1Action", automation.Action
)
SetComponentBackgroundColorAction = nextion_intelligent_ns.class_(
    "SetComponentBackgroundColorAction", automation.Action
)
SetComponentFontColorAction = nextion_intelligent_ns.class_(
    "SetComponentFontColorAction", automation.Action
)
SetPageAction = nextion_intelligent_ns.class_(
    "SetPageAction", automation.Action
)

# Trigger for Nextion boot events
NextionBootTrigger = nextion_intelligent_ns.class_(
    "NextionBootTrigger", automation.Trigger.template()
)

# Display platform configuration schema
PLATFORM_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NextionIntelligent),
    cv.Optional(CONF_LAMBDA): cv.lambda_,
    cv.Optional(CONF_ON_BOOT): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionBootTrigger),
    }),
})

# Register the display platform
async def setup_nextion_intelligent_display(config):
    var = await cg.get_variable(config[CONF_ID])
    
    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], 
            [(NextionIntelligent.operator("ptr"), "it")],
            return_type=cg.void
        )
        cg.add(var.set_lambda(lambda_))
    
    # Set up on_boot callbacks
    if CONF_ON_BOOT in config:
        for conf in config[CONF_ON_BOOT]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)
    
    return var

# Action schema for set_component_value
SET_COMPONENT_VALUE_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.Any(cv.int_, cv.float_)),
})

@automation.register_action(
    "nextion_intelligent.set_component_value", 
    SetComponentValueAction, 
    SET_COMPONENT_VALUE_SCHEMA
)
async def set_component_value_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_VALUE], args, float)
    cg.add(var.set_component_value(config[CONF_COMPONENT_NAME], template_))
    return var

# Action schema for set_component_text
SET_COMPONENT_TEXT_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.string),
})

@automation.register_action(
    "nextion_intelligent.set_component_text", 
    SetComponentTextAction, 
    SET_COMPONENT_TEXT_SCHEMA
)
async def set_component_text_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_VALUE], args, cg.std_string)
    cg.add(var.set_component_text(config[CONF_COMPONENT_NAME], template_))
    return var

# Action schema for set_component_text_printf
SET_COMPONENT_TEXT_PRINTF_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_FORMAT): cv.string,
    cv.Optional(CONF_ARGS, default=[]): cv.ensure_list(cv.lambda_),
})

@automation.register_action(
    "nextion_intelligent.set_component_text_printf", 
    SetComponentTextPrintfAction, 
    SET_COMPONENT_TEXT_PRINTF_SCHEMA
)
async def set_component_text_printf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_component_name(config[CONF_COMPONENT_NAME]))
    cg.add(var.set_format(config[CONF_FORMAT]))
    for arg in config[CONF_ARGS]:
        arg_var = await cg.process_lambda(arg, args, return_type=cg.float_)
        cg.add(var.add_argument(arg_var))
    return var

# Additional action schemas for other functions
# (Abbreviated for clarity - implement following the same pattern)

SET_COMPONENT_PICTURE_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_PICTURE_ID): cv.templatable(cv.int_),
})

@automation.register_action(
    "nextion_intelligent.set_component_picture", 
    SetComponentPictureAction, 
    SET_COMPONENT_PICTURE_SCHEMA
)
async def set_component_picture_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PICTURE_ID], args, int)
    cg.add(var.set_component_picture(config[CONF_COMPONENT_NAME], template_))
    return var

SET_COMPONENT_PICTURE1_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_PICTURE_ID): cv.templatable(cv.int_),
})

@automation.register_action(
    "nextion_intelligent.set_component_picture1", 
    SetComponentPicture1Action, 
    SET_COMPONENT_PICTURE1_SCHEMA
)
async def set_component_picture1_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PICTURE_ID], args, int)
    cg.add(var.set_component_picture1(config[CONF_COMPONENT_NAME], template_))
    return var

SET_PAGE_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(NextionIntelligent),
    cv.Required(CONF_PAGE): cv.templatable(cv.int_),
})

@automation.register_action(
    "nextion_intelligent.set_page", 
    SetPageAction, 
    SET_PAGE_SCHEMA
)
async def set_page_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PAGE], args, int)
    cg.add(var.set_page(template_))
    return var

# Generate code from the config
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)