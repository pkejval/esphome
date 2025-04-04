import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID, CONF_LAMBDA, CONF_UART_ID, CONF_TRIGGER_ID

DEPENDENCIES = ["uart"]

nextion_simple_ns = cg.esphome_ns.namespace("nextion_simple")
NextionSimple = nextion_simple_ns.class_("NextionSimple", cg.Component)
NextionSetupTrigger = nextion_simple_ns.class_("NextionSetupTrigger", automation.Trigger.template())
NextionPageTrigger = nextion_simple_ns.class_("NextionPageTrigger", automation.Trigger.template())
NextionReadyTrigger = nextion_simple_ns.class_("NextionReadyTrigger", automation.Trigger.template())

# Actions
SetComponentValueAction = nextion_simple_ns.class_("SetComponentValueAction", automation.Action)
SetComponentFloatValueAction = nextion_simple_ns.class_("SetComponentFloatValueAction", automation.Action)
SetComponentTextAction = nextion_simple_ns.class_("SetComponentTextAction", automation.Action)
SetComponentTextPrintfAction = nextion_simple_ns.class_("SetComponentTextPrintfAction", automation.Action)
SetComponentPiccAction = nextion_simple_ns.class_("SetComponentPiccAction", automation.Action)
SetComponentPicc1Action = nextion_simple_ns.class_("SetComponentPicc1Action", automation.Action)
SetComponentBackgroundColorAction = nextion_simple_ns.class_("SetComponentBackgroundColorAction", automation.Action)
SetComponentBackgroundColorRGBAction = nextion_simple_ns.class_("SetComponentBackgroundColorRGBAction", automation.Action)
SetComponentFontColorAction = nextion_simple_ns.class_("SetComponentFontColorAction", automation.Action)
SetComponentFontColorRGBAction = nextion_simple_ns.class_("SetComponentFontColorRGBAction", automation.Action)
SetPageAction = nextion_simple_ns.class_("SetPageAction", automation.Action)
UploadTftAction = nextion_simple_ns.class_("UploadTftAction", automation.Action)

CONF_COMPONENT_NAME = "objname"
CONF_VALUE = "value"
CONF_COLOR = "color"
CONF_PAGE = "page"
CONF_FORMAT = "format"
CONF_TEXT = "text"
CONF_ARGS = "args"
CONF_TFT_URL = "tft_url"
CONF_ON_SETUP = "on_setup"
CONF_ON_PAGE = "on_page"
CONF_ON_NEXTION_READY = "on_nextion_ready"
CONF_NEXTION_READY_COOLDOWN = "nextion_ready_cooldown"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NextionSimple),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_TFT_URL): cv.string,
    cv.Optional(CONF_ON_SETUP): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionSetupTrigger),
    }),
    cv.Optional(CONF_ON_PAGE): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionPageTrigger),
    }),
    cv.Optional(CONF_ON_NEXTION_READY): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionReadyTrigger),
    }),
    cv.Optional(CONF_NEXTION_READY_COOLDOWN, default="1s"): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)

# Action schemas

SET_COMPONENT_VALUE_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(NextionSimple),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.Any(cv.int_, cv.float_)),
})

SET_COMPONENT_TEXT_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(NextionSimple),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.string),
})

SET_COMPONENT_PICC_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(NextionSimple),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.int_),
})

SET_COMPONENT_PICC1_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(NextionSimple),
    cv.Required(CONF_COMPONENT_NAME): cv.string,
    cv.Required(CONF_VALUE): cv.templatable(cv.int_),
})

# New validation function to handle both formats
def validate_set_page_action(config):
    if isinstance(config, int):
        config = {CONF_PAGE: config}
    
    return cv.Schema({
        cv.GenerateID(): cv.use_id(NextionSimple),
        cv.Required(CONF_PAGE): cv.templatable(cv.int_),
    })(config)

UPLOAD_TFT_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(NextionSimple),
})

@automation.register_action("nextion.set_value", SetComponentValueAction, SET_COMPONENT_VALUE_SCHEMA)
async def nextion_simple_set_component_value_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_component_name(config[CONF_COMPONENT_NAME]))
    template_ = await cg.templatable(config[CONF_VALUE], args, float)
    cg.add(var.set_value(template_))
    return var

@automation.register_action("nextion.set_text", SetComponentTextAction, SET_COMPONENT_TEXT_SCHEMA)
async def nextion_simple_set_component_text_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT_NAME]))
    template_ = await cg.templatable(config[CONF_VALUE], args, cg.std_string)
    cg.add(var.set_value(template_))
    return var

@automation.register_action("nextion.set_picc", SetComponentPiccAction, SET_COMPONENT_PICC_SCHEMA)
async def nextion_simple_set_component_picc_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_component_name(config[CONF_COMPONENT_NAME]))
    cg.add(var.set_value(config[CONF_VALUE]))
    return var

@automation.register_action("nextion.set_picc1", SetComponentPicc1Action, SET_COMPONENT_PICC1_SCHEMA)
async def nextion_simple_set_component_picc1_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_component_name(config[CONF_COMPONENT_NAME]))
    cg.add(var.set_value(config[CONF_VALUE]))
    return var

@automation.register_action("nextion.set_page", SetPageAction, validate_set_page_action)
async def nextion_simple_set_page_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    page = await cg.templatable(config[CONF_PAGE], args, int)
    cg.add(var.set_page(page))
    return var

@automation.register_action("nextion.upload_tft", UploadTftAction, UPLOAD_TFT_SCHEMA)
async def nextion_simple_upload_tft_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    return var

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_component))

    if CONF_TFT_URL in config:
        cg.add(var.set_tft_url(config[CONF_TFT_URL]))

    if CONF_ON_SETUP in config:
        for conf in config.get(CONF_ON_SETUP, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_PAGE in config:
        for conf in config.get(CONF_ON_PAGE, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [(cg.int_, "x")], conf)

    if CONF_ON_NEXTION_READY in config:
        for conf in config.get(CONF_ON_NEXTION_READY, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_NEXTION_READY_COOLDOWN in config:
        cg.add(var.set_nextion_ready_cooldown(config[CONF_NEXTION_READY_COOLDOWN]))
