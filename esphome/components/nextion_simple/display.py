import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID, CONF_UART_ID, CONF_TRIGGER_ID

DEPENDENCIES = ["uart"]

nextion_simple_ns = cg.esphome_ns.namespace("nextion_simple")
NextionSimple = nextion_simple_ns.class_("NextionSimple", cg.Component)
NextionSetupTrigger = nextion_simple_ns.class_("NextionSetupTrigger", automation.Trigger.template())
NextionPageTrigger = nextion_simple_ns.class_("NextionPageTrigger", automation.Trigger.template())
NextionReadyTrigger = nextion_simple_ns.class_("NextionReadyTrigger", automation.Trigger.template())

CONF_TFT_URL = "tft_url"
CONF_ON_SETUP = "on_setup"
CONF_ON_PAGE = "on_page"
CONF_ON_NEXTION_READY = "on_nextion_ready"
CONF_NEXTION_READY_COOLDOWN = "nextion_ready_cooldown"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NextionSimple),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_TFT_URL): cv.string,

    # on_setup nemá parametry → jde to přímo přes callback registrátor
    cv.Optional(CONF_ON_SETUP): automation.validate_automation(single=True),

    # on_page má parametr (int page) → definuj trigger_id
    cv.Optional(CONF_ON_PAGE): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionPageTrigger),
    }),

    # on_nextion_ready bez parametrů → taky trigger s id
    cv.Optional(CONF_ON_NEXTION_READY): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionReadyTrigger),
    }),

    # POZOR: default musí mít jednotku
    cv.Optional(CONF_NEXTION_READY_COOLDOWN, default="500ms"): cv.positive_time_period_milliseconds,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_par = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_par))

    if CONF_TFT_URL in config:
        cg.add(var.set_tft_url(config[CONF_TFT_URL]))

    if CONF_NEXTION_READY_COOLDOWN in config:
        cg.add(var.set_nextion_ready_cooldown(int(config[CONF_NEXTION_READY_COOLDOWN].total_milliseconds)))

    # on_setup – přímo přes registrátor
    if CONF_ON_SETUP in config:
        await automation.build_automation(var.add_on_setup_callback, [], config[CONF_ON_SETUP])

    # on_page – vytvoř Trigger instanci s daným ID a parentem var
    if CONF_ON_PAGE in config:
        for conf in config[CONF_ON_PAGE]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [(cg.int_, "page")], conf)

    # on_nextion_ready – bez parametrů, ale stejně Trigger s ID
    if CONF_ON_NEXTION_READY in config:
        for conf in config[CONF_ON_NEXTION_READY]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [], conf)

# Actions
SetComponentValueAction = nextion_simple_ns.class_("SetComponentValueAction", automation.Action.template())
SetComponentTextAction = nextion_simple_ns.class_("SetComponentTextAction", automation.Action.template())
SetComponentTextPrintfAction = nextion_simple_ns.class_("SetComponentTextPrintfAction", automation.Action.template())
SetComponentPiccAction = nextion_simple_ns.class_("SetComponentPiccAction", automation.Action.template())
SetComponentPicc1Action = nextion_simple_ns.class_("SetComponentPicc1Action", automation.Action.template())
SetComponentBackgroundColorAction = nextion_simple_ns.class_("SetComponentBackgroundColorAction", automation.Action.template())
SetComponentFontColorAction = nextion_simple_ns.class_("SetComponentFontColorAction", automation.Action.template())
SetComponentVisibilityAction = nextion_simple_ns.class_("SetComponentVisibilityAction", automation.Action.template())
SetPageAction = nextion_simple_ns.class_("SetPageAction", automation.Action.template())
