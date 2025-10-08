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

# C++ Action třídy
SetComponentValueAction = nextion_simple_ns.class_("SetComponentValueAction", automation.Action.template())
SetComponentTextAction = nextion_simple_ns.class_("SetComponentTextAction", automation.Action.template())
SetComponentPiccAction = nextion_simple_ns.class_("SetComponentPiccAction", automation.Action.template())
SetComponentPicc1Action = nextion_simple_ns.class_("SetComponentPicc1Action", automation.Action.template())
SetComponentBackgroundColorAction = nextion_simple_ns.class_("SetComponentBackgroundColorAction", automation.Action.template())
SetComponentFontColorAction = nextion_simple_ns.class_("SetComponentFontColorAction", automation.Action.template())
SetComponentVisibilityAction = nextion_simple_ns.class_("SetComponentVisibilityAction", automation.Action.template())

# set_page: máme dvě C++ akce (int a string)
SetPageAction = nextion_simple_ns.class_("SetPageAction", automation.Action.template())
SetPageByNameAction = nextion_simple_ns.class_("SetPageByNameAction", automation.Action.template())

# registr instancí pro auto-ID
_NEXTION_INSTANCES = []

CONF_TFT_URL = "tft_url"
CONF_ON_SETUP = "on_setup"
CONF_ON_PAGE = "on_page"
CONF_ON_NEXTION_READY = "on_nextion_ready"
CONF_NEXTION_READY_COOLDOWN = "nextion_ready_cooldown"

CONF_PAGE = "page"
CONF_COMPONENT = "component"
CONF_VALUE = "value"
CONF_TEXT = "text"
CONF_COLOR = "color"
CONF_STATE = "state"


# ================= CONFIG =================
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

    cv.Optional(CONF_NEXTION_READY_COOLDOWN, default="500ms"): cv.positive_time_period_milliseconds,
})

async def to_code(config):
    # 🔧 FIX: používej CONF_ID z esphome.const
    var = cg.new_Pvariable(config[CONF_ID])
    _NEXTION_INSTANCES.append(var)
    await cg.register_component(var, config)

    uart_par = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_par))

    if CONF_TFT_URL in config:
        cg.add(var.set_tft_url(config[CONF_TFT_URL]))

    if CONF_NEXTION_READY_COOLDOWN in config:
        cg.add(var.set_nextion_ready_cooldown(int(config[CONF_NEXTION_READY_COOLDOWN].total_milliseconds)))

    if CONF_ON_SETUP in config:
        for conf in config[CONF_ON_SETUP]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [], conf)

    if CONF_ON_PAGE in config:
        for conf in config[CONF_ON_PAGE]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [(cg.int_, "page")], conf)

    if CONF_ON_NEXTION_READY in config:
        for conf in config[CONF_ON_NEXTION_READY]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [], conf)


# ---------- helper pro auto-ID ----------
async def _get_parent_from_config(config):
    if CONF_ID in config:
        return await cg.get_variable(config[CONF_ID])
    if len(_NEXTION_INSTANCES) == 1:
        return _NEXTION_INSTANCES[0]
    raise cv.Invalid("Je více než jedna instance 'nextion_simple' – uveď 'id:' v akci.")


# ================== AKCE ==================

# ---- sjednocené nextion_simple.set_page (int nebo string; podporuje shorthand) ----
@automation.register_action(
    "nextion_simple.set_page",
    automation.Action.template(),  # rozhodneme dynamicky, kterou C++ akci vytvořit
    cv.Any(
        cv.templatable(cv.int_),    # shorthand: - nextion_simple.set_page: 2
        cv.templatable(cv.string),  # shorthand: - nextion_simple.set_page: "home"
        cv.Schema({                 # plný tvar (s volitelným id)
            cv.Optional(CONF_ID): cv.use_id(NextionSimple),
            cv.Required(CONF_PAGE): cv.Any(
                cv.templatable(cv.int_),
                cv.templatable(cv.string),
            ),
        }),
    ),
)
async def nextion_set_page_to_code(config, action_id, template_args):
    # Shorthand: samotná hodnota (int nebo string)
    if isinstance(config, (int, str)) or (hasattr(config, "is_template") and config.is_template):
        parent = await _get_parent_from_config({})
        if isinstance(config, int):
            var = cg.new_Pvariable(action_id, SetPageAction(parent))
            page_t = await cg.templatable(config, template_args, cg.int_)
            cg.add(var.set_page(page_t))
            return var
        else:
            var = cg.new_Pvariable(action_id, SetPageByNameAction(parent))
            name_t = await cg.templatable(config, template_args, cg.std_string)
            cg.add(var.set_page_name(name_t))
            return var

    # Plný tvar se slovníkem
    parent = await _get_parent_from_config(config)
    page = config[CONF_PAGE]

    # Nejprve zkusíme jako int
    try:
        var = cg.new_Pvariable(action_id, SetPageAction(parent))
        page_t = await cg.templatable(page, template_args, cg.int_)
        cg.add(var.set_page(page_t))
        return var
    except Exception:
        pass

    # Fallback: string
    var = cg.new_Pvariable(action_id, SetPageByNameAction(parent))
    name_t = await cg.templatable(page, template_args, cg.std_string)
    cg.add(var.set_page_name(name_t))
    return var


# ---- set_component_value ----
@automation.register_action(
    "nextion_simple.set_component_value",
    SetComponentValueAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_VALUE): cv.templatable(cv.float_),
    }),
)
async def nextion_set_component_value_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    value = await cg.templatable(config[CONF_VALUE], template_args, cg.float_)
    cg.add(var.set_value(value))
    return var


# ---- set_component_text ----
@automation.register_action(
    "nextion_simple.set_component_text",
    SetComponentTextAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_TEXT): cv.templatable(cv.string),
    }),
)
async def nextion_set_component_text_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    text = await cg.templatable(config[CONF_TEXT], template_args, cg.std_string)
    cg.add(var.set_text(text))
    return var


# ---- set_component_picc ----
@automation.register_action(
    "nextion_simple.set_component_picc",
    SetComponentPiccAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_VALUE): cv.templatable(cv.int_),
    }),
)
async def nextion_set_component_picc_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    value = await cg.templatable(config[CONF_VALUE], template_args, cg.int_)
    cg.add(var.set_value(value))
    return var


# ---- set_component_picc1 ----
@automation.register_action(
    "nextion_simple.set_component_picc1",
    SetComponentPicc1Action,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_VALUE): cv.templatable(cv.int_),
    }),
)
async def nextion_set_component_picc1_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    value = await cg.templatable(config[CONF_VALUE], template_args, cg.int_)
    cg.add(var.set_value(value))
    return var


# ---- set_component_background_color ----
@automation.register_action(
    "nextion_simple.set_component_background_color",
    SetComponentBackgroundColorAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_COLOR): cv.templatable(cv.int_),
    }),
)
async def nextion_set_component_bco_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    color = await cg.templatable(config[CONF_COLOR], template_args, cg.int_)
    cg.add(var.set_color(color))
    return var


# ---- set_component_font_color ----
@automation.register_action(
    "nextion_simple.set_component_font_color",
    SetComponentFontColorAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_COLOR): cv.templatable(cv.int_),
    }),
)
async def nextion_set_component_pco_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    color = await cg.templatable(config[CONF_COLOR], template_args, cg.int_)
    cg.add(var.set_color(color))
    return var


# ---- set_component_visibility ----
@automation.register_action(
    "nextion_simple.set_component_visibility",
    SetComponentVisibilityAction,
    cv.Schema({
        cv.Optional(CONF_ID): cv.use_id(NextionSimple),
        cv.Required(CONF_COMPONENT): cv.string,
        cv.Required(CONF_STATE): cv.templatable(cv.boolean),
    }),
)
async def nextion_set_component_visibility_to_code(config, action_id, template_args):
    parent = await _get_parent_from_config(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_component_name(config[CONF_COMPONENT]))
    state = await cg.templatable(config[CONF_STATE], template_args, cg.bool_)
    cg.add(var.set_state(state))
    return var
