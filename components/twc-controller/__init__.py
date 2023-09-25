import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.cpp_helpers import gpio_pin_expression
from esphome import pins
from esphome.components import (
    number,
    sensor,
    text_sensor,
    uart,
)

from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_STEP,
    CONF_RESTORE_VALUE,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_CURRENT,
    CONF_STATE,
    CONF_FLOW_CONTROL_PIN,

    UNIT_AMPERE,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,

    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,

    STATE_CLASS_MEASUREMENT,
)

CONF_MAX_ALLOWABLE_CURRENT = "max_allowable_current"
CONF_TOTAL_KWH = "total_kwh_delivered"
CONF_SERIAL = "serial"
CONF_PHASE_1_VOLTAGE = "phase_1_voltage"
CONF_PHASE_2_VOLTAGE = "phase_2_voltage"
CONF_PHASE_3_VOLTAGE = "phase_3_voltage"
CONF_PHASE_1_CURRENT = "phase_1_current"
CONF_PHASE_2_CURRENT = "phase_2_current"
CONF_PHASE_3_CURRENT = "phase_3_current"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_ACTUAL_CURRENT = "actual_current"
CONF_VIN = "connected_vin"

CONF_MIN_CURRENT = "min_current"
CONF_MAX_CURRENT = "max_current"
CONF_SET_CURRENT = "set_current"
CONF_TWCID = "twc_id"

ICON_CURRENT_AC = "mdi:current-ac"
ICON_CAR = "mdi:car"
ICON_NUMERIC = "mdi:numeric"
ICON_LIGHTNING_BOLT = "mdi:lightning-bolt"

AUTO_LOAD = ["number", "sensor", "text_sensor"]
DEPENDENCIES = ["uart"]

TEXT_TYPES = [
    CONF_SERIAL,
    CONF_FIRMWARE_VERSION,
    CONF_VIN
]

TYPES = [
    CONF_MAX_ALLOWABLE_CURRENT,
    CONF_CURRENT,
    CONF_STATE,
    CONF_TOTAL_KWH,
    CONF_PHASE_1_VOLTAGE,
    CONF_PHASE_2_VOLTAGE,
    CONF_PHASE_3_VOLTAGE,
    CONF_PHASE_1_CURRENT,
    CONF_PHASE_2_CURRENT,
    CONF_PHASE_3_CURRENT,
    CONF_ACTUAL_CURRENT,
]

twc_controller_ns = cg.esphome_ns.namespace("twc_controller")
TWCController = twc_controller_ns.class_(
    "TWCController", number.Number, cg.Component, uart.UARTDevice
)

def validate_min_max(config):
    if config[CONF_MAX_CURRENT] <= config[CONF_MIN_CURRENT]:
        raise cv.Invalid(f"{CONF_MAX_CURRENT} must be greater than {CONF_MIN_CURRENT}")

    if CONF_SET_CURRENT in config:
        if (config[CONF_SET_CURRENT] > config[CONF_MAX_CURRENT]) or (config[CONF_SET_CURRENT] < config[CONF_MIN_CURRENT]):
            raise cv.Invalid(f"{CONF_SET_CURRENT} must be between {CONF_MIN_CURRENT} and {CONF_MAX_CURRENT}")

    return config

CONFIG_SCHEMA = cv.All(
    number.number_schema(TWCController).extend(
        {
            cv.Optional(CONF_NAME, default="Set Current"): cv.string,
            cv.Optional(CONF_MIN_CURRENT, default=6): cv.int_range(min=6, max=32),
            cv.Optional(CONF_MAX_CURRENT, default=32): cv.int_range(min=6, max=32),
            cv.Optional(CONF_TWCID, default=0xABCD): cv.hex_int_range(0, 65535),
            cv.Optional(CONF_SET_CURRENT): cv.positive_int,
            cv.Optional(CONF_STEP, default=1): cv.positive_int,
            cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_AMPERE): cv.one_of(UNIT_AMPERE),
            cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MAX_ALLOWABLE_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TOTAL_KWH): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SERIAL): text_sensor.text_sensor_schema(
                icon=ICON_CURRENT_AC,
            ),
            cv.Optional(CONF_PHASE_1_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_2_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_3_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_1_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_2_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_3_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
                icon=ICON_NUMERIC,
            ),
            cv.Optional(CONF_VIN): text_sensor.text_sensor_schema(
                icon=ICON_CAR,
            ),
            cv.Optional(CONF_ACTUAL_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_STATE): sensor.sensor_schema(
                icon=ICON_CURRENT_AC,
                accuracy_decimals=0,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    validate_min_max
)

async def setup_sensor(config, key, hub):
    if sensor_config := config.get(key):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))

async def setup_text_sensor(config, key, hub):
    if sensor_config := config.get(key):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))

async def to_code(config):
    num_var = await number.new_number(
        config,
        min_value=config[CONF_MIN_CURRENT],
        max_value=config[CONF_MAX_CURRENT],
        step=config[CONF_STEP],
    )

    await cg.register_component(num_var, config)
    await uart.register_uart_device(num_var, config)

    cg.add(num_var.set_min_current(config[CONF_MIN_CURRENT]))
    cg.add(num_var.set_max_current(config[CONF_MAX_CURRENT]))
    cg.add(num_var.set_twcid(config[CONF_TWCID]))

    for key in TYPES:
        await setup_sensor(config, key, num_var)

    for key in TEXT_TYPES:
        await setup_text_sensor(config, key, num_var)

    pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
    cg.add(num_var.set_flow_control_pin(pin))
