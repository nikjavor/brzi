#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    OBD_SIGNAL_ENGINE_LOAD,
    OBD_SIGNAL_COOLANT_TEMP,
    OBD_SIGNAL_RPM,
    OBD_SIGNAL_SPEED,
    OBD_SIGNAL_THROTTLE_POSITION,
    OBD_SIGNAL_FUEL_LEVEL,
    OBD_SIGNAL_BAROMETRIC_PRESSURE,
    OBD_SIGNAL_BATTERY_VOLTAGE,
    OBD_SIGNAL_RELATIVE_THROTTLE_POSITION,
    OBD_SIGNAL_AMBIENT_TEMPERATURE,
    OBD_SIGNAL_THROTTLE_ACTUATOR,
    OBD_SIGNAL_OIL_TEMP,
    OBD_SIGNAL_MANIFOLD_PRESSURE,
    OBD_SIGNAL_INTAKE_AIR_TEMP,
    OBD_SIGNAL_MASS_AIR_FLOW,
    OBD_SIGNAL_FUEL_RATE,
    OBD_SIGNAL_PEDAL_POSITION_D,
    OBD_SIGNAL_PEDAL_POSITION_E,
    OBD_SIGNAL_PEDAL_POSITION_F,
    OBD_SIGNAL_RELATIVE_PEDAL_POSITION,
    OBD_SIGNAL_THROTTLE_POSITION_B,
    OBD_SIGNAL_THROTTLE_POSITION_C,
    OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_1,
    OBD_SIGNAL_LONG_FUEL_TRIM_BANK_1,
    OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_2,
    OBD_SIGNAL_LONG_FUEL_TRIM_BANK_2,
    OBD_SIGNAL_ENGINE_RUN_TIME,
    OBD_SIGNAL_TIMING_ADVANCE,
    OBD_SIGNAL_DEMANDED_TORQUE,
    OBD_SIGNAL_ACTUAL_TORQUE,
    OBD_SIGNAL_REFERENCE_TORQUE,
    OBD_SIGNAL_FUEL_RAIL_PRESSURE,
    OBD_SIGNAL_COUNT
} obd_signal_t;

typedef enum
{
    OBD_VALUE_U16,
    OBD_VALUE_I16,
    OBD_VALUE_U32,
    OBD_VALUE_FLOAT,
} obd_value_type_t;

typedef struct
{
    obd_signal_t signal;
    obd_value_type_t type;
    union
    {
        uint16_t u16;
        int16_t i16;
        uint32_t u32;
        float f32;
    } value;
} obd_decoded_value_t;

typedef struct
{
    const char *name;
    const char *unit;
    uint8_t pid;
    obd_value_type_t value_type;
} obd_signal_info_t;

const obd_signal_info_t *obd_signal_get_info(obd_signal_t signal);

// Decode a Mode 01 ISO-TP single frame, including its length/service/PID bytes.
// Values use the integer units returned by obd_signal_get_info().
bool obd_signal_decode(obd_signal_t signal, const uint8_t *frame, size_t len,
                       obd_decoded_value_t *out);
