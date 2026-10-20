#include "obd_signals.h"

typedef struct
{
    obd_signal_info_t info;
    uint8_t data_bytes;
    int32_t multiplier;
    int32_t offset;
    uint16_t divisor;
} obd_signal_def_t;

// value = (raw * multiplier + offset) / divisor.
// PID definitions: SAE J1979 Mode 01; see the reference links in README.md.
#define SIGNAL(id, pid_hex, label, units, type, bytes, mul, off, div) \
    [id] = {{label, units, pid_hex, type}, bytes, mul, off, div}

static const obd_signal_def_t signal_defs[OBD_SIGNAL_COUNT] = {
    SIGNAL(OBD_SIGNAL_ENGINE_LOAD, 0x04, "LOAD", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_COOLANT_TEMP, 0x05, "COOLANT", "C", OBD_VALUE_I16, 1, 1, -40, 1),
    SIGNAL(OBD_SIGNAL_RPM, 0x0C, "RPM", "rpm", OBD_VALUE_U16, 2, 1, 0, 4),
    SIGNAL(OBD_SIGNAL_SPEED, 0x0D, "SPEED", "km/h", OBD_VALUE_U16, 1, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_THROTTLE_POSITION, 0x11, "THROTTLE", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_FUEL_LEVEL, 0x2F, "FUEL_LEVEL", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_BAROMETRIC_PRESSURE, 0x33, "BARO", "kPa", OBD_VALUE_U16, 1, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_BATTERY_VOLTAGE, 0x42, "MODULE_VOLTAGE", "mV", OBD_VALUE_U16, 2, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_RELATIVE_THROTTLE_POSITION, 0x45, "REL_THROTTLE", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_AMBIENT_TEMPERATURE, 0x46, "AMBIENT", "C", OBD_VALUE_I16, 1, 1, -40, 1),
    SIGNAL(OBD_SIGNAL_THROTTLE_ACTUATOR, 0x4C, "COMMANDED_THROTTLE", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_OIL_TEMP, 0x5C, "OIL", "C", OBD_VALUE_I16, 1, 1, -40, 1),
    SIGNAL(OBD_SIGNAL_MANIFOLD_PRESSURE, 0x0B, "MAP", "kPa", OBD_VALUE_U16, 1, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_INTAKE_AIR_TEMP, 0x0F, "INTAKE_AIR", "C", OBD_VALUE_I16, 1, 1, -40, 1),
    SIGNAL(OBD_SIGNAL_MASS_AIR_FLOW, 0x10, "MAF", "mg/s", OBD_VALUE_U32, 2, 10, 0, 1),
    SIGNAL(OBD_SIGNAL_FUEL_RATE, 0x5E, "FUEL_RATE", "mL/h", OBD_VALUE_U32, 2, 50, 0, 1),
    SIGNAL(OBD_SIGNAL_PEDAL_POSITION_D, 0x49, "PEDAL_D", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_PEDAL_POSITION_E, 0x4A, "PEDAL_E", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_PEDAL_POSITION_F, 0x4B, "PEDAL_F", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_RELATIVE_PEDAL_POSITION, 0x5A, "REL_PEDAL", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_THROTTLE_POSITION_B, 0x47, "THROTTLE_B", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_THROTTLE_POSITION_C, 0x48, "THROTTLE_C", "%", OBD_VALUE_U16, 1, 100, 0, 255),
    SIGNAL(OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_1, 0x06, "STFT_B1", "0.01%", OBD_VALUE_I16, 1, 10000, -1280000, 128),
    SIGNAL(OBD_SIGNAL_LONG_FUEL_TRIM_BANK_1, 0x07, "LTFT_B1", "0.01%", OBD_VALUE_I16, 1, 10000, -1280000, 128),
    SIGNAL(OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_2, 0x08, "STFT_B2", "0.01%", OBD_VALUE_I16, 1, 10000, -1280000, 128),
    SIGNAL(OBD_SIGNAL_LONG_FUEL_TRIM_BANK_2, 0x09, "LTFT_B2", "0.01%", OBD_VALUE_I16, 1, 10000, -1280000, 128),
    SIGNAL(OBD_SIGNAL_ENGINE_RUN_TIME, 0x1F, "RUN_TIME", "s", OBD_VALUE_U16, 2, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_TIMING_ADVANCE, 0x0E, "IGNITION_ADVANCE", "0.1deg", OBD_VALUE_I16, 1, 5, -640, 1),
    SIGNAL(OBD_SIGNAL_DEMANDED_TORQUE, 0x61, "DEMANDED_TORQUE", "%", OBD_VALUE_I16, 1, 1, -125, 1),
    SIGNAL(OBD_SIGNAL_ACTUAL_TORQUE, 0x62, "ACTUAL_TORQUE", "%", OBD_VALUE_I16, 1, 1, -125, 1),
    SIGNAL(OBD_SIGNAL_REFERENCE_TORQUE, 0x63, "REFERENCE_TORQUE", "Nm", OBD_VALUE_U16, 2, 1, 0, 1),
    SIGNAL(OBD_SIGNAL_FUEL_RAIL_PRESSURE, 0x23, "FUEL_RAIL", "kPa", OBD_VALUE_U32, 2, 10, 0, 1),
};

#undef SIGNAL

const obd_signal_info_t *obd_signal_get_info(obd_signal_t signal)
{
    if ((unsigned)signal >= OBD_SIGNAL_COUNT)
        return NULL;

    return &signal_defs[signal].info;
}

bool obd_signal_decode(obd_signal_t signal, const uint8_t *frame, size_t len,
                       obd_decoded_value_t *out)
{
    if ((unsigned)signal >= OBD_SIGNAL_COUNT || frame == NULL || out == NULL || len < 4)
        return false;

    const obd_signal_def_t *def = &signal_defs[signal];
    if (frame[0] < 2u + def->data_bytes || frame[0] > 7 ||
        len < (size_t)frame[0] + 1u || frame[1] != 0x41 || frame[2] != def->info.pid)
        return false;

    uint16_t raw = frame[3];
    if (def->data_bytes == 2)
        raw = ((uint16_t)frame[3] << 8) | frame[4];

    int32_t value = ((int32_t)raw * def->multiplier + def->offset) / def->divisor;
    out->signal = signal;
    out->type = def->info.value_type;
    switch (out->type)
    {
    case OBD_VALUE_U16:
        out->value.u16 = (uint16_t)value;
        break;
    case OBD_VALUE_I16:
        out->value.i16 = (int16_t)value;
        break;
    case OBD_VALUE_U32:
        out->value.u32 = (uint32_t)value;
        break;
    default:
        return false;
    }
    return true;
}
