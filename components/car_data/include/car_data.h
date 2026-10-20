// lib/car_data/car_data.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    CAR_DATA_RPM,
    CAR_DATA_SPEED_KPH,
    CAR_DATA_COOLANT_TEMP_C,
    CAR_DATA_THROTTLE_PCT,
    CAR_DATA_ENGINE_LOAD_PCT,
    CAR_DATA_FUEL_LEVEL_PCT,
    CAR_DATA_BAROMETRIC_PRESSURE_KPA,
    CAR_DATA_MODULE_VOLTAGE_MV,
    CAR_DATA_RELATIVE_THROTTLE_PCT,
    CAR_DATA_AMBIENT_TEMP_C,
    CAR_DATA_THROTTLE_ACTUATOR_PCT,
    CAR_DATA_OIL_TEMP_C,
    CAR_DATA_MANIFOLD_PRESSURE_KPA,
    CAR_DATA_INTAKE_AIR_TEMP_C,
    CAR_DATA_MASS_AIR_FLOW_MG_S,
    CAR_DATA_FUEL_RATE_ML_H,
    CAR_DATA_PEDAL_POSITION_D_PCT,
    CAR_DATA_PEDAL_POSITION_E_PCT,
    CAR_DATA_PEDAL_POSITION_F_PCT,
    CAR_DATA_RELATIVE_PEDAL_POSITION_PCT,
    CAR_DATA_THROTTLE_POSITION_B_PCT,
    CAR_DATA_THROTTLE_POSITION_C_PCT,
    CAR_DATA_SHORT_FUEL_TRIM_BANK_1_PCT_X100,
    CAR_DATA_LONG_FUEL_TRIM_BANK_1_PCT_X100,
    CAR_DATA_SHORT_FUEL_TRIM_BANK_2_PCT_X100,
    CAR_DATA_LONG_FUEL_TRIM_BANK_2_PCT_X100,
    CAR_DATA_ENGINE_RUN_TIME_S,
    CAR_DATA_TIMING_ADVANCE_DEG_X10,
    CAR_DATA_DEMANDED_TORQUE_PCT,
    CAR_DATA_ACTUAL_TORQUE_PCT,
    CAR_DATA_REFERENCE_TORQUE_NM,
    CAR_DATA_FUEL_RAIL_PRESSURE_KPA,
    CAR_DATA_BOOST_PRESSURE_KPA,
    CAR_DATA_FUEL_CONSUMPTION_ML_100KM,
    CAR_DATA_ACTUAL_TORQUE_NM,

    CAR_DATA_COUNT,
    CAR_DATA_ID_INVALID,
} car_data_id_t;

typedef struct
{
    bool valid;
    uint32_t timestamp_ms;
    int32_t value;
} car_value_t;

typedef struct
{
    car_value_t rpm;            // rpm
    car_value_t speed_kph;      // km/h
    car_value_t coolant_temp_c; // Celsius
    car_value_t throttle_pct;   // percent
    car_value_t ambient_temp_c; // Celsius
    car_value_t engine_load_pct;
    car_value_t fuel_level_pct;
    car_value_t barometric_pressure_kpa;
    car_value_t module_voltage_mv; // ECU supply voltage, not battery state of charge
    car_value_t relative_throttle_pct;
    car_value_t throttle_actuator_pct;
    car_value_t oil_temp_c;
    car_value_t manifold_pressure_kpa; // absolute pressure
    car_value_t intake_air_temp_c;
    car_value_t mass_air_flow_mg_s; // divide by 1000 for g/s
    car_value_t fuel_rate_ml_h;     // divide by 1000 for L/h
    car_value_t pedal_position_d_pct;
    car_value_t pedal_position_e_pct;
    car_value_t pedal_position_f_pct;
    car_value_t relative_pedal_position_pct;
    car_value_t throttle_position_b_pct;
    car_value_t throttle_position_c_pct;
    car_value_t short_fuel_trim_bank_1_pct_x100;
    car_value_t long_fuel_trim_bank_1_pct_x100;
    car_value_t short_fuel_trim_bank_2_pct_x100;
    car_value_t long_fuel_trim_bank_2_pct_x100;
    car_value_t engine_run_time_s;
    car_value_t timing_advance_deg_x10;
    car_value_t demanded_torque_pct;
    car_value_t actual_torque_pct;
    car_value_t reference_torque_nm;
    car_value_t fuel_rail_pressure_kpa;
    car_value_t boost_pressure_kpa;          // MAP minus barometric pressure
    car_value_t fuel_consumption_ml_100km;  // divide by 1000 for L/100 km
    car_value_t actual_torque_nm;           // ECU-reported percent * reference
} car_engine_data_t;

typedef struct
{
    car_engine_data_t engine;
} car_data_t;

void car_data_init(void);

bool car_data_update(car_data_id_t id, int32_t value, uint32_t timestamp_ms);
bool car_data_invalidate(car_data_id_t id);
bool car_data_get(car_data_id_t id, int32_t *value);
bool car_data_get_full(car_data_id_t id, car_value_t *out);
bool car_data_get_snapshot(car_data_t *out);
