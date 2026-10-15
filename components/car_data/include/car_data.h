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
} car_engine_data_t;

typedef struct
{
    car_engine_data_t engine;
} car_data_t;

void car_data_init(void);

bool car_data_update(car_data_id_t id, int32_t value, uint32_t timestamp_ms);
bool car_data_get(car_data_id_t id, int32_t *value);
bool car_data_get_full(car_data_id_t id, car_value_t *out);
bool car_data_get_snapshot(car_data_t *out);
