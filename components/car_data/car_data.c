#include <stdio.h>
#include "car_data.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "CAR_DATA";

static SemaphoreHandle_t s_lock;

static car_data_t s_car;

static car_value_t *car_data_get_field(car_data_id_t id)
{
  switch (id)
  {
  case CAR_DATA_RPM:
    return &s_car.engine.rpm;

  case CAR_DATA_SPEED_KPH:
    return &s_car.engine.speed_kph;

  case CAR_DATA_COOLANT_TEMP_C:
    return &s_car.engine.coolant_temp_c;

  case CAR_DATA_THROTTLE_PCT:
    return &s_car.engine.throttle_pct;

  case CAR_DATA_ENGINE_LOAD_PCT:
    return &s_car.engine.engine_load_pct;
  case CAR_DATA_FUEL_LEVEL_PCT:
    return &s_car.engine.fuel_level_pct;
  case CAR_DATA_BAROMETRIC_PRESSURE_KPA:
    return &s_car.engine.barometric_pressure_kpa;
  case CAR_DATA_MODULE_VOLTAGE_MV:
    return &s_car.engine.module_voltage_mv;
  case CAR_DATA_RELATIVE_THROTTLE_PCT:
    return &s_car.engine.relative_throttle_pct;
  case CAR_DATA_AMBIENT_TEMP_C:
    return &s_car.engine.ambient_temp_c;
  case CAR_DATA_THROTTLE_ACTUATOR_PCT:
    return &s_car.engine.throttle_actuator_pct;
  case CAR_DATA_OIL_TEMP_C:
    return &s_car.engine.oil_temp_c;
  case CAR_DATA_MANIFOLD_PRESSURE_KPA:
    return &s_car.engine.manifold_pressure_kpa;
  case CAR_DATA_INTAKE_AIR_TEMP_C:
    return &s_car.engine.intake_air_temp_c;
  case CAR_DATA_MASS_AIR_FLOW_MG_S:
    return &s_car.engine.mass_air_flow_mg_s;
  case CAR_DATA_FUEL_RATE_ML_H:
    return &s_car.engine.fuel_rate_ml_h;
  case CAR_DATA_PEDAL_POSITION_D_PCT:
    return &s_car.engine.pedal_position_d_pct;
  case CAR_DATA_PEDAL_POSITION_E_PCT:
    return &s_car.engine.pedal_position_e_pct;
  case CAR_DATA_PEDAL_POSITION_F_PCT:
    return &s_car.engine.pedal_position_f_pct;
  case CAR_DATA_RELATIVE_PEDAL_POSITION_PCT:
    return &s_car.engine.relative_pedal_position_pct;
  case CAR_DATA_THROTTLE_POSITION_B_PCT:
    return &s_car.engine.throttle_position_b_pct;
  case CAR_DATA_THROTTLE_POSITION_C_PCT:
    return &s_car.engine.throttle_position_c_pct;
  case CAR_DATA_SHORT_FUEL_TRIM_BANK_1_PCT_X100:
    return &s_car.engine.short_fuel_trim_bank_1_pct_x100;
  case CAR_DATA_LONG_FUEL_TRIM_BANK_1_PCT_X100:
    return &s_car.engine.long_fuel_trim_bank_1_pct_x100;
  case CAR_DATA_SHORT_FUEL_TRIM_BANK_2_PCT_X100:
    return &s_car.engine.short_fuel_trim_bank_2_pct_x100;
  case CAR_DATA_LONG_FUEL_TRIM_BANK_2_PCT_X100:
    return &s_car.engine.long_fuel_trim_bank_2_pct_x100;
  case CAR_DATA_ENGINE_RUN_TIME_S:
    return &s_car.engine.engine_run_time_s;
  case CAR_DATA_TIMING_ADVANCE_DEG_X10:
    return &s_car.engine.timing_advance_deg_x10;
  case CAR_DATA_DEMANDED_TORQUE_PCT:
    return &s_car.engine.demanded_torque_pct;
  case CAR_DATA_ACTUAL_TORQUE_PCT:
    return &s_car.engine.actual_torque_pct;
  case CAR_DATA_REFERENCE_TORQUE_NM:
    return &s_car.engine.reference_torque_nm;
  case CAR_DATA_FUEL_RAIL_PRESSURE_KPA:
    return &s_car.engine.fuel_rail_pressure_kpa;
  case CAR_DATA_BOOST_PRESSURE_KPA:
    return &s_car.engine.boost_pressure_kpa;
  case CAR_DATA_FUEL_CONSUMPTION_ML_100KM:
    return &s_car.engine.fuel_consumption_ml_100km;
  case CAR_DATA_ACTUAL_TORQUE_NM:
    return &s_car.engine.actual_torque_nm;

  default:
    return NULL;
  }
}

static bool is_fresh(const car_value_t *field, uint32_t now, uint32_t max_age_ms)
{
  return field->valid && (uint32_t)(now - field->timestamp_ms) <= max_age_ms;
}

// Caller holds s_lock. Derived values require valid, reasonably recent inputs.
static void refresh_derived(uint32_t now)
{
  car_engine_data_t *engine = &s_car.engine;
  engine->boost_pressure_kpa.valid =
      is_fresh(&engine->manifold_pressure_kpa, now, 3000) &&
      is_fresh(&engine->barometric_pressure_kpa, now, 15000);
  if (engine->boost_pressure_kpa.valid)
  {
    engine->boost_pressure_kpa.value =
        engine->manifold_pressure_kpa.value - engine->barometric_pressure_kpa.value;
    engine->boost_pressure_kpa.timestamp_ms = engine->manifold_pressure_kpa.timestamp_ms;
  }

  engine->fuel_consumption_ml_100km.valid =
      is_fresh(&engine->speed_kph, now, 2000) && engine->speed_kph.value > 0 &&
      is_fresh(&engine->fuel_rate_ml_h, now, 5000);
  if (engine->fuel_consumption_ml_100km.valid)
  {
    engine->fuel_consumption_ml_100km.value =
        (int32_t)((int64_t)engine->fuel_rate_ml_h.value * 100 / engine->speed_kph.value);
    engine->fuel_consumption_ml_100km.timestamp_ms = engine->fuel_rate_ml_h.timestamp_ms;
  }

  engine->actual_torque_nm.valid =
      is_fresh(&engine->actual_torque_pct, now, 5000) &&
      is_fresh(&engine->reference_torque_nm, now, 90000);
  if (engine->actual_torque_nm.valid)
  {
    engine->actual_torque_nm.value =
        engine->actual_torque_pct.value * engine->reference_torque_nm.value / 100;
    engine->actual_torque_nm.timestamp_ms = engine->actual_torque_pct.timestamp_ms;
  }
}

void car_data_init(void)
{

  if (s_lock == NULL)
  {
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);
  }

  xSemaphoreTake(s_lock, portMAX_DELAY);
  memset(&s_car, 0, sizeof(s_car));
  xSemaphoreGive(s_lock);

  ESP_LOGI(TAG, "Car data initialized");
}

bool car_data_update(car_data_id_t id, int32_t value, uint32_t timestamp_ms)
{
  car_value_t *field = car_data_get_field(id);

  if (s_lock == NULL || field == NULL)
    return false;

  xSemaphoreTake(s_lock, portMAX_DELAY);
  field->timestamp_ms = timestamp_ms;
  field->value = value;
  field->valid = true;
  xSemaphoreGive(s_lock);

  return true;
}

bool car_data_get(car_data_id_t id, int32_t *value)
{
  if (s_lock == NULL || value == NULL)
    return false;

  car_value_t *field = car_data_get_field(id);

  xSemaphoreTake(s_lock, portMAX_DELAY);
  refresh_derived(pdTICKS_TO_MS(xTaskGetTickCount()));

  if (!field || !value || !field->valid)
  {
    xSemaphoreGive(s_lock);
    return false;
  }

  *value = field->value;

  xSemaphoreGive(s_lock);

  return true;
}

bool car_data_get_full(car_data_id_t id, car_value_t *out)
{
  if (s_lock == NULL || out == NULL)
    return false;

  car_value_t *field = car_data_get_field(id);

  xSemaphoreTake(s_lock, portMAX_DELAY);
  refresh_derived(pdTICKS_TO_MS(xTaskGetTickCount()));

  if (!field || !out || !field->valid)
  {
    xSemaphoreGive(s_lock);
    return false;
  }

  *out = *field;

  xSemaphoreGive(s_lock);

  return true;
}

bool car_data_get_snapshot(car_data_t *out)
{
  if (s_lock == NULL || out == NULL)
    return false;

  xSemaphoreTake(s_lock, portMAX_DELAY);
  refresh_derived(pdTICKS_TO_MS(xTaskGetTickCount()));
  memcpy(out, &s_car, sizeof(s_car));
  xSemaphoreGive(s_lock);

  return true;
}

bool car_data_invalidate(car_data_id_t id)
{
  car_value_t *field = car_data_get_field(id);
  if (s_lock == NULL || field == NULL)
    return false;

  xSemaphoreTake(s_lock, portMAX_DELAY);
  field->valid = false;
  xSemaphoreGive(s_lock);
  return true;
}

