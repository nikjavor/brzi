#include <stdio.h>
#include "car_data.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

  default:
    return NULL;
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
  memcpy(out, &s_car, sizeof(s_car));
  xSemaphoreGive(s_lock);

  return true;
}

