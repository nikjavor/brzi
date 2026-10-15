#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "driver/gpio.h"

typedef struct
{
  gpio_num_t tx_gpio;
  gpio_num_t rx_gpio;
  uint32_t request_period_ms;
  uint32_t requester_stack_size;
  uint32_t handler_stack_size;
  int task_priority;
  uint8_t queue_len;
} obd_twai_config_t;

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

#define OBD_TWAI_DEFAULT_CONFIG() ((obd_twai_config_t){ \
    .tx_gpio = GPIO_NUM_16,                              \
    .rx_gpio = GPIO_NUM_17,                              \
    .request_period_ms = 1000,                          \
    .requester_stack_size = 2048,                       \
    .handler_stack_size = 2048,                         \
    .task_priority = 5,                                 \
    .queue_len = 16})


esp_err_t obd_twai_start(const obd_twai_config_t *);
bool obd_twai_query_signal(obd_signal_t, obd_decoded_value_t *, TickType_t);




