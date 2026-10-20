#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "driver/gpio.h"
#include "obd_signals.h"

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
  OBD_SUPPORT_UNKNOWN,
  OBD_SUPPORT_SUPPORTED,
  OBD_SUPPORT_UNSUPPORTED,
} obd_support_t;

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
// Call discovery and queries from the same task. Each support page waits timeout ticks.
bool obd_twai_discover_supported_signals(TickType_t timeout);
obd_support_t obd_twai_signal_support(obd_signal_t signal);
uint32_t obd_twai_get_rx_drop_count(void);




